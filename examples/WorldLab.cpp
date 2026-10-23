#include "WorldLab.h"
#include <AYEntity/EntityModule.h>
#include <AYEntity/EntityImpl.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace navlab {
GraphDefinition initialGraph() {
    GraphDefinition d;d.stamp={1,1};
    d.nodes={{1,{1,{-8,0,-1.5}}},{2,{1,{-8,0,0}}},{3,{1,{-8,0,1.5}}},
        {4,{1,{-4,0,0}}},{5,{1,{-1,0,0}}},{6,{1,{2,1,0}}},{7,{1,{5,1,0}}},
        {8,{1,{8,1,-1.5}}},{9,{1,{8,1,0}}},{10,{1,{8,1,1.5}}}};
    const auto add=[&](NodeId from,NodeId to,double width,std::uint64_t resource=0,TraversalKind traversal=0) {
        d.links.push_back({d.links.size()+1,from,to,width,3,4,traversal?1u:0u,resource,resource?1u:0u,traversal,true});
    };
    for(NodeId n=1;n<=3;++n){add(n,4,1.4,69);add(4,n,1.4,69);}
    add(4,5,.9,70);add(5,4,.9,70);add(5,6,1.2,71,1);add(6,5,1.2,71,1);
    add(6,7,1.4);add(7,6,1.4);
    for(NodeId n=8;n<=10;++n){add(7,n,1.4);add(n,7,1.4);}
    return d;
}
WorldLab::WorldLab() {
    if(world.isInitialized())throw std::runtime_error("WorldLab needs exclusive ownership of the process World");
    if(!ayt::entity::registerEntityCoreComponents(ayt::entity::ComponentRegistry::instance()))throw std::runtime_error("Cannot register Entity core");
    profile.radius=.18;profile.capabilities=1;profile.maxSpeed=2.5;profile.maxAcceleration=12;
    profile.arrivalDistance=.035;profile.arrivalSpeed=.06;obstacles.lab=this;
    if(!install(initialGraph()))throw std::runtime_error(message);
    refreshCamera(1280,800);
}
WorldLab::~WorldLab(){shutdown();}
void WorldLab::shutdown() {
    if(initialized)world.shutdown();initialized=false;system=nullptr;
    navigator.reset();motor.reset();units.clear();
}
bool WorldLab::install(GraphDefinition candidate) {
    const auto compiled=GraphSpace::compile(candidate);
    if(!compiled || candidate.nodes.size()<3 || std::any_of(candidate.nodes.begin(),candidate.nodes.end(),[](auto& n){return n.location.region!=1;})) {
        message="Lab requires a valid graph with at least three nodes in region1; existing content retained.";return false;
    }
    if(navigator)for(const auto handle:units) {
        const auto id=behavior::unitId(handle);const auto state=system->runtime().state(id);
        if(state&&state->active&&!navigator->canInterrupt(id,state->active->id)) {message="Finish the locked traversal before loading/resetting.";return false;}
    }
    shutdown();definition=compiled.value->definition();graph=compiled.value;
    if(!world.initialize()){message="World initialization failed.";return false;}initialized=true;
    motor=std::make_unique<RootKinematicMotor>(world);
    Config config;config.maxBlockedSteps=600;config.maxReplans=4;
    navigator=std::make_unique<Navigator>(graph,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),*motor,obstacles,config);
    if(navigator->registerTraversal(1,std::make_shared<TimedArcTraversal>(36,1.6))!=Error::None)throw std::runtime_error("Cannot register jump");
    const auto installed=behavior::installBehavior(world);if(!installed)throw std::runtime_error("Cannot install BehaviorSystem");system=installed.value;
    if(system->registerExecutor(kNavigate,std::make_shared<NavigationExecutor>(*navigator))!=behavior::Error::None ||
       system->registerExecutor(behavior::kWait,std::make_shared<behavior::WaitExecutor>())!=behavior::Error::None)
        throw std::runtime_error("Cannot register navigation/wait executors");
    for(std::size_t i=0;i<3;++i) {
        auto* entity=world.createSpatialEntity();entity->setName(("Unit "+std::to_string(i+1)).c_str());const auto p=definition.nodes[i].location.position;
        if(!entity->setWorldPose({float(p.x),float(p.y),float(p.z)},{0,0,0,1}))throw std::runtime_error("Invalid spawn");
        const auto handle=world.getEntityHandle(entity->getId());units.push_back(handle);
        if(navigator->attach(behavior::unitId(handle),profile)!=Error::None||system->track(handle)!=behavior::Error::None)throw std::runtime_error("Cannot attach unit");
    }
    selection=behavior::SelectionSet{};gesture.cancel();reports.clear();events.clear();ticks=0;obstacle=false;
    message="Three units ready. Drag to select; right-click a destination.";return true;
}
bool WorldLab::idle() const {
    if(!system)return true;
    for(const auto handle:units)if(const auto state=system->runtime().state(behavior::unitId(handle));state && state->active)return false;
    return true;
}
bool WorldLab::edit(GraphDefinition candidate) {
    if(!idle()){message="Stop active work before editing the graph.";return false;}
    candidate.stamp={definition.stamp.space,definition.stamp.version+1};
    const auto compiled=GraphSpace::compile(candidate);
    if(!compiled){message="Invalid graph edit: previous graph retained.";return false;}
    for(const auto handle:units) {
        const auto state=motor->sample(behavior::unitId(handle));
        if(!state||!compiled.value->contains(state->location,profile)){message="Edit would strand a unit outside the network; reset/load instead.";return false;}
    }
    if(navigator->publish(compiled.value)!=Error::None){message="Cannot publish edited graph.";return false;}
    definition=compiled.value->definition();graph=compiled.value;message="Graph edit published.";return true;
}
bool WorldLab::load(const std::filesystem::path& path) {
    const auto loaded=readGraphFile(path);
    if(!loaded){message=loaded.diagnostics.empty()?"Load failed.":"Line "+std::to_string(loaded.diagnostics.front().line)+": "+loaded.diagnostics.front().message;return false;}
    if(!install(loaded.definition))return false;
    message="Loaded graph; units and task queues reset. Diagnostics: "+std::to_string(loaded.diagnostics.size());return true;
}
bool WorldLab::save(const std::filesystem::path& path,bool replace) {
    const auto error=writeGraphFile(path,definition,replace);
    message=error==AssetError::None?"Graph saved.":error==AssetError::Exists?"File already exists; explicit overwrite required.":"Save failed; previous file retained.";
    return error==AssetError::None;
}
ObstacleSnapshot WorldLab::Obstacles::query(AgentId id,const MotionState& state,double) const {
    ObstacleSnapshot result;
    for(const auto handle:lab->units) {
        const auto other=behavior::unitId(handle);if(other==id)continue;
        const auto body=lab->motor->sample(other);
        if(!body){result.valid=false;return result;}
        if(body->location.region==state.location.region)result.obstacles.push_back({other,body->location,body->velocity,lab->profile.radius});
    }
    if(lab->obstacle && state.location.region==1)result.obstacles.push_back({UINT64_MAX,{1,{-2.5,0,0}},{},.3});
    return result;
}
void WorldLab::tick() {
    world.fixedUpdate(1.f/30.f);++ticks;
    auto added=system->drainEvents();events.insert(events.end(),added.begin(),added.end());
    if(events.size()>64)events.erase(events.begin(),events.end()-64);
    std::vector<AgentId> live;
    for(const auto handle:units)if(world.getEntityByHandle(handle))live.push_back(behavior::unitId(handle));
    (void)selection.retain(live);
}
std::vector<Location> WorldLab::goals(Location center) const {
    std::vector<Location> result;
    const auto append=[&](Location value) {
        if(!graph->contains(value,profile))return;
        for(const auto& old:result)if(old.region==value.region&&length(old.position-value.position)<.65)return;
        result.push_back(value);
    };
    // Finite example policy: authored nearby nodes first, then projected offsets. No general formation service.
    std::vector<Node> nodes=definition.nodes;
    std::stable_sort(nodes.begin(),nodes.end(),[&](auto& a,auto& b){return length(a.location.position-center.position)<length(b.location.position-center.position);});
    for(const auto& node:nodes)if(node.location.region==center.region&&length(node.location.position-center.position)<=2)append(node.location);
    for(int ring=0;ring<=3;++ring)for(int x=-ring;x<=ring;++x)for(int z=-ring;z<=ring;++z) {
        if(std::max(std::abs(x),std::abs(z))!=ring)continue;
        const auto snapped=graph->nearest({center.region,center.position+Vec3{double(x)*.8,0,double(z)*.8}},profile,.5);
        if(snapped)append(snapped.value);
    }
    return result;
}
bool WorldLab::owns(AgentId id) const {return std::any_of(units.begin(),units.end(),[&](auto handle){return behavior::unitId(handle)==id;});}
void WorldLab::issue(Location center,behavior::SubmitMode mode) {
    reports.clear();auto slots=goals(center);std::vector<Location> assigned;GraphPlanner planner;
    for(const auto id:selection.selected()) {
        CommandReport report;report.unit=id;bool found=false;const auto motion=motor->sample(id);
        if(!owns(id)||!motion){report.behaviorError=behavior::Error::UnknownUnit;reports.push_back(report);continue;}
        const auto current=unit(id);
        if(mode!=behavior::SubmitMode::Append && current->active && !current->active->interruptibleNow) {
            report.behaviorError=behavior::Error::NotInterruptible;reports.push_back(report);continue;
        }
        Location from=motion->location;
        if(mode==behavior::SubmitMode::Append) {
            const auto predict=[&](const behavior::TaskView& task) {
                const auto* point=std::get_if<behavior::Point3>(&task.order.target);
                if(task.order.kind!=kNavigate || !point || task.order.payload.size()!=8)return;
                RegionId region=0;for(unsigned i=0;i<8;++i)region|=RegionId(task.order.payload[i])<<(i*8);
                from={region,{point->x,point->y,point->z}};
            };
            if(current->active)predict(*current->active);
            for(auto task=current->suspended.rbegin();task!=current->suspended.rend();++task)predict(*task);
            for(const auto& task:current->queued)predict(task);
        }
        for(const auto slot:slots) {
            if(std::any_of(assigned.begin(),assigned.end(),[&](auto& value){return value.region==slot.region&&length(value.position-slot.position)<.65;}))continue;
            bool occupied=false;
            for(const auto handle:units) {
                const auto other=behavior::unitId(handle);if(std::binary_search(selection.selected().begin(),selection.selected().end(),other))continue;
                const auto body=motor->sample(other);if(body&&body->location.region==slot.region&&length(body->location.position-slot.position)<.65)occupied=true;
            }
            if(occupied)continue;
            PlanRequest request;request.id=1;request.agent=id;request.task=1;request.space=graph;request.from=from;
            request.goal=slot;request.profile=profile;request.traversals={1};
            if(planner.poll(request).status!=PlanStatus::Found)continue;
            const std::array handles{behavior::entityHandle(id)};
            const auto rows=system->submit(handles,navigationOrder(slot,1,mode==behavior::SubmitMode::Interrupt?100:0),mode);
            report.goal=slot;report.behaviorError=rows[0].error;report.task=rows[0].task;found=true;
            if(report.behaviorError==behavior::Error::None)assigned.push_back(slot);
            break;
        }
        if(!found)report.navigationError=Error::NoPath;
        reports.push_back(report);
    }
    message=reports.empty()?"Select units first.":"Command results shown per unit; rejected commands retain previous tasks.";
}
void WorldLab::stop() {
    reports.clear();for(const auto id:selection.selected())reports.push_back({id,system->stop(behavior::entityHandle(id))});
    message="Stop results shown per unit. In-air traversal stays locked until landing.";
}
void WorldLab::wait() {
    reports.clear();for(const auto id:selection.selected()) {
        const std::array handles{behavior::entityHandle(id)};const auto row=system->submit(handles,behavior::waitOrder(60,1),behavior::SubmitMode::Append)[0];
        reports.push_back({id,row.error,Error::None,{},row.task});
    }
    message="Appended two seconds of work after navigation.";
}
void WorldLab::refreshCamera(float width,float height) {
    viewport={0,140,width,std::max(1.f,height-340)};
    camera.position={0,12.5f,-21.94f};camera.orientation={float(std::sin(.25)),0,0,float(std::cos(.25))};
    camera.projectionMode=ayt::entity::CameraProjection::Orthographic;camera.verticalSize=10;camera.aspect=viewport.width/viewport.height;
    camera.nearZ=.1f;camera.farZ=100;camera.identity=1;
    if(!ayt::entity::rebuildCameraFrame(camera))throw std::runtime_error("Invalid lab camera");
}
behavior::Point2 WorldLab::screen(Vec3 p) const {
    ayt::math::FVector3 output;if(!ayt::entity::worldToScreen(camera,viewport,{float(p.x),float(p.y),float(p.z)},output))return {-1e6,-1e6};
    return {output.x,output.y};
}
std::optional<Vec3> WorldLab::ground(behavior::Point2 p,double height) const {
    ayt::entity::CameraRay ray;
    if(!ayt::entity::screenPointToRay(camera,viewport,{float(p.x),float(p.y)},ray)||std::abs(ray.direction.y)<1e-6)return {};
    const double distance=(height-ray.origin.y)/ray.direction.y;if(distance<0)return {};
    return Vec3{ray.origin.x+ray.direction.x*distance,height,ray.origin.z+ray.direction.z*distance};
}
std::vector<behavior::ScreenCandidate> WorldLab::candidates() const {
    std::vector<behavior::SelectionProxy> proxies;for(auto handle:units)proxies.push_back({handle,{},14,true});
    const auto projected=behavior::projectSelection(world,camera,viewport,proxies,1);
    return projected?projected.value:std::vector<behavior::ScreenCandidate>{};
}
std::string WorldLab::status(AgentId id) const {
    const auto state=unit(id);const auto nav=navigation(id);if(!state)return "Removed";
    std::string result;
    if(state->active&&state->active->order.kind==behavior::kWait)result="Working";
    else if(!nav)result="Idle";
    else switch(nav->phase) {
        case Phase::Idle:result="Idle";break;case Phase::Planning:result="Planning";break;
        case Phase::Following:result="Moving";break;case Phase::WaitingForLink:result="Waiting: link capacity";break;
        case Phase::Traversing:result="Jumping (stop locked)";break;case Phase::Blocked:result="Blocked: obstacle / neighbor";break;
        case Phase::Succeeded:result="Arrived";break;
        case Phase::Failed:
            switch(nav->error) {
                case Error::NoPath:result="Failed: no reachable path";break;
                case Error::Budget:result="Failed: planning/wait budget exhausted";break;
                case Error::Blocked:result="Failed: movement blocked";break;
                case Error::QueryFailed:result="Failed: obstacle query unavailable";break;
                case Error::TraversalFailed:result="Failed: special movement could not finish";break;
                case Error::MotionFailed:result="Failed: movement authority rejected motion";break;
                default:result="Failed: invalid route or configuration";break;
            }break;
    }
    return result+" | queue="+std::to_string(state->queued.size())+" suspended="+std::to_string(state->suspended.size());
}
} // namespace navlab
