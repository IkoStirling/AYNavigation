#pragma once
#include <AYNavigation/Graph.h>
#include <algorithm>
#include <string>
#include <stdexcept>

namespace navexample {
using namespace ayt::navigation;
struct Motor final:ayt::navigation::Motor {
    MotionState body{{1,{-6,0,0}},{}};TaskId token=0;
    std::optional<MotionState> sample(AgentId id) const override {return id==1?std::optional<MotionState>{body}:std::nullopt;}
    bool drive(AgentId id,TaskId task,Vec3 velocity,Location target,double dt) override {
        if(id!=1 || target.region!=body.location.region || (token && task<token)) return false;
        token=task;body.velocity=velocity;body.location.position=body.location.position+velocity*dt;return true;
    }
    void stop(AgentId id,TaskId task) noexcept override {if(id==1 && token==task){token=0;body.velocity={};}}
    bool traversalPose(AgentId id,TaskId task,Location location) override {
        if(id!=1 || location.region!=body.location.region || (token && task<token)) return false;
        token=task;body={location,{}};return true;
    }
};
struct Obstacles final:ObstacleSource {
    bool enabled=false;
    ObstacleSnapshot query(AgentId,const MotionState&,double) const override {
        return {true,enabled?std::vector<Obstacle>{{100,{1,{-4,0,0}},{},.45}}:std::vector<Obstacle>{}};
    }
};
inline GraphDefinition initialGraph() {
    GraphDefinition d;d.stamp={1,1};
    d.nodes={{1,{1,{-6,0,0}}},{2,{1,{-2,0,0}}},{3,{1,{2,1.5,0}}},{4,{1,{6,1.5,0}}},
        {5,{1,{-2,0,4}}},{6,{1,{2,1.5,4}}}};
    d.links={{1,1,2,1.2,3,4},{2,2,1,1.2,3,4},{3,2,3,1.2,3,4,1,7,1,1},
        {4,3,2,1.2,3,4,1,7,1,1},{5,3,4,1.2,3,4},{6,4,3,1.2,3,4},
        {7,2,5,1.2,3,4},{8,5,6,1.2,3,5,1,8,1,1},{9,6,3,1.2,3,4}};
    return d;
}
struct World {
    Motor motor;Obstacles obstacles;
    GraphDefinition definition=initialGraph();
    std::shared_ptr<const GraphSpace> graph;
    std::unique_ptr<Navigator> navigator;
    int mode=1;NodeId selected=0;TaskId task=0;double editHeight=0;
    std::wstring message=L"Right-click node 4 to follow the path and jump.";
    World(){reset(1);}
    void reset(int nextMode) {
        navigator.reset();mode=nextMode;motor.body={{1,{-6,0,0}},{}};motor.token=0;selected=0;editHeight=0;
        Profile profile;profile.capabilities=1;profile.radius=.2;profile.maxSpeed=2;profile.maxAcceleration=12;
        Config config;config.maxBlockedSteps=90;config.maxReplans=4;
        std::shared_ptr<const Space> space;std::shared_ptr<Planner> planner;
        if(mode==1) {definition=initialGraph();graph=GraphSpace::compile(definition).value;space=graph;planner=std::make_shared<GraphPlanner>();}
        else {
            space=VolumeSpace::compile({std::uint64_t(mode),1},{{1,{-12,-10,-12},{12,10,12}}}).value;
            planner=std::make_shared<DirectPlanner>();profile.steering=mode==2?Steering::Planar:Steering::Spatial;
        }
        navigator=std::make_unique<Navigator>(space,planner,std::make_shared<ConservativeAvoidance>(),motor,obstacles,config);
        if(navigator->registerTraversal(1,std::make_shared<TimedArcTraversal>(30,2))!=Error::None || navigator->attach(1,profile)!=Error::None)
            throw std::runtime_error("Cannot construct navigation example");
        message=mode==1?L"Authored 3D corridors + special jump links":mode==2?L"Planar movement: targets stay on the horizontal plane":L"Free 3D motion: choose a target height with Up / Down";
    }
    bool publish(GraphDefinition candidate) {
        if(mode!=1) return false;
        candidate.stamp={1,definition.stamp.version+1};const auto compiled=GraphSpace::compile(candidate);
        if(!compiled || navigator->publish(compiled.value)!=Error::None) {message=L"This edit creates an invalid path.";return false;}
        definition=std::move(candidate);graph=compiled.value;message=L"Path updated; active navigation will replan.";return true;
    }
    bool addNode(Vec3 position) {
        auto candidate=definition;NodeId id=1;for(const auto& node:candidate.nodes) id=std::max(id,node.id+1);
        candidate.nodes.push_back({id,{1,position}});if(!publish(std::move(candidate))) return false;selected=id;return true;
    }
    bool connect(NodeId from,NodeId to,bool jump) {
        if(!from || !to || from==to) return false;
        auto candidate=definition;LinkId id=1;for(const auto& link:candidate.links) id=std::max(id,link.id+1);
        Link link;link.id=id;link.from=from;link.to=to;link.width=1.2;link.height=3;link.cost=4;
        if(jump){link.traversal=1;link.requiredCapabilities=1;link.resource=1000+id;link.capacity=1;}
        candidate.links.push_back(link);return publish(std::move(candidate));
    }
    bool moveNode(NodeId id,Vec3 position) {
        auto candidate=definition;for(auto& node:candidate.nodes) if(node.id==id) node.location.position=position;
        return publish(std::move(candidate));
    }
    void go(Location goal) {
        if(!navigator->canInterrupt(1,task)) {message=L"Traversal in progress: finish landing before changing orders.";return;}
        const auto error=navigator->begin(1,task+1,goal);
        if(error==Error::None){++task;message=L"Moving to the selected destination.";}else message=L"Destination is outside the allowed space.";
    }
    void tick(){const auto state=navigator->state(1);if(state && state->task && state->phase!=Phase::Idle)(void)navigator->update(1,state->task);}
};
} // namespace navexample
