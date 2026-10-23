// design reference: ../design.md; real planner routes, swept constraints, plugin/lifetime boundaries.
#include "Test.h"
#include "Support.h"
#include <limits>
using namespace ayt::navigation;
namespace {
void graph_validation_and_directed_interior_routes() {
    auto d=navtest::graph();auto space=GraphSpace::compile(d);CHECK(space);
    GraphPlanner planner;auto q=navtest::request(space.value,{1,{1,0,0}},{1,{2,0,0}});
    const auto result=planner.poll(q);CHECK(result.status==PlanStatus::Found);CHECK(result.route.steps.size()==1);
    CHECK(result.route.steps[0].from==q.from && result.route.steps[0].to==q.goal);
    q.from={1,{2,0,0}};q.goal={1,{1,0,0}};CHECK(planner.poll(q).status==PlanStatus::Unreachable);
    d.links.push_back(d.links[0]);CHECK(GraphSpace::compile(d).error==Error::Duplicate);
    d=navtest::graph();d.links[0].to=999;CHECK(!GraphSpace::compile(d));
    d=navtest::graph();d.links[0].width=std::numeric_limits<double>::quiet_NaN();CHECK(!GraphSpace::compile(d));
}
void graph_clearance_capabilities_special_links_and_budget() {
    auto d=navtest::graph();d.links[1].traversal=5;d.links[1].requiredCapabilities=4;
    auto space=GraphSpace::compile(d);CHECK(space);GraphPlanner planner;
    auto q=navtest::request(space.value,d.nodes[0].location,d.nodes[3].location);
    CHECK(planner.poll(q).status==PlanStatus::Unreachable);
    q.profile.capabilities=4;q.traversals={5};CHECK(planner.poll(q).status==PlanStatus::Found);
    q.profile.radius=2;CHECK(planner.poll(q).status==PlanStatus::Unreachable);
    q.profile=navtest::profile();q.profile.capabilities=4;q.expansionBudget=1;CHECK(planner.poll(q).error==Error::Budget);
    q.expansionBudget=100;q.profile.steering=Steering::Planar;q.profile.up={0,1,0};
    // A special action can change altitude; the ordinary horizontal last segment remains usable.
    CHECK(planner.poll(q).status==PlanStatus::Found);
}
void graph_rejects_forged_routes_and_constrains_whole_corridor() {
    const auto space=GraphSpace::compile(navtest::graph()).value;const auto p=navtest::profile();
    auto step=space->step(*space->link(1),{1,{0,0,0}},{1,{3,0,0}});CHECK(space->accepts(step,p));
    CHECK(space->permits(step,{1,{1,0,.5}},{1,{2,0,.5}},p));
    CHECK(!space->permits(step,{1,{1,0,.5}},{1,{2,0,1}},p));
    step.to={1,{6,1,0}};CHECK(!space->accepts(step,p));
    step=space->step(*space->link(1),{1,{0,0,0}},{1,{3,0,0}});step.capacity=1;CHECK(!space->accepts(step,p));
    const auto nearest=space->nearest({1,{1,0,4}},p,5);CHECK(nearest);CHECK(nearest.value.position==Vec3{1,0,0});
    CHECK(!space->nearest({1,{1,0,4}},p,1));
}
void navigator_follows_authored_three_dimensional_network() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    const auto definition=navtest::graph();const auto space=GraphSpace::compile(definition).value;
    Navigator n(space,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
    CHECK(n.attach(1,navtest::profile())==Error::None);CHECK(n.begin(1,1,definition.nodes.back().location)==Error::None);
    const auto result=navtest::finish(n);CHECK(result.status==TravelStatus::Succeeded);
    CHECK(length(motor.bodies.at(1).location.position-definition.nodes.back().location.position)<=.03);
    CHECK(length(motor.bodies.at(1).velocity)==0);CHECK(motor.commands.size()>50);
}
struct WrongPlanner final:Planner {
    PlanResult poll(const PlanRequest& q) override {
        Step step;step.from=q.from;step.to=q.goal;
        return {q.id,PlanStatus::Found,{q.space->stamp(),q.goal,{step}}};
    }
};
void custom_planner_results_are_independently_validated() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    auto space=GraphSpace::compile(navtest::graph()).value;
    Navigator n(space,std::make_shared<WrongPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles);
    CHECK(n.attach(1)==Error::None);CHECK(n.begin(1,1,{1,{9,1,0}})==Error::None);
    CHECK(n.update(1,1).error==Error::InvalidRoute);CHECK(motor.commands.empty());
}
struct PendingPlanner final:Planner {
    RequestId first=0;unsigned polls=0,cancelled=0;bool stale=false;
    PlanResult poll(const PlanRequest& q) override {
        ++polls;if(!first) first=q.id;
        if(stale) {stale=false;return {first,PlanStatus::Found,{}};}
        if(polls==1) return {q.id};
        return DirectPlanner{}.poll(q);
    }
    void cancel(RequestId) noexcept override {++cancelled;}
};
void pending_plans_stale_responses_and_old_cancellation_are_safe() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    auto space=VolumeSpace::compile({1,1},{{1,{-20,-20,-20},{20,20,20}}}).value;
    auto planner=std::make_shared<PendingPlanner>();Navigator n(space,planner,std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
    CHECK(n.attach(1)==Error::None);CHECK(n.begin(1,1,{1,{2,0,0}})==Error::None);
    CHECK(n.update(1,1).status==TravelStatus::Waiting);CHECK(n.cancel(1,1));
    CHECK(n.begin(1,2,{1,{3,0,0}})==Error::None);planner->stale=true;
    CHECK(n.update(1,2).status==TravelStatus::Waiting);CHECK(n.state(1)->phase==Phase::Planning);
    CHECK(!n.cancel(1,1));CHECK(n.state(1)->task==2);
    CHECK(navtest::finish(n,1,2).status==TravelStatus::Succeeded);CHECK(planner->cancelled>=2);
}
void changed_space_replans_from_actual_interior_position() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    auto d=navtest::graph();auto space=GraphSpace::compile(d).value;
    Navigator n(space,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
    CHECK(n.attach(1,navtest::profile())==Error::None);CHECK(n.begin(1,1,d.nodes.back().location)==Error::None);
    for(int i=0;i<10;++i) CHECK(n.update(1,1).status!=TravelStatus::Failed);
    const auto old=n.state(1)->request;CHECK(motor.bodies[1].location.position.x>0);
    d.stamp.version=2;CHECK(n.publish(GraphSpace::compile(d).value)==Error::None);
    CHECK(navtest::finish(n).status==TravelStatus::Succeeded);CHECK(n.state(1)->request>old);CHECK(n.state(1)->replans==1);
    CHECK(n.publish(space)==Error::Stale);
}
void arc_traversal_locks_interruptions_and_finishes_frozen_revision() {
    navtest::Motor motor;navtest::Obstacles obstacles;auto d=navtest::graph();d.links[0].traversal=1;
    motor.bodies[1]={{1,{0,0,0}},{}};auto space=GraphSpace::compile(d).value;
    Navigator n(space,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
    CHECK(n.registerTraversal(1,std::make_shared<TimedArcTraversal>(5,2))==Error::None);
    CHECK(n.attach(1,navtest::profile())==Error::None);CHECK(n.begin(1,1,d.nodes.back().location)==Error::None);
    CHECK(n.update(1,1).status==TravelStatus::Waiting);CHECK(!n.update(1,1).interruptible);
    CHECK(motor.bodies[1].location.position.y>0);CHECK(!n.canInterrupt(1,1));
    CHECK(n.begin(1,2,d.nodes.back().location)==Error::Busy);
    d.stamp.version=2;d.links[0].enabled=false;CHECK(n.publish(GraphSpace::compile(d).value)==Error::None);
    CHECK(navtest::finish(n).status==TravelStatus::Succeeded);CHECK(n.state(1)->replans==1);
}
void link_capacity_waits_and_releases_on_cancel() {
    navtest::Motor motor;navtest::Obstacles obstacles;auto d=navtest::graph();d.links[0].traversal=1;d.links[0].resource=99;d.links[0].capacity=1;
    motor.bodies[1]={{1,{0,0,0}},{}};motor.bodies[2]=motor.bodies[1];
    Navigator n(GraphSpace::compile(d).value,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles);
    CHECK(n.registerTraversal(1,std::make_shared<TimedArcTraversal>(5,2))==Error::None);
    CHECK(n.attach(1)==Error::None && n.attach(2)==Error::None);
    CHECK(n.begin(1,1,d.nodes[1].location)==Error::None && n.begin(2,2,d.nodes[1].location)==Error::None);
    n.update(1,1);n.update(2,2);n.update(1,1);
    CHECK(n.update(2,2).status==TravelStatus::Waiting);CHECK(n.state(2)->phase==Phase::WaitingForLink);
    CHECK(n.cancel(1,1));CHECK(!n.update(2,2).interruptible);
}
void planar_and_free_volume_movement_use_different_constraints() {
    for(const auto steering:{Steering::Planar,Steering::Spatial}) {
        navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
        auto p=navtest::profile();p.steering=steering;p.up={0,0,1};
        const auto space=VolumeSpace::compile({2,1},{{1,{-20,-20,-20},{20,20,20}}}).value;
        Navigator n(space,std::make_shared<DirectPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
        CHECK(n.attach(1,p)==Error::None);const Location goal{1,{3,2,steering==Steering::Planar?0.0:4.0}};
        CHECK(n.begin(1,1,goal)==Error::None);CHECK(navtest::finish(n).status==TravelStatus::Succeeded);
        for(const auto velocity:motor.commands) if(steering==Steering::Planar) CHECK(std::abs(velocity.z)<1e-7);
    }
}
void arrival_requires_braking_and_obstacle_queries_are_not_assumed_empty() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{2,0,0}};
    auto space=VolumeSpace::compile({1,1},{{1,{-20,-20,-20},{20,20,20}}}).value;
    Navigator n(space,std::make_shared<DirectPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
    CHECK(n.attach(1,navtest::profile())==Error::None);CHECK(n.begin(1,1,{1,{0,0,0}})==Error::None);
    CHECK(n.update(1,1).status==TravelStatus::Waiting);CHECK(n.update(1,1).status!=TravelStatus::Succeeded);
    CHECK(navtest::finish(n).status==TravelStatus::Succeeded);
    CHECK(n.begin(1,2,{1,{2,0,0}})==Error::None);n.update(1,2);obstacles.snapshot.valid=false;
    CHECK(n.update(1,2).error==Error::QueryFailed);
}
struct UnsafeAvoidance final:Avoidance {
    AvoidanceResult choose(const AvoidanceQuery&) const override {return {AvoidanceStatus::Clear,{0,0,999}};}
};
void custom_avoidance_cannot_leave_corridor_or_violate_acceleration() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    auto c=navtest::config();c.maxBlockedSteps=2;c.maxReplans=1;
    Navigator n(GraphSpace::compile(navtest::graph()).value,std::make_shared<GraphPlanner>(),std::make_shared<UnsafeAvoidance>(),motor,obstacles,c);
    CHECK(n.attach(1)==Error::None);CHECK(n.begin(1,1,{1,{3,0,0}})==Error::None);
    CHECK(navtest::finish(n).status==TravelStatus::Failed);CHECK(motor.commands.empty());
}
void moving_obstacle_sweep_detects_crossing_and_planar_avoidance_stays_in_plane() {
    const auto space=VolumeSpace::compile({1,1},{{1,{-20,-20,-20},{20,20,20}}}).value;
    auto p=navtest::profile();p.steering=Steering::Planar;p.up={0,0,1};p.maxAcceleration=100;
    MotionState motion{{1,{0,0,0}},{}};Step step;step.from=motion.location;step.to={1,{10,0,0}};
    const std::array<Obstacle,1> obstacles{{{5,{1,{.5,1,0}},{0,-4,0},.25}}};
    AvoidanceQuery q{1,motion,p,step,*space,{2,0,0},.5,obstacles};
    CHECK(!feasible(q,{2,0,0}));const auto result=ConservativeAvoidance{}.choose(q);
    if(result.status==AvoidanceStatus::Clear) {CHECK(feasible(q,result.velocity));CHECK(result.velocity.z==0);}
    else CHECK(result.status==AvoidanceStatus::Wait);
}
void native_callbacks_cannot_reenter_navigation_mutations() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    auto space=VolumeSpace::compile({1,1},{{1,{-20,-20,-20},{20,20,20}}}).value;
    Navigator n(space,std::make_shared<DirectPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
    CHECK(n.attach(1)==Error::None);CHECK(n.begin(1,1,{1,{2,0,0}})==Error::None);
    motor.callback=[&]{CHECK(n.publish(space)==Error::Busy);CHECK(n.detach(1)==Error::Busy);CHECK(!n.cancel(1,1));};
    CHECK(navtest::finish(n).status==TravelStatus::Succeeded);motor.callback={};
}
struct CustomSphere final:Space {
    Stamp stamp() const noexcept override {return {42,1};}
    bool contains(Location p,const Profile& a) const override {return p.region==9 && finite(p.position) && length(p.position)+a.radius<20;}
    bool accepts(const Step& s,const Profile& p) const override {return !s.link && !s.traversal && !s.resource && !s.capacity && contains(s.from,p) && contains(s.to,p);}
    bool permits(const Step& s,Location from,Location to,const Profile& p) const override {return accepts(s,p) && contains(from,p) && contains(to,p);}
};
struct CustomDogleg final:Planner {
    unsigned calls=0;
    PlanResult poll(const PlanRequest& q) override {
        ++calls;Step a,b;a.from=q.from;a.to={9,{0,2,3}};b.from=a.to;b.to=q.goal;
        return {q.id,PlanStatus::Found,{q.space->stamp(),q.goal,{a,b}}};
    }
};
struct CustomAvoidance final:Avoidance {
    mutable unsigned calls=0;
    AvoidanceResult choose(const AvoidanceQuery& q) const override {
        ++calls;return feasible(q,q.desiredVelocity)?AvoidanceResult{AvoidanceStatus::Clear,q.desiredVelocity}:ConservativeAvoidance{}.choose(q);
    }
};
void custom_space_planner_and_avoidance_execute_without_graph_assumptions() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{9,{0,0,0}},{}};
    auto planner=std::make_shared<CustomDogleg>();auto avoidance=std::make_shared<CustomAvoidance>();
    Navigator n(std::make_shared<CustomSphere>(),planner,avoidance,motor,obstacles,navtest::config());
    CHECK(n.attach(1,navtest::profile())==Error::None);CHECK(n.begin(1,1,{9,{5,3,-1}})==Error::None);
    CHECK(navtest::finish(n).status==TravelStatus::Succeeded);CHECK(planner->calls==1 && avoidance->calls>20);
    CHECK(n.state(1)->route.steps.size()==2);CHECK(length(motor.bodies[1].location.position-Vec3{5,3,-1})<=.03);
}
void narrow_corridor_waits_instead_of_leaving_its_authored_boundary() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    auto d=navtest::graph();d.links[0].width=.6;
    obstacles.snapshot.obstacles={{100,{1,{1.5,0,0}},{},.4}};
    Navigator n(GraphSpace::compile(d).value,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
    CHECK(n.attach(1,navtest::profile())==Error::None);CHECK(n.begin(1,1,{1,{3,0,0}})==Error::None);
    const auto result=navtest::finish(n);CHECK(result.status==TravelStatus::Failed);
    CHECK(motor.bodies[1].location.position.x<1.5);CHECK(std::abs(motor.bodies[1].location.position.z)<=.050001);
    CHECK(motor.stopped>0);
}
void motor_defined_final_arrival_condition_can_wait_for_heading_or_docking() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};motor.arrivalDelay=4;
    Navigator n(VolumeSpace::compile({1,1},{{1,{-10,-10,-10},{10,10,10}}}).value,std::make_shared<DirectPlanner>(),
        std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
    CHECK(n.attach(1)==Error::None);CHECK(n.begin(1,1,{1,{0,0,0}})==Error::None);
    CHECK(n.update(1,1).status==TravelStatus::Waiting);
    for(int i=0;i<4;++i)CHECK(n.update(1,1).status!=TravelStatus::Succeeded);
    CHECK(n.update(1,1).status==TravelStatus::Succeeded);CHECK(motor.commands.size()==4);
}
void open_volume_avoidance_routes_around_a_sphere_without_intersection() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{-3,0,0}},{}};
    obstacles.snapshot.obstacles={{100,{1,{0,0,0}},{},.5}};
    auto p=navtest::profile();p.maxAcceleration=100;auto c=navtest::config();c.maxBlockedSteps=120;
    Navigator n(VolumeSpace::compile({1,1},{{1,{-10,-10,-10},{10,10,10}}}).value,std::make_shared<DirectPlanner>(),
        std::make_shared<ConservativeAvoidance>(),motor,obstacles,c);
    CHECK(n.attach(1,p)==Error::None);CHECK(n.begin(1,1,{1,{3,0,0}})==Error::None);
    bool arrived=false;double detour=0;
    for(int i=0;i<2000;++i) {
        const auto before=motor.bodies[1].location.position;const auto result=n.update(1,1);const auto after=motor.bodies[1].location.position;
        const auto delta=after-before;const double squared=dot(delta,delta);
        const double t=squared>0?std::clamp(-dot(before,delta)/squared,0.0,1.0):0;
        CHECK(length(before+delta*t)>=.75-1e-7);detour=std::max(detour,std::hypot(after.y,after.z));
        CHECK(result.status!=TravelStatus::Failed);if(result.status==TravelStatus::Succeeded){arrived=true;break;}
    }
    CHECK(arrived && detour>.3);
}
struct StalledPlanner final:Planner {
    unsigned cancelled=0;
    PlanResult poll(const PlanRequest& q) override {return {q.id};}
    void cancel(RequestId) noexcept override {++cancelled;}
};
void pending_planning_timeout_cancels_request_without_publishing_motion() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    auto planner=std::make_shared<StalledPlanner>();auto c=navtest::config();c.maxPlanningSteps=2;
    Navigator n(VolumeSpace::compile({1,1},{{1,{-10,-10,-10},{10,10,10}}}).value,planner,
        std::make_shared<ConservativeAvoidance>(),motor,obstacles,c);
    CHECK(n.attach(1)==Error::None);CHECK(n.begin(1,1,{1,{3,0,0}})==Error::None);
    CHECK(n.update(1,1).status==TravelStatus::Waiting);CHECK(n.update(1,1).status==TravelStatus::Waiting);
    CHECK(n.update(1,1).error==Error::Budget);CHECK(planner->cancelled==1 && motor.commands.empty());
}
void failed_traversal_releases_capacity_for_another_agent() {
    navtest::Motor motor;navtest::Obstacles obstacles;auto d=navtest::graph();d.links[0].traversal=1;d.links[0].resource=99;d.links[0].capacity=1;
    motor.bodies[1]={{1,{0,0,0}},{}};motor.bodies[2]=motor.bodies[1];
    Navigator n(GraphSpace::compile(d).value,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles);
    CHECK(n.registerTraversal(1,std::make_shared<TimedArcTraversal>(5,2))==Error::None);
    CHECK(n.attach(1)==Error::None && n.attach(2)==Error::None);
    CHECK(n.begin(1,1,d.nodes[1].location)==Error::None && n.begin(2,2,d.nodes[1].location)==Error::None);
    n.update(1,1);n.update(2,2);motor.rejectPose=true;CHECK(n.update(1,1).error==Error::TraversalFailed);
    motor.rejectPose=false;CHECK(n.update(2,2).status==TravelStatus::Running);CHECK(n.state(2)->phase==Phase::Traversing);
}
struct CustomRegionTransfer final:Traversal {
    bool accepts(const Step& s,const Profile&) const override {return s.from.region!=s.to.region;}
    TravelResult update(TraversalContext& c,bool) override {
        return {c.motor.traversalPose(c.agent,c.task,c.step.to)?TravelStatus::Succeeded:TravelStatus::Failed,true};
    }
};
void region_aware_custom_traversal_uses_backend_confirmed_destination() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    GraphDefinition d;d.stamp={1,1};d.nodes={{1,{1,{0,0,0}}},{2,{2,{100,5,9}}}};
    Link link;link.id=1;link.from=1;link.to=2;link.traversal=2;d.links={link};
    Navigator n(GraphSpace::compile(d).value,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles);
    CHECK(n.registerTraversal(2,std::make_shared<CustomRegionTransfer>())==Error::None);CHECK(n.attach(1)==Error::None);
    CHECK(n.begin(1,1,d.nodes.back().location)==Error::None);CHECK(navtest::finish(n).status==TravelStatus::Succeeded);
    CHECK(motor.bodies[1].location==d.nodes.back().location);CHECK(motor.commands.empty());
}
struct CruiseFollower final:Follower {
    mutable unsigned calls=0;
    FollowResult follow(const FollowQuery& q) const override {
        ++calls;if(q.final)return BrakingFollower{}.follow(q);
        const auto delta=q.step.to.position-q.motion.location.position;
        return {normalized(delta)*q.profile.maxSpeed,length(delta)<=q.profile.arrivalDistance,false};
    }
};
void custom_follower_can_pass_intermediate_nodes_without_forcing_a_stop() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    auto follower=std::make_shared<CruiseFollower>();auto p=navtest::profile();p.arrivalDistance=.15;
    Navigator n(GraphSpace::compile(navtest::graph()).value,std::make_shared<GraphPlanner>(),
        std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config(),follower);
    CHECK(n.attach(1,p)==Error::None);CHECK(n.begin(1,1,{1,{6,1,0}})==Error::None);
    bool continuous=false,arrived=false;
    for(int i=0;i<2000;++i){const auto old=n.state(1)->step;const auto result=n.update(1,1);
        if(old==0 && n.state(1)->step==1)continuous=length(motor.bodies[1].velocity)>p.arrivalSpeed;
        CHECK(result.status!=TravelStatus::Failed);if(result.status==TravelStatus::Succeeded){arrived=true;break;}}
    CHECK(continuous && arrived && follower->calls>20);
}
}
void explicit_resume_restores_older_owner_task_without_weakening_begin_staleness() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    auto space=VolumeSpace::compile({1,1},{{1,{-20,-20,-20},{20,20,20}}}).value;
    Navigator n(space,std::make_shared<DirectPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());CHECK(n.attach(1)==Error::None);
    CHECK(n.begin(1,1,{1,{8,0,0}})==Error::None);const auto original=n.state(1)->request;CHECK(n.cancel(1,1));
    CHECK(n.begin(1,3,{1,{1,0,0}})==Error::None);CHECK(n.resume(1,1,{1,{8,0,0}})==Error::Busy);
    CHECK(navtest::finish(n,1,3).status==TravelStatus::Succeeded);
    CHECK(n.begin(1,1,{1,{8,0,0}})==Error::Stale);CHECK(n.resume(1,4,{1,{8,0,0}})==Error::Stale);
    CHECK(n.resume(1,1,{1,{8,0,0}})==Error::None);CHECK(n.state(1)->request>original);
    CHECK(n.begin(1,2,{1,{2,0,0}})==Error::Stale);CHECK(n.begin(1,3,{1,{1,0,0}})==Error::Stale);CHECK(!n.cancel(1,3));
    CHECK(navtest::finish(n,1,1).status==TravelStatus::Succeeded);CHECK(n.begin(1,4,{1,{4,0,0}})==Error::None);
}
TEST_MAIN(
    RUN(graph_validation_and_directed_interior_routes);
    RUN(graph_clearance_capabilities_special_links_and_budget);
    RUN(graph_rejects_forged_routes_and_constrains_whole_corridor);
    RUN(navigator_follows_authored_three_dimensional_network);
    RUN(custom_planner_results_are_independently_validated);
    RUN(pending_plans_stale_responses_and_old_cancellation_are_safe);
    RUN(changed_space_replans_from_actual_interior_position);
    RUN(arc_traversal_locks_interruptions_and_finishes_frozen_revision);
    RUN(link_capacity_waits_and_releases_on_cancel);
    RUN(planar_and_free_volume_movement_use_different_constraints);
    RUN(arrival_requires_braking_and_obstacle_queries_are_not_assumed_empty);
    RUN(custom_avoidance_cannot_leave_corridor_or_violate_acceleration);
    RUN(moving_obstacle_sweep_detects_crossing_and_planar_avoidance_stays_in_plane);
    RUN(native_callbacks_cannot_reenter_navigation_mutations);
    RUN(custom_space_planner_and_avoidance_execute_without_graph_assumptions);
    RUN(narrow_corridor_waits_instead_of_leaving_its_authored_boundary);
    RUN(motor_defined_final_arrival_condition_can_wait_for_heading_or_docking);
    RUN(open_volume_avoidance_routes_around_a_sphere_without_intersection);
    RUN(pending_planning_timeout_cancels_request_without_publishing_motion);
    RUN(failed_traversal_releases_capacity_for_another_agent);
    RUN(region_aware_custom_traversal_uses_backend_confirmed_destination);
    RUN(custom_follower_can_pass_intermediate_nodes_without_forcing_a_stop);
    RUN(explicit_resume_restores_older_owner_task_without_weakening_begin_staleness);
)
