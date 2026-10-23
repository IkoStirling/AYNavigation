// design reference: ../design.md; actual AYBehavior scheduling and navigation plugin lifecycle.
#include "Test.h"
#include "Support.h"
#include <AYNavigation/BehaviorIntegration.h>
using namespace ayt::navigation;
namespace {
void jump_stage_blocks_preemption_then_allows_it_after_landing() {
    navtest::Motor motor;navtest::Obstacles obstacles;auto d=navtest::graph();d.links[0].traversal=1;
    motor.bodies[1]={{1,{0,0,0}},{}};
    Navigator n(GraphSpace::compile(d).value,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
    CHECK(n.registerTraversal(1,std::make_shared<TimedArcTraversal>(4,2))==Error::None);CHECK(n.attach(1)==Error::None);
    ayt::behavior::Runtime runtime;
    CHECK(runtime.registerExecutor(kNavigate,std::make_shared<NavigationExecutor>(n))==ayt::behavior::Error::None);
    CHECK(runtime.registerExecutor(ayt::behavior::kWait,std::make_shared<ayt::behavior::WaitExecutor>())==ayt::behavior::Error::None);
    CHECK(runtime.addUnit(1)==ayt::behavior::Error::None);CHECK(runtime.submit(1,navigationOrder(d.nodes.back().location,1)));
    auto tick=[&]{CHECK(runtime.advance(runtime.nextTick())==ayt::behavior::Error::None);};
    tick();tick();CHECK(!runtime.state(1)->active->interruptibleNow);
    CHECK(runtime.submit(1,ayt::behavior::waitOrder(0,1,10),ayt::behavior::SubmitMode::Interrupt).error==ayt::behavior::Error::NotInterruptible);
    CHECK(runtime.stop(1)==ayt::behavior::Error::NotInterruptible);
    tick();tick();tick();CHECK(runtime.state(1)->active->interruptibleNow);
    CHECK(runtime.submit(1,ayt::behavior::waitOrder(0,1,10),ayt::behavior::SubmitMode::Interrupt));tick();
    for(int i=0;i<1000 && runtime.state(1)->active;++i) tick();
    CHECK(!runtime.state(1)->active);CHECK(length(motor.bodies[1].location.position-d.nodes.back().location.position)<=.025);
}
void old_motion_port_contract_preserves_tokens_and_dynamic_windows() {
    navtest::Motor motor;navtest::Obstacles obstacles;auto d=navtest::graph();d.links[0].traversal=1;motor.bodies[1]={{1,{0,0,0}},{}};
    Navigator n(GraphSpace::compile(d).value,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
    CHECK(n.registerTraversal(1,std::make_shared<TimedArcTraversal>(3,1))==Error::None);CHECK(n.attach(1)==Error::None);
    NavigationMotionPort port(n,1);ayt::behavior::Runtime runtime;
    CHECK(runtime.registerExecutor(ayt::behavior::kMove,std::make_shared<ayt::behavior::MoveExecutor>(port))==ayt::behavior::Error::None);
    CHECK(runtime.addUnit(1)==ayt::behavior::Error::None);
    ayt::behavior::Order order;order.kind=ayt::behavior::kMove;order.source=1;order.target=ayt::behavior::Point3{3,0,0};CHECK(runtime.submit(1,order));
    runtime.advance(0);runtime.advance(1);CHECK(!runtime.state(1)->active->interruptibleNow);
    CHECK(runtime.removeUnit(1)==ayt::behavior::Error::None);CHECK(n.state(1)->phase==Phase::Idle);
}
void region_goals_are_owned_and_invalid_order_rejection_preserves_current_task() {
    navtest::Motor motor;navtest::Obstacles obstacles;motor.bodies[1]={{1,{0,0,0}},{}};
    Navigator n(VolumeSpace::compile({1,1},{{1,{-10,-10,-10},{10,10,10}}}).value,
        std::make_shared<DirectPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());
    CHECK(n.attach(1)==Error::None);ayt::behavior::Runtime runtime;
    runtime.registerExecutor(kNavigate,std::make_shared<NavigationExecutor>(n));runtime.addUnit(1);
    auto order=navigationOrder({1,{3,0,0}},1);CHECK(runtime.submit(1,order));runtime.advance(0);
    order.payload.clear();CHECK(runtime.submit(1,order).error==ayt::behavior::Error::Unsupported);
    CHECK(runtime.state(1)->active->id==1);CHECK(n.state(1)->task==1);
}
}
TEST_MAIN(
    RUN(jump_stage_blocks_preemption_then_allows_it_after_landing);
    RUN(old_motion_port_contract_preserves_tokens_and_dynamic_windows);
    RUN(region_goals_are_owned_and_invalid_order_rejection_preserves_current_task);
)
