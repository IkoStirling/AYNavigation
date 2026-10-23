// design reference: ../design.md; actual World/Behavior owner and root motor contract.
#include "Test.h"
#include "Support.h"
#include <AYNavigation/EntityMotor.h>
#include <AYEntity/World.h>
#include <AYEntity/EntityModule.h>
#include <AYEntity/EntityImpl.h>
using namespace ayt::navigation;
namespace {
struct WorldFixture {
    ayt::entity::World& world=ayt::entity::World::processWorld();
    WorldFixture(){CHECK(ayt::entity::registerEntityCoreComponents(ayt::entity::ComponentRegistry::instance()));CHECK(world.initialize());}
    ~WorldFixture(){world.shutdown();}
};
struct Shutdown {ayt::entity::World& world;~Shutdown(){world.shutdown();}};
void real_world_navigation_uses_fixed_ticks_and_honors_jump_window() {
    WorldFixture f;RootKinematicMotor motor(f.world);navtest::Obstacles obstacles;
    auto definition=navtest::graph();definition.links[0].traversal=1;
    Navigator navigator(GraphSpace::compile(definition).value,std::make_shared<GraphPlanner>(),
        std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());Shutdown shutdown{f.world};
    auto* entity=f.world.createSpatialEntity();const auto handle=f.world.getEntityHandle(entity->getId());const auto id=ayt::behavior::unitId(handle);
    CHECK(navigator.attach(id,navtest::profile())==Error::None);
    CHECK(navigator.registerTraversal(1,std::make_shared<TimedArcTraversal>(4,2))==Error::None);
    auto installed=ayt::behavior::installBehavior(f.world);CHECK(installed);auto& behavior=*installed.value;
    CHECK(behavior.registerExecutor(kNavigate,std::make_shared<NavigationExecutor>(navigator))==ayt::behavior::Error::None);
    CHECK(behavior.track(handle)==ayt::behavior::Error::None);
    CHECK(behavior.submit(std::array<ayt::entity::EntityHandle,1>{handle},navigationOrder(definition.nodes.back().location,1))[0].error==ayt::behavior::Error::None);
    f.world.updatePresentation(.02f,.5f);CHECK(navigator.state(id)->phase==Phase::Idle);
    f.world.fixedUpdate(.05f);f.world.fixedUpdate(.05f);
    CHECK(!behavior.runtime().state(id)->active->interruptibleNow);CHECK(behavior.stop(handle)==ayt::behavior::Error::NotInterruptible);
    for(int i=0;i<2000 && behavior.runtime().state(id)->active;++i) f.world.fixedUpdate(.05f);
    CHECK(!behavior.runtime().state(id)->active);CHECK(navigator.state(id)->phase==Phase::Succeeded);
    CHECK(length(motor.sample(id)->location.position-definition.nodes.back().location.position)<=.03);
}
void destroyed_entity_and_world_shutdown_release_navigation_operations() {
    WorldFixture f;RootKinematicMotor motor(f.world);navtest::Obstacles obstacles;
    Navigator navigator(GraphSpace::compile(navtest::graph()).value,std::make_shared<GraphPlanner>(),
        std::make_shared<ConservativeAvoidance>(),motor,obstacles,navtest::config());Shutdown shutdown{f.world};
    auto& behavior=*ayt::behavior::installBehavior(f.world).value;
    behavior.registerExecutor(kNavigate,std::make_shared<NavigationExecutor>(navigator));
    auto* entity=f.world.createSpatialEntity();const auto handle=f.world.getEntityHandle(entity->getId());const auto id=ayt::behavior::unitId(handle);
    CHECK(navigator.attach(id)==Error::None);CHECK(behavior.track(handle)==ayt::behavior::Error::None);
    CHECK(behavior.submit(std::array<ayt::entity::EntityHandle,1>{handle},navigationOrder({1,{9,1,0}},1))[0].error==ayt::behavior::Error::None);
    f.world.fixedUpdate(.05f);f.world.fixedUpdate(.05f);f.world.destroyEntity(entity);f.world.fixedUpdate(.05f);
    CHECK(navigator.state(id)->phase==Phase::Idle);CHECK(!motor.sample(id));
    auto* second=f.world.createSpatialEntity();const auto next=f.world.getEntityHandle(second->getId());const auto nextId=ayt::behavior::unitId(next);
    CHECK(nextId!=id);CHECK(navigator.attach(nextId)==Error::None);CHECK(behavior.track(next)==ayt::behavior::Error::None);
    CHECK(behavior.submit(std::array<ayt::entity::EntityHandle,1>{next},navigationOrder({1,{9,1,0}},1))[0].error==ayt::behavior::Error::None);
    f.world.fixedUpdate(.05f);f.world.shutdown();CHECK(navigator.state(nextId)->phase==Phase::Idle);
}
void root_motor_keeps_authoritative_regions_and_ignores_stale_stop() {
    WorldFixture f;RootKinematicMotor motor(f.world);
    auto* entity=f.world.createSpatialEntity();const auto handle=f.world.getEntityHandle(entity->getId());const auto id=ayt::behavior::unitId(handle);
    CHECK(f.world.connectedSpaces().addRegion(7));CHECK(entity->setSpaceRegion(7));CHECK(motor.sample(id)->location.region==7);
    CHECK(motor.drive(id,2,{1,0,0},{7,{3,0,0}},.1));motor.stop(id,1);CHECK(motor.sample(id)->velocity.x==1);
    CHECK(!motor.traversalPose(id,2,{8,{100,0,0}}));CHECK(entity->effectiveSpaceRegion()==7);
    auto* parent=f.world.createSpatialEntity();CHECK(entity->setSpatialParent(parent,ayt::entity::ReparentMode::KeepWorld));CHECK(!motor.sample(id));
}
}
TEST_MAIN(
    RUN(real_world_navigation_uses_fixed_ticks_and_honors_jump_window);
    RUN(destroyed_entity_and_world_shutdown_release_navigation_operations);
    RUN(root_motor_keeps_authoritative_regions_and_ignores_stale_stop);
)
