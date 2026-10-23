#include "Test.h"
#include <AYNavigation/CharacterMotor.h>
#include <AYNavigation/Graph.h>
#include <AYPhysics/PhysicsManager.h>
#include <array>
using namespace ayt::navigation;
namespace {
using ayt::physics::PhysResult;
struct Scene {
    std::unique_ptr<ayt::physics::PhysicsManager> manager;
    ayt::physics::PhysicsWorld3D* physics;
    ayt::physics::CharacterHandle handle=0;
    Scene() {
        ayt::physics::PhysicsBackendDescriptor config;config.kind3D=ayt::physics::BackendKind::DefaultJolt;config.kind2D=ayt::physics::BackendKind::Null;
        manager=ayt::physics::PhysicsManager::create(config);CHECK(manager&&manager->isRunning());physics=manager->world3D();
        box({-5.5,-.25,0},{4.5,.25,4});box({6,.75,0},{4,.25,4});
        ayt::physics::CharacterController3DDesc desc;desc.position={-6,.05f,0};desc.radius=.2f;desc.height=1.4f;desc.groundSnapDistance=.1f;desc.maxStepHeight=.2f;
        CHECK(physics->createCharacter(desc,handle)==PhysResult::Ok);for(int i=0;i<30;++i)tick();
    }
    void box(Vec3 center,Vec3 half) {
        ayt::physics::RigidbodyDesc body;body.type=ayt::physics::BodyType::Static;body.position={float(center.x),float(center.y),float(center.z)};
        ayt::physics::BodyHandle handle;CHECK(physics->createRigidbody(body,handle)==PhysResult::Ok);
        ayt::physics::ColliderDesc shape;shape.body=handle;shape.shape=ayt::physics::ColliderShape::Box;shape.halfExtents={float(half.x),float(half.y),float(half.z)};
        ayt::physics::ColliderHandle collider;CHECK(physics->createCollider(shape,collider)==PhysResult::Ok);
    }
    void tick(){CHECK(manager->stepAndWait(1.f/60.f)==PhysResult::Ok);}
};
struct Empty final:ObstacleSource {ObstacleSnapshot query(AgentId,const MotionState&,double) const override {return {};}};
GraphDefinition graph() {
    GraphDefinition result;result.stamp={1,1};result.nodes={{1,{1,{-6,0,0}}},{2,{1,{-1.5,0,0}}},{3,{1,{2.6,1,0}}},{4,{1,{7,1,0}}}};
    result.links={{1,1,2,1.4,3,4},{2,2,3,1.4,3,5,0,9,1,1},{3,3,4,1.4,3,4}};return result;
}
Profile profile(){Profile p;p.radius=.2;p.height=1.4;p.maxSpeed=2.5;p.maxAcceleration=10;p.arrivalDistance=.06;p.arrivalSpeed=.1;return p;}
Config config(){Config c;c.secondsPerStep=1.0/60;c.maxBlockedSteps=120;c.maxReplans=2;return c;}
void actual_jolt_jump_crosses_a_gap_and_waits_for_grounded_arrival() {
    Scene scene;CharacterMotor motor(*scene.physics);CHECK(motor.attach(1,scene.handle,1)==Error::None);Empty obstacles;
    Navigator navigator(GraphSpace::compile(graph()).value,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,config());
    CHECK(navigator.attach(1,profile())==Error::None);CHECK(navigator.registerTraversal(1,std::make_shared<CharacterJumpTraversal>(motor))==Error::None);
    CHECK(navigator.begin(1,1,{1,{7,1,0}})==Error::None);bool airborne=false,locked=false;double highest=0;Update result;
    for(int i=0;i<2000;++i) {
        result=navigator.update(1,1);if(!result.interruptible)locked=true;scene.tick();const auto state=motor.character(1);CHECK(state);
        highest=std::max(highest,double(state->position.y));airborne|=!state->isGrounded();
        if(result.status!=TravelStatus::Running&&result.status!=TravelStatus::Waiting)break;
    }
    if(result.status!=TravelStatus::Succeeded){const auto s=motor.sample(1);std::fprintf(stderr,"navigation error %d phase %d feet %g %g velocity %g %g %g grounded %d\n",int(result.error),int(navigator.state(1)->phase),s->location.position.x,s->location.position.y,s->velocity.x,s->velocity.y,s->velocity.z,int(motor.character(1)->isGrounded()));}
    CHECK(result.status==TravelStatus::Succeeded);CHECK(airborne&&locked&&highest>1.5);CHECK(motor.character(1)->isGrounded());
    CHECK(length(motor.sample(1)->location.position-Vec3{7,1,0})<=.06);
}
void real_wall_collision_and_stale_stop_do_not_fake_motion() {
    Scene scene;scene.box({-4,1,0},{.2,1,3});scene.tick();CharacterMotor motor(*scene.physics);CHECK(motor.attach(1,scene.handle,1)==Error::None);
    CHECK(motor.drive(1,1,{3,0,0},{1,{0,0,0}},1.0/60));for(int i=0;i<120;++i)scene.tick();
    CHECK(motor.sample(1)->location.position.x<-4.3);motor.stop(1,1);
    CHECK(motor.drive(1,2,{-1,0,0},{1,{-8,0,0}},1.0/60));motor.stop(1,1);const auto before=motor.sample(1)->location.position.x;
    for(int i=0;i<10;++i)scene.tick();CHECK(motor.sample(1)->location.position.x<before-.1);
    CHECK(!motor.traversalPose(1,2,{1,{100,0,0}}));CHECK(!motor.drive(1,2,{1,0,0},{2,{0,0,0}},1.0/60));
    CHECK(motor.detach(1)==Error::None);CHECK(!motor.sample(1));
}
void obstructed_physical_jump_fails_without_writing_a_landing_pose() {
    Scene scene;scene.box({-1.5,1.9,0},{1.2,.1,2});scene.tick();CharacterMotor motor(*scene.physics);CHECK(motor.attach(1,scene.handle,1)==Error::None);Empty obstacles;
    Navigator navigator(GraphSpace::compile(graph()).value,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,config());
    CHECK(navigator.attach(1,profile())==Error::None);JumpSettings jump;jump.maxSeconds=2;
    CHECK(navigator.registerTraversal(1,std::make_shared<CharacterJumpTraversal>(motor,jump))==Error::None);
    CHECK(navigator.begin(1,1,{1,{7,1,0}})==Error::None);Update result;
    for(int i=0;i<1500;++i){result=navigator.update(1,1);scene.tick();if(result.status==TravelStatus::Failed||result.status==TravelStatus::Succeeded)break;}
    CHECK(result.status==TravelStatus::Failed);CHECK(result.error==Error::TraversalFailed);
    CHECK(length(motor.sample(1)->location.position-Vec3{7,1,0})>1);CHECK(!motor.traversalPose(1,1,{1,{7,1,0}}));
}
void forced_cancellation_stops_input_without_freezing_airborne_physics() {
    Scene scene;CharacterMotor motor(*scene.physics);CHECK(motor.attach(1,scene.handle,1)==Error::None);Empty obstacles;
    Navigator navigator(GraphSpace::compile(graph()).value,std::make_shared<GraphPlanner>(),std::make_shared<ConservativeAvoidance>(),motor,obstacles,config());
    CHECK(navigator.attach(1,profile())==Error::None);navigator.registerTraversal(1,std::make_shared<CharacterJumpTraversal>(motor));
    CHECK(navigator.begin(1,1,{1,{7,1,0}})==Error::None);bool airborne=false;
    for(int i=0;i<1000;++i){const auto update=navigator.update(1,1);CHECK(update.status!=TravelStatus::Failed);scene.tick();
        if(!motor.character(1)->isGrounded()&&navigator.state(1)->phase==Phase::Traversing){airborne=true;break;}}
    CHECK(airborne&&!navigator.canInterrupt(1,1));const auto before=motor.sample(1)->location.position;
    CHECK(navigator.cancel(1,1));for(int i=0;i<6;++i)scene.tick();const auto after=motor.sample(1)->location.position;
    CHECK(std::abs(after.y-before.y)>.01);CHECK(std::abs(after.x-before.x)<.03);
    CHECK(scene.physics->destroyCharacter(scene.handle)==PhysResult::Ok);CHECK(!motor.sample(1));CHECK(motor.detach(1)==Error::MotionFailed);
}
}
TEST_MAIN(
    RUN(actual_jolt_jump_crosses_a_gap_and_waits_for_grounded_arrival);
    RUN(real_wall_collision_and_stale_stop_do_not_fake_motion);
    RUN(obstructed_physical_jump_fails_without_writing_a_landing_pose);
    RUN(forced_cancellation_stops_input_without_freezing_airborne_physics);
)
