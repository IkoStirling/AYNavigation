#include "Test.h"

#include <AYNavigation/PhysicsVisibility.h>
#include <AYNavigation/WorldBake.h>
#include <AYEntity.h>
#include <AYEntity/World.h>
#include <AYEntity/EntityModule.h>
#include <AYEntity/ComponentRegistration.h>
#include <AYEntity/components/ColliderComponent.h>
#include <AYEntity/components/RigidBodyComponent.h>
#include <AYEntity/components/TransformComponent.h>
#include <array>
#include <cmath>

using namespace ayt::navigation;

namespace {
struct Fixture {
    ayt::entity::World& world = ayt::entity::World::processWorld();
    Fixture()
    {
        CHECK(ayt::entity::registerEntityCoreComponents(
            ayt::entity::ComponentRegistry::instance()));
        CHECK(ayt::entity::registerComponent<ayt::entity::RigidBodyComponent>(
            ayt::entity::ComponentRegistry::instance(), "RigidBodyComponent"));
        CHECK(ayt::entity::registerComponent<ayt::entity::ColliderComponent>(
            ayt::entity::ComponentRegistry::instance(), "ColliderComponent"));
        CHECK(world.initialize());
        CHECK(world.connectedSpaces().addRegion(1));
    }
    ~Fixture() { world.shutdown(); }

    ayt::entity::Entity* box(const char* id, ayt::math::FVector3 center,
                            ayt::math::FVector3 half,
                            bool isStatic = true)
    {
        auto* entity = world.createSpatialEntity();
        CHECK(entity && entity->restorePersistentId(id));
        CHECK(entity->setSpaceRegion(1));
        entity->transform()->setPosition(center.x, center.y, center.z);
        auto* rigid = entity->addComponent<ayt::entity::RigidBodyComponent>();
        CHECK(rigid);
        rigid->setStatic(isStatic);
        auto* collider = entity->addComponent<ayt::entity::ColliderComponent>();
        CHECK(collider);
        collider->addShape().halfExtents = half;
        return entity;
    }
};

constexpr auto floorId = "00000000000000000000000000000011";
constexpr auto obstacleId = "00000000000000000000000000000012";
constexpr auto targetId = "00000000000000000000000000000013";
constexpr auto observerId = "00000000000000000000000000000014";
constexpr auto upperFloorId = "00000000000000000000000000000015";
constexpr auto rampId = "00000000000000000000000000000016";

void tilted_box_extracts_a_bounded_ramp()
{
    Fixture fixture;
    fixture.box(floorId, {3, -0.5f, 3}, {3, 0.5f, 3});
    auto* ramp = fixture.box(rampId, {8.1f, 0.1f, 3}, {2, 0.25f, 1});
    constexpr float halfAngle = 0.1f;
    ramp->transform()->setRotation(0, 0,
        std::sin(halfAngle), std::cos(halfAngle));
    const std::array<std::string_view, 2> floors{floorId, rampId};
    auto extracted = extractWorldBakeRegion(fixture.world, 1, 1, floors);
    CHECK(extracted && extracted.value.ramps.size() == 1
        && extracted.value.additionalFloors.empty());
    StaticBakeWorld bake(82);
    WorldBakeAdapter adapter(fixture.world, bake);
    auto published = adapter.publishRegion(1, 1, floors);
    CHECK(published && published.value && bake.report().nodes > 36);
}

void stacked_world_floor_extraction_preserves_published_graph()
{
    Fixture fixture;
    fixture.box(floorId, {3, -0.5f, 3}, {3, 0.5f, 3});
    fixture.box(upperFloorId, {3, 3.5f, 3}, {3, 0.5f, 3});
    StaticBakeWorld bake(81);
    WorldBakeAdapter adapter(fixture.world, bake);
    const std::array<std::string_view, 2> floors{floorId, upperFloorId};
    auto published = adapter.publishRegion(1, 1, floors);
    CHECK(published && published.value && bake.report().nodes == 72);
    const auto before = bake.current();
    const std::array<std::string_view, 2> duplicate{floorId, floorId};
    CHECK(adapter.publishRegion(1, 2, duplicate).error == Error::Invalid);
    const std::array<std::string_view, 2> missing{floorId, targetId};
    CHECK(adapter.publishRegion(1, 2, missing).error == Error::Missing);
    CHECK(bake.current() == before);
}

void world_geometry_and_unload()
{
    Fixture fixture;
    auto* floor = fixture.box(floorId, {3, -0.5f, 3}, {3, 0.5f, 3});
    auto* blocker = fixture.box(obstacleId, {3, 1, 3}, {1, 1, 1});
    auto* actor = fixture.box(targetId, {4, 1, 5}, {0.5f, 1, 0.5f}, false);
    StaticBakeWorld bake(77);
    WorldBakeAdapter adapter(fixture.world, bake);
    auto first = adapter.publishRegion(1, 1, floorId);
    CHECK(first && first.value && bake.report().blockedCells > 0);
    const auto revision = bake.report().stamp.version;
    CHECK(adapter.publishRegion(1, 1, floorId).value == first.value);
    fixture.world.destroyEntity(blocker);
    auto changed = adapter.publishRegion(1, 2, floorId);
    CHECK(changed && changed.value && bake.report().stamp.version > revision);
    CHECK(bake.report().blockedCells == 0);
    auto* unsupported = fixture.box(obstacleId, {2, 1, 2}, {1, 1, 1});
    unsupported->getComponent<ayt::entity::ColliderComponent>()
        ->shapes.front().shape = ayt::entity::ColliderShapeType::Sphere;
    CHECK(adapter.publishRegion(1, 3, floorId).error == Error::Unsupported);
    CHECK(adapter.publishRegion(1, 2, floorId).error == Error::Unsupported);
    CHECK(bake.current() == changed.value);
    fixture.world.destroyEntity(unsupported);
    fixture.world.destroyEntity(floor);
    fixture.world.destroyEntity(actor);
    CHECK(fixture.world.connectedSpaces().removeRegion(1));
    auto unloaded = adapter.unloadRegion(1);
    CHECK(unloaded && !unloaded.value && !bake.current());
    CHECK(adapter.publishRegion(1, 3, floorId).error == Error::Invalid);
}

void observed_static_geometry_refreshes_without_manual_version()
{
    Fixture fixture;
    fixture.box(floorId, {3, -0.5f, 3}, {3, 0.5f, 3});
    auto* blocker = fixture.box(obstacleId, {3, 1, 3}, {1, 1, 1});
    StaticBakeWorld bake(83);
    WorldBakeAdapter adapter(fixture.world, bake);
    const std::array<std::string_view, 1> floors{floorId};
    const auto first = adapter.refreshRegion(1, floors);
    CHECK(first && first.value && bake.report().geometryRebakes == 1);
    CHECK(bake.report().blockedCells > 0);
    const auto initialRevision = bake.report().stamp.version;
    const auto same = adapter.refreshRegion(1, floors);
    CHECK(same && same.value == first.value);
    CHECK(bake.report().stamp.version == initialRevision);
    blocker->transform()->setPosition(20, 1, 20);
    const auto changed = adapter.refreshRegion(1, floors);
    CHECK(changed && changed.value != first.value);
    CHECK(bake.report().stamp.version == initialRevision + 1);
    CHECK(bake.report().geometryRebakes == 1);
    CHECK(bake.report().blockedCells == 0);
    blocker->getComponent<ayt::entity::ColliderComponent>()
        ->shapes.front().shape = ayt::entity::ColliderShapeType::Sphere;
    CHECK(adapter.refreshRegion(1, floors).error == Error::Unsupported);
    CHECK(bake.current() == changed.value);
    CHECK(bake.report().stamp.version == initialRevision + 1);
}

struct FakeQuery final : ayt::physics::IPhysicsQuery {
    bool ready = true;
    ayt::physics::PhysResult status = ayt::physics::PhysResult::Ok;
    ayt::physics::RaycastHit next{};
    ayt::physics::PhysLayerMask lastMask = 0;
    ayt::physics::BodyHandle lastIgnored = ayt::physics::InvalidBodyHandle;
    bool isReady() const override { return ready; }
    ayt::physics::PhysResult raycastSync(const ayt::math::Ray&,
        ayt::physics::RaycastHit& hit,
        ayt::physics::PhysLayerMask mask) override
    { lastMask = mask; hit = next; return status; }
    ayt::physics::PhysResult raycastSyncIgnoringBody(const ayt::math::Ray& ray,
        ayt::physics::RaycastHit& hit, ayt::physics::PhysLayerMask mask,
        ayt::physics::BodyHandle ignored) override
    { lastIgnored = ignored; return raycastSync(ray, hit, mask); }
    ayt::physics::PhysResult overlapSphereSync(
        const ayt::math::FVector3&, float,
        std::vector<ayt::physics::BodyHandle>&,
        ayt::physics::PhysLayerMask) override
    { return ayt::physics::PhysResult::Unsupported; }
};

void physics_ray_uses_world_identity_and_region_mask()
{
    Fixture fixture;
    auto* observer = fixture.box(observerId, {0, 1, 1}, {.3f, 1, .3f});
    auto* target = fixture.box(targetId, {4, 1, 1}, {.3f, 1, .3f}, false);
    observer->getComponent<ayt::entity::RigidBodyComponent>()->setBodyHandle(41);
    target->getComponent<ayt::entity::RigidBodyComponent>()->setBodyHandle(42);
    ayt::entity::SpacePhysicsRoutes routes;
    CHECK(routes.set(1, 16));
    FakeQuery query;
    PhysicsRayVisibility visibility(query, fixture.world, routes, observerId);
    SightTarget sight{targetId, {1, {4, 0, 1}}, {1, {4, 1.5, 1}}};
    const Location eye{1, {0, 1.5, 1}};
    CHECK(visibility.clear(eye, sight).value);
    CHECK(query.lastMask == (1u << 16));
    CHECK(query.lastIgnored == 41);
    query.next = {.hit = true, .body = 99, .distance = 2.0f};
    CHECK(!visibility.clear(eye, sight).value);
    query.next.body = 42;
    CHECK(visibility.clear(eye, sight).value);
    query.next.body = 41;
    CHECK(visibility.clear(eye, sight).error == Error::QueryFailed);
    query.status = ayt::physics::PhysResult::InvalidState;
    CHECK(visibility.clear(eye, sight).error == Error::QueryFailed);
    query.status = ayt::physics::PhysResult::Ok;
    CHECK(visibility.clear({2, eye.position}, sight).error == Error::QueryFailed);
    fixture.world.destroyEntity(target);
    CHECK(visibility.clear(eye, sight).error == Error::QueryFailed);
}
} // namespace

TEST_MAIN(
    RUN(stacked_world_floor_extraction_preserves_published_graph);
    RUN(tilted_box_extracts_a_bounded_ramp);
    RUN(world_geometry_and_unload);
    RUN(observed_static_geometry_refreshes_without_manual_version);
    RUN(physics_ray_uses_world_identity_and_region_mask);
)
