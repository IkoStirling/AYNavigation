#include "Test.h"

#include <AYNavigation/PhysicsVisibility.h>
#include <AYNavigation/ConnectedSpaceSound.h>
#include <AYNavigation/ConnectedSpaceSight.h>
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
#include <limits>

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

void connected_space_sound_follows_directed_resident_portals()
{
    Fixture fixture;
    auto& spaces = fixture.world.connectedSpaces();
    CHECK(spaces.addRegion(2));
    CHECK(spaces.addRegion(3));
    ayt::entity::SpacePortal first;
    first.id = 10;
    first.source = 1;
    first.destination = 2;
    first.entrance.position = {1, 0, 0};
    first.exit.position = {100, 0, 0};
    CHECK(spaces.setPortal(first));
    ayt::entity::SpacePortal second = first;
    second.id = 11;
    second.source = 2;
    second.destination = 3;
    second.entrance.position = {102, 0, 0};
    second.exit.position = {0, 0, 0};
    CHECK(spaces.setPortal(second));
    ConnectedSpaceSoundTravel travel(spaces);
    const Location ear{1, {0, 0, 0}};
    const SoundStimulus sound{101, "visitor", {2, {101, 0, 0}}, 2.5};
    const auto path = travel.distance(ear, sound);
    CHECK(path && path.value && std::abs(*path.value - 2.0) < 1e-9);
    const auto chain = travel.distance(ear,
        SoundStimulus{102, "far-visitor", {3, {1, 0, 0}}, 4.5});
    CHECK(chain && chain.value && std::abs(*chain.value - 4.0) < 1e-9);
    const auto reverse = travel.distance({2, {101, 0, 0}},
        SoundStimulus{103, "reverse", ear, 100.0});
    CHECK(reverse && !reverse.value);
    ayt::entity::SpacePortal returnPortal = first;
    returnPortal.id = 12;
    returnPortal.source = 2;
    returnPortal.destination = 1;
    returnPortal.entrance.position = {101, 0, 0};
    returnPortal.exit.position = {20, 0, 0};
    CHECK(spaces.setPortal(returnPortal));
    const auto sameRegionShortcut = travel.distance(ear,
        SoundStimulus{105, "shortcut", {1, {21, 0, 0}}, 3.0});
    CHECK(sameRegionShortcut && sameRegionShortcut.value
        && std::abs(*sameRegionShortcut.value - 3.0) < 1e-9);
    CHECK(spaces.removePortal(returnPortal.id));

    struct Clear final : VisibilityQuery {
        Result<bool> clear(Location, const SightTarget&) override
        { return {Error::None, true}; }
    } clear;
    PerceptionMemory memory({10.0, 0.8, 3, 4});
    auto unheard = memory.update(1, ear, {1, 0, 0}, {},
        std::array{sound}, ear, clear);
    CHECK(unheard && unheard.value.mode == PursuitMode::Return);
    auto heard = memory.update(2, ear, {1, 0, 0}, {},
        std::array{sound}, ear, clear, &travel);
    CHECK(heard && heard.value.mode == PursuitMode::Search
        && heard.value.goal == sound.location);
    memory.forgetRegion(2);
    CHECK(memory.rememberedCount() == 0);
    auto replayed = memory.update(3, ear, {1, 0, 0}, {},
        std::array{sound}, ear, clear, &travel);
    CHECK(replayed && replayed.value.mode == PursuitMode::Search);

    ConnectedSpaceSoundTravel budgeted(spaces, 1);
    PerceptionMemory atomicMemory;
    CHECK(atomicMemory.update(1, ear, {1, 0, 0}, {},
        std::array{sound}, ear, clear, &budgeted).error == Error::Budget);
    CHECK(atomicMemory.rememberedCount() == 0);
    CHECK(atomicMemory.update(1, ear, {1, 0, 0}, {},
        std::array{sound}, ear, clear, &travel));

    struct InvalidSecondRoute final : SoundTravelQuery {
        Result<std::optional<double>> distance(Location,
            const SoundStimulus& event) override
        {
            return {Error::None, event.eventId == 106
                ? std::optional<double>{std::numeric_limits<double>::quiet_NaN()}
                : std::optional<double>{0.0}};
        }
    } invalidRoute;
    PerceptionMemory rollbackMemory;
    const std::array pair{sound,
        SoundStimulus{106, "invalid-route", {2, {100, 0, 0}}, 3.0}};
    CHECK(rollbackMemory.update(1, ear, {1, 0, 0}, {},
        pair, ear, clear, &invalidRoute).error == Error::QueryFailed);
    CHECK(rollbackMemory.rememberedCount() == 0);
    CHECK(rollbackMemory.update(1, ear, {1, 0, 0}, {},
        std::array{sound}, ear, clear, &travel));

    ConnectedSpaceSoundTravel resident(spaces, 128,
        [](RegionId region) { return region != 2; });
    const auto unavailable = resident.distance(ear,
        SoundStimulus{104, "far-visitor", {3, {1, 0, 0}}, 100.0});
    CHECK(unavailable && !unavailable.value);
    first.enabled = false;
    CHECK(spaces.setPortal(first));
    const auto disabled = travel.distance(ear, sound);
    CHECK(disabled && !disabled.value);
}

void connected_space_sound_accounts_for_portal_crossing_cost()
{
    Fixture fixture;
    auto& spaces = fixture.world.connectedSpaces();
    CHECK(spaces.addRegion(2));
    CHECK(spaces.addRegion(3));
    ayt::entity::SpacePortal direct;
    direct.id = 20;
    direct.source = 1;
    direct.destination = 2;
    direct.entrance.position = {1, 0, 0};
    direct.exit.position = {1, 0, 0};
    CHECK(spaces.setPortal(direct));
    ayt::entity::SpacePortal detour = direct;
    detour.id = 21;
    detour.destination = 3;
    detour.entrance.position = {2, 0, 0};
    detour.exit.position = {0, 0, 0};
    CHECK(spaces.setPortal(detour));
    detour.id = 22;
    detour.source = 3;
    detour.destination = 2;
    detour.entrance.position = {0, 0, 0};
    detour.exit.position = {1, 0, 0};
    CHECK(spaces.setPortal(detour));

    double directCost = 5.0;
    ConnectedSpaceSoundTravel travel(spaces, 128, {},
        [&](const ayt::entity::SpacePortal& portal) {
            return portal.id == direct.id ? directCost : 0.0;
        });
    const Location ear{1, {0, 0, 0}};
    const SoundStimulus sound{201, "through-door", {2, {2, 0, 0}}, 2.5};
    const auto detoured = travel.distance(ear, sound);
    CHECK(detoured && detoured.value && std::abs(*detoured.value - 3.0) < 1e-9);
    directCost = 0.0;
    const auto opened = travel.distance(ear, sound);
    CHECK(opened && opened.value && std::abs(*opened.value - 2.0) < 1e-9);

    struct Clear final : VisibilityQuery {
        Result<bool> clear(Location, const SightTarget&) override
        { return {Error::None, true}; }
    } clear;
    PerceptionMemory memory;
    directCost = 5.0;
    CHECK(memory.update(1, ear, {1, 0, 0}, {},
        std::array{sound}, ear, clear, &travel));
    CHECK(memory.rememberedCount() == 0);
    directCost = 0.0;
    CHECK(memory.update(2, ear, {1, 0, 0}, {},
        std::array{sound}, ear, clear, &travel));
    CHECK(memory.rememberedCount() == 1);

    for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()}) {
        directCost = invalid;
        CHECK(travel.distance(ear, sound).error == Error::QueryFailed);
        PerceptionMemory rollback;
        CHECK(rollback.update(1, ear, {1, 0, 0}, {},
            std::array{sound}, ear, clear, &travel).error == Error::QueryFailed);
        CHECK(rollback.rememberedCount() == 0);
    }
}

void connected_space_sight_uses_the_actual_opening_and_keeps_memory_atomic()
{
    Fixture fixture;
    auto& spaces = fixture.world.connectedSpaces();
    CHECK(spaces.addRegion(2));
    ayt::entity::SpacePortal portal;
    portal.id = 30;
    portal.source = 1;
    portal.destination = 2;
    portal.entrance.position = {0, 0, 1};
    portal.exit.position = {0, 0, 100};
    portal.halfWidth = 0.5f;
    portal.halfHeight = 1.0f;
    CHECK(spaces.setPortal(portal));
    const Location eye{1, {0, 0, 2}};
    const SightTarget visitor{"visitor", {2, {0, 0, 102}}, {}};
    const ayt::entity::SpaceRayQuery unblocked = [](
        ayt::entity::SpaceRegionId, const ayt::math::Ray&, float) {
        return std::optional<ayt::entity::SpaceLocalHit>{{false, 0}};
    };
    ConnectedSpaceSightTravel sight(spaces, unblocked);
    auto path = sight.path(eye, visitor);
    CHECK(path && path.value && std::abs(path.value->distance - 3.0) < 0.01);
    CHECK(path.value->apparentAim.z < eye.position.z);
    struct Clear final : VisibilityQuery {
        int calls = 0;
        Result<bool> clear(Location, const SightTarget&) override {
            ++calls;
            return {Error::None, true};
        }
    } clear;
    PerceptionMemory memory({5.0, 0.6, 4, 4});
    auto seen = memory.update(1, eye, {0, 0, -1}, std::array{visitor}, {},
        eye, clear, nullptr, &sight);
    CHECK(seen && seen.value.mode == PursuitMode::Chase
        && seen.value.goal == visitor.location && clear.calls == 0);
    CHECK(memory.rememberedCount() == 1);

    const ayt::entity::SpaceRayQuery blocked = [](
        ayt::entity::SpaceRegionId region, const ayt::math::Ray&, float) {
        return std::optional<ayt::entity::SpaceLocalHit>{
            {region == 1, 0.5f}};
    };
    ConnectedSpaceSightTravel wall(spaces, blocked);
    CHECK(wall.path(eye, visitor) && !wall.path(eye, visitor).value);
    PerceptionMemory hidden;
    auto unseen = hidden.update(1, eye, {0, 0, -1}, std::array{visitor}, {},
        eye, clear, nullptr, &wall);
    CHECK(unseen && unseen.value.mode == PursuitMode::Return);
    const ayt::entity::SpaceRayQuery failed = [](
        ayt::entity::SpaceRegionId, const ayt::math::Ray&, float)
        -> std::optional<ayt::entity::SpaceLocalHit> { return std::nullopt; };
    ConnectedSpaceSightTravel broken(spaces, failed);
    PerceptionMemory rollback;
    CHECK(rollback.update(1, eye, {0, 0, -1}, std::array{visitor}, {},
        eye, clear, nullptr, &broken).error == Error::QueryFailed);
    CHECK(rollback.rememberedCount() == 0);
    CHECK(rollback.update(1, eye, {0, 0, -1}, std::array{visitor}, {},
        eye, clear, nullptr, &sight));
    ConnectedSpaceSightTravel unavailable(spaces, unblocked, 128,
        [](RegionId region) { return region != 2; });
    CHECK(unavailable.path(eye, visitor) && !unavailable.path(eye, visitor).value);
    ConnectedSpaceSightTravel budget(spaces, unblocked, 0);
    CHECK(budget.path(eye, visitor).error == Error::Budget);
    portal.enabled = false;
    CHECK(spaces.setPortal(portal));
    CHECK(sight.path(eye, visitor) && !sight.path(eye, visitor).value);
}
} // namespace

TEST_MAIN(
    RUN(stacked_world_floor_extraction_preserves_published_graph);
    RUN(tilted_box_extracts_a_bounded_ramp);
    RUN(world_geometry_and_unload);
    RUN(observed_static_geometry_refreshes_without_manual_version);
    RUN(physics_ray_uses_world_identity_and_region_mask);
    RUN(connected_space_sound_follows_directed_resident_portals);
    RUN(connected_space_sound_accounts_for_portal_crossing_cost);
    RUN(connected_space_sight_uses_the_actual_opening_and_keeps_memory_atomic);
)
