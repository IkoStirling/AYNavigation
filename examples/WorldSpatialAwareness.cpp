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
#include <iostream>

using namespace ayt::navigation;

namespace {
constexpr auto floorId = "00000000000000000000000000000021";
constexpr auto wallId = "00000000000000000000000000000022";
constexpr auto npcId = "00000000000000000000000000000023";
constexpr auto visitorId = "00000000000000000000000000000024";

ayt::entity::Entity* box(ayt::entity::World& world, const char* id,
    ayt::math::FVector3 center, ayt::math::FVector3 half, bool staticBody)
{
    auto* object = world.createSpatialEntity();
    if (!object || !object->restorePersistentId(id)
        || !object->setSpaceRegion(1)) return nullptr;
    object->transform()->setPosition(center.x, center.y, center.z);
    auto* body = object->addComponent<ayt::entity::RigidBodyComponent>();
    auto* shape = object->addComponent<ayt::entity::ColliderComponent>();
    if (!body || !shape) return nullptr;
    body->setStatic(staticBody);
    shape->addShape().halfExtents = half;
    return object;
}

// A substitute Host query keeps this executable independent of a Jolt setup.
// Replace it with host->physicsQuery() in an assembled game.
struct DemoQuery final : ayt::physics::IPhysicsQuery {
    bool wallOccludes = true;
    bool isReady() const override { return true; }
    ayt::physics::PhysResult raycastSync(const ayt::math::Ray&,
        ayt::physics::RaycastHit& out,
        ayt::physics::PhysLayerMask mask) override
    {
        if (mask != (1u << 16)) return ayt::physics::PhysResult::InvalidParam;
        out = wallOccludes
            ? ayt::physics::RaycastHit{.hit = true, .body = 99,
                                      .distance = 2.0f}
            : ayt::physics::RaycastHit{};
        return ayt::physics::PhysResult::Ok;
    }
    ayt::physics::PhysResult overlapSphereSync(
        const ayt::math::FVector3&, float,
        std::vector<ayt::physics::BodyHandle>&,
        ayt::physics::PhysLayerMask) override
    { return ayt::physics::PhysResult::Unsupported; }
};
} // namespace

int main()
{
    auto& registry = ayt::entity::ComponentRegistry::instance();
    if (!ayt::entity::registerEntityCoreComponents(registry)
        || !ayt::entity::registerComponent<ayt::entity::RigidBodyComponent>(
            registry, "RigidBodyComponent")
        || !ayt::entity::registerComponent<ayt::entity::ColliderComponent>(
            registry, "ColliderComponent")) return 1;
    auto& world = ayt::entity::World::processWorld();
    if (!world.initialize() || !world.connectedSpaces().addRegion(1)) return 1;
    auto* floor = box(world, floorId, {3, -.5f, 3}, {3, .5f, 3}, true);
    auto* wall = box(world, wallId, {3, 1, 3}, {1, 1, 1}, true);
    auto* npc = box(world, npcId, {.5f, 1, 2.5f}, {.25f, 1, .25f}, false);
    auto* visitor = box(world, visitorId, {5.5f, 1, 2.5f}, {.25f, 1, .25f}, false);
    if (!floor || !wall || !npc || !visitor) return 1;
    StaticBakeWorld bake(901);
    WorldBakeAdapter worldBake(world, bake);
    auto graph = worldBake.publishRegion(1, 1, floorId);
    if (!graph || !graph.value || !bake.report().blockedCells) return 1;
    ayt::entity::SpacePhysicsRoutes routes;
    if (!routes.set(1, 16)) return 1;
    DemoQuery query;
    PhysicsRayVisibility sight(query, world, routes, npcId);
    PerceptionMemory memory;
    const Location eye{1, {.5, 1.7, 2.5}};
    const Location home{1, {.5, 0, 2.5}};
    const std::array target{SightTarget{visitorId,
        {1, {5.5, 0, 2.5}}, {1, {5.5, 1.7, 2.5}}}};
    auto unseen = memory.update(1, eye, {1, 0, 0}, target, {}, home, sight);
    if (!unseen || unseen.value.mode != PursuitMode::Return) return 1;
    query.wallOccludes = false;
    auto seen = memory.update(2, eye, {1, 0, 0}, target, {}, home, sight);
    if (!seen || seen.value.mode != PursuitMode::Chase) return 1;
    PlanRequest request;
    request.id = request.agent = request.task = 1;
    request.from = home;
    request.goal = seen.value.goal;
    request.profile.steering = Steering::Planar;
    request.space = graph.value;
    auto route = GraphPlanner{}.poll(request);
    if (route.status != PlanStatus::Found || route.route.steps.size() < 6)
        return 1;
    world.destroyEntity(wall);
    auto rebuilt = worldBake.publishRegion(1, 2, floorId);
    if (!rebuilt || !rebuilt.value || bake.report().blockedCells) return 1;
    std::cout << "World-backed room: " << graph.value->definition().nodes.size()
              << " walkable cells before wall removal, "
              << rebuilt.value->definition().nodes.size()
              << " after; physics query adapter changed sight Return -> Chase\n";
    world.shutdown();
    return 0;
}
