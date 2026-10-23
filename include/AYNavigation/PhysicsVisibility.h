#pragma once

#include <AYNavigation/Perception.h>
#include <AYEntity/ConnectedSpacePhysicsRoutes.h>
#include <AYPhysics/IPhysicsQuery.h>

#include <string>

namespace ayt::entity { class World; }

namespace ayt::navigation {

/// Optional current-World 3D ray adapter. The borrowed query/World/routes must
/// outlive it. Only same-region sight is supported. A ray that hits the
/// observer's own body fails explicitly because raycastSync cannot ignore it;
/// place the eye outside self collision or choose a suitable physics layer.
class PhysicsRayVisibility final : public VisibilityQuery {
public:
    PhysicsRayVisibility(physics::IPhysicsQuery& query,
        const entity::World& world, const entity::SpacePhysicsRoutes& routes,
        std::string observerPersistentId)
        : _query(query), _world(world), _routes(routes),
          _observerPersistentId(std::move(observerPersistentId)) {}
    Result<bool> clear(Location eye, const SightTarget& target) override;
private:
    physics::IPhysicsQuery& _query;
    const entity::World& _world;
    const entity::SpacePhysicsRoutes& _routes;
    std::string _observerPersistentId;
};

} // namespace ayt::navigation
