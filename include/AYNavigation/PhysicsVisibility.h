#pragma once

#include <AYNavigation/Perception.h>
#include <AYEntity/ConnectedSpacePhysicsRoutes.h>
#include <AYPhysics/IPhysicsQuery.h>

#include <string>

namespace ayt::entity { class World; }

namespace ayt::navigation {

/// Optional current-World 3D ray adapter. The borrowed query/World/routes must
/// outlive it. Only same-region sight is supported. The observer body is
/// excluded by the 3D query when one is registered; unsupported query backends
/// fail explicitly instead of treating self collision as clear visibility.
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
