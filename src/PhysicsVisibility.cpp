#include <AYNavigation/PhysicsVisibility.h>

#include <AYEntity.h>
#include <AYEntity/World.h>
#include <AYEntity/components/RigidBodyComponent.h>

#include <cmath>
#include <cstdint>

namespace ayt::navigation {
Result<bool> PhysicsRayVisibility::clear(Location eye,
    const SightTarget& target)
{
    const auto aim = target.sightPoint.region
        ? target.sightPoint : target.location;
    if (!eye.region || eye.region != aim.region
        || eye.region > UINT32_MAX || !finite(eye.position)
        || !finite(aim.position) || target.persistentId.empty()
        || !_query.isReady()) return {Error::QueryFailed};
    const auto route = _routes.resolve(
        static_cast<entity::SpaceRegionId>(eye.region));
    if (!route) return {Error::QueryFailed};
    auto* targetEntity = _world.findEntityByPersistentId(target.persistentId);
    if (!targetEntity || !targetEntity->isSpatial()
        || targetEntity->effectiveSpaceRegion() != eye.region)
        return {Error::QueryFailed};
    physics::BodyHandle targetBody = physics::InvalidBodyHandle;
    if (auto* body = targetEntity->getComponent<entity::RigidBodyComponent>())
        targetBody = body->getBodyHandle();
    physics::BodyHandle observerBody = physics::InvalidBodyHandle;
    if (!_observerPersistentId.empty()) {
        auto* observer = _world.findEntityByPersistentId(_observerPersistentId);
        if (!observer || !observer->isSpatial()
            || observer->effectiveSpaceRegion() != eye.region)
            return {Error::QueryFailed};
        if (auto* body = observer->getComponent<entity::RigidBodyComponent>())
            observerBody = body->getBodyHandle();
    }
    const auto delta = aim.position - eye.position;
    const auto distance = length(delta);
    if (!std::isfinite(distance) || distance <= 0.0 || distance > 10000.0)
        return {Error::QueryFailed};
    const math::Ray ray{
        {float(eye.position.x), float(eye.position.y), float(eye.position.z)},
        {float(delta.x / distance), float(delta.y / distance),
         float(delta.z / distance)}};
    physics::RaycastHit hit;
    if (_query.raycastSync(ray, hit, route->collideMask)
        != physics::PhysResult::Ok)
        return {Error::QueryFailed};
    if (hit.hit && (!std::isfinite(hit.distance) || hit.distance < 0))
        return {Error::QueryFailed};
    if (!hit.hit || double(hit.distance) >= distance - 0.01)
        return {Error::None, true};
    if (observerBody != physics::InvalidBodyHandle
        && hit.body == observerBody)
        return {Error::QueryFailed};
    return {Error::None, targetBody != physics::InvalidBodyHandle
        && hit.body == targetBody};
}
} // namespace ayt::navigation
