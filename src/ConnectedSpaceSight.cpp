#include <AYNavigation/ConnectedSpaceSight.h>

#include <cmath>
#include <limits>

namespace ayt::navigation {
namespace {
math::FVector3 point(Vec3 value) {
    return {float(value.x), float(value.y), float(value.z)};
}
Vec3 point(const math::FVector3& value) {
    return {value.x, value.y, value.z};
}
bool validRegion(const entity::ConnectedSpaces& spaces, RegionId region) {
    return region && region <= std::numeric_limits<entity::SpaceRegionId>::max()
        && spaces.containsRegion(static_cast<entity::SpaceRegionId>(region));
}
} // namespace

Result<std::optional<SightPath>> ConnectedSpaceSightTravel::path(
    Location eye, const SightTarget& target) {
    const auto aim = target.sightPoint.region
        ? target.sightPoint : target.location;
    if (!_rayQuery || !validRegion(_spaces, eye.region)
        || !validRegion(_spaces, aim.region)
        || target.location.region != aim.region
        || target.persistentId.empty() || !finite(eye.position)
        || !finite(aim.position))
        return {Error::Invalid};
    try {
        if (_available && (!_available(eye.region) || !_available(aim.region)))
            return {Error::None, std::nullopt};
        std::optional<SightPath> best;
        auto checkRay = [&](math::FVector3 apparent, std::uint32_t hops)
            -> Error {
            const auto displacement = apparent - point(eye.position);
            const float distance = displacement.length();
            if (!std::isfinite(distance) || distance <= 0.0f
                || distance > 10000.0f) return Error::QueryFailed;
            const auto traced = _spaces.traceRay(
                static_cast<entity::SpaceRegionId>(eye.region),
                {point(eye.position), displacement / distance},
                distance, hops, _rayQuery);
            if (traced.status == entity::SpaceTraceStatus::QueryFailed)
                return Error::QueryFailed;
            if (traced.status != entity::SpaceTraceStatus::Miss
                || traced.crossings != hops || traced.region != aim.region
                || (traced.position - point(aim.position)).length() > 0.05f)
                return Error::None;
            const SightPath candidate{point(apparent), double(distance)};
            if (!best || candidate.distance < best->distance)
                best = candidate;
            return Error::None;
        };
        if (eye.region == aim.region) {
            const auto error = checkRay(point(aim.position), 0);
            return error == Error::None
                ? Result<std::optional<SightPath>>{Error::None, best}
                : Result<std::optional<SightPath>>{error};
        }
        std::size_t inspected = 0;
        for (const auto& portal : _spaces.portals()) {
            if (!portal.enabled || portal.source != eye.region
                || portal.destination != aim.region) continue;
            if (++inspected > _maxPortals) return {Error::Budget};
            entity::ConnectedSpaces reverse;
            if (!reverse.addRegion(portal.source)
                || !reverse.addRegion(portal.destination))
                return {Error::Invalid};
            auto reversed = portal;
            std::swap(reversed.source, reversed.destination);
            std::swap(reversed.entrance, reversed.exit);
            if (!reverse.setPortal(reversed)) return {Error::Invalid};
            entity::SpacePose apparent;
            if (!reverse.mapPose(reversed.id,
                {point(aim.position), {0, 0, 0, 1}}, apparent))
                return {Error::QueryFailed};
            const auto error = checkRay(apparent.position, 1);
            if (error != Error::None) return {error};
        }
        return {Error::None, best};
    } catch (...) {
        return {Error::QueryFailed};
    }
}

} // namespace ayt::navigation
