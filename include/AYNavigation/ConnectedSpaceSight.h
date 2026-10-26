#pragma once

#include <AYNavigation/Perception.h>
#include <AYEntity/ConnectedSpaces.h>

#include <cstddef>
#include <functional>

namespace ayt::navigation {

/// One-hop portal sight over the same rigid topology used by movement and
/// rendering. The borrowed region-local ray callback checks static/dynamic
/// occluders up to its supplied segment length. It should treat the observed
/// target collider as transparent and exclude the observer's own body.
/// The caller keeps topology and residency stable during one perception tick.
class ConnectedSpaceSightTravel final : public SightTravelQuery {
public:
    using RegionAvailable = std::function<bool(RegionId)>;
    ConnectedSpaceSightTravel(const entity::ConnectedSpaces& spaces,
        entity::SpaceRayQuery rayQuery, std::size_t maxPortals = 128,
        RegionAvailable available = {})
        : _spaces(spaces), _rayQuery(std::move(rayQuery)),
          _maxPortals(maxPortals), _available(std::move(available)) {}

    /// Returns the shortest clear one-hop view, nullopt if no opening admits
    /// the ray or an occluder blocks it, and QueryFailed on invalid ray input.
    Result<std::optional<SightPath>> path(
        Location eye, const SightTarget& target) override;

private:
    const entity::ConnectedSpaces& _spaces;
    entity::SpaceRayQuery _rayQuery;
    std::size_t _maxPortals;
    RegionAvailable _available;
};

} // namespace ayt::navigation
