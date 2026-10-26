#pragma once

#include <AYNavigation/Perception.h>
#include <AYEntity/ConnectedSpaces.h>

#include <cstddef>
#include <functional>
#include <utility>

namespace ayt::navigation {

/// Approximate sound travel through the same directed ConnectedSpaces portals
/// used by movement and view planning. Intra-region travel uses straight-line
/// distance to portal centers; opening geometry, walls, diffraction and
/// acoustic attenuation remain Host/game policy. Add reverse portals when
/// sound should travel both ways. Query at a quiescent topology boundary.
class ConnectedSpaceSoundTravel final : public SoundTravelQuery {
public:
    using RegionAvailable = std::function<bool(RegionId)>;
    /// Optional equivalent-distance penalty for crossing one directed portal.
    /// Return a finite, nonnegative value; an invalid value fails the query.
    /// Keep the callback pure during one fixed-step perception update.
    using PortalCost = std::function<double(const entity::SpacePortal&)>;
    /// A residency predicate may reference WorldRegionStream::isLoaded so
    /// unloaded intermediate regions cannot conduct sound. It must be pure
    /// for the duration of one fixed-step perception update.
    explicit ConnectedSpaceSoundTravel(const entity::ConnectedSpaces& spaces,
        std::size_t maxPortals = 128, RegionAvailable available = {},
        PortalCost portalCost = {})
        : _spaces(spaces), _maxPortals(maxPortals),
          _available(std::move(available)), _portalCost(std::move(portalCost)) {}

    Result<std::optional<double>> distance(
        Location listener, const SoundStimulus& sound) override;

private:
    const entity::ConnectedSpaces& _spaces;
    std::size_t _maxPortals;
    RegionAvailable _available;
    PortalCost _portalCost;
};

} // namespace ayt::navigation
