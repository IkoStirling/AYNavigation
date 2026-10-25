#include <AYNavigation/ConnectedSpaceSound.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace ayt::navigation {
namespace {
Vec3 position(const math::FVector3& value) noexcept
{
    return {value.x, value.y, value.z};
}

bool validRegion(const entity::ConnectedSpaces& spaces, RegionId region) noexcept
{
    return region && region <= std::numeric_limits<entity::SpaceRegionId>::max()
        && spaces.containsRegion(static_cast<entity::SpaceRegionId>(region));
}
} // namespace

Result<std::optional<double>> ConnectedSpaceSoundTravel::distance(
    Location listener, const SoundStimulus& sound)
{
    if (!validRegion(_spaces, listener.region)
        || !validRegion(_spaces, sound.location.region)
        || !finite(listener.position) || !finite(sound.location.position))
        return {Error::Invalid};
    if (_available && (!_available(listener.region)
        || !_available(sound.location.region)))
        return {Error::None, std::nullopt};

    std::vector<const entity::SpacePortal*> portals;
    for (const auto& portal : _spaces.portals()) {
        if (!portal.enabled || (_available &&
            (!_available(portal.source) || !_available(portal.destination))))
            continue;
        if (portals.size() == _maxPortals) return {Error::Budget};
        portals.push_back(&portal);
    }

    const double infinity = std::numeric_limits<double>::infinity();
    double best = listener.region == sound.location.region
        ? length(listener.position - sound.location.position) : infinity;
    std::vector<double> costs(portals.size(), infinity);
    std::vector<bool> visited(portals.size(), false);
    for (std::size_t i = 0; i < portals.size(); ++i)
        if (portals[i]->source == listener.region)
            costs[i] = length(listener.position - position(portals[i]->entrance.position));

    for (std::size_t step = 0; step < portals.size(); ++step) {
        std::size_t next = portals.size();
        for (std::size_t i = 0; i < portals.size(); ++i)
            if (!visited[i] && costs[i] < best
                && (next == portals.size() || costs[i] < costs[next])) next = i;
        if (next == portals.size()) break;
        visited[next] = true;
        const auto& crossed = *portals[next];
        const auto exit = position(crossed.exit.position);
        if (crossed.destination == sound.location.region)
            best = std::min(best, costs[next]
                + length(exit - sound.location.position));
        for (std::size_t i = 0; i < portals.size(); ++i)
            if (!visited[i] && portals[i]->source == crossed.destination)
                costs[i] = std::min(costs[i], costs[next]
                    + length(exit - position(portals[i]->entrance.position)));
    }
    if (std::isnan(best)) return {Error::QueryFailed};
    return {Error::None, std::isfinite(best)
        ? std::optional<double>{best} : std::nullopt};
}
} // namespace ayt::navigation
