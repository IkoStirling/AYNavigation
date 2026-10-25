#include <AYNavigation/Perception.h>

#include <algorithm>
#include <cmath>
#include <set>

namespace ayt::navigation {

Result<PursuitDecision> PerceptionMemory::update(
    std::uint64_t tick, Location eye, Vec3 forward,
    std::span<const SightTarget> targets,
    std::span<const SoundStimulus> sounds,
    Location home, VisibilityQuery& visibility,
    SoundTravelQuery* soundTravel)
{
    if (!tick || tick <= _lastTick || !eye.region || !home.region
        || !finite(eye.position) || !finite(home.position)
        || !finite(forward) || length(forward) <= 0.0
        || !std::isfinite(_config.visualRange) || _config.visualRange <= 0.0
        || !std::isfinite(_config.halfAngleRadians)
        || _config.halfAngleRadians <= 0.0
        || _config.halfAngleRadians > 3.14159265358979323846
        || !_config.memoryTicks || !_config.maxEntries || _config.maxEntries > 1024
        || targets.size() > _config.maxEntries
        || sounds.size() > _config.maxEntries)
        return {Error::Invalid};

    std::set<std::string> targetIds;
    for (const auto& target : targets) {
        if (target.persistentId.empty() || !target.location.region
            || !finite(target.location.position)
            || (target.sightPoint.region != 0
                && (target.sightPoint.region != target.location.region
                    || !finite(target.sightPoint.position)))
            || !targetIds.insert(target.persistentId).second)
            return {Error::Invalid};
    }
    std::set<std::uint64_t> soundIds;
    for (const auto& sound : sounds) {
        if (!sound.eventId || sound.sourcePersistentId.empty()
            || !sound.location.region || !finite(sound.location.position)
            || !std::isfinite(sound.audibleRadius) || sound.audibleRadius < 0.0
            || !soundIds.insert(sound.eventId).second)
            return {Error::Invalid};
    }

    auto stagedMemory = _memory;
    auto stagedSounds = _seenSounds;
    for (auto it = stagedMemory.begin(); it != stagedMemory.end();) {
        if (tick - it->second.lastTick > _config.memoryTicks)
            it = stagedMemory.erase(it);
        else ++it;
    }
    for (auto it = stagedSounds.begin(); it != stagedSounds.end();) {
        if (tick - it->second.lastTick > _config.memoryTicks)
            it = stagedSounds.erase(it);
        else ++it;
    }

    for (const auto& sound : sounds) {
        if (stagedSounds.contains(sound.eventId)) continue;
        double distance = 0.0;
        if (soundTravel) {
            const auto path = soundTravel->distance(eye, sound);
            if (!path) return {path.error};
            if (!path.value) continue;
            distance = *path.value;
            if (!std::isfinite(distance) || distance < 0.0)
                return {Error::QueryFailed};
        } else {
            if (sound.location.region != eye.region) continue;
            distance = length(sound.location.position - eye.position);
        }
        if (distance > sound.audibleRadius) continue;
        stagedSounds.emplace(sound.eventId,
                             SeenSound{sound.location.region, tick});
        stagedMemory[sound.sourcePersistentId] = {sound.location, tick};
    }

    const auto facing = normalized(forward);
    const double minCosine = std::cos(_config.halfAngleRadians);
    std::string nearestVisible;
    double nearestDistance = _config.visualRange;
    Location visibleGoal;
    for (const auto& target : targets) {
        if (target.location.region != eye.region) continue;
        const auto sightPoint = target.sightPoint.region
            ? target.sightPoint : target.location;
        const auto delta = sightPoint.position - eye.position;
        const double distance = length(delta);
        if (distance > _config.visualRange) continue;
        if (distance > 0.0 && dot(facing, delta * (1.0 / distance)) < minCosine)
            continue;
        const auto line = visibility.clear(eye, target);
        if (!line) return {Error::QueryFailed};
        if (!line.value) continue;
        stagedMemory[target.persistentId] = {target.location, tick};
        if (nearestVisible.empty() || distance < nearestDistance
            || (distance == nearestDistance
                && target.persistentId < nearestVisible)) {
            nearestVisible = target.persistentId;
            nearestDistance = distance;
            visibleGoal = target.location;
        }
    }

    while (stagedMemory.size() > _config.maxEntries) {
        auto oldest = stagedMemory.end();
        for (auto it = stagedMemory.begin(); it != stagedMemory.end(); ++it)
            if (it->first != nearestVisible
                && (oldest == stagedMemory.end()
                    || it->second.lastTick < oldest->second.lastTick
                    || (it->second.lastTick == oldest->second.lastTick
                        && it->first > oldest->first))) oldest = it;
        if (oldest == stagedMemory.end()) break;
        stagedMemory.erase(oldest);
    }
    const std::size_t maxSoundIds = _config.maxEntries * 4;
    while (stagedSounds.size() > maxSoundIds) {
        auto oldest = stagedSounds.begin();
        for (auto it = stagedSounds.begin(); it != stagedSounds.end(); ++it)
            if (it->second.lastTick < oldest->second.lastTick
                || (it->second.lastTick == oldest->second.lastTick
                    && it->first > oldest->first))
                oldest = it;
        stagedSounds.erase(oldest);
    }

    PursuitDecision decision;
    if (!nearestVisible.empty()) {
        decision = {PursuitMode::Chase, nearestVisible, visibleGoal};
    } else if (!stagedMemory.empty()) {
        auto newest = stagedMemory.begin();
        for (auto it = stagedMemory.begin(); it != stagedMemory.end(); ++it)
            if (it->second.lastTick > newest->second.lastTick
                || (it->second.lastTick == newest->second.lastTick
                    && it->first < newest->first)) newest = it;
        decision = {PursuitMode::Search, newest->first,
                    newest->second.lastKnown};
    } else {
        decision = {PursuitMode::Return, {}, home};
    }
    _memory.swap(stagedMemory);
    _seenSounds.swap(stagedSounds);
    _lastTick = tick;
    return {Error::None, std::move(decision)};
}

void PerceptionMemory::forgetRegion(RegionId region) noexcept
{
    for (auto it = _memory.begin(); it != _memory.end();) {
        if (it->second.lastKnown.region == region) it = _memory.erase(it);
        else ++it;
    }
    for (auto it = _seenSounds.begin(); it != _seenSounds.end();) {
        if (it->second.region == region) it = _seenSounds.erase(it);
        else ++it;
    }
}

} // namespace ayt::navigation
