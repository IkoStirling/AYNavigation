#pragma once

#include <AYNavigation/Navigation.h>

#include <map>
#include <span>
#include <string>

namespace ayt::navigation {

/// Stable object identity and current-world location, supplied by the Host.
struct SightTarget {
    std::string persistentId;
    Location location; // Feet/navigation goal.
    Location sightPoint; // Optional head/torso aim point; zero region uses location.
};

/// A current fixed-step sound fact. Event IDs must be unique while retained;
/// repeated delivery of the same ID does not refresh memory.
struct SoundStimulus {
    std::uint64_t eventId = 0;
    std::string sourcePersistentId;
    Location location;
    double audibleRadius = 0.0;
};

/// Host implements this with a current-world ray or equivalent occlusion
/// query. Failure is distinct from an unobstructed line and aborts update.
class VisibilityQuery {
public:
    virtual ~VisibilityQuery() = default;
    virtual Result<bool> clear(Location eye, const SightTarget& target) = 0;
};

struct PerceptionConfig {
    double visualRange = 15.0;
    double halfAngleRadians = 1.0;
    std::uint64_t memoryTicks = 90;
    std::size_t maxEntries = 32;
};

enum class PursuitMode { Return, Search, Chase };
struct PursuitDecision {
    PursuitMode mode = PursuitMode::Return;
    std::string targetPersistentId;
    Location goal;
};

/// Bounded sensory memory for one NPC. It makes a generic chase/search/return
/// goal suggestion; a game or AYBehavior owner chooses task priority and
/// submission. No Entity/Physics handle or external pointer is retained.
class PerceptionMemory {
public:
    explicit PerceptionMemory(PerceptionConfig config = {}) : _config(config) {}
    Result<PursuitDecision> update(std::uint64_t tick, Location eye, Vec3 forward,
        std::span<const SightTarget> targets,
        std::span<const SoundStimulus> sounds,
        Location home, VisibilityQuery& visibility);
    /// Call from a region-unload boundary before its transient entities die.
    /// Clears both remembered targets and sound deduplication for that region.
    void forgetRegion(RegionId region) noexcept;
    [[nodiscard]] std::size_t rememberedCount() const noexcept { return _memory.size(); }

private:
    struct Remembered { Location lastKnown; std::uint64_t lastTick = 0; };
    struct SeenSound { RegionId region = 0; std::uint64_t lastTick = 0; };
    PerceptionConfig _config;
    std::uint64_t _lastTick = 0;
    std::map<std::string, Remembered> _memory;
    std::map<std::uint64_t, SeenSound> _seenSounds;
};

} // namespace ayt::navigation
