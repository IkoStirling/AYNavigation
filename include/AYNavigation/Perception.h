#pragma once

#include <AYNavigation/Navigation.h>

#include <map>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace ayt::navigation {

/// Stable object identity and current-world location, supplied by the Host.
struct SightTarget {
    std::string persistentId;
    Location location; // Feet/navigation goal.
    Location sightPoint; // Optional head/torso aim point; zero region uses location.
};

/// A current fixed-step sound fact. Event IDs must be unique while retained;
/// repeated delivery of the same ID after it is heard does not refresh memory.
/// An inaudible event is not consumed, so a listener entering its radius while
/// the Host still supplies it can hear it once.
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

/// Optional acoustic route distance. No route is std::nullopt; query failure
/// aborts the perception update without publishing memory or sound receipts.
/// The default without this service hears only within the listener's region.
class SoundTravelQuery {
public:
    virtual ~SoundTravelQuery() = default;
    virtual Result<std::optional<double>> distance(
        Location listener, const SoundStimulus& sound) = 0;
};

/// A cross-region visible route and the target's apparent aim in the
/// observer's region. Distance includes portal travel, not Euclidean distance
/// between unrelated region coordinates. The query owns occlusion checking.
struct SightPath {
    Vec3 apparentAim;
    double distance = 0.0;
};
class SightTravelQuery {
public:
    virtual ~SightTravelQuery() = default;
    /// No route/blocked sight is nullopt; failure aborts memory publication.
    virtual Result<std::optional<SightPath>> path(
        Location eye, const SightTarget& target) = 0;
};

struct PerceptionConfig {
    double visualRange = 15.0;
    double halfAngleRadians = 1.0;
    std::uint64_t memoryTicks = 90;
    std::size_t maxEntries = 32;
    bool operator==(const PerceptionConfig&) const = default;
};

/// Owned fixed-tick state for a single NPC. Persist with its matching config;
/// restore before resuming updates and after any unloaded regions are forgotten.
struct PerceptionMemorySnapshot {
    static constexpr std::uint32_t currentSchema = 1;
    struct Target {
        std::string persistentId;
        Location lastKnown;
        std::uint64_t lastTick = 0;
    };
    struct HeardSound {
        std::uint64_t eventId = 0;
        RegionId region = 0;
        std::uint64_t lastTick = 0;
    };
    std::uint32_t schema = currentSchema;
    PerceptionConfig config;
    std::uint64_t lastTick = 0;
    std::vector<Target> targets;
    std::vector<HeardSound> heardSounds;
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
        Location home, VisibilityQuery& visibility,
        SoundTravelQuery* soundTravel = nullptr,
        SightTravelQuery* sightTravel = nullptr);
    /// Call after a successful region unload, or when retiring all stimuli in
    /// that region. Clears remembered targets and sound deduplication without
    /// retaining or querying transient World entities.
    void forgetRegion(RegionId region) noexcept;
    /// Retire a destroyed or otherwise invalid stable target within a live region.
    /// Sound event receipts remain, so an already-heard event cannot recreate it;
    /// the Host must also stop supplying new stimuli and cancel any active task.
    [[nodiscard]] bool forgetTarget(const std::string& persistentId) noexcept;
    [[nodiscard]] std::size_t rememberedCount() const noexcept { return _memory.size(); }
    /// Copy stable IDs, positions and sound receipts; no live handles are retained.
    [[nodiscard]] PerceptionMemorySnapshot snapshot() const;
    /// Validate and publish an entire snapshot atomically. Invalid or incompatible
    /// data leaves the prior memory and tick untouched.
    Error restore(const PerceptionMemorySnapshot& snapshot);

private:
    struct Remembered { Location lastKnown; std::uint64_t lastTick = 0; };
    struct SeenSound { RegionId region = 0; std::uint64_t lastTick = 0; };
    PerceptionConfig _config;
    std::uint64_t _lastTick = 0;
    std::map<std::string, Remembered> _memory;
    std::map<std::uint64_t, SeenSound> _seenSounds;
};

} // namespace ayt::navigation
