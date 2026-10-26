#pragma once

#include <AYNavigation/PerceptionJson.h>

#include <functional>

namespace ayt::navigation {

/// Stable NPC identity captured with the same World incarnation as memory.
/// The game maps this value to its current Entity and checks it in preflight.
struct PerceptionOwnerIdentity {
    std::string persistentId;
    std::uint64_t generation = 0;
    std::uint64_t worldEpoch = 0;
    bool operator==(const PerceptionOwnerIdentity&) const = default;
};
struct PerceptionWorldLedger {
    PerceptionOwnerIdentity owner;
    std::uint64_t lastTick = 0;
    std::string lastFingerprint;
    PursuitDecision lastDecision;
};
struct PerceptionWorldCandidate {
    PerceptionWorldLedger ledger;
    PerceptionMemorySnapshot memory;
    PursuitDecision decision;
};
enum class PerceptionWorldCommitStatus {
    Committed, AlreadyCommitted, Rejected, SaveRejected
};

using PerceptionOwnerPreflight = std::function<bool(
    const PerceptionOwnerIdentity&)>;
using PerceptionWorldSave = std::function<bool(
    const PerceptionWorldCandidate&)>;

/// Encode a bounded version-1 section containing identity, tick receipt and
/// PerceptionMemory. The game saves it with Entity and World state in one slot.
[[nodiscard]] bool encodePerceptionWorldCandidate(
    const PerceptionWorldCandidate& candidate,
    std::string& out, std::string* error = nullptr);
/// Decode to a temporary value and validate the memory against its config.
[[nodiscard]] bool decodePerceptionWorldCandidate(
    std::string_view text, PerceptionWorldCandidate& out,
    std::string* error = nullptr);

/** Update a private memory copy, check live NPC/World identity, save the
 * complete candidate, then publish memory and receipt without allocation.
 * The save callback is failure atomic and does not reenter owners. Decisions
 * are returned only after publication; the game submits Behavior commands
 * afterward under its own priority/interrupt policy. A matching tick and
 * fingerprint replay skips queries and save; a changed fingerprint conflicts.
 */
[[nodiscard]] PerceptionWorldCommitStatus commitPerceptionWorldTick(
    PerceptionMemory& memory, PerceptionWorldLedger& ledger,
    std::string fingerprint, std::uint64_t tick, Location eye, Vec3 forward,
    std::span<const SightTarget> targets,
    std::span<const SoundStimulus> sounds, Location home,
    VisibilityQuery& visibility, const PerceptionOwnerPreflight& preflight,
    const PerceptionWorldSave& save, PursuitDecision& decision,
    SoundTravelQuery* soundTravel = nullptr,
    SightTravelQuery* sightTravel = nullptr);

} // namespace ayt::navigation
