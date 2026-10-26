#include <AYNavigation/PerceptionWorldCommit.h>

#include <nlohmann/json.hpp>

#include <cmath>
#include <type_traits>

namespace ayt::navigation {
namespace {
using Json = nlohmann::json;
constexpr std::size_t kMaxBytes = 2u * 1024u * 1024u;

bool fail(std::string* error, const char* message) {
    if (error) *error = message;
    return false;
}
bool validOwner(const PerceptionOwnerIdentity& value) {
    return !value.persistentId.empty() && value.persistentId.size() <= 128
        && value.generation && value.worldEpoch;
}
bool validDecision(const PursuitDecision& value) {
    if (value.mode != PursuitMode::Return
        && value.mode != PursuitMode::Search
        && value.mode != PursuitMode::Chase) return false;
    if (value.targetPersistentId.size() > 128
        || (value.mode == PursuitMode::Return
            && !value.targetPersistentId.empty())
        || (value.mode != PursuitMode::Return
            && value.targetPersistentId.empty())) return false;
    return value.goal.region && finite(value.goal.position);
}
Json locationJson(Location value) {
    return Json::array({value.region, value.position.x,
        value.position.y, value.position.z});
}
Location readLocation(const Json& value) {
    if (!value.is_array() || value.size() != 4)
        throw std::runtime_error("invalid location");
    return {value.at(0).get<RegionId>(),
        {value.at(1).get<double>(), value.at(2).get<double>(),
         value.at(3).get<double>()}};
}
} // namespace

bool encodePerceptionWorldCandidate(const PerceptionWorldCandidate& candidate,
                                    std::string& out, std::string* error) {
    if (!validOwner(candidate.ledger.owner)
        || !candidate.ledger.lastTick
        || candidate.ledger.lastFingerprint.empty()
        || candidate.ledger.lastFingerprint.size() > 256
        || candidate.memory.lastTick != candidate.ledger.lastTick
        || !validDecision(candidate.decision)
        || !validDecision(candidate.ledger.lastDecision)
        || candidate.decision.mode != candidate.ledger.lastDecision.mode
        || candidate.decision.targetPersistentId
            != candidate.ledger.lastDecision.targetPersistentId
        || !(candidate.decision.goal == candidate.ledger.lastDecision.goal))
        return fail(error, "invalid perception World candidate");
    try {
        std::string memoryJson;
        if (!encodePerceptionMemorySnapshot(candidate.memory, memoryJson, error))
            return false;
        const auto& identity = candidate.ledger.owner;
        Json root{{"format", "AliyatPerceptionWorld"}, {"version", 1},
            {"owner", Json::array({identity.persistentId,
                identity.generation, identity.worldEpoch})},
            {"tick", candidate.ledger.lastTick},
            {"fingerprint", candidate.ledger.lastFingerprint},
            {"decision", Json::array({static_cast<int>(candidate.decision.mode),
                candidate.decision.targetPersistentId,
                locationJson(candidate.decision.goal)})},
            {"memory", std::move(memoryJson)}};
        auto encoded = root.dump();
        if (encoded.size() > kMaxBytes)
            return fail(error, "perception World section exceeds 2 MiB");
        out = std::move(encoded);
        if (error) error->clear();
        return true;
    } catch (...) {
        return fail(error, "cannot encode perception World candidate");
    }
}

bool decodePerceptionWorldCandidate(std::string_view text,
                                    PerceptionWorldCandidate& out,
                                    std::string* error) {
    if (text.empty() || text.size() > kMaxBytes)
        return fail(error, "invalid perception World section size");
    try {
        const auto root = Json::parse(text.begin(), text.end());
        if (!root.is_object() || root.size() != 7
            || root.at("format") != "AliyatPerceptionWorld"
            || root.at("version") != 1)
            return fail(error, "invalid perception World section format");
        const auto& owner = root.at("owner");
        const auto& decision = root.at("decision");
        if (!owner.is_array() || owner.size() != 3
            || !owner.at(1).is_number_unsigned()
            || !owner.at(2).is_number_unsigned()
            || !root.at("tick").is_number_unsigned()
            || !decision.is_array() || decision.size() != 3)
            return fail(error, "invalid perception World section shape");
        PerceptionWorldCandidate candidate;
        candidate.ledger.owner = {owner.at(0).get<std::string>(),
            owner.at(1).get<std::uint64_t>(),
            owner.at(2).get<std::uint64_t>()};
        candidate.ledger.lastTick = root.at("tick").get<std::uint64_t>();
        candidate.ledger.lastFingerprint =
            root.at("fingerprint").get<std::string>();
        candidate.decision = {
            static_cast<PursuitMode>(decision.at(0).get<int>()),
            decision.at(1).get<std::string>(), readLocation(decision.at(2))};
        candidate.ledger.lastDecision = candidate.decision;
        if (!decodePerceptionMemorySnapshot(
            root.at("memory").get<std::string>(), candidate.memory, error))
            return false;
        std::string checked;
        if (!encodePerceptionWorldCandidate(candidate, checked, error))
            return false;
        out = std::move(candidate);
        if (error) error->clear();
        return true;
    } catch (...) {
        return fail(error, "invalid perception World section JSON");
    }
}

PerceptionWorldCommitStatus commitPerceptionWorldTick(
    PerceptionMemory& memory, PerceptionWorldLedger& ledger,
    std::string fingerprint, std::uint64_t tick, Location eye, Vec3 forward,
    std::span<const SightTarget> targets,
    std::span<const SoundStimulus> sounds, Location home,
    VisibilityQuery& visibility, const PerceptionOwnerPreflight& preflight,
    const PerceptionWorldSave& save, PursuitDecision& decision,
    SoundTravelQuery* soundTravel, SightTravelQuery* sightTravel) {
    static_assert(std::is_nothrow_move_assignable_v<PerceptionMemory>);
    static_assert(std::is_nothrow_move_assignable_v<PerceptionWorldLedger>);
    static_assert(std::is_nothrow_move_assignable_v<PursuitDecision>);
    if (!validOwner(ledger.owner) || !preflight || !save
        || fingerprint.empty() || fingerprint.size() > 256 || !tick
        || (!ledger.lastTick && !ledger.lastFingerprint.empty())
        || (ledger.lastTick && (ledger.lastFingerprint.empty()
            || !validDecision(ledger.lastDecision))))
        return PerceptionWorldCommitStatus::Rejected;
    try {
        if (memory.snapshot().lastTick != ledger.lastTick)
            return PerceptionWorldCommitStatus::Rejected;
        if (tick <= ledger.lastTick) {
            if (tick != ledger.lastTick
                || fingerprint != ledger.lastFingerprint
                || !preflight(ledger.owner))
                return PerceptionWorldCommitStatus::Rejected;
            auto remembered = ledger.lastDecision;
            decision = std::move(remembered);
            return PerceptionWorldCommitStatus::AlreadyCommitted;
        }
        auto staged = memory;
        auto updated = staged.update(tick, eye, forward, targets, sounds,
            home, visibility, soundTravel, sightTravel);
        if (!updated || !preflight(ledger.owner))
            return PerceptionWorldCommitStatus::Rejected;
        PerceptionWorldCandidate candidate;
        candidate.ledger = ledger;
        candidate.ledger.lastTick = tick;
        candidate.ledger.lastFingerprint = std::move(fingerprint);
        candidate.ledger.lastDecision = updated.value;
        candidate.memory = staged.snapshot();
        candidate.decision = std::move(updated.value);
        try {
            if (!save(candidate)) return PerceptionWorldCommitStatus::SaveRejected;
        } catch (...) {
            return PerceptionWorldCommitStatus::SaveRejected;
        }
        memory = std::move(staged);
        ledger = std::move(candidate.ledger);
        decision = std::move(candidate.decision);
        return PerceptionWorldCommitStatus::Committed;
    } catch (...) {
        return PerceptionWorldCommitStatus::Rejected;
    }
}

} // namespace ayt::navigation
