#include <AYNavigation/PerceptionWorldCommit.h>
#include "Test.h"

#include <nlohmann/json.hpp>
#include <array>

using namespace ayt::navigation;

namespace {
struct Clear final : VisibilityQuery {
    int calls = 0;
    Result<bool> clear(Location, const SightTarget&) override {
        ++calls;
        return {Error::None, true};
    }
};

void saved_npc_memory_replays_without_resensing_or_losing_sound_receipts() {
    PerceptionMemory memory;
    PerceptionWorldLedger ledger{{"npc/a", 1, 4}};
    Clear clear;
    const Location home{1, {0, 0, 0}};
    const SightTarget target{"target/a", {1, {3, 0, 0}}, {}};
    const SoundStimulus sound{7, "target/a", target.location, 5};
    const std::array targets{target};
    const std::array sounds{sound};
    bool currentOwner = true, allowSave = false;
    int saves = 0;
    std::string saved;
    const PerceptionOwnerPreflight preflight = [&](const auto& identity) {
        return currentOwner && identity == ledger.owner;
    };
    const PerceptionWorldSave save = [&](const auto& candidate) {
        ++saves;
        CHECK(candidate.ledger.owner.persistentId == "npc/a");
        CHECK(candidate.memory.lastTick == 1);
        CHECK(candidate.memory.heardSounds.size() == 1);
        CHECK(candidate.decision.mode == PursuitMode::Chase);
        if (!allowSave) return false;
        return encodePerceptionWorldCandidate(candidate, saved);
    };
    PursuitDecision decision{PursuitMode::Return, "keep", home};
    const auto commit = [&](std::string fingerprint) {
        return commitPerceptionWorldTick(memory, ledger,
            std::move(fingerprint), 1, home, {1, 0, 0}, targets, sounds,
            home, clear, preflight, save, decision);
    };
    CHECK(commit("sensors-v1") == PerceptionWorldCommitStatus::SaveRejected);
    CHECK(memory.snapshot().lastTick == 0 && ledger.lastTick == 0);
    CHECK(decision.targetPersistentId == "keep" && saved.empty());
    currentOwner = false;
    CHECK(commit("sensors-v1") == PerceptionWorldCommitStatus::Rejected);
    CHECK(saves == 1 && memory.snapshot().lastTick == 0);
    currentOwner = true;
    allowSave = true;
    CHECK(commit("sensors-v1") == PerceptionWorldCommitStatus::Committed);
    CHECK(saves == 2 && ledger.lastTick == 1
        && ledger.lastFingerprint == "sensors-v1");
    CHECK(memory.snapshot().heardSounds.size() == 1);
    CHECK(decision.mode == PursuitMode::Chase
        && decision.targetPersistentId == "target/a");
    const int priorQueries = clear.calls;
    decision = {};
    CHECK(commit("sensors-v1") == PerceptionWorldCommitStatus::AlreadyCommitted);
    CHECK(clear.calls == priorQueries && saves == 2
        && decision.mode == PursuitMode::Chase);
    CHECK(commit("changed") == PerceptionWorldCommitStatus::Rejected);
    CHECK(clear.calls == priorQueries && saves == 2);

    PerceptionWorldCandidate restored;
    CHECK(decodePerceptionWorldCandidate(saved, restored));
    CHECK(restored.ledger.owner == ledger.owner
        && restored.memory.lastTick == 1
        && restored.memory.heardSounds[0].eventId == 7);
    PerceptionMemory restarted(restored.memory.config);
    CHECK(restarted.restore(restored.memory) == Error::None);
    auto restartedLedger = restored.ledger;
    PursuitDecision restartedDecision;
    const auto replay = commitPerceptionWorldTick(restarted, restartedLedger,
        "sensors-v1", 1, home, {1, 0, 0}, targets, sounds, home, clear,
        [](const auto&) { return true; },
        [&](const auto&) { ++saves; return false; }, restartedDecision);
    CHECK(replay == PerceptionWorldCommitStatus::AlreadyCommitted
        && restartedDecision.targetPersistentId == "target/a"
        && saves == 2 && clear.calls == priorQueries);
    const auto before = restarted.snapshot();
    const auto rejected = commitPerceptionWorldTick(restarted, restartedLedger,
        "sensors-v2", 2, home, {1, 0, 0}, targets, sounds, home, clear,
        [](const auto&) { return false; },
        [](const auto&) { return true; }, restartedDecision);
    CHECK(rejected == PerceptionWorldCommitStatus::Rejected
        && restarted.snapshot().lastTick == before.lastTick);
    PerceptionWorldCandidate unchanged = restored;
    auto forged = nlohmann::json::parse(saved);
    forged["decision"][0] = 99;
    CHECK(!decodePerceptionWorldCandidate(forged.dump(), unchanged));
    CHECK(unchanged.ledger.lastTick == 1);
    forged = nlohmann::json::parse(saved);
    forged["owner"][1] = -1;
    CHECK(!decodePerceptionWorldCandidate(forged.dump(), unchanged));
    CHECK(unchanged.ledger.owner.generation == 1);
}
}

TEST_MAIN(
    RUN(saved_npc_memory_replays_without_resensing_or_losing_sound_receipts);
)
