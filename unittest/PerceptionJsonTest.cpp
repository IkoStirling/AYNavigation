#include <AYNavigation/PerceptionJson.h>
#include "Test.h"

using namespace ayt::navigation;

namespace {
void round_trip_and_reject_changed_schema_without_overwriting_output() {
    PerceptionMemorySnapshot snapshot;
    snapshot.lastTick = 100;
    snapshot.targets.push_back({"target/b", {2, {3, 4, 5}}, 99});
    snapshot.targets.push_back({"target/a", {1, {1, 2, 3}}, 98});
    snapshot.heardSounds.push_back({42, 2, 97});
    std::string document;
    CHECK(encodePerceptionMemorySnapshot(snapshot, document));
    PerceptionMemorySnapshot restored;
    CHECK(decodePerceptionMemorySnapshot(document, restored));
    CHECK(restored.targets.size() == 2);
    CHECK(restored.targets[0].persistentId == "target/a");
    CHECK(restored.targets[1].lastKnown.position.z == 5);
    CHECK(restored.heardSounds[0].eventId == 42);
    const auto prior = restored;
    const auto schemaPos = document.find("\"schema\":1");
    CHECK(schemaPos != std::string::npos);
    document.replace(schemaPos, 10, "\"schema\":2");
    CHECK(!decodePerceptionMemorySnapshot(document, restored));
    CHECK(restored.targets.size() == prior.targets.size());
    CHECK(restored.targets[0].persistentId == prior.targets[0].persistentId);
}

void invalid_payload_and_limit_preserve_output() {
    PerceptionMemorySnapshot current;
    current.lastTick = 7;
    CHECK(!decodePerceptionMemorySnapshot("{", current));
    CHECK(!decodePerceptionMemorySnapshot(std::string(1024 * 1024 + 1, 'x'),
        current));
    CHECK(current.lastTick == 7);
    PerceptionMemorySnapshot invalid;
    invalid.lastTick = 2;
    invalid.targets.push_back({"stale", {1, {0, 0, 0}}, 1});
    invalid.targets.push_back({"stale", {1, {0, 0, 0}}, 2});
    std::string output = "keep";
    CHECK(!encodePerceptionMemorySnapshot(invalid, output));
    CHECK(output == "keep");
}
}

TEST_MAIN(
    RUN(round_trip_and_reject_changed_schema_without_overwriting_output);
    RUN(invalid_payload_and_limit_preserve_output);
)
