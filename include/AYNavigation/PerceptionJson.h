#pragma once

#include <AYNavigation/Perception.h>

#include <string>
#include <string_view>

namespace ayt::navigation {

/// Bounded schema-1 JSON for one NPC's owned perception memory. The game puts
/// this section in the same complete SaveGame slot as its NPC/region state.
/// Both functions validate the complete snapshot and preserve out on failure.
[[nodiscard]] bool encodePerceptionMemorySnapshot(
    const PerceptionMemorySnapshot& snapshot, std::string& out,
    std::string* error = nullptr);
[[nodiscard]] bool decodePerceptionMemorySnapshot(
    std::string_view text, PerceptionMemorySnapshot& out,
    std::string* error = nullptr);

} // namespace ayt::navigation
