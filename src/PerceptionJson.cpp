#include <AYNavigation/PerceptionJson.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace ayt::navigation {
namespace {
using Json = nlohmann::json;
constexpr std::size_t maxDocumentBytes = 1024 * 1024;

bool fail(std::string* error, const char* message) {
    if (error) *error = message;
    return false;
}
std::uint64_t unsignedField(const Json& value, const char* name) {
    const auto& field = value.at(name);
    if (!field.is_number_unsigned()) throw std::runtime_error("integer field is invalid");
    return field.get<std::uint64_t>();
}
double finiteField(const Json& value, const char* name) {
    const auto& field = value.at(name);
    if (!field.is_number()) throw std::runtime_error("coordinate is invalid");
    const auto result = field.get<double>();
    if (!std::isfinite(result)) throw std::runtime_error("coordinate is not finite");
    return result;
}
Location locationOf(const Json& value) {
    if (!value.is_object()) throw std::runtime_error("location is invalid");
    return {unsignedField(value, "region"),
        {finiteField(value, "x"), finiteField(value, "y"),
         finiteField(value, "z")}};
}
Json locationJson(Location value) {
    return {{"region", value.region}, {"x", value.position.x},
        {"y", value.position.y}, {"z", value.position.z}};
}
}

bool encodePerceptionMemorySnapshot(const PerceptionMemorySnapshot& snapshot,
    std::string& out, std::string* error) {
    try {
        PerceptionMemory check(snapshot.config);
        if (check.restore(snapshot) != Error::None)
            return fail(error, "Perception snapshot is invalid.");
        const auto canonical = check.snapshot();
        Json targets = Json::array(), sounds = Json::array();
        for (const auto& target : canonical.targets)
            targets.push_back({{"id", target.persistentId},
                {"location", locationJson(target.lastKnown)},
                {"tick", target.lastTick}});
        for (const auto& sound : canonical.heardSounds)
            sounds.push_back({{"eventId", sound.eventId},
                {"region", sound.region}, {"tick", sound.lastTick}});
        const Json document{{"format", "AliyatPerceptionMemory"},
            {"schema", canonical.schema},
            {"config", {{"visualRange", canonical.config.visualRange},
                {"halfAngleRadians", canonical.config.halfAngleRadians},
                {"memoryTicks", canonical.config.memoryTicks},
                {"maxEntries", canonical.config.maxEntries}}},
            {"lastTick", canonical.lastTick},
            {"targets", std::move(targets)},
            {"heardSounds", std::move(sounds)}};
        auto encoded = document.dump();
        if (encoded.size() > maxDocumentBytes)
            return fail(error, "Perception document exceeds 1 MiB.");
        out = std::move(encoded);
        return true;
    } catch (...) {
        return fail(error, "Perception snapshot encoding failed.");
    }
}

bool decodePerceptionMemorySnapshot(std::string_view text,
    PerceptionMemorySnapshot& out, std::string* error) {
    if (text.empty() || text.size() > maxDocumentBytes)
        return fail(error, "Perception document size is invalid.");
    try {
        const auto document = Json::parse(text);
        if (!document.is_object()
            || document.at("format") != "AliyatPerceptionMemory"
            || unsignedField(document, "schema") != 1)
            return fail(error, "Perception document format is invalid.");
        PerceptionMemorySnapshot candidate;
        const auto& config = document.at("config");
        candidate.config.visualRange = finiteField(config, "visualRange");
        candidate.config.halfAngleRadians = finiteField(config, "halfAngleRadians");
        candidate.config.memoryTicks = unsignedField(config, "memoryTicks");
        const auto entries = unsignedField(config, "maxEntries");
        if (entries > std::numeric_limits<std::size_t>::max())
            return fail(error, "Perception entry limit is invalid.");
        candidate.config.maxEntries = static_cast<std::size_t>(entries);
        candidate.lastTick = unsignedField(document, "lastTick");
        const auto& targets = document.at("targets");
        const auto& sounds = document.at("heardSounds");
        if (!targets.is_array() || !sounds.is_array()
            || targets.size() > 1024 || sounds.size() > 4096)
            return fail(error, "Perception entry count is invalid.");
        for (const auto& target : targets)
            candidate.targets.push_back({target.at("id").get<std::string>(),
                locationOf(target.at("location")),
                unsignedField(target, "tick")});
        for (const auto& sound : sounds)
            candidate.heardSounds.push_back({unsignedField(sound, "eventId"),
                unsignedField(sound, "region"),
                unsignedField(sound, "tick")});
        PerceptionMemory check(candidate.config);
        if (check.restore(candidate) != Error::None)
            return fail(error, "Perception snapshot payload is invalid.");
        out = check.snapshot();
        return true;
    } catch (...) {
        return fail(error, "Perception document decoding failed.");
    }
}

} // namespace ayt::navigation
