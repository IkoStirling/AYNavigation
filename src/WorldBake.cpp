#include <AYNavigation/WorldBake.h>

#include <AYEntity.h>
#include <AYEntity/World.h>
#include <AYEntity/components/ColliderComponent.h>
#include <AYEntity/components/RigidBodyComponent.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

namespace ayt::navigation {
namespace {
bool finite(ayt::math::FVector3 value)
{
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z);
}

Result<BakeBox> boxOf(const entity::Entity& entity,
                      const entity::ColliderShapeSpec& shape,
                      bool floor, std::optional<BakeRamp>& ramp)
{
    if (shape.shape != entity::ColliderShapeType::Box || shape.isTrigger)
        return {Error::Unsupported};
    const auto half = shape.halfExtents;
    if (!finite(half) || !finite(shape.offset)
        || half.x <= 0 || half.y <= 0 || half.z <= 0)
        return {Error::Invalid};
    const auto matrix = entity.worldMatrix(1.0f);
    BakeBox result{
        {std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity()},
        {-std::numeric_limits<double>::infinity(),
         -std::numeric_limits<double>::infinity(),
         -std::numeric_limits<double>::infinity()}};
    std::array<ayt::math::FVector3, 4> top{};
    std::size_t topCount = 0;
    double topSum = 0.0, bottomSum = 0.0;
    for (int x : {-1, 1}) for (int y : {-1, 1}) for (int z : {-1, 1}) {
        const ayt::math::FVector3 local{
            shape.offset.x + x * half.x,
            shape.offset.y + y * half.y,
            shape.offset.z + z * half.z};
        const auto point = matrix.transformPoint(local);
        if (!finite(point)) return {Error::Invalid};
        result.minimum.x = std::min(result.minimum.x, double(point.x));
        result.minimum.y = std::min(result.minimum.y, double(point.y));
        result.minimum.z = std::min(result.minimum.z, double(point.z));
        result.maximum.x = std::max(result.maximum.x, double(point.x));
        result.maximum.y = std::max(result.maximum.y, double(point.y));
        result.maximum.z = std::max(result.maximum.z, double(point.z));
        if (floor && y == 1) {
            top[topCount++] = point;
            topSum += point.y;
        }
        if (floor && y == -1) bottomSum += point.y;
    }
    if (floor) {
        if (topCount != 4 || topSum <= bottomSum) return {Error::Unsupported};
        const auto [lowestTop, highestTop] = std::minmax_element(top.begin(), top.end(),
            [](const auto& a, const auto& b) { return a.y < b.y; });
        if (highestTop->y - lowestTop->y <= 1e-4)
            return {Error::None, result};
        BakeBox topFootprint = result;
        topFootprint.minimum.x = topFootprint.minimum.z =
            std::numeric_limits<double>::infinity();
        topFootprint.maximum.x = topFootprint.maximum.z =
            -std::numeric_limits<double>::infinity();
        for (const auto& point : top) {
            topFootprint.minimum.x = std::min(topFootprint.minimum.x, double(point.x));
            topFootprint.maximum.x = std::max(topFootprint.maximum.x, double(point.x));
            topFootprint.minimum.z = std::min(topFootprint.minimum.z, double(point.z));
            topFootprint.maximum.z = std::max(topFootprint.maximum.z, double(point.z));
        }
        std::array<double, 4> heights{};
        std::array<bool, 4> seen{};
        for (const auto& point : top) {
            const bool lowX = std::abs(double(point.x) - topFootprint.minimum.x) <= 1e-3;
            const bool highX = std::abs(double(point.x) - topFootprint.maximum.x) <= 1e-3;
            const bool lowZ = std::abs(double(point.z) - topFootprint.minimum.z) <= 1e-3;
            const bool highZ = std::abs(double(point.z) - topFootprint.maximum.z) <= 1e-3;
            if (lowX == highX || lowZ == highZ) return {Error::Unsupported};
            const std::size_t corner = (highX ? 1u : 0u) + (highZ ? 2u : 0u);
            if (seen[corner]) return {Error::Unsupported};
            seen[corner] = true;
            heights[corner] = point.y;
        }
        if (std::any_of(seen.begin(), seen.end(), [](bool value) { return !value; }))
            return {Error::Unsupported};
        ramp = BakeRamp{topFootprint, heights[0], heights[1], heights[2], heights[3]};
    }
    return {Error::None, result};
}
} // namespace

Result<BakeRegion> extractWorldBakeRegion(const entity::World& world,
    RegionId region, std::uint64_t contentVersion,
    std::string_view floorPersistentId)
{
    return extractWorldBakeRegion(world, region, contentVersion,
        std::span<const std::string_view>(&floorPersistentId, 1));
}

Result<BakeRegion> extractWorldBakeRegion(const entity::World& world,
    RegionId region, std::uint64_t contentVersion,
    std::span<const std::string_view> floorPersistentIds)
{
    if (!region || region > std::numeric_limits<entity::SpaceRegionId>::max()
        || !contentVersion || floorPersistentIds.empty()
        || floorPersistentIds.size() > 17
        || !world.connectedSpaces().containsRegion(
            static_cast<entity::SpaceRegionId>(region)))
        return {Error::Invalid};
    for (std::size_t i = 0; i < floorPersistentIds.size(); ++i)
        if (floorPersistentIds[i].empty()
            || std::find(floorPersistentIds.begin(),
                         floorPersistentIds.begin() + i, floorPersistentIds[i])
                != floorPersistentIds.begin() + i)
            return {Error::Invalid};
    BakeRegion result;
    result.id = region;
    result.contentVersion = contentVersion;
    std::vector<bool> foundFloor(floorPersistentIds.size());
    std::vector<BakeBox> floorBoxes(floorPersistentIds.size());
    std::vector<std::optional<BakeRamp>> floorRamps(floorPersistentIds.size());
    for (auto* entity : world.getAllEntities()) {
        if (!entity || !entity->isSpatial()
            || entity->effectiveSpaceRegion() != region) continue;
        auto* rigid = entity->getComponent<entity::RigidBodyComponent>();
        auto* collider = entity->getComponent<entity::ColliderComponent>();
        if (!rigid || !collider || !rigid->isStatic()
            || rigid->getPhysicsDimension()
                != entity::RigidBodyComponent::PhysicsDimension::ThreeD)
            continue;
        const auto floorIt = std::find(floorPersistentIds.begin(),
            floorPersistentIds.end(), entity->persistentId());
        const bool floor = floorIt != floorPersistentIds.end();
        const auto floorIndex = static_cast<std::size_t>(
            floorIt - floorPersistentIds.begin());
        std::size_t solids = 0;
        for (const auto& shape : collider->shapes) {
            if (shape.isTrigger) continue;
            ++solids;
            std::optional<BakeRamp> ramp;
            auto box = boxOf(*entity, shape, floor, ramp);
            if (!box) return {box.error};
            if (floor) {
                floorBoxes[floorIndex] = box.value;
                floorRamps[floorIndex] = std::move(ramp);
            } else result.blockers.push_back(box.value);
        }
        if (floor) {
            if (solids != 1) return {Error::Unsupported};
            if (foundFloor[floorIndex]) return {Error::Invalid};
            foundFloor[floorIndex] = true;
        }
    }
    if (std::find(foundFloor.begin(), foundFloor.end(), false)
        != foundFloor.end()) return {Error::Missing};
    if (floorRamps.front()) return {Error::Unsupported};
    result.floor = floorBoxes.front();
    for (std::size_t i = 1; i < floorBoxes.size(); ++i) {
        if (floorRamps[i]) result.ramps.push_back(*floorRamps[i]);
        else result.additionalFloors.push_back(floorBoxes[i]);
    }
    return {Error::None, std::move(result)};
}

Result<std::shared_ptr<const GraphSpace>> WorldBakeAdapter::publishRegion(
    RegionId region, std::uint64_t contentVersion,
    std::string_view floorPersistentId)
{
    auto extracted = extractWorldBakeRegion(
        _world, region, contentVersion, floorPersistentId);
    if (!extracted) return {extracted.error};
    return _bake.publishRegion(std::move(extracted.value));
}

Result<std::shared_ptr<const GraphSpace>> WorldBakeAdapter::publishRegion(
    RegionId region, std::uint64_t contentVersion,
    std::span<const std::string_view> floorPersistentIds)
{
    auto extracted = extractWorldBakeRegion(
        _world, region, contentVersion, floorPersistentIds);
    if (!extracted) return {extracted.error};
    return _bake.publishRegion(std::move(extracted.value));
}

} // namespace ayt::navigation
