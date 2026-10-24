#include <AYNavigation/WorldBake.h>

#include <AYEntity.h>
#include <AYEntity/World.h>
#include <AYEntity/components/ColliderComponent.h>
#include <AYEntity/components/RigidBodyComponent.h>

#include <algorithm>
#include <cmath>
#include <limits>
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
                      bool floor)
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
    double topY = 0.0;
    bool haveTop = false;
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
            if (haveTop && std::abs(double(point.y) - topY) > 1e-4)
                return {Error::Unsupported};
            topY = point.y;
            haveTop = true;
        }
    }
    if (floor && std::abs(result.maximum.y - topY) > 1e-4)
        return {Error::Unsupported};
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
    result.additionalFloors.resize(floorPersistentIds.size() - 1);
    std::vector<bool> foundFloor(floorPersistentIds.size());
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
            auto box = boxOf(*entity, shape, floor);
            if (!box) return {box.error};
            if (floor && floorIndex == 0) result.floor = box.value;
            else if (floor) result.additionalFloors[floorIndex - 1] = box.value;
            else result.blockers.push_back(box.value);
        }
        if (floor) {
            if (solids != 1) return {Error::Unsupported};
            if (foundFloor[floorIndex]) return {Error::Invalid};
            foundFloor[floorIndex] = true;
        }
    }
    if (std::find(foundFloor.begin(), foundFloor.end(), false)
        != foundFloor.end()) return {Error::Missing};
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
