#pragma once

#include <AYNavigation/StaticBake.h>

#include <string_view>
#include <span>

namespace ayt::entity { class World; }

namespace ayt::navigation {

/// Optional Scene-World adapter. Call only after Scene/RegionStream has attached
/// all colliders at a quiescent boundary. The floor is one static Box entity;
/// other static non-trigger Box colliders in that region become blockers.
/// Unsupported static shapes fail the whole extraction instead of opening an
/// unsafe path. The caller increments contentVersion after any static change.
Result<BakeRegion> extractWorldBakeRegion(const entity::World& world,
    RegionId region, std::uint64_t contentVersion,
    std::string_view floorPersistentId);

/// Extract an authored primary floor and up to sixteen additional floors.
/// IDs must be unique and each name exactly one static Box entity. Other
/// static solids remain blockers; no floor or blocker is silently skipped.
Result<BakeRegion> extractWorldBakeRegion(const entity::World& world,
    RegionId region, std::uint64_t contentVersion,
    std::span<const std::string_view> floorPersistentIds);

/// Publishes World geometry into an existing bake. On region unload call
/// unloadRegion after WorldRegionStream::unload and before issuing new routes.
class WorldBakeAdapter {
public:
    WorldBakeAdapter(const entity::World& world, StaticBakeWorld& bake)
        : _world(world), _bake(bake) {}
    Result<std::shared_ptr<const GraphSpace>> publishRegion(
        RegionId region, std::uint64_t contentVersion,
        std::string_view floorPersistentId);
    Result<std::shared_ptr<const GraphSpace>> publishRegion(
        RegionId region, std::uint64_t contentVersion,
        std::span<const std::string_view> floorPersistentIds);
    Result<std::shared_ptr<const GraphSpace>> unloadRegion(RegionId region)
    { return _bake.unloadRegion(region); }
private:
    const entity::World& _world;
    StaticBakeWorld& _bake;
};

} // namespace ayt::navigation
