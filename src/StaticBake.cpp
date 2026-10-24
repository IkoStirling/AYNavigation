#include <AYNavigation/StaticBake.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace ayt::navigation {
namespace {
bool validBox(const BakeBox& box, bool floor)
{
    return finite(box.minimum) && finite(box.maximum)
        && std::abs(box.minimum.x) <= 1e12 && std::abs(box.minimum.y) <= 1e12
        && std::abs(box.minimum.z) <= 1e12 && std::abs(box.maximum.x) <= 1e12
        && std::abs(box.maximum.y) <= 1e12 && std::abs(box.maximum.z) <= 1e12
        && box.minimum.x < box.maximum.x
        && box.minimum.z < box.maximum.z
        && (floor ? box.minimum.y <= box.maximum.y
                  : box.minimum.y < box.maximum.y);
}

bool overlapXZ(const BakeBox& a, const BakeBox& b) noexcept
{
    return a.minimum.x < b.maximum.x && a.maximum.x > b.minimum.x
        && a.minimum.z < b.maximum.z && a.maximum.z > b.minimum.z;
}

bool validRamp(const BakeRamp& ramp)
{
    if (!validBox(ramp.footprint, true)) return false;
    const double heights[]{ramp.y00, ramp.y10, ramp.y01, ramp.y11};
    for (double y : heights)
        if (!std::isfinite(y) || y < ramp.footprint.minimum.y
            || y > ramp.footprint.maximum.y) return false;
    return std::abs((ramp.y00 + ramp.y11) - (ramp.y10 + ramp.y01))
        <= 1e-4;
}

double surfaceY(const BakeBox& floor, const BakeRamp* ramp,
                double x, double z) noexcept
{
    if (!ramp) return floor.maximum.y;
    const double u = (x - floor.minimum.x) /
        (floor.maximum.x - floor.minimum.x);
    const double v = (z - floor.minimum.z) /
        (floor.maximum.z - floor.minimum.z);
    return (1.0 - u) * (1.0 - v) * ramp->y00
        + u * (1.0 - v) * ramp->y10
        + (1.0 - u) * v * ramp->y01 + u * v * ramp->y11;
}

bool intersects(double minX, double maxX, double minZ, double maxZ,
                const BakeBox& blocker, double minFloorY,
                double maxFloorY, double height)
{
    if (blocker.maximum.y <= minFloorY
        || blocker.minimum.y >= maxFloorY + height)
        return false;
    return blocker.minimum.x < maxX && blocker.maximum.x > minX
        && blocker.minimum.z < maxZ && blocker.maximum.z > minZ;
}

bool clearCorridor(const BakeRegion& region, const BakeBox& floor, Vec3 a, Vec3 b,
                   double width, double height)
{
    const double half = width * 0.5;
    const double minX = std::min(a.x, b.x) - half;
    const double maxX = std::max(a.x, b.x) + half;
    const double minZ = std::min(a.z, b.z) - half;
    const double maxZ = std::max(a.z, b.z) + half;
    if (minX < floor.minimum.x || maxX > floor.maximum.x
        || minZ < floor.minimum.z || maxZ > floor.maximum.z)
        return false;
    for (const auto& blocker : region.blockers)
        if (intersects(minX, maxX, minZ, maxZ, blocker,
                       std::min(a.y, b.y), std::max(a.y, b.y), height))
            return false;
    return true;
}

struct RegionGrid {
    std::size_t nx = 0, nz = 0;
    std::vector<NodeId> nodes;
};
} // namespace

StaticBakeWorld::StaticBakeWorld(std::uint64_t spaceId, BakeConfig config)
    : _spaceId(spaceId), _config(config)
{
    _report.stamp.space = spaceId;
}

Result<std::shared_ptr<const GraphSpace>> StaticBakeWorld::publishRegion(
    BakeRegion region)
{
    if (!region.id || !region.contentVersion || !validBox(region.floor, true)
        || region.additionalFloors.size() + region.ramps.size() > 16)
        return {Error::Invalid};
    for (const auto& floor : region.additionalFloors)
        if (!validBox(floor, true)) return {Error::Invalid};
    for (std::size_t i = 0; i <= region.additionalFloors.size(); ++i) {
        const auto& a = i == 0 ? region.floor : region.additionalFloors[i - 1];
        for (std::size_t j = i + 1; j <= region.additionalFloors.size(); ++j) {
            const auto& b = j == 0 ? region.floor : region.additionalFloors[j - 1];
            if (std::abs(a.maximum.y - b.maximum.y) < 0.001
                && a.minimum.x < b.maximum.x && a.maximum.x > b.minimum.x
                && a.minimum.z < b.maximum.z && a.maximum.z > b.minimum.z)
                return {Error::Invalid};
        }
    }
    for (const auto& blocker : region.blockers)
        if (!validBox(blocker, false)) return {Error::Invalid};
    for (std::size_t i = 0; i < region.ramps.size(); ++i) {
        const auto& ramp = region.ramps[i];
        if (!validRamp(ramp) || overlapXZ(region.floor, ramp.footprint))
            return {Error::Invalid};
        for (const auto& flat : region.additionalFloors)
            if (overlapXZ(flat, ramp.footprint)) return {Error::Invalid};
        for (std::size_t j = 0; j < i; ++j)
            if (overlapXZ(region.ramps[j].footprint, ramp.footprint))
                return {Error::Invalid};
    }
    if (const auto old = _regions.find(region.id); old != _regions.end()) {
        if (region.contentVersion < old->second.contentVersion)
            return {Error::Stale};
        if (region.contentVersion == old->second.contentVersion)
            return old->second == region
                ? Result<std::shared_ptr<const GraphSpace>>{Error::None, _graph}
                : Result<std::shared_ptr<const GraphSpace>>{Error::Stale};
    }
    auto staged = _regions;
    staged[region.id] = std::move(region);
    return rebuild(std::move(staged), _connections);
}

Result<std::shared_ptr<const GraphSpace>> StaticBakeWorld::unloadRegion(RegionId id)
{
    if (!_regions.contains(id)) return {Error::Missing};
    auto staged = _regions;
    staged.erase(id);
    return rebuild(std::move(staged), _connections);
}

Result<std::shared_ptr<const GraphSpace>> StaticBakeWorld::setConnections(
    std::vector<BakeConnection> connections)
{
    for (const auto& connection : connections) {
        if (!connection.from || !connection.to
            || !finite(connection.departure) || !finite(connection.arrival)
            || (connection.from == connection.to
                && length(connection.departure - connection.arrival) < 0.001)
            || !connection.traversal
            || bool(connection.resource) != bool(connection.capacity))
            return {Error::Invalid};
    }
    if (connections == _connections) return {Error::None, _graph};
    return rebuild(_regions, std::move(connections));
}

Result<std::shared_ptr<const GraphSpace>> StaticBakeWorld::rebuild(
    std::map<RegionId, BakeRegion> regions,
    std::vector<BakeConnection> connections)
{
    if (!_spaceId || !std::isfinite(_config.cellSize)
        || !std::isfinite(_config.corridorWidth)
        || !std::isfinite(_config.agentHeight)
        || !std::isfinite(_config.maxRisePerHorizontal)
        || _config.cellSize <= 0.0 || _config.cellSize > 1e6
        || _config.corridorWidth <= 0.0
        || _config.corridorWidth > _config.cellSize
        || _config.agentHeight <= 0.0 || _config.agentHeight > 1e6
        || _config.maxRisePerHorizontal <= 0.0
        || _config.maxRisePerHorizontal > 10.0
        || !_config.maxCells || !_config.maxLinks
        || _report.stamp.version == std::numeric_limits<std::uint64_t>::max())
        return {Error::Invalid};

    BakeReport report;
    report.stamp = {_spaceId, _report.stamp.version + 1};
    report.regions = regions.size();
    if (regions.empty()) {
        _regions = std::move(regions);
        _connections = std::move(connections);
        _graph.reset();
        _report = report;
        return {Error::None, _graph};
    }
    GraphDefinition graph;
    graph.stamp = report.stamp;
    std::map<std::pair<RegionId, std::size_t>, RegionGrid> grids;
    NodeId nextNode = 1;
    LinkId nextLink = 1;
    std::size_t cellCount = 0;
    const auto maxCells = std::min<std::size_t>(_config.maxCells, 10000);
    const auto cost = static_cast<std::uint64_t>(std::ceil(_config.cellSize * 1000.0));
    const auto surfaceCount = [](const BakeRegion& region) {
        return 1 + region.additionalFloors.size() + region.ramps.size();
    };
    const auto floorAt = [](const BakeRegion& region, std::size_t index)
        -> const BakeBox& {
        if (index == 0) return region.floor;
        if (index <= region.additionalFloors.size())
            return region.additionalFloors[index - 1];
        return region.ramps[index - 1 - region.additionalFloors.size()].footprint;
    };
    const auto rampAt = [](const BakeRegion& region, std::size_t index)
        -> const BakeRamp* {
        if (index <= region.additionalFloors.size()) return nullptr;
        return &region.ramps[index - 1 - region.additionalFloors.size()];
    };
    for (const auto& [regionId, region] : regions) {
      for (std::size_t floorIndex = 0; floorIndex < surfaceCount(region); ++floorIndex) {
        const auto& floor = floorAt(region, floorIndex);
        const auto* ramp = rampAt(region, floorIndex);
        const double dx = floor.maximum.x - floor.minimum.x;
        const double dz = floor.maximum.z - floor.minimum.z;
        if (!std::isfinite(dx) || !std::isfinite(dz)
            || dx < _config.cellSize || dz < _config.cellSize)
            return {Error::Invalid};
        const double cellsX = std::floor(dx / _config.cellSize);
        const double cellsZ = std::floor(dz / _config.cellSize);
        if (!std::isfinite(cellsX) || !std::isfinite(cellsZ)
            || cellsX > double(maxCells)
            || cellsZ > double(maxCells)) return {Error::Budget};
        const auto nx = static_cast<std::size_t>(cellsX);
        const auto nz = static_cast<std::size_t>(cellsZ);
        if (!nx || !nz || nx > maxCells / nz
            || nx * nz > maxCells - cellCount)
            return {Error::Budget};
        cellCount += nx * nz;
        RegionGrid grid{nx, nz, std::vector<NodeId>(nx * nz, 0)};
        for (std::size_t z = 0; z < nz; ++z) {
            for (std::size_t x = 0; x < nx; ++x) {
                const Vec3 center{
                    floor.minimum.x + (double(x) + 0.5) * _config.cellSize,
                    surfaceY(floor, ramp,
                        floor.minimum.x + (double(x) + 0.5) * _config.cellSize,
                        floor.minimum.z + (double(z) + 0.5) * _config.cellSize),
                    floor.minimum.z + (double(z) + 0.5) * _config.cellSize};
                if (!clearCorridor(region, floor, center, center,
                                   _config.corridorWidth, _config.agentHeight)) {
                    ++report.blockedCells;
                    continue;
                }
                grid.nodes[z * nx + x] = nextNode;
                graph.nodes.push_back({nextNode++, {regionId, center}});
            }
        }
        grids.emplace(std::pair{regionId, floorIndex}, std::move(grid));
      }
    }
    if (graph.nodes.empty()) return {Error::NoPath};
    for (const auto& [regionId, region] : regions) {
      for (std::size_t floorIndex = 0; floorIndex < surfaceCount(region); ++floorIndex) {
        const auto& floor = floorAt(region, floorIndex);
        const auto& grid = grids.at({regionId, floorIndex});
        for (std::size_t z = 0; z < grid.nz; ++z) {
            for (std::size_t x = 0; x < grid.nx; ++x) {
                const auto id = grid.nodes[z * grid.nx + x];
                if (!id) continue;
                for (const auto [nx, nz] : {
                         std::pair{x + 1, z}, std::pair{x, z + 1}}) {
                    if (nx >= grid.nx || nz >= grid.nz) continue;
                    const auto neighbor = grid.nodes[nz * grid.nx + nx];
                    if (!neighbor) continue;
                    const auto a = graph.nodes[id - 1].location.position;
                    const auto b = graph.nodes[neighbor - 1].location.position;
                    const double horizontal = std::hypot(a.x - b.x, a.z - b.z);
                    if (std::abs(a.y - b.y)
                        > _config.maxRisePerHorizontal * horizontal + 1e-9)
                        continue;
                    if (!clearCorridor(region, floor, a, b,
                                       _config.corridorWidth, _config.agentHeight))
                        continue;
                    if (graph.links.size() + 2 > std::min<std::size_t>(
                            _config.maxLinks, 50000)) return {Error::Budget};
                    const auto linkCost = static_cast<std::uint64_t>(
                        std::ceil(length(b - a) * 1000.0));
                    graph.links.push_back({nextLink++, id, neighbor,
                                           _config.corridorWidth, _config.agentHeight,
                                           linkCost});
                    graph.links.push_back({nextLink++, neighbor, id,
                                           _config.corridorWidth, _config.agentHeight,
                                           linkCost});
                }
            }
        }
      }
    }
    auto nearest = [&](RegionId regionId, Vec3 target) -> NodeId {
        NodeId bestId = 0;
        double best = _config.cellSize;
        for (const auto& node : graph.nodes) {
            if (node.location.region != regionId) continue;
            const double distance = length(node.location.position - target);
            if (distance <= best && (!bestId || distance < best)) {
                best = distance;
                bestId = node.id;
            }
        }
        return bestId;
    };
    for (const auto& connection : connections) {
        if (!regions.contains(connection.from) || !regions.contains(connection.to)) {
            ++report.disabledConnections;
            continue;
        }
        const auto from = nearest(connection.from, connection.departure);
        const auto to = nearest(connection.to, connection.arrival);
        if (!from || !to || from == to) return {Error::NoPath};
        if (graph.links.size() + 1 > std::min<std::size_t>(
                _config.maxLinks, 50000)) return {Error::Budget};
        graph.links.push_back({nextLink++, from, to, _config.corridorWidth,
                               _config.agentHeight, cost, 0, connection.resource,
                               connection.capacity, connection.traversal});
    }
    auto compiled = GraphSpace::compile(std::move(graph));
    if (!compiled) return compiled;
    report.nodes = compiled.value->definition().nodes.size();
    report.links = compiled.value->definition().links.size();
    _regions = std::move(regions);
    _connections = std::move(connections);
    _graph = compiled.value;
    _report = report;
    return {Error::None, _graph};
}

} // namespace ayt::navigation
