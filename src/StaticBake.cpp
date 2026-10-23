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

bool intersects(double minX, double maxX, double minZ, double maxZ,
                const BakeBox& blocker, double floorY, double height)
{
    if (blocker.maximum.y <= floorY || blocker.minimum.y >= floorY + height)
        return false;
    return blocker.minimum.x < maxX && blocker.maximum.x > minX
        && blocker.minimum.z < maxZ && blocker.maximum.z > minZ;
}

bool clearCorridor(const BakeRegion& region, Vec3 a, Vec3 b,
                   double width, double height)
{
    const double half = width * 0.5;
    const double minX = std::min(a.x, b.x) - half;
    const double maxX = std::max(a.x, b.x) + half;
    const double minZ = std::min(a.z, b.z) - half;
    const double maxZ = std::max(a.z, b.z) + half;
    if (minX < region.floor.minimum.x || maxX > region.floor.maximum.x
        || minZ < region.floor.minimum.z || maxZ > region.floor.maximum.z)
        return false;
    for (const auto& blocker : region.blockers)
        if (intersects(minX, maxX, minZ, maxZ, blocker,
                       region.floor.maximum.y, height)) return false;
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
    if (!region.id || !region.contentVersion || !validBox(region.floor, true))
        return {Error::Invalid};
    for (const auto& blocker : region.blockers)
        if (!validBox(blocker, false)) return {Error::Invalid};
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
        if (!connection.from || !connection.to || connection.from == connection.to
            || !finite(connection.departure) || !finite(connection.arrival)
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
        || _config.cellSize <= 0.0 || _config.cellSize > 1e6
        || _config.corridorWidth <= 0.0
        || _config.corridorWidth > _config.cellSize
        || _config.agentHeight <= 0.0 || _config.agentHeight > 1e6
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
    std::map<RegionId, RegionGrid> grids;
    NodeId nextNode = 1;
    LinkId nextLink = 1;
    std::size_t cellCount = 0;
    const auto maxCells = std::min<std::size_t>(_config.maxCells, 10000);
    const auto cost = static_cast<std::uint64_t>(std::ceil(_config.cellSize * 1000.0));
    for (const auto& [regionId, region] : regions) {
        const double dx = region.floor.maximum.x - region.floor.minimum.x;
        const double dz = region.floor.maximum.z - region.floor.minimum.z;
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
                    region.floor.minimum.x + (double(x) + 0.5) * _config.cellSize,
                    region.floor.maximum.y,
                    region.floor.minimum.z + (double(z) + 0.5) * _config.cellSize};
                if (!clearCorridor(region, center, center,
                                   _config.corridorWidth, _config.agentHeight)) {
                    ++report.blockedCells;
                    continue;
                }
                grid.nodes[z * nx + x] = nextNode;
                graph.nodes.push_back({nextNode++, {regionId, center}});
            }
        }
        grids.emplace(regionId, std::move(grid));
    }
    if (graph.nodes.empty()) return {Error::NoPath};
    for (const auto& [regionId, region] : regions) {
        const auto& grid = grids.at(regionId);
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
                    if (!clearCorridor(region, a, b,
                                       _config.corridorWidth, _config.agentHeight))
                        continue;
                    if (graph.links.size() + 2 > std::min<std::size_t>(
                            _config.maxLinks, 50000)) return {Error::Budget};
                    graph.links.push_back({nextLink++, id, neighbor,
                                           _config.corridorWidth, _config.agentHeight,
                                           cost});
                    graph.links.push_back({nextLink++, neighbor, id,
                                           _config.corridorWidth, _config.agentHeight,
                                           cost});
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
        if (!from || !to) return {Error::NoPath};
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
