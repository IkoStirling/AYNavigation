#pragma once

#include <AYNavigation/Graph.h>

#include <map>

namespace ayt::navigation {

/// Static box extracted by a Scene/Physics adapter. Dynamic actors remain in
/// ObstacleSource and are never baked into this topology.
struct BakeBox {
    Vec3 minimum;
    Vec3 maximum;
    bool operator==(const BakeBox&) const = default;
};

/// Axis-aligned XZ footprint with one planar walkable top. Heights correspond
/// to (minX,minZ), (maxX,minZ), (minX,maxZ), (maxX,maxZ). Adjacent horizontal
/// floors still require an authored traversal connection.
struct BakeRamp {
    BakeBox footprint;
    double y00 = 0, y10 = 0, y01 = 0, y11 = 0;
    bool operator==(const BakeRamp&) const = default;
};

/// One primary horizontal floor, optional extra flat/ramp surfaces and static
/// blockers. The authoring tool/Host supplies finite world-space geometry and
/// a persistent region ID.
struct BakeRegion {
    RegionId id = 0;
    std::uint64_t contentVersion = 0;
    BakeBox floor;
    std::vector<BakeBox> blockers;
    /// Optional stacked walkable surfaces in the same region. Each surface
    /// bakes an independent grid; authored same-region BakeConnections join
    /// floors by a registered traversal (stairs, ladder, lift, etc.).
    std::vector<BakeBox> additionalFloors;
    /// Optional planar ramps, each on its own grid; footprints must not
    /// overlap flat floors or other ramps in XZ interior.
    std::vector<BakeRamp> ramps;
    bool operator==(const BakeRegion&) const = default;
};

/// Explicit directed access across regions or floors of one region. A
/// traversal adapter must implement the same kind in Navigator.
struct BakeConnection {
    RegionId from = 0, to = 0;
    Vec3 departure, arrival;
    TraversalKind traversal = 0;
    std::uint64_t resource = 0;
    std::uint32_t capacity = 0;
    bool operator==(const BakeConnection&) const = default;
};

struct BakeConfig {
    double cellSize = 1.0;
    double corridorWidth = 1.0;
    double agentHeight = 1.8;
    /// Maximum rise per horizontal unit for an ordinary grid link.
    double maxRisePerHorizontal = 1.0;
    std::size_t maxCells = 10000;
    std::size_t maxLinks = 50000;
};

struct BakeReport {
    Stamp stamp;
    std::size_t regions = 0;
    /// Number of region geometries recalculated for this publication. A
    /// connection-only revision reuses all cached region fragments.
    std::size_t geometryRebakes = 0;
    std::size_t nodes = 0;
    std::size_t links = 0;
    std::size_t blockedCells = 0;
    std::size_t disabledConnections = 0;
};

/// Bounded, deterministic 2.5D flat/ramp grid bake into the existing immutable
/// GraphSpace. Changed region geometry is rebaked locally, then the complete
/// immutable graph is recompiled for one atomic revision. Failed bakes leave
/// the previous graph and authoring inputs untouched. Not a NavMesh solver.
class StaticBakeWorld {
public:
    StaticBakeWorld(std::uint64_t spaceId, BakeConfig config = {});
    Result<std::shared_ptr<const GraphSpace>> publishRegion(BakeRegion region);
    Result<std::shared_ptr<const GraphSpace>> unloadRegion(RegionId id);
    Result<std::shared_ptr<const GraphSpace>> setConnections(
        std::vector<BakeConnection> connections);
    [[nodiscard]] std::shared_ptr<const GraphSpace> current() const noexcept { return _graph; }
    [[nodiscard]] BakeReport report() const noexcept { return _report; }

private:
    struct RegionFragment {
        std::size_t cells = 0, blockedCells = 0;
        std::vector<Node> nodes;
        std::vector<Link> links;
    };
    Result<RegionFragment> bakeRegion(const BakeRegion& region) const;
    Result<std::shared_ptr<const GraphSpace>> rebuild(
        std::map<RegionId, BakeRegion> regions,
        std::vector<BakeConnection> connections);
    std::uint64_t _spaceId;
    BakeConfig _config;
    std::map<RegionId, BakeRegion> _regions;
    std::map<RegionId, std::shared_ptr<const RegionFragment>> _fragments;
    std::vector<BakeConnection> _connections;
    std::shared_ptr<const GraphSpace> _graph;
    BakeReport _report;
};

} // namespace ayt::navigation
