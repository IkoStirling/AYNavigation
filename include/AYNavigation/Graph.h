#pragma once
#include <AYNavigation/Navigation.h>

namespace ayt::navigation {
struct Node {NodeId id=0;Location location;};
struct Link {
    LinkId id=0;NodeId from=0,to=0;
    double width=1,height=3;
    std::uint64_t cost=1,requiredCapabilities=0,resource=0;
    std::uint32_t capacity=0;
    TraversalKind traversal=0;
    bool enabled=true;
};
struct GraphDefinition {Stamp stamp;std::vector<Node> nodes;std::vector<Link> links;};
/// Immutable hand-authored directed path network. Ordinary links are straight swept corridors;
/// author polylines as connected links. Special links connect explicit endpoints, including regions.
class GraphSpace final:public Space {
public:
    /// Atomic validation of IDs, references, geometry, capacities and authoring budgets.
    static Result<std::shared_ptr<const GraphSpace>> compile(GraphDefinition);
    Stamp stamp() const noexcept override {return _definition.stamp;}
    bool contains(Location,const Profile&) const override;
    bool accepts(const Step&,const Profile&) const override;
    bool permits(const Step&,Location,Location,const Profile&) const override;
    /// Explicit target projection onto an eligible ordinary corridor. Does not authorize travelling
    /// from an off-network start; callers choose a finite maximum snapping distance.
    Result<Location> nearest(Location,const Profile&,double maxDistance) const;
    const GraphDefinition& definition() const noexcept {return _definition;}
    const Node* node(NodeId) const;
    const Link* link(LinkId) const;
    bool eligible(const Link&,const Profile&) const;
    Step step(const Link&,Location from,Location to) const;
private:
    GraphDefinition _definition;
    std::map<NodeId,std::size_t> _nodes;std::map<LinkId,std::size_t> _links;
};
/// Default bounded Dijkstra planner for GraphSpace. Stable (cost,node/link ID) ordering, capability
/// and dimension-aware clearance, interior directed-corridor starts/goals and cross-region links.
class GraphPlanner final:public Planner {
public:
    PlanResult poll(const PlanRequest&) override;
};
struct RegionBox {RegionId region=0;Vec3 minimum,maximum;};
/// Explicit free-motion box domains for 2D planes or 3D volume examples. No static obstacle
/// occupancy is inferred; supply ObstacleSource or replace this space/planner with game geometry.
class VolumeSpace final:public Space {
public:
    static Result<std::shared_ptr<const VolumeSpace>> compile(Stamp,std::vector<RegionBox>);
    Stamp stamp() const noexcept override {return _stamp;}
    bool contains(Location,const Profile&) const override;
    bool accepts(const Step&,const Profile&) const override;
    bool permits(const Step&,Location,Location,const Profile&) const override;
private:
    Stamp _stamp;std::vector<RegionBox> _boxes;
};
/// Straight route planner for any Space that validates a direct ordinary step. Replace it for
/// nontrivial free-space global planning; local obstacle avoidance is a separate policy.
class DirectPlanner final:public Planner {
public:
    PlanResult poll(const PlanRequest&) override;
};
} // namespace ayt::navigation
