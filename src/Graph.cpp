#include <AYNavigation/Graph.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace ayt::navigation {
namespace {
constexpr double epsilon=1e-7;
bool near(Location a,Location b) {return a.region==b.region && length(a.position-b.position)<=epsilon;}
struct Projection {double along=0,distance=0;Vec3 point;};
Projection project(Vec3 p,Vec3 a,Vec3 b) {
    const auto delta=b-a;const auto squared=dot(delta,delta);
    const auto along=squared>0?std::clamp(dot(p-a,delta)/squared,0.0,1.0):0;
    const auto point=a+delta*along;return {along,length(p-point),point};
}
bool on(const GraphSpace& graph,const Link& link,Location location,const Profile& profile,Projection* out=nullptr) {
    const auto& a=graph.node(link.from)->location;const auto& b=graph.node(link.to)->location;
    if(link.traversal || location.region!=a.region || !finite(location.position)) return false;
    const auto projection=project(location.position,a.position,b.position);
    if(out) *out=projection;
    return projection.distance<=link.width*.5-profile.radius+epsilon;
}
std::uint64_t partCost(const Link& link,double fraction) {
    return std::uint64_t(std::ceil(double(link.cost)*std::clamp(fraction,0.0,1.0)));
}
bool allowed(const PlanRequest& request,TraversalKind kind) {
    return !kind || std::find(request.traversals.begin(),request.traversals.end(),kind)!=request.traversals.end();
}
}
Result<std::shared_ptr<const GraphSpace>> GraphSpace::compile(GraphDefinition definition) {
    if(!definition.stamp.space || !definition.stamp.version || definition.nodes.empty()
        || definition.nodes.size()>10000 || definition.links.size()>50000) return {Error::Invalid};
    auto graph=std::shared_ptr<GraphSpace>(new GraphSpace);
    std::sort(definition.nodes.begin(),definition.nodes.end(),[](const auto& a,const auto& b){return a.id<b.id;});
    std::sort(definition.links.begin(),definition.links.end(),[](const auto& a,const auto& b){return a.id<b.id;});
    graph->_definition=std::move(definition);
    for(std::size_t i=0;i<graph->_definition.nodes.size();++i) {
        const auto& node=graph->_definition.nodes[i];
        if(!node.id || !node.location.region || !finite(node.location.position)) return {Error::Invalid};
        if(!graph->_nodes.emplace(node.id,i).second) return {Error::Duplicate};
    }
    std::map<std::uint64_t,std::uint32_t> capacities;
    for(std::size_t i=0;i<graph->_definition.links.size();++i) {
        const auto& link=graph->_definition.links[i];
        const auto* from=graph->node(link.from);const auto* to=graph->node(link.to);
        if(!link.id || !from || !to || link.from==link.to || !std::isfinite(link.width)
            || !std::isfinite(link.height) || link.width<=0 || link.width>1e9 || link.height<=0 || link.height>1e9
            || !link.cost || link.cost>1000000000000ULL || bool(link.resource)!=bool(link.capacity)
            || (!link.traversal && (from->location.region!=to->location.region
                || length(from->location.position-to->location.position)<=epsilon))) return {Error::Invalid};
        if(!graph->_links.emplace(link.id,i).second) return {Error::Duplicate};
        if(link.resource) {
            const auto [it,inserted]=capacities.emplace(link.resource,link.capacity);
            if(!inserted && it->second!=link.capacity) return {Error::Invalid};
        }
    }
    return {Error::None,std::move(graph)};
}
const Node* GraphSpace::node(NodeId id) const {const auto it=_nodes.find(id);return it==_nodes.end()?nullptr:&_definition.nodes[it->second];}
const Link* GraphSpace::link(LinkId id) const {const auto it=_links.find(id);return it==_links.end()?nullptr:&_definition.links[it->second];}
bool GraphSpace::eligible(const Link& link,const Profile& profile) const {
    if(!valid(profile) || !link.enabled || link.width+epsilon<profile.radius*2 || link.height+epsilon<profile.height
        || (profile.capabilities&link.requiredCapabilities)!=link.requiredCapabilities) return false;
    if(!link.traversal && profile.steering==Steering::Planar
        && std::abs(dot(node(link.to)->location.position-node(link.from)->location.position,normalized(profile.up)))>epsilon)
        return false;
    return true;
}
bool GraphSpace::contains(Location location,const Profile& profile) const {
    if(!valid(profile) || !location.region || !finite(location.position)) return false;
    for(const auto& node:_definition.nodes) if(near(location,node.location)) return true;
    for(const auto& link:_definition.links) if(eligible(link,profile) && on(*this,link,location,profile)) return true;
    return false;
}
Step GraphSpace::step(const Link& link,Location from,Location to) const {
    return {link.id,link.traversal,from,to,link.width*.5,link.resource,link.capacity};
}
bool GraphSpace::accepts(const Step& step,const Profile& profile) const {
    const auto* edge=link(step.link);
    if(!edge || !eligible(*edge,profile) || step.traversal!=edge->traversal || step.resource!=edge->resource
        || step.capacity!=edge->capacity || step.halfWidth!=edge->width*.5) return false;
    if(edge->traversal) return near(step.from,node(edge->from)->location) && near(step.to,node(edge->to)->location);
    Projection a,b;
    return on(*this,*edge,step.from,profile,&a) && on(*this,*edge,step.to,profile,&b) && b.along+epsilon>=a.along;
}
bool GraphSpace::permits(const Step& step,Location from,Location to,const Profile& profile) const {
    if(step.traversal || from.region!=to.region) return false;
    // A capsule around one straight link is convex: testing both endpoints proves the whole sweep.
    if(!step.link) {
        for(const auto& edge:_definition.links) if(eligible(edge,profile)
            && on(*this,edge,from,profile) && on(*this,edge,to,profile)) return true;
        return near(from,to) && contains(from,profile);
    }
    const auto* edge=link(step.link);
    return edge && accepts(step,profile) && on(*this,*edge,from,profile) && on(*this,*edge,to,profile);
}
Result<Location> GraphSpace::nearest(Location location,const Profile& profile,double maxDistance) const {
    if(!finite(location.position) || !location.region || !valid(profile) || !std::isfinite(maxDistance) || maxDistance<0)
        return {Error::Invalid};
    double best=maxDistance;std::optional<Location> result;
    for(const auto& edge:_definition.links) if(eligible(edge,profile) && !edge.traversal && node(edge.from)->location.region==location.region) {
        const auto projected=project(location.position,node(edge.from)->location.position,node(edge.to)->location.position);
        if(projected.distance<=best && (!result || projected.distance<best)) {
            best=projected.distance;result=Location{location.region,projected.point};
        }
    }
    for(const auto& node:_definition.nodes) if(node.location.region==location.region) {
        const double distance=length(location.position-node.location.position);
        if(distance<=best && (!result || distance<best)) {best=distance;result=node.location;}
    }
    return result?Result<Location>{Error::None,*result}:Result<Location>{Error::Missing};
}
PlanResult GraphPlanner::poll(const PlanRequest& request) {
    PlanResult result;result.request=request.id;result.status=PlanStatus::Failed;
    const auto* graph=dynamic_cast<const GraphSpace*>(request.space.get());
    if(!graph) {result.error=Error::Unsupported;return result;}
    if(!request.id || !request.expansionBudget || !graph->contains(request.from,request.profile)
        || !graph->contains(request.goal,request.profile)) {result.error=Error::Invalid;return result;}
    struct Previous {NodeId parent=0;LinkId link=0;std::optional<Step> first;};
    std::map<NodeId,std::uint64_t> distance;std::map<NodeId,Previous> previous;
    using QueueItem=std::pair<std::uint64_t,NodeId>;
    std::priority_queue<QueueItem,std::vector<QueueItem>,std::greater<QueueItem>> queue;
    auto seed=[&](NodeId id,std::uint64_t cost,std::optional<Step> first) {
        const auto it=distance.find(id);if(it==distance.end() || cost<it->second) {
            distance[id]=cost;previous[id]={0,0,std::move(first)};queue.emplace(cost,id);
        }
    };
    for(const auto& node:graph->definition().nodes) if(near(request.from,node.location)) seed(node.id,0,{});
    for(const auto& link:graph->definition().links) if(graph->eligible(link,request.profile) && !link.traversal) {
        Projection point;if(on(*graph,link,request.from,request.profile,&point))
            seed(link.to,partCost(link,1-point.along),graph->step(link,request.from,graph->node(link.to)->location));
    }
    std::map<NodeId,std::vector<const Link*>> outgoing;
    for(const auto& link:graph->definition().links) if(graph->eligible(link,request.profile) && allowed(request,link.traversal)) outgoing[link.from].push_back(&link);
    std::size_t expansions=0;
    while(!queue.empty()) {
        const auto [cost,id]=queue.top();queue.pop();if(distance.at(id)!=cost) continue;
        if(++expansions>request.expansionBudget) {result.error=Error::Budget;return result;}
        for(const auto* link:outgoing[id]) {
            if(cost>UINT64_MAX-link->cost) {result.error=Error::Budget;return result;}
            const auto next=cost+link->cost;const auto it=distance.find(link->to);
            if(it==distance.end() || next<it->second) {
                distance[link->to]=next;previous[link->to]={id,link->id,{}};queue.emplace(next,link->to);
            }
        }
    }
    std::optional<std::uint64_t> best;NodeId end=0;std::optional<Step> suffix;std::vector<Step> direct;
    auto choose=[&](std::uint64_t cost,NodeId node,std::optional<Step> tail,std::vector<Step> route={}) {
        if(!best || cost<*best) {best=cost;end=node;suffix=std::move(tail);direct=std::move(route);}
    };
    if(near(request.from,request.goal)) choose(0,0,{});
    for(const auto& node:graph->definition().nodes) if(near(request.goal,node.location) && distance.contains(node.id))
        choose(distance.at(node.id),node.id,{});
    for(const auto& link:graph->definition().links) if(graph->eligible(link,request.profile) && !link.traversal) {
        Projection goal,start;if(!on(*graph,link,request.goal,request.profile,&goal)) continue;
        if(distance.contains(link.from)) choose(distance.at(link.from)+partCost(link,goal.along),link.from,
            graph->step(link,graph->node(link.from)->location,request.goal));
        if(on(*graph,link,request.from,request.profile,&start) && start.along<=goal.along+epsilon)
            choose(partCost(link,goal.along-start.along),0,{}, {graph->step(link,request.from,request.goal)});
    }
    if(!best) {result.status=PlanStatus::Unreachable;result.error=Error::NoPath;return result;}
    std::vector<Step> reverse;
    for(auto current=end;current;) {
        const auto& entry=previous.at(current);
        if(!entry.parent) {if(entry.first) reverse.push_back(*entry.first);break;}
        const auto* link=graph->link(entry.link);
        reverse.push_back(graph->step(*link,graph->node(link->from)->location,graph->node(link->to)->location));
        current=entry.parent;
    }
    std::reverse(reverse.begin(),reverse.end());
    if(end) direct=std::move(reverse);
    if(suffix) direct.push_back(*suffix);
    result.status=PlanStatus::Found;result.route={graph->stamp(),request.goal,std::move(direct)};return result;
}
Result<std::shared_ptr<const VolumeSpace>> VolumeSpace::compile(Stamp stamp,std::vector<RegionBox> boxes) {
    if(!stamp.space || !stamp.version || boxes.empty() || boxes.size()>10000) return {Error::Invalid};
    std::sort(boxes.begin(),boxes.end(),[](const auto& a,const auto& b){return a.region<b.region;});
    RegionId last=0;
    for(const auto& box:boxes) {
        if(!box.region || box.region==last || !finite(box.minimum) || !finite(box.maximum)
            || box.minimum.x>=box.maximum.x || box.minimum.y>=box.maximum.y || box.minimum.z>=box.maximum.z) return {Error::Invalid};
        last=box.region;
    }
    auto space=std::shared_ptr<VolumeSpace>(new VolumeSpace);space->_stamp=stamp;space->_boxes=std::move(boxes);
    return {Error::None,std::move(space)};
}
bool VolumeSpace::contains(Location location,const Profile& profile) const {
    if(!finite(location.position) || !valid(profile)) return false;
    for(const auto& box:_boxes) if(box.region==location.region) {
        const auto p=location.position;const auto r=profile.radius;
        return p.x>=box.minimum.x+r && p.x<=box.maximum.x-r && p.y>=box.minimum.y+r && p.y<=box.maximum.y-r
            && p.z>=box.minimum.z+r && p.z<=box.maximum.z-r;
    }
    return false;
}
bool VolumeSpace::accepts(const Step& step,const Profile& profile) const {
    return !step.link && !step.traversal && !step.resource && !step.capacity && step.halfWidth==0
        && step.from.region==step.to.region && contains(step.from,profile) && contains(step.to,profile)
        && (profile.steering!=Steering::Planar || std::abs(dot(step.to.position-step.from.position,normalized(profile.up)))<=epsilon);
}
bool VolumeSpace::permits(const Step& step,Location from,Location to,const Profile& profile) const {
    return accepts(step,profile) && from.region==step.from.region && to.region==from.region
        && contains(from,profile) && contains(to,profile)
        && (profile.steering!=Steering::Planar || std::abs(dot(to.position-from.position,normalized(profile.up)))<=epsilon);
}
PlanResult DirectPlanner::poll(const PlanRequest& request) {
    PlanResult result;result.request=request.id;
    Step step;step.from=request.from;step.to=request.goal;
    if(!request.id || !request.space || !valid(request.profile)) {result.status=PlanStatus::Failed;result.error=Error::Invalid;}
    else if(!request.space->accepts(step,request.profile)) {result.status=PlanStatus::Unreachable;result.error=Error::NoPath;}
    else {result.status=PlanStatus::Found;result.route={request.space->stamp(),request.goal,{step}};}
    return result;
}
} // namespace ayt::navigation
