#include <AYNavigation/Navigation.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ayt::navigation {
Vec3 operator+(Vec3 a,Vec3 b) {return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec3 operator-(Vec3 a,Vec3 b) {return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec3 operator*(Vec3 a,double value) {return {a.x*value,a.y*value,a.z*value};}
double dot(Vec3 a,Vec3 b) {return a.x*b.x+a.y*b.y+a.z*b.z;}
double length(Vec3 a) {return std::hypot(a.x,a.y,a.z);}
Vec3 normalized(Vec3 a) {const double size=length(a);return size>0 && std::isfinite(size)?a*(1/size):Vec3{};}
bool finite(Vec3 a) {return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z)
    && std::abs(a.x)<=1e12 && std::abs(a.y)<=1e12 && std::abs(a.z)<=1e12;}
bool valid(const Profile& p) {
    return std::isfinite(p.radius) && p.radius>0 && p.radius<=1e6 && std::isfinite(p.height) && p.height>0 && p.height<=1e6
        && std::isfinite(p.maxSpeed) && p.maxSpeed>0 && p.maxSpeed<=1e9 && std::isfinite(p.maxAcceleration)
        && p.maxAcceleration>0 && p.maxAcceleration<=1e9 && std::isfinite(p.arrivalDistance) && p.arrivalDistance>=0
        && p.arrivalDistance<=1e6 && std::isfinite(p.arrivalSpeed) && p.arrivalSpeed>=0 && p.arrivalSpeed<=p.maxSpeed
        && finite(p.up) && length(p.up)>1e-9 && (p.steering==Steering::Spatial || p.steering==Steering::Planar);
}
bool feasible(const AvoidanceQuery& query,Vec3 velocity) {
    if(!finite(velocity) || !valid(query.profile) || !query.motion.location.region || !finite(query.motion.location.position)
        || !finite(query.motion.velocity) || !std::isfinite(query.seconds) || query.seconds<=0 || query.seconds>1) return false;
    const double epsilon=1e-7;
    if(length(velocity)>std::max(query.profile.maxSpeed,length(query.motion.velocity))+epsilon
        || length(velocity-query.motion.velocity)>query.profile.maxAcceleration*query.seconds+epsilon) return false;
    if(query.profile.steering==Steering::Planar && std::abs(dot(velocity,normalized(query.profile.up)))>epsilon) return false;
    const Location next{query.motion.location.region,query.motion.location.position+velocity*query.seconds};
    if(!finite(next.position))return false;
    if(!query.space.permits(query.step,query.motion.location,next,query.profile)) return false;
    for(const auto& obstacle:query.obstacles) {
        if(!obstacle.id || !obstacle.location.region || !finite(obstacle.location.position) || !finite(obstacle.velocity)
            || !std::isfinite(obstacle.radius) || obstacle.radius<0 || obstacle.radius>1e6) return false;
        if(obstacle.id==query.agent || obstacle.location.region!=query.motion.location.region) continue;
        const auto delta=query.motion.location.position-obstacle.location.position;
        const auto relative=velocity-obstacle.velocity;const double squared=dot(relative,relative);
        const double t=squared>0?std::clamp(-dot(delta,relative)/squared,0.0,query.seconds):0;
        const double separation=query.profile.radius+obstacle.radius+1e-5;
        if(length(delta+relative*t)<separation) return false;
    }
    return true;
}
AvoidanceResult ConservativeAvoidance::choose(const AvoidanceQuery& query) const {
    if(feasible(query,query.desiredVelocity)) return {AvoidanceStatus::Clear,query.desiredVelocity};
    const auto desired=query.desiredVelocity;
    Vec3 direction=normalized(desired);
    if(length(direction)==0) direction={1,0,0};
    const auto up=normalized(query.profile.up);
    auto cross=[](Vec3 a,Vec3 b){return Vec3{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};};
    Vec3 side=normalized(cross(up,direction));
    if(length(side)==0) side=normalized(cross(std::abs(direction.x)<.8?Vec3{1,0,0}:Vec3{0,0,1},direction));
    const double sign=(query.agent&1)?1:-1;
    const double speed=std::max(length(desired),query.profile.maxSpeed*.5);
    std::vector<Vec3> candidates{desired*.5,desired+side*(speed*sign),desired-side*(speed*sign),side*(speed*sign),side*(-speed*sign)};
    if(query.profile.steering==Steering::Spatial) {
        const auto vertical=normalized(cross(direction,side));
        candidates.push_back(desired+vertical*speed);candidates.push_back(desired-vertical*speed);
    }
    candidates.push_back({});
    for(auto candidate:candidates) {
        if(length(candidate)>query.profile.maxSpeed) candidate=normalized(candidate)*query.profile.maxSpeed;
        const auto delta=candidate-query.motion.velocity;const double limit=query.profile.maxAcceleration*query.seconds;
        if(length(delta)>limit) candidate=query.motion.velocity+normalized(delta)*limit;
        if(feasible(query,candidate)) return {length(candidate)>1e-8?AvoidanceStatus::Clear:AvoidanceStatus::Wait,candidate};
    }
    return {AvoidanceStatus::Wait,{}};
}
TimedArcTraversal::TimedArcTraversal(std::uint32_t steps,double height):_steps(steps),_height(height) {
    if(steps<2 || steps>1000000 || !std::isfinite(height) || height<0 || height>1e6)
        throw std::invalid_argument("Invalid kinematic arc");
}
bool TimedArcTraversal::accepts(const Step& step,const Profile& profile) const {
    return step.traversal && step.from.region==step.to.region && valid(profile);
}
TravelResult TimedArcTraversal::update(TraversalContext& context,bool) {
    const double t=std::min(1.0,double(context.elapsedSteps+1)/_steps);
    const auto position=context.step.from.position+(context.step.to.position-context.step.from.position)*t
        +normalized(context.profile.up)*(4*_height*t*(1-t));
    if(!context.motor.traversalPose(context.agent,context.task,{context.step.from.region,position})) return {TravelStatus::Failed,false};
    return {t>=1?TravelStatus::Succeeded:TravelStatus::Running,t>=1};
}
} // namespace ayt::navigation
