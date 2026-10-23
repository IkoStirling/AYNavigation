#include <AYNavigation/Navigation.h>
#include <algorithm>
#include <cmath>

namespace ayt::navigation {
FollowResult BrakingFollower::follow(const FollowQuery& query) const {
    const auto delta=query.step.to.position-query.motion.location.position;
    const double distance=length(delta);
    const double speed=std::min(query.profile.maxSpeed,
        std::sqrt(2*query.profile.maxAcceleration*std::max(0.0,distance-query.profile.arrivalDistance*.25)));
    auto velocity=normalized(delta)*std::min(speed,distance/query.seconds);
    if(distance<=query.profile.arrivalDistance*.25)velocity={};
    const auto change=velocity-query.motion.velocity;const double limit=query.profile.maxAcceleration*query.seconds;
    if(length(change)>limit)velocity=query.motion.velocity+normalized(change)*limit;
    return {velocity,distance<=query.profile.arrivalDistance && length(query.motion.velocity)<=query.profile.arrivalSpeed,true};
}
} // namespace ayt::navigation
