#include <AYNavigation/Navigation.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace ayt::navigation {
namespace {
struct Guard {bool& flag;explicit Guard(bool& f):flag(f){flag=true;}~Guard(){flag=false;}};
bool usable(const MotionState& state) {return state.location.region && finite(state.location.position) && finite(state.velocity);}
bool near(Location a,Location b,double tolerance) {return a.region==b.region && length(a.position-b.position)<=tolerance;}
bool terminal(Phase phase) {return phase==Phase::Idle || phase==Phase::Succeeded || phase==Phase::Failed;}
}
Navigator::Navigator(std::shared_ptr<const Space> space,std::shared_ptr<Planner> planner,
    std::shared_ptr<const Avoidance> avoidance,Motor& motor,const ObstacleSource& obstacles,Config config,std::shared_ptr<const Follower> follower)
    :_config(config),_space(std::move(space)),_planner(std::move(planner)),_avoidance(std::move(avoidance)),
    _follower(follower?std::move(follower):std::make_shared<BrakingFollower>()),_motor(motor),_obstacles(obstacles) {
    if(!_space || !_space->stamp().space || !_space->stamp().version || !_planner || !_avoidance
        || !std::isfinite(config.secondsPerStep) || config.secondsPerStep<=0 || config.secondsPerStep>1
        || !config.maxAgents || !config.maxRouteSteps || !config.expansionBudget || !config.maxPlanningSteps || !config.maxBlockedSteps)
        throw std::invalid_argument("Invalid navigation configuration");
}
Navigator::~Navigator() {_busy=true;for(auto& [_,agent]:_agents) cleanup(agent);}
Error Navigator::attach(AgentId id,Profile profile) {
    if(_busy) return Error::Busy;
    if(!id || !valid(profile)) return Error::Invalid;
    if(_agents.contains(id)) return Error::Duplicate;
    if(_agents.size()>=_config.maxAgents) return Error::Capacity;
    Guard guard(_busy);
    try {
        const auto sample=_motor.sample(id);
        if(!sample || !usable(*sample) || !_space->contains(sample->location,profile)) return Error::Missing;
        Agent agent;agent.profile=profile;agent.observed=*sample;agent.view.agent=id;
        _agents.emplace(id,std::move(agent));return Error::None;
    } catch(...) {return Error::MotionFailed;}
}
Error Navigator::detach(AgentId id) {
    if(_busy) return Error::Busy;
    const auto it=_agents.find(id);if(it==_agents.end()) return Error::Missing;
    Guard guard(_busy);cleanup(it->second);_agents.erase(it);return Error::None;
}
Error Navigator::registerTraversal(TraversalKind kind,std::shared_ptr<Traversal> executor) {
    if(_busy) return Error::Busy;
    if(!kind || !executor) return Error::Invalid;
    if(_traversals.contains(kind)) return Error::Duplicate;
    _traversals.emplace(kind,std::move(executor));return Error::None;
}
Error Navigator::publish(std::shared_ptr<const Space> space) {
    if(_busy) return Error::Busy;
    if(!space || !space->stamp().space || !space->stamp().version || space->stamp().space!=_space->stamp().space) return Error::Invalid;
    if(space==_space) return Error::None;
    if(space->stamp().version<=_space->stamp().version) return Error::Stale;
    _space=std::move(space);return Error::None;
}
bool Navigator::available(AgentId id) const {
    if(_busy || !_agents.contains(id)) return false;
    Guard guard(_busy);try {const auto sample=_motor.sample(id);return sample && usable(*sample);} catch(...) {return false;}
}
bool Navigator::canQueue(AgentId id,Location goal) const {
    if(_busy || !goal.region || !finite(goal.position))return false;
    const auto it=_agents.find(id);if(it==_agents.end())return false;
    Guard guard(_busy);
    try {const auto sample=_motor.sample(id);return sample&&usable(*sample)&&_space->contains(goal,it->second.profile);}
    catch(...){return false;}
}
bool Navigator::canBegin(AgentId id,Location goal) const {
    if(_busy || !goal.region || !finite(goal.position)) return false;
    const auto it=_agents.find(id);if(it==_agents.end()) return false;
    Guard guard(_busy);
    try {const auto sample=_motor.sample(id);return sample && usable(*sample) && _space->contains(goal,it->second.profile)
        && _space->contains(sample->location,it->second.profile);} catch(...) {return false;}
}
TraversalContext Navigator::context(Agent& agent,const Step& step) {
    return {agent.view.agent,agent.view.task,step,agent.profile,agent.observed,_motor,agent.memory,agent.elapsed,_config.secondsPerStep};
}
void Navigator::release(Agent& agent) noexcept {
    if(!agent.reserved || agent.view.step>=agent.view.route.steps.size()) return;
    const auto resource=agent.view.route.steps[agent.view.step].resource;
    const auto it=_reservations.find(resource);
    if(it!=_reservations.end()) {
        const auto owner=it->second.find(agent.view.agent);
        if(owner!=it->second.end() && owner->second==agent.view.task) it->second.erase(owner);
        if(it->second.empty()) _reservations.erase(it);
    }
    agent.reserved=false;
}
void Navigator::cleanup(Agent& agent) noexcept {
    if(agent.request) {_planner->cancel(agent.request->id);agent.request.reset();}
    if(agent.traversalStarted && agent.view.step<agent.view.route.steps.size()) {
        const auto& step=agent.view.route.steps[agent.view.step];
        const auto it=_traversals.find(step.traversal);
        if(it!=_traversals.end()) {auto ctx=context(agent,step);it->second->cancel(ctx);}
    }
    agent.traversalStarted=false;release(agent);
    if(agent.view.task) _motor.stop(agent.view.agent,agent.view.task);
    agent.view.interruptible=true;
}
Update Navigator::fail(Agent& agent,Error error) {
    cleanup(agent);agent.view.phase=Phase::Failed;agent.view.error=error;
    return {TravelStatus::Failed,error,true};
}
Error Navigator::plan(Agent& agent) {
    if(_lastRequest==UINT64_MAX) return Error::Capacity;
    if(agent.view.request && agent.view.replans>=_config.maxReplans) return Error::Budget;
    if(!_space->contains(agent.observed.location,agent.profile) || !_space->contains(agent.goal,agent.profile)) return Error::NoPath;
    if(agent.request) {_planner->cancel(agent.request->id);agent.request.reset();}
    release(agent);
    _motor.stop(agent.view.agent,agent.view.task);
    if(agent.view.request) ++agent.view.replans;
    PlanRequest request;request.id=++_lastRequest;request.agent=agent.view.agent;request.task=agent.view.task;
    request.from=agent.observed.location;request.goal=agent.goal;request.profile=agent.profile;
    request.space=_space;request.expansionBudget=_config.expansionBudget;
    for(const auto& [kind,_]:_traversals) request.traversals.push_back(kind);
    agent.view.request=request.id;agent.request=std::move(request);agent.view.phase=Phase::Planning;
    agent.view.interruptible=true;agent.elapsed=0;agent.lastDistance=1e100;return Error::None;
}
Error Navigator::begin(AgentId id,TaskId task,Location goal) {
    return start(id,task,goal,false);
}
Error Navigator::resume(AgentId id,TaskId task,Location goal) {
    return start(id,task,goal,true);
}
Error Navigator::start(AgentId id,TaskId task,Location goal,bool restoring) {
    if(_busy) return Error::Busy;
    if(!task || !goal.region || !finite(goal.position)) return Error::Invalid;
    const auto it=_agents.find(id);if(it==_agents.end()) return Error::Missing;
    auto& agent=it->second;
    if((!restoring && (task<agent.highestTask || (task==agent.highestTask && task!=agent.view.task)))
        || (restoring && (!agent.highestTask || task>agent.highestTask))
        || (task==agent.view.task && agent.goal!=goal)) return Error::Stale;
    if(restoring && !terminal(agent.view.phase))return Error::Busy;
    if(task==agent.view.task && !terminal(agent.view.phase)) return Error::None;
    if(!terminal(agent.view.phase) && !agent.view.interruptible) return Error::Busy;
    Guard guard(_busy);
    try {
        const auto sample=_motor.sample(id);
        if(!sample || !usable(*sample) || !_space->contains(sample->location,agent.profile) || !_space->contains(goal,agent.profile)) return Error::Invalid;
        cleanup(agent);agent.observed=*sample;agent.goal=goal;agent.view={};agent.view.agent=id;agent.view.task=task;
        agent.highestTask=std::max(agent.highestTask,task);
        agent.routeSpace.reset();agent.memory={};agent.elapsed=0;
        if(const auto error=plan(agent);error!=Error::None) {fail(agent,error);return error;}
        return Error::None;
    } catch(...) {fail(agent,Error::MotionFailed);return Error::MotionFailed;}
}
bool Navigator::validateRoute(const Agent& agent,const Route& route) const {
    if(!agent.request || route.stamp!=agent.request->space->stamp() || route.goal!=agent.goal || route.steps.size()>_config.maxRouteSteps) return false;
    Location cursor=agent.request->from;
    for(const auto& step:route.steps) {
        if(!step.from.region || !step.to.region || !finite(step.from.position) || !finite(step.to.position)
            || !std::isfinite(step.halfWidth) || step.halfWidth<0 || step.halfWidth>1e12
            || bool(step.resource)!=bool(step.capacity) || (!step.traversal && step.from.region!=step.to.region))return false;
        if(!near(step.from,cursor,1e-6) || !agent.request->space->accepts(step,agent.profile)) return false;
        if(step.traversal) {
            const auto it=_traversals.find(step.traversal);
            if(it==_traversals.end() || !it->second->accepts(step,agent.profile)) return false;
        }
        cursor=step.to;
    }
    return near(cursor,agent.goal,1e-6);
}
bool Navigator::reserve(Agent& agent,const Step& step) {
    if(!step.resource || !step.capacity || agent.reserved) return true;
    auto& owners=_reservations[step.resource];
    if(owners.size()>=step.capacity) {if(owners.empty()) _reservations.erase(step.resource);return false;}
    owners[agent.view.agent]=agent.view.task;agent.reserved=true;return true;
}
Update Navigator::advance(Agent& agent) {
    if(agent.view.phase==Phase::Planning) {
        if(agent.request->space->stamp()!=_space->stamp()) {
            if(const auto error=plan(agent);error!=Error::None) return fail(agent,error);
        }
        if(++agent.elapsed>_config.maxPlanningSteps) return fail(agent,Error::Budget);
        auto result=_planner->poll(*agent.request);
        if(result.request!=agent.request->id || result.status==PlanStatus::Pending) return {TravelStatus::Waiting};
        if(result.status==PlanStatus::Unreachable) return fail(agent,Error::NoPath);
        if(result.status!=PlanStatus::Found) return fail(agent,result.error==Error::None?Error::InvalidRoute:result.error);
        if(!validateRoute(agent,result.route)) return fail(agent,Error::InvalidRoute);
        const bool startStillValid=result.route.steps.empty()?near(agent.observed.location,agent.goal,agent.profile.arrivalDistance)
            :result.route.steps.front().traversal?near(agent.observed.location,result.route.steps.front().from,agent.profile.arrivalDistance)
            :agent.request->space->permits(result.route.steps.front(),agent.observed.location,agent.observed.location,agent.profile);
        if(!startStillValid) {
            if(const auto error=plan(agent);error!=Error::None)return fail(agent,error);
            return {TravelStatus::Waiting};
        }
        agent.routeSpace=agent.request->space;agent.view.route=std::move(result.route);agent.view.step=0;
        _planner->cancel(agent.request->id);agent.request.reset();agent.view.phase=Phase::Following;
        agent.view.blockedSteps=0;agent.elapsed=0;return {TravelStatus::Waiting};
    }
    if(agent.view.phase!=Phase::Traversing && agent.routeSpace->stamp()!=_space->stamp()) {
        if(const auto error=plan(agent);error!=Error::None) return fail(agent,error);
        return {TravelStatus::Waiting};
    }
    const bool empty=agent.view.route.steps.empty() || agent.view.step>=agent.view.route.steps.size();
    Step hold;hold.from=hold.to=agent.goal;
    const auto& step=empty?hold:agent.view.route.steps.at(agent.view.step);
    if(!step.traversal) {
        if(!agent.routeSpace->contains(agent.observed.location,agent.profile)) return fail(agent,Error::MotionFailed);
        if(agent.observed.location.region!=step.to.region) {
            if(const auto error=plan(agent);error!=Error::None)return fail(agent,error);
            return {TravelStatus::Waiting};
        }
        const double distance=length(step.to.position-agent.observed.location.position);
        const bool final=empty || agent.view.step+1>=agent.view.route.steps.size();
        const Step* next=final?nullptr:&agent.view.route.steps[agent.view.step+1];
        const bool boundaryStop=final || (next && (next->traversal || next->resource));
        const auto follow=_follower->follow({agent.view.agent,agent.observed,agent.profile,step,next,_config.secondsPerStep,final});
        if(!finite(follow.velocity))return fail(agent,Error::MotionFailed);
        if(follow.advance && distance<=agent.profile.arrivalDistance
            && (!boundaryStop || length(agent.observed.velocity)<=agent.profile.arrivalSpeed)
            && (!final || _motor.acceptArrival(agent.view.agent,agent.view.task,agent.goal,agent.profile))) {
            release(agent);if(follow.brakeOnAdvance || boundaryStop)_motor.stop(agent.view.agent,agent.view.task);
            if(!empty)++agent.view.step;agent.lastDistance=1e100;agent.view.blockedSteps=0;
            if(agent.view.step>=agent.view.route.steps.size()) {
                cleanup(agent);agent.view.phase=Phase::Succeeded;return {TravelStatus::Succeeded};
            }
            return {};
        }
        if(!reserve(agent,step)) {
            agent.view.phase=Phase::WaitingForLink;agent.view.interruptible=true;
            _motor.stop(agent.view.agent,agent.view.task);
            if(++agent.view.blockedSteps>=_config.maxBlockedSteps) {
                if(const auto error=plan(agent);error!=Error::None)return fail(agent,error);
            }
            return {TravelStatus::Waiting};
        }
        const auto desired=follow.velocity;
        const double range=agent.profile.radius+std::max(agent.profile.maxSpeed,length(agent.observed.velocity))*_config.secondsPerStep*2;
        const auto obstacles=_obstacles.query(agent.view.agent,agent.observed,range);
        if(!obstacles.valid || obstacles.obstacles.size()>65536) return fail(agent,Error::QueryFailed);
        std::set<std::uint64_t> ids;
        for(const auto& obstacle:obstacles.obstacles) if(!obstacle.id || !obstacle.location.region || !finite(obstacle.location.position)
            || !finite(obstacle.velocity) || !std::isfinite(obstacle.radius) || obstacle.radius<0 || obstacle.radius>1e6
            || !ids.insert(obstacle.id).second) return fail(agent,Error::QueryFailed);
        const AvoidanceQuery query{agent.view.agent,agent.observed,agent.profile,step,*agent.routeSpace,desired,_config.secondsPerStep,obstacles.obstacles};
        const auto chosen=_avoidance->choose(query);
        if(chosen.status==AvoidanceStatus::Failed) return fail(agent,Error::Blocked);
        if((chosen.status!=AvoidanceStatus::Clear && chosen.status!=AvoidanceStatus::Wait) || !feasible(query,chosen.velocity)) {
            // Invalidate a previous drive rather than publishing an infeasible instantaneous zero
            // velocity. The real motor owns emergency braking/collision response.
            _motor.stop(agent.view.agent,agent.view.task);
            agent.view.phase=Phase::Blocked;agent.view.interruptible=true;
            if(++agent.view.blockedSteps>=_config.maxBlockedSteps) {
                if(const auto error=plan(agent);error!=Error::None) return fail(agent,error);
            }
            return {TravelStatus::Waiting};
        }
        if(!_motor.drive(agent.view.agent,agent.view.task,chosen.velocity,step.to,_config.secondsPerStep)) return fail(agent,Error::MotionFailed);
        if(distance<agent.lastDistance-1e-6 && chosen.status==AvoidanceStatus::Clear) agent.view.blockedSteps=0;
        else ++agent.view.blockedSteps;
        agent.lastDistance=distance;agent.view.phase=chosen.status==AvoidanceStatus::Wait?Phase::Blocked:Phase::Following;
        agent.view.interruptible=true;
        if(agent.view.blockedSteps>=_config.maxBlockedSteps) {
            if(const auto error=plan(agent);error!=Error::None) return fail(agent,error);
        }
        return {chosen.status==AvoidanceStatus::Wait?TravelStatus::Waiting:TravelStatus::Running};
    }
    if(!agent.traversalStarted) {
        if(!near(agent.observed.location,step.from,agent.profile.arrivalDistance) || length(agent.observed.velocity)>agent.profile.arrivalSpeed)
            return fail(agent,Error::TraversalFailed);
        if(!reserve(agent,step)) {
            agent.view.phase=Phase::WaitingForLink;_motor.stop(agent.view.agent,agent.view.task);
            if(++agent.view.blockedSteps>=_config.maxBlockedSteps) {
                if(const auto error=plan(agent);error!=Error::None)return fail(agent,error);
            }
            return {TravelStatus::Waiting};
        }
        agent.elapsed=0;agent.memory={};
    }
    const auto it=_traversals.find(step.traversal);if(it==_traversals.end()) return fail(agent,Error::Unsupported);
    auto ctx=context(agent,step);const bool starting=!agent.traversalStarted;
    agent.traversalStarted=true;agent.view.phase=Phase::Traversing;
    const auto result=it->second->update(ctx,starting);++agent.elapsed;agent.view.interruptible=result.interruptible;
    if(result.status==TravelStatus::Failed) return fail(agent,Error::TraversalFailed);
    if(result.status==TravelStatus::Succeeded) {
        const auto landed=_motor.sample(agent.view.agent);
        if(!landed || !usable(*landed) || !near(landed->location,step.to,agent.profile.arrivalDistance)
            || length(landed->velocity)>agent.profile.arrivalSpeed) return fail(agent,Error::TraversalFailed);
        agent.observed=*landed;agent.traversalStarted=false;release(agent);++agent.view.step;
        agent.elapsed=0;agent.lastDistance=1e100;agent.view.blockedSteps=0;agent.view.interruptible=true;
        if(agent.routeSpace->stamp()!=_space->stamp()) {
            if(const auto error=plan(agent);error!=Error::None) return fail(agent,error);
            return {TravelStatus::Waiting};
        }
        if(agent.view.step>=agent.view.route.steps.size()) {agent.view.phase=Phase::Following;return {TravelStatus::Waiting};}
        agent.view.phase=Phase::Following;return {};
    }
    if(result.status!=TravelStatus::Running && result.status!=TravelStatus::Waiting) return fail(agent,Error::TraversalFailed);
    return {result.status,Error::None,result.interruptible};
}
Update Navigator::update(AgentId id,TaskId task) {
    if(_busy) return {TravelStatus::Failed,Error::Busy};
    const auto it=_agents.find(id);if(it==_agents.end()) return {TravelStatus::Failed,Error::Missing};
    auto& agent=it->second;if(task!=agent.view.task || agent.view.phase==Phase::Idle) return {TravelStatus::Failed,Error::Stale};
    if(agent.view.phase==Phase::Succeeded) return {TravelStatus::Succeeded};
    if(agent.view.phase==Phase::Failed) return {TravelStatus::Failed,agent.view.error};
    Guard guard(_busy);
    try {
        const auto sample=_motor.sample(id);if(!sample || !usable(*sample)) return fail(agent,Error::MotionFailed);
        agent.observed=*sample;return advance(agent);
    } catch(...) {return fail(agent,Error::MotionFailed);}
}
bool Navigator::cancel(AgentId id,TaskId task) noexcept {
    if(_busy) return false;
    const auto it=_agents.find(id);if(it==_agents.end() || it->second.view.task!=task) return false;
    Guard guard(_busy);const bool finished=terminal(it->second.view.phase);
    cleanup(it->second);if(!finished)it->second.view.phase=Phase::Idle;return true;
}
bool Navigator::canInterrupt(AgentId id,TaskId task) const noexcept {
    const auto it=_agents.find(id);return it==_agents.end() || it->second.view.task!=task || it->second.view.interruptible;
}
std::optional<AgentView> Navigator::state(AgentId id) const {
    const auto it=_agents.find(id);return it==_agents.end()?std::nullopt:std::optional<AgentView>{it->second.view};
}
} // namespace ayt::navigation
