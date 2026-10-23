#pragma once
#include <AYNavigation/Graph.h>
#include <algorithm>
#include <functional>
#include <stdexcept>

namespace navtest {
using namespace ayt::navigation;
struct Motor final:ayt::navigation::Motor {
    std::map<AgentId,MotionState> bodies;
    std::map<AgentId,TaskId> tokens;
    std::vector<Vec3> commands;
    bool rejectDrive=false,rejectPose=false;
    unsigned stopped=0;
    unsigned arrivalDelay=0;mutable unsigned arrivalChecks=0;
    std::function<void()> callback;
    std::optional<MotionState> sample(AgentId id) const override {
        const auto it=bodies.find(id);return it==bodies.end()?std::nullopt:std::optional<MotionState>{it->second};
    }
    bool drive(AgentId id,TaskId task,Vec3 velocity,Location target,double seconds) override {
        if(callback) callback();
        if(rejectDrive || !bodies.contains(id) || bodies.at(id).location.region!=target.region || (tokens[id] && task<tokens[id])) return false;
        tokens[id]=task;commands.push_back(velocity);auto& body=bodies.at(id);
        body.location.position=body.location.position+velocity*seconds;body.velocity=velocity;return true;
    }
    void stop(AgentId id,TaskId task) noexcept override {
        const auto it=tokens.find(id);if(it!=tokens.end() && it->second==task) {
            if(bodies.contains(id)) bodies.at(id).velocity={};tokens.erase(it);++stopped;
        }
    }
    bool acceptArrival(AgentId,TaskId,Location,const Profile&) const override {return ++arrivalChecks>arrivalDelay;}
    bool traversalPose(AgentId id,TaskId task,Location location) override {
        if(rejectPose || !bodies.contains(id) || (tokens[id] && task<tokens[id])) return false;
        tokens[id]=task;bodies.at(id)={location,{}};return true;
    }
};
struct Obstacles final:ObstacleSource {
    ObstacleSnapshot snapshot;
    ObstacleSnapshot query(AgentId,const MotionState&,double) const override {return snapshot;}
};
inline GraphDefinition graph(std::uint64_t version=1) {
    GraphDefinition d;d.stamp={1,version};
    d.nodes={{1,{1,{0,0,0}}},{2,{1,{3,0,0}}},{3,{1,{6,1,0}}},{4,{1,{9,1,0}}}};
    d.links={{1,1,2,2,3,3},{2,2,3,2,3,3},{3,3,4,2,3,3}};return d;
}
inline Profile profile() {Profile p;p.maxSpeed=2;p.maxAcceleration=10;p.arrivalDistance=.03;p.arrivalSpeed=.05;return p;}
inline Config config() {Config c;c.secondsPerStep=.05;c.maxBlockedSteps=20;c.maxReplans=3;return c;}
inline PlanRequest request(std::shared_ptr<const Space> space,Location from,Location to) {
    PlanRequest r;r.id=1;r.agent=1;r.task=1;r.space=std::move(space);r.from=from;r.goal=to;r.profile=profile();return r;
}
inline Update finish(Navigator& navigator,AgentId id=1,TaskId task=1,int maximum=2000) {
    Update result;
    for(int i=0;i<maximum;++i) {result=navigator.update(id,task);if(result.status==TravelStatus::Succeeded || result.status==TravelStatus::Failed) return result;}
    throw std::runtime_error("Navigation did not finish within the fixture budget");
}
}
