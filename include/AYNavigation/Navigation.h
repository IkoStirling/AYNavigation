#pragma once
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace ayt::navigation {
using AgentId=std::uint64_t; using TaskId=std::uint64_t; using RequestId=std::uint64_t;
using RegionId=std::uint64_t; using NodeId=std::uint64_t; using LinkId=std::uint64_t;
using TraversalKind=std::uint32_t;
enum class Error { None, Invalid, Duplicate, Missing, Unsupported, NoPath, Budget, Busy,
    InvalidRoute, MotionFailed, QueryFailed, Blocked, Stale, Capacity, TraversalFailed };
template<class T> struct Result {
    Error error=Error::None; T value{};
    explicit operator bool() const noexcept {return error==Error::None;}
};
struct Vec3 {double x=0,y=0,z=0; bool operator==(const Vec3&) const=default;};
Vec3 operator+(Vec3,Vec3); Vec3 operator-(Vec3,Vec3); Vec3 operator*(Vec3,double);
double dot(Vec3,Vec3); double length(Vec3); Vec3 normalized(Vec3); bool finite(Vec3);
struct Location {RegionId region=0;Vec3 position;bool operator==(const Location&) const=default;};
struct Stamp {std::uint64_t space=0,version=0;bool operator==(const Stamp&) const=default;};
enum class Steering {Spatial,Planar};
struct Profile {
    double radius=.25,height=1.8,maxSpeed=3,maxAcceleration=12;
    double arrivalDistance=.025,arrivalSpeed=.05;
    std::uint64_t capabilities=0;
    Steering steering=Steering::Spatial;
    Vec3 up{0,1,0};
};
bool valid(const Profile&);
struct Step {
    LinkId link=0;TraversalKind traversal=0;
    Location from,to;
    double halfWidth=0;
    std::uint64_t resource=0;
    std::uint32_t capacity=0; // zero resource/capacity means no reservation
};
struct Route {Stamp stamp;Location goal;std::vector<Step> steps;};

/// Immutable navigability authority. Custom grids/surfaces/volumes/topologies implement this
/// independently of the planner. permits must validate the complete swept movement, not endpoints alone.
class Space {
public:
    virtual ~Space()=default;
    virtual Stamp stamp() const noexcept=0;
    virtual bool contains(Location,const Profile&) const=0;
    virtual bool accepts(const Step&,const Profile&) const=0;
    virtual bool permits(const Step&,Location from,Location to,const Profile&) const=0;
};
struct PlanRequest {
    RequestId id=0;AgentId agent=0;TaskId task=0;
    Location from,goal;Profile profile;
    std::shared_ptr<const Space> space;
    std::vector<TraversalKind> traversals;
    std::size_t expansionBudget=65536;
};
enum class PlanStatus {Pending,Found,Unreachable,Failed};
struct PlanResult {RequestId request=0;PlanStatus status=PlanStatus::Pending;Route route;Error error=Error::None;};
/// Replaceable synchronous or owner-polled asynchronous planner. Returned request ID must match;
/// late mismatched results are ignored. Retain only owned request/snapshot data. cancel is noexcept.
class Planner {
public:
    virtual ~Planner()=default;
    virtual PlanResult poll(const PlanRequest&)=0;
    virtual void cancel(RequestId) noexcept {}
};
struct MotionState {Location location;Vec3 velocity;};
struct FollowQuery {
    AgentId agent;const MotionState& motion;const Profile& profile;const Step& step;
    const Step* next=nullptr;double seconds=0;bool final=false;
};
struct FollowResult {Vec3 velocity;bool advance=false;bool brakeOnAdvance=true;};
/// Replaceable path guidance and intermediate progression policy. Intermediate advance still
/// requires endpoint proximity; final/special/capacity boundaries also require real low speed.
/// Final arrival additionally requires motor approval.
/// A continuous-flight follower can retain motion across ordinary nodes with brakeOnAdvance=false.
class Follower {
public:
    virtual ~Follower()=default;
    virtual FollowResult follow(const FollowQuery&) const=0;
};
/// Default conservative guidance: acceleration-aware approach and a low-speed stop at every node.
class BrakingFollower final:public Follower {
public:
    FollowResult follow(const FollowQuery&) const override;
};
/// Borrowed authoritative movement owner. drive receives a velocity feasible under the configured
/// profile, plus the route target; completion is confirmed from later sample(), never enqueue success.
class Motor {
public:
    virtual ~Motor()=default;
    virtual std::optional<MotionState> sample(AgentId) const=0;
    virtual bool drive(AgentId,TaskId,Vec3 velocity,Location target,double seconds)=0;
    virtual void stop(AgentId,TaskId) noexcept=0;
    /// Additional final heading/docking/authority condition, evaluated after position and speed.
    /// A false result keeps the task active; the motor can converge orientation on later drives.
    virtual bool acceptArrival(AgentId,TaskId,Location,const Profile&) const {return true;}
    /// Explicit opt-in kinematic special movement. Default unsupported; physics games provide their
    /// own Traversal/controller instead. Region migration must use the actual space authority.
    virtual bool traversalPose(AgentId,TaskId,Location) {return false;}
};
struct Obstacle {std::uint64_t id=0;Location location;Vec3 velocity;double radius=0;};
struct ObstacleSnapshot {bool valid=true;std::vector<Obstacle> obstacles;};
/// Complete region-isolated candidate snapshot supplied by the simulation owner, including static
/// geometry approximations and moving neighbors. Query failure must not be reported as an empty success.
class ObstacleSource {
public:
    virtual ~ObstacleSource()=default;
    virtual ObstacleSnapshot query(AgentId,const MotionState&,double range) const=0;
};
struct AvoidanceQuery {
    AgentId agent;const MotionState& motion;const Profile& profile;const Step& step;
    const Space& space;Vec3 desiredVelocity;double seconds;
    std::span<const Obstacle> obstacles;
};
enum class AvoidanceStatus {Clear,Wait,Failed};
struct AvoidanceResult {AvoidanceStatus status=AvoidanceStatus::Clear;Vec3 velocity;};
/// Replaceable local avoidance policy. Navigator independently rechecks speed/acceleration,
/// planar constraints, swept space validity and obstacle clearance before publishing a drive.
class Avoidance {
public:
    virtual ~Avoidance()=default;
    virtual AvoidanceResult choose(const AvoidanceQuery&) const=0;
};
class ConservativeAvoidance final:public Avoidance {
public:
    AvoidanceResult choose(const AvoidanceQuery&) const override;
};
/// Shared safety predicate for candidate solvers and the Navigator's final validation.
bool feasible(const AvoidanceQuery&,Vec3 velocity);

struct TraversalMemory {std::array<std::uint64_t,8> words{};std::array<double,8> values{};};
struct TraversalContext {
    AgentId agent;TaskId task;const Step& step;const Profile& profile;
    const MotionState& motion;Motor& motor;TraversalMemory& memory;
    std::uint64_t elapsedSteps;double seconds;
};
enum class TravelStatus {Running,Waiting,Succeeded,Failed};
struct TravelResult {TravelStatus status=TravelStatus::Running;bool interruptible=false;};
/// Custom jump/vault/climb/portal/docking executor. No universal gravity or animation is assumed.
/// Report safe interrupt windows; mandatory cancellation must release only the matching operation.
class Traversal {
public:
    virtual ~Traversal()=default;
    virtual bool accepts(const Step&,const Profile&) const=0;
    virtual TravelResult update(TraversalContext&,bool starting)=0;
    virtual void cancel(TraversalContext&) noexcept {}
};
/// Optional explicit kinematic demonstration of an arc, not a collision-tested physical jump.
class TimedArcTraversal final:public Traversal {
public:
    TimedArcTraversal(std::uint32_t steps,double height);
    bool accepts(const Step&,const Profile&) const override;
    TravelResult update(TraversalContext&,bool starting) override;
private:
    std::uint32_t _steps;double _height;
};
enum class Phase {Idle,Planning,Following,WaitingForLink,Traversing,Blocked,Succeeded,Failed};
struct Update {TravelStatus status=TravelStatus::Running;Error error=Error::None;bool interruptible=true;};
struct AgentView {
    AgentId agent=0;TaskId task=0;RequestId request=0;
    Phase phase=Phase::Idle;Error error=Error::None;
    bool interruptible=true;std::size_t step=0;std::uint32_t replans=0,blockedSteps=0;
    Route route;
};
struct Config {
    double secondsPerStep=1.0/30.0;
    std::size_t maxAgents=4096,maxRouteSteps=4096,expansionBudget=65536;
    std::uint32_t maxPlanningSteps=120,maxBlockedSteps=120,maxReplans=16;
};

/// Single-owner navigation session with borrowed Motor/ObstacleSource and owned immutable space,
/// planner, follower, avoidance and traversal plugins. Uses actual sampled motion, bounded planning/replanning,
/// per-link capacity, route-version checks and token-specific cleanup. No implicit tick/thread owner.
class Navigator {
public:
    Navigator(std::shared_ptr<const Space>,std::shared_ptr<Planner>,std::shared_ptr<const Avoidance>,
              Motor&,const ObstacleSource&,Config={},std::shared_ptr<const Follower> follower={});
    ~Navigator();
    Navigator(const Navigator&)=delete;Navigator& operator=(const Navigator&)=delete;
    Error attach(AgentId,Profile={});
    Error detach(AgentId);
    Error registerTraversal(TraversalKind,std::shared_ptr<Traversal>);
    /// Publish a strictly newer immutable revision of the same space. Ordinary routes replan;
    /// an executing special traversal finishes using its frozen step, then replans if needed.
    Error publish(std::shared_ptr<const Space>);
    bool available(AgentId) const;
    bool canBegin(AgentId,Location goal) const;
    Error begin(AgentId,TaskId,Location goal);
    /// One active update per caller's fixed step. A motor may apply later; sampled arrival decides.
    Update update(AgentId,TaskId);
    /// Idempotent token-specific cancellation. Old task IDs never cancel the current task.
    bool cancel(AgentId,TaskId) noexcept;
    bool canInterrupt(AgentId,TaskId) const noexcept;
    std::optional<AgentView> state(AgentId) const;
private:
    struct Agent {
        Profile profile;AgentView view;Location goal;
        MotionState observed;std::optional<PlanRequest> request;
        std::shared_ptr<const Space> routeSpace;
        TraversalMemory memory;std::uint64_t elapsed=0;
        bool traversalStarted=false,reserved=false;
        double lastDistance=1e100;
    };
    Config _config;std::shared_ptr<const Space> _space;
    std::shared_ptr<Planner> _planner;std::shared_ptr<const Avoidance> _avoidance;
    std::shared_ptr<const Follower> _follower;
    Motor& _motor;const ObstacleSource& _obstacles;
    std::map<AgentId,Agent> _agents;
    std::map<TraversalKind,std::shared_ptr<Traversal>> _traversals;
    std::map<std::uint64_t,std::map<AgentId,TaskId>> _reservations;
    RequestId _lastRequest=0;mutable bool _busy=false;
    void cleanup(Agent&) noexcept;
    Update fail(Agent&,Error);
    Error plan(Agent&);
    bool validateRoute(const Agent&,const Route&) const;
    bool reserve(Agent&,const Step&);
    void release(Agent&) noexcept;
    TraversalContext context(Agent&,const Step&);
    Update advance(Agent&);
};
} // namespace ayt::navigation
