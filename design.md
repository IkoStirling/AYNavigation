# AYNavigation design

**Version:** 0.1.0

**Date:** 2026-10-23

**Status:** Active

**Owner:** Runtime gameplay

## 1. Separation

Space, Planner, Follower, Avoidance, Traversal and Motor are independent virtual contracts. The Navigator
coordinates their lifecycle, validates their outputs, bounds work and owns no physical body,
thread, render context or second clock. Behavior is an optional adapter. The native World motor
is an optional explicit kinematic implementation, not a recommended replacement for physics.

Location combines a nonzero region with position. Cross-region distance is not Euclidean;
GraphPlanner uses authored nonnegative integer costs (strictly positive for full links).
Profiles express clearance, speed/acceleration, arrival thresholds, capability bits and
arbitrary-up planar versus spatial steering. Additional final heading/docking checks live in
Motor::acceptArrival. Region IDs must correspond to the host's actual topology authority.

## 2. Immutable spaces and routes

Stamp identifies one space and a nonzero increasing revision. publish accepts a newer version
of the same identity; the same shared snapshot is idempotent. Space implementations must be
immutable while retained; there is no sandbox for a backend mutating hidden state.

GraphSpace compile sorts IDs and validates references, unique IDs, finite coordinates,
clearances, cost and capacity agreement for shared resources. Maximum10000nodes/50000links,
cost<=10^12 per link, width/height<=10^9; coordinates/velocities are bounded to +/-10^12.
Ordinary links are straight capsules with usable radius=width/2-agent.radius; their endpoints
share a region. Special links use exact endpoints, optional capabilities and a traversal kind.
Control points/curves are represented by explicit connected links in v0.1.

Graph contains allows exact authored nodes or eligible ordinary corridors. nearest is an
explicit goal projection with a maximum distance; it never authorizes an off-network start.
Interior requests attach along the directed edge. The solver seeds reachable tails, supports
interior goals and direct forward travel on the same edge, then reconstructs stable Dijkstra
predecessors. Positive costs prevent predecessor cycles. Search is bounded by expansionBudget.

Every accepted route binds request ID, stamp, goal, continuity, step count and Space::accepts.
Graph steps must match current link metadata and progress direction; special steps must match
the authored endpoints. A custom planner cannot invent a corridor, fake a special action or
override reservations. A custom Space is the authority for its own geometry and must validate
whole swept movements. For a straight capsule/box/sphere, convexity makes endpoint membership
sufficient; this does not generalize to arbitrary custom nonconvex geometry.

## 3. Navigator state and planning

Agents have stable nonzero caller identities; Entity adapters use full generation handles.
Task IDs increase per agent; restarting the same cancelled goal/task is permitted for behavior
resume. A lower task or a changed goal under the same task is stale. Each planning attempt gets
a new session-wide RequestId. poll may return Pending; mismatched IDs are ignored until the
planning budget expires. Cancellation closes the exact request; no asynchronous thread may
mutate the Navigator. Polling/ingress belongs to the owner thread.

Updates sample real motion first. Found routes start following on a later update. A new space
revision cancels ordinary planning/following and starts from the actual sampled position.
A started special traversal keeps its owned step/frozen route snapshot through landing, then
replans on the latest space. Deleting the landing area can therefore fail after a real landing;
the session does not teleport to repaired content. Suspend/resume cancels and replans; it never
blindly restores a cursor into an old route. copied AgentView exposes progress and diagnostics.

Default budgets:4096agents,4096route steps,65536expansions,120planning polls,120blocked updates,
16replans, dt=1/30. dt must be positive finite<=1. Custom long operations/large worlds must
configure budgets intentionally. No checkpoints/serialization/replay closure are supplied.

## 4. Following, avoidance and actual authority

Each ordinary step approaches its endpoint using bounded acceleration and stopping-distance
speed in the default BrakingFollower, stopping at corners conservatively. A custom Follower
can supply guidance and retain velocity across intermediate nodes. Intermediate advance still
requires actual endpoint proximity; final advance always checks sampled low speed and motor
approval. The actual sweep remains independently validated. Completion checks region/distance/speed,
plus the optional final motor condition. Drive acceptance is not completion. A completed special
last step transitions to an ordinary final hold to evaluate arrival without replaying traversal
side effects. RootKinematicMotor writes checked native root poses and records commanded velocity;
it is explicit, has no presentation interpolation and requires exclusive pose ownership.

ObstacleSource returns complete owned region-isolated sphere candidates. IDs are unique in the
snapshot and share physical-agent identity for self exclusion. At most65536 candidates; finite
centers/velocities and radius0..10^6 are validated. Failed/invalid queries fail the operation.
For local safety the final gate validates speed (allowing gradual deceleration from overspeed),
acceleration, planar projection, full space sweep and closed relative sphere sweep over dt.

ConservativeAvoidance tries desired, slower, lateral and optional spatial candidates, with
agent-ID side preference. This sampling is local and does not prove global convergence or
long-term crowd safety. No feasible candidate invalidates the old motor command and reports
Blocked; actual emergency stopping/collision remains the motor's responsibility. Waiting and
non-progress trigger bounded replanning. The default graph does not incorporate obstacle costs,
so a repeated route may stay blocked; publish topology/cost changes or supply a custom planner.

## 5. Traversal and reservations

One shared registry maps nonzero traversal kinds to executors. Planning filters unregistered
kinds; acceptance also calls the executor's capability check. A traversal starts only at its
entry with sufficiently low speed, and succeeds only after the motor confirms its exit.
Callbacks receive owned step data, progress lanes, dt and actual motor state. Each update
reports interruptibility. TimedArcTraversal is explicitly a kinematic sample using profile.up
and Motor::traversalPose, with no physical jump collision/ballistics/animation claim.

Resource/capacity reserves a link before entry. Opposite directions may share one resource.
One operation holds at most one resource; it releases before the next. Cancellation, failure,
completion, detach and Navigator destruction release matching(agent,task) ownership. This
prevents hold-and-wait across links, but provides no fairness/starvation/crowd-deadlock promise.
Resource identity is stable across revisions; deleted resources remain held until old traversal
completion. Publishers must not reuse an ID for a different semantic resource.

## 6. Behavior and failure boundaries

NavigationExecutor owns no navigator; the host must keep Navigator/Motor/ObstacleSource alive
through Runtime or World shutdown. Its region-aware order owns an eight-byte region and Point3
target. The in-process layout is not a stable wire/asset format. The optional compatibility
MotionPort uses one explicitly configured destination region.

AYBehavior v0.2 stores the last committed StepResult.interruptible. Both authored and dynamic
permissions must allow ordinary Replace/Interrupt/Stop. A rejected command is not deferred;
Append or caller retry is explicit. Unit removal and shutdown still mandate cleanup. Direct
Navigator::cancel is also a mandatory cleanup API; ordinary callers check canInterrupt first.
The custom traversal/motor owns emergency recovery; the runtime cannot undo external effects.

Mutation callbacks are guarded against reentrancy; step/plugin exceptions fail the operation
and release its resources. noexcept cancel/stop contracts are programming obligations. A failed
operation retains copied diagnostics and can be replaced with a newer task. Allocation follows
normal C++ behavior; there is no arbitrary external side-effect transaction.

## 7. Acceptance and planned work

Core fixtures exercise custom sphere-space/dogleg planning, stale/Pending planners, forged routes,
clearance/capabilities, interior direction, path revision while moving/in air, occupancy cleanup,
2D/3D steering, braking/final conditions, moving sphere sweep and a blocked narrow corridor.
Behavior fixtures exercise real interruption windows and both adapters; World fixtures exercise
actual fixed-step scheduling, lifetime and region handling. Windows smoke drives real editing/
navigation messages and actual GDI paint. No production GPU, physics jump/flight, cross-platform,
large-crowd/performance, Release or deterministic qualification is inferred.

Future backends: NavMesh/grid/voxel, curve sampling, physical traversal/controllers, authoritative
connected-space transfer adapters, editor asset/cook pipeline, richer shapes/crowd strategies,
orientation-aware planning and registered deterministic state.
