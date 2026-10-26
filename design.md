# AYNavigation design

**Version:** 0.4.0

**Date:** 2026-10-24

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
begin retains a task high-water mark; lower IDs, a changed goal under the current ID, and an old
high-water ID resubmitted while an earlier task is restored are stale. Explicit owner-authorized
resume permits an earlier task<=high-water only after current work is terminal, preserving that
high-water. The owner validates suspended task/goal identity; resume is never asynchronous ingress.
This supports navigation interrupting navigation, then restoring the earlier behavior task.
Each planning attempt gets
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
Cancellation after Succeeded/Failed performs cleanup but preserves terminal phase/error, so
the behavior executor's automatic failure cleanup does not erase the reason.
Queue preflight canQueue checks live sampled state and goal membership, without requiring the
current position on the graph. The behavior adapter uses it so an in-air unit can append another
navigation order; actual begin/resume still validates the sampled start when that work executes.

## 7. Graph authoring file and World workbench

Optional Authoring does not add filesystem code to Core. Experimental `.nvg` v1 is an ASCII
token text profile (`experimental.navigation-graph`);16MiB maximum,10000nodes/50000links.
Whitespace is space/tab/CR/LF; there are no comments, optional/unknown fields or trailing tokens.
Unsigned integers use decimal, doubles use locale-independent from_chars/to_chars. Stable-ID
sorting plus shortest round-trippable finite double output defines canonical encoding.

```text
AYNavigation.Graph 1
space <spaceId> <revision>
nodes <count>
node <id> <region> <x> <y> <z>
... repeated count times
links <count>
link <id> <from> <to> <width> <height> <cost> <capabilityBits> <resource> <capacity> <traversalKind> <enabled0or1>
... repeated count times
end
```

All GraphSpace validation applies before success; partial/invalid definitions and encodings
are never returned. Errors identify invalid node/link IDs or a graph constraint group; syntax
errors include line location. Warnings identify isolated nodes and directed dead ends. Valid
dead ends are permitted; diagnostics are not a whole-world reachability/physical-safety proof.
First version has no N-1 format; unsupported versions reject rather than silently migrate.
No file dependencies, scene entities, sessions, save state or task cursors are encoded.

Paths are supplied explicitly by the host, without an implied asset root or suffix trust.
The reader bounds size before allocation and requires a complete stable file; hosts keep input
quiescent. There is no root confinement or symlink policy. Writer first validates/encodes all
content, creates an exclusive sibling temporary, writes/flushes/closes, then publishes. Windows
uses MoveFileExW (replace flag only on explicit opt-in); POSIX uses rename for replacement and
link/unlink for atomic fresh publication. Ordinary failures clean up the owned temporary and
leave prior destination intact; no crash durability, multi-file transaction or concurrent-writer
arbitration is promised. Native Windows Unicode path/create/refusal/replace checks are exercised;
physical write/flush/rename faults and POSIX execution are not separately injected/verified.

There is no ResourceManager loader, Cook/Pak or AYEditor handler. Apps explicitly ship/load loose
graphs; existing source path authoring and runtime task checkpoints have separate lifecycles.
The World workbench owns only its exclusive process World and a programmatically assembled
Spatial unit scene, not a serialized `.scn`/Host registration. World fixedUpdate is the sole tick
owner at1/30. CameraFrame/projectSelection and native Entity handles are shared by drawing and
selection. Its GDI UI is a diagnostic frontend, not an engine GPU renderer.

Goal slots use a finite stable-unit greedy policy: nearby authored nodes then projected offsets,
>=.65 separation, native graph planner reachability and exclusion of unselected occupied slots.
This is a sample, not global matching/formation. Author-specified shared merge capacity prevents
the sample's three feeders entering the same funnel together; separate narrow/jump resources
and neighbor obstacles exercise waiting. It cannot guarantee arbitrary crowd deadlock freedom.
Loaded definitions compile before installation, locked traversal refuses replacement, and failed
decode retains live work. Successful installation recreates units/queues; old generation handles
and selection are discarded. Editing while active or stranding an attached unit is refused.

## 8. Acceptance and planned work

Core fixtures exercise custom sphere-space/dogleg planning, stale/Pending planners, forged routes,
clearance/capabilities, interior direction, path revision while moving/in air, occupancy cleanup,
2D/3D steering, braking/final conditions, moving sphere sweep and a blocked narrow corridor.
Behavior fixtures exercise real interruption windows and both adapters; World fixtures exercise
actual fixed-step scheduling, lifetime and region handling. Windows smoke drives real editing/
navigation messages and actual GDI paint. No production GPU, physics jump/flight, cross-platform,
large-crowd/performance, Release or deterministic qualification is inferred.

Future backends: general NavMesh/voxel, curve sampling, further physical/animation traversal, authoritative
connected-space transfer adapters, editor asset/cook pipeline, richer shapes/crowd strategies,
orientation-aware planning and registered deterministic state.

## 9. Optional character physics (v0.3)

AYNavigation::Physics is off by default, with no new dependencies in Core. CharacterMotor borrows
PhysicsWorld3D and live character handles after completed creation. It owns agent/handle/region/token
bindings only. Repeated handles/agents reject; unsupported/missing physical snapshots never fake
attachment. Profile dimensions and initial up must match the capsule. Constant nonzero gravity,
exclusive input and fixed-step ownership are host responsibilities; the sample uses gravityScale1.

drive submits tangent intent, not displacement or a Transform write. sample reads completed feet
and velocity. A grounded Jolt controller retains a downward support velocity bias even at rest;
remove only the inward normal component relative to ground velocity. Preserve airborne velocity
and moving support contributions. Final arrival requires walkable physical support and the actual
position/speed threshold. Controller deletion invalidates reads. Stop is token-specific; cleanup
submits zero tangent input without freezing gravity. Physics admission failure is not physical
completion, and noexcept stop cannot certify emergency braking when the queue/service refuses.

CharacterJumpTraversal proposes a ballistic launch under motor gravity, submits jumpSpeed once,
observes actual airborne state, and uses remaining-height/vertical-speed feedback plus braking
speed bounds for tangent control. On actual landing it permits bounded alignment, then the Navigator
independently checks the sampled exit/speed. Default apex1.6, horizontal speed6, jump speed9,
duration4seconds, landing error.65. Grounded alignment permits ordinary interruption; airborne
updates lock it. No clearance/reachability promise or landing pose correction is made. Obstructions,
failed launch, wrong support/landing or timeout fail. Mandatory cancellation leaves real airborne
motion to physics and host recovery. No cross-region transfer or variable-gravity route policy exists.

The3D sample uses normal GameProject/Scene, Entity fixed pre-physics behavior, Physics fixed barrier,
post-physics Transform copies, and native render extraction/UI. It does not step the Host a second
time. Character creation/reset/load waits for actual completed grounding before attachment. Cleanup
untracks/detaches first, destroys handles while physics is alive, and keeps emptied borrowed service
objects through World teardown. Existing `.nvg` data is reused; no format/type is added. Platform
collision comes from sample-authored unit-box Scene meshes, graph edits do not create geometry, and
neighbor/blocker sphere approximations are owned by the sample. This is not a generic shape collector.

Finite Jolt tests cover real gap/grounded arrival, wall collision, old-stop safety, blocked jump,
mandatory airborne cancellation and stale character lifetime. Separate sample tests cover actual
Scene/multiple controllers/file reload/blocker, standard headless Host, and strict D3D11 geometry/
HUD/camera selection/three grounded arrivals. The GPU gate captures after rendering/UI flush and
before endFrame, then checks PNG completion. Placeholder cubes do not establish animation quality,
and no manual desktop/Release/backend matrix/cross-platform/crowd qualification is inferred.

## 10. Static topology bake and perception (v0.4)

StaticBakeWorld owns versioned authoring input by persistent region ID. Each BakeRegion
describes one primary floor rectangle with an optional sampled nonplanar top,
up to sixteen additional flat floors or planar ramps, and static axis-aligned
blocker boxes. The Host
extracts these from current Scene content; dynamic actors remain in ObstacleSource. The
config bounds cell/link count and fixes cell size, corridor width and agent height. Every
cell center must have a clear rectangular corridor footprint, and every adjacent link must
have a clear swept rectangle through full agent height. The result is a deterministic
directed GraphDefinition compiled by existing GraphSpace. New local node/link IDs are
allocated above the previous graph's IDs; unchanged resident fragments reuse their IDs.
Do not save them as persistent object IDs.
Each region's local node/link fragment is cached by its full authored value.
`publishRegion` rebakes only changed geometry, while `setConnections` and
`unloadRegion` reuse surviving fragments. All fragments are assembled and
GraphSpace is recompiled for every published revision. Unchanged resident region
IDs survive other regions' edits; changed/unloaded regions and authored connection
links may get new IDs. Old immutable graphs remain valid. `BakeReport::geometryRebakes` exposes
the actual recalculation count; it is not a full graph compilation cost metric.
GraphSpace keeps deterministic sorted node/link arrays and uses reserved hash
lookup indices, avoiding per-element tree allocation on each revision. The
1/16/64-region scale test reports elapsed replacement time and a lower bound
for candidate vector memory; it does not promise a frame-time upper bound.

Region publication checks increasing contentVersion. A same-version identical region is
idempotent; changed content with the same/older version is stale. Rebuild happens in a
temporary graph, then atomically replaces the current immutable snapshot with an increasing
Stamp version. Failed validation, blocked portal attachment and budget failure leave the
old graph and region input unchanged. `regionContentVersion` observes only the
committed input token; it returns empty after unload and does not rebuild.
`Navigator::publish` invalidates ordinary routes and replans from sampled actual motion.
`navigationSpace()` supplies the immutable graph or a revisioned empty authority when no
regions remain. The empty authority denies all locations and steps; it starts at revision 1,
and unloading the last region advances it, allowing a running Navigator to reject stale routes.
A started special traversal keeps its frozen step and
must be resolved by its authoritative movement adapter. Unloading a region removes its
nodes; connections referring to it become dormant and reactivate on reload. If the final
region unloads, `current()` becomes null; `navigationSpace()` still publishes a denying
revision. The Host cancels gameplay work or detaches agents according to its own lifecycle.
The optional `refreshRegion` compares observed floor, blocker, ramp and
heightfield geometry
with the last accepted region, ignoring the caller's contentVersion. Equal
geometry returns the current graph without a new stamp. Changed geometry uses
the prior region version plus one (or one for a new region) and enters the same
atomic publish path. A failed refresh leaves the stored version unchanged.

Connections are explicit directed special links whose endpoints snap to a free cell within
one cell size. Their traversal kind/resource/capacity are authored from the same connected
space topology as movement; the bake does not move an Entity or physical body. A real
Traversal must use the existing connected-space authority. The simple bake intentionally
excludes arbitrary mesh NavMesh, exact dynamic obstacle shapes and
incremental tile updates; a region edit rebuilds the bounded graph.
Up to sixteen additional flat floors or planar ramps may be authored in one region.
Each surface receives an independent bounded grid and shares the region's
content version and blockers. Same-height overlapping surfaces are invalid;
different heights stay disconnected unless an authored same-region special
connection snaps to distinct nodes. The World extraction adapter accepts an
ordered list of persistent static Box floor IDs and validates one solid shape
per ID. The primary Box stays horizontal; a secondary tilted Box must project
its upward top onto an axis-aligned XZ rectangle. The top's four heights must
form one plane. A ramp's cell heights are interpolated from those corners;
ordinary links are admitted only within `maxRisePerHorizontal`, with 3D
distance reflected in cost. A ramp cannot overlap another authored surface
in XZ interior. Spatial steering and a compatible motor are required for
ascending ordinary links; Planar steering rejects height-changing motion.
This does not infer stairs or arbitrary mesh surfaces.
An optional `BakeHeightfield` replaces the primary floor's constant top Y.
Its 2..64 by 2..64 row-major finite samples span the full XZ footprint and
stay inside the floor's Y bounds. Spacing must be at least one quarter of the
bake cell size, bounding subcell detail relative to the route grid. Cell
centers use bilinear height; each ordinary link samples sixteen intermediate
positions and checks local rise against `maxRisePerHorizontal`. The sampled
height envelope also enters blocker headroom checks along a link, so a ridge
between equal-height endpoint nodes cannot silently connect them. This is a
bounded terrain grid, not a triangle-mesh NavMesh or exact continuous terrain
collision proof. A Spatial Motor must follow the same authored surface, and
the game/terrain tool supplies height samples because static Box extraction
cannot reconstruct a heightfield.

PerceptionMemory is one-NPC, single-thread state. Sight targets carry stable persistent IDs,
feet/navigation locations and optional aim points. A valid in-range/cone target is visible
only when the Host's current-world occlusion query returns clear. Query failure aborts the
whole update without changing tick, memory or sound receipts. Unique sound event IDs prevent
short-window redelivery from refreshing memory.
Sound receipts are inserted only for actually audible events. An event first
observed out of region or radius remains eligible if the Host supplies that
same event when the listener later enters range. Once heard, its ID is
deduplicated until expiry or region unload.
Heard locations and last-seen feet become
bounded, expiring records; visual observations win over sound in the current tick. The
returned Chase/Search/Return goal is a suggestion for an AYBehavior/NavigationExecutor
command, not authority over game permissions or interruption. After confirmed unload, `forgetRegion` removes
unloaded-region records and their sound-ID receipts without querying transient Entities;
a reloaded region may reuse its sound facts immediately. No Entity or Physics handles are
retained across frames. The standalone sample uses a simulated motor and visibility query;
live Jolt/audio/Scene lifecycle wiring remains Host work.
For a permanently retired target in a still-loaded region, `forgetTarget` removes
only that stable ID's remembered location. Heard-event receipts remain to prevent
redelivery of an old sound from recreating the target; the Host stops new stimuli
and cancels any already-issued chase/search task separately.
`PerceptionMemorySnapshot` is an owned schema-1 save boundary for one NPC. It records
the configured limits, last committed tick, target locations/observation ticks and
heard sound IDs/regions/ticks. Ordered maps make emitted records deterministic.
Restore validates schema, exact config, finite locations, unique nonzero IDs,
retention age and entry budgets in temporary maps before swapping them into the
live memory. A rejected restore keeps the previous tick and records. The Host
persists the snapshot with the NPC's stable identity, restores before advancing
fixed ticks and drops records for regions no longer loaded with `forgetRegion`.
The optional `AYNavigation::PerceptionJson` codec stores memory alone in an
`AliyatPerceptionMemory` schema-1 JSON section capped at 1 MiB. Encode
canonicalizes target/sound order via a temporary validated memory; decode
validates types, finite positions, counts and the full `restore` contract
before replacing the caller output. This codec has no independent file path;
the game owns complete-slot SaveGame composition and NPC identity mapping.
`PerceptionWorldCommit` adds an optional identity-bound candidate. It copies
memory, updates one fixed tick, rechecks the stable NPC ID, Entity generation
and World epoch through a game-supplied preflight, then lets the game save its
whole World slot before publishing memory and tick receipt. The saved decision
is retained with the fingerprint so exact replay can resubmit a Behavior
command without re-sensing; the game uses a stable task ID and its own priority
policy to avoid duplicate commands. The bounded version-1 JSON wrapper records
identity, receipt, last decision and the existing memory section. Decode
validates the whole wrapper in a temporary value; the game restores only after
matching the current NPC and World identities. This is not automatic ECS save.
An optional `SoundTravelQuery` can supply a route distance for each unheard
sound. A successful empty route leaves the event eligible; a failed or invalid
query rolls back the entire fixed-step update. The default remains same-region
Euclidean range. `ConnectedSpaceSoundTravel` uses the live `ConnectedSpaces`
directed enabled portals as a bounded shortest-path graph. Each intra-region
leg runs directly between listener, portal centers and sound; crossing costs
zero, matching the rigid portal connection rather than physical distance
between its two poses. It may find a shortcut back into the source region.
An optional `PortalCost` adds an equivalent-distance crossing penalty to each
directed edge. The callback is evaluated once per eligible portal before route
search; nonfinite or negative costs fail the query, preserving the perception
update's atomicity. The Host may vary costs between fixed-step updates as door
state changes, while keeping the callback pure during an update.
An optional pure residency predicate excludes unloaded endpoints and intermediate
regions. Query only at a quiescent topology boundary; no portal pointer survives
the call. The bounded O(P²) search fails when the eligible portal count exceeds
the configured limit. This does not model aperture, wall occlusion, diffraction,
frequency-dependent attenuation or acoustic material response. A Host may implement `SoundTravelQuery`
with those rules without changing perception memory.
Optional `SightTravelQuery` extends visual memory beyond one region. A successful
one-hop route supplies an apparent aim in the listener's region and a traveled
distance for cone/range ordering; the service itself owns occlusion. An empty
route is invisible, and a failed query rolls back the whole memory update.
`ConnectedSpaceSightTravel` reverse-maps the aim through each eligible direct
opening, then uses `ConnectedSpaces::traceRay` and a region-filtered local ray
callback to confirm the ray actually crosses that opening without a nearer
occluder. It rejects disabled/unloaded destinations, caps candidate portals,
and never assumes Euclidean distance across unrelated region coordinates.
It is one-hop; recursive portal sight remains explicit Host policy.

## 11. Optional Scene World and Physics query adapter

`AYNavigation::WorldAdapter` is an opt-in assembly over public Entity World and
Physics query interfaces. It accepts `scene.world()` after Scene or RegionStream
attachment on the owner thread. A floor is identified by its persistent Entity ID
and must have exactly one static, non-trigger 3D Box shape. The first ID must
have a horizontal top; later IDs may describe the bounded planar ramp above.
All other static non-trigger 3D Box colliders in the same effective region become
blocker AABBs, transformed through their composed World matrix. Rotated blocker
boxes use conservative world AABBs; non-Box static solids reject the whole bake
because silently omitting them would publish unsafe routes. Dynamic bodies stay
in `ObstacleSource`. Region/version publication and rollback remain owned by
`StaticBakeWorld`; unload the corresponding graph region after WorldRegionStream
has removed its resident content, then detach or move affected agents.
`WorldBakeAdapter::refreshRegion` extracts current static geometry and lets
`StaticBakeWorld` assign the next version only when it differs. The Host still
calls it after collider edits; World changes are not watched automatically.

`PhysicsRayVisibility` borrows the Host's ready `IPhysicsQuery`, current World,
and `SpacePhysicsRoutes`. It resolves target identity afresh each query, casts only
within one region's physics layer, accepts a current target-body hit, and fails
closed on query error, missing identity/route, or an incorrectly returned
observer self-hit. `raycastSyncIgnoringBody` excludes one exact generation-checked
observer body in Jolt; unsupported query backends fail closed. The one-hop
portal sight adapter uses a supplied per-region ray callback; adapting actual
Jolt filters for both observer and target, sensor scheduling, sound emission,
live Jolt integration tests, and arbitrary mesh
geometry extraction remain separate Host/adapter work. The focused adapter test
uses real World entities and a fake `IPhysicsQuery` to validate the query contract;
AYPhysics tests cover the actual nearest-hit skip in Jolt.
