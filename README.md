# AYNavigation

**Version:** 0.4.0

**Date:** 2026-10-24

**Status:** Active — experimental public API

**Owner:** Runtime gameplay

可替换导航空间、规划算法、路径跟随、特殊通行、局部避障与实际运动控制。
独立本地 Git 子仓库，无远端；主仓记录 gitlink，跨机器需单独携带仓库或 bundle。
根构建默认 `AY_ENABLE_NAVIGATION=OFF`。Core 仅依赖 C++23 标准库。

| Target | Scope |
|---|---|
| `AYNavigation::Core` | Immutable spaces, default planners, navigation session, traversal and avoidance contracts |
| `AYNavigation::Authoring` | Optional explicit versioned .nvg graph files, validation and replacement save |
| `AYNavigation::Behavior` | Optional AYBehavior v0.2 orders/executor and MotionPort compatibility |
| `AYNavigation::Entity` | Optional explicit native root kinematic motor over AYBehavior::Entity / AYEntityCore |
| `AYNavigation::Physics` | Optional borrowed Jolt capsule motor and real grounded jump traversal; defaults OFF |
| `AYNavigation::WorldAdapter` | Optional current Scene World static Box extraction and Host physics ray visibility; defaults OFF |

## Static walkable bake and NPC perception

`StaticBakeWorld` converts bounded horizontal floors, planar ramp rectangles and static blocker
boxes into an immutable `GraphSpace` revision. The caller extracts these shapes from Scene
content, supplies persistent region IDs and increasing content versions, and publishes the
new graph to `Navigator::publish` at a quiescent frame boundary. A failed bake retains the
previous graph. Unloading one region removes its nodes and disables its cross-region links;
`navigationSpace()` also supplies a versioned denying `Space` before the first region and
after the last unload, so a live Navigator can discard routes into removed content. Its
initial empty revision is 1; the first successful bake advances the revision.
`regionContentVersion(region)` reads the published region token and returns
empty after unload, so a stream integration can reject an already-owned bake.
reloading the region restores links declared with `setConnections`. Each directed connection
names a `TraversalKind` that the caller must register with Navigator and implement with the
real connected-space movement authority.
An updated region recalculates only its own walkable cells and local links;
unchanged region fragments are shared across revisions. The complete graph is
still recompiled for atomic publication. Nodes and local links in unchanged
resident regions retain their IDs when another region or its connections change;
routes still use the new graph stamp and must be replanned. `BakeReport::geometryRebakes` reports
how many region geometries were recalculated in the last publication.
For current Scene geometry, `WorldBakeAdapter::refreshRegion` extracts the
region and advances its content version only when floor, ramp or blocker geometry
changes. Repeating an unchanged refresh returns the existing graph; failed
extraction or bake leaves that graph live. Call it at the quiescent boundary
after static collider edits, then publish a changed graph to Navigator.
One region may contain one primary horizontal floor and up to 16 additional flat
floors or planar ramps. Each surface has an independent grid; an authored directed
same-region `BakeConnection` with a registered traversal kind joins selected levels.
Overlapping flat floors at the same height, and any ramp overlapping another
surface in XZ interior, are rejected before publication. Ordinary ramp links
respect `BakeConfig::maxRisePerHorizontal`; use Spatial steering and a motor
that can follow the slope because Planar steering rejects vertical velocity.

`PerceptionMemory` receives stable target IDs, target feet/aim points, current-step sound
stimuli, and a Host-owned visibility query. It returns a generic Chase, Search or Return
goal; a game or AYBehavior owner chooses command priority and interruption. Sight requires
range, cone and successful occlusion query. Sound has a bounded audible radius and event
identity. Old memories expire by fixed tick; `forgetRegion` removes unloaded-region targets.
When a stable target is permanently removed inside a resident region, call
`forgetTarget` and cancel any active chase/search command for it; this preserves
other targets and heard-event receipts, so old sound IDs cannot revive it.
An event is marked heard only after the listener is inside its region and radius;
a still-present event can be heard once after the listener approaches it.
It also clears sound-event deduplication owned by that region, so a reloaded
region can deliver its current sound facts without waiting for old IDs to age out.
Failed visibility queries leave memory unchanged. No Entity or Physics handles are retained.
`snapshot` copies the fixed-tick target records and heard-event receipts for save/restore;
`restore` accepts only schema 1 with an identical `PerceptionConfig`, valid bounded
records and tick order, publishing the whole state atomically. Restore before the next
fixed tick, then call `forgetRegion` for regions that are not resident.
The optional `SoundTravelQuery` replaces the same-region Euclidean test with a
Host-supplied nonnegative route distance. A missing route remains unheard; a
failed query aborts the whole update without consuming any sound event.
`AYNavigation::WorldAdapter` provides `ConnectedSpaceSoundTravel` over the live
`ConnectedSpaces` topology. It chooses the shortest directed route through
enabled portal centers, including multi-hop paths and same-region shortcuts.
Call it at a quiescent topology boundary; supply a `RegionAvailable` predicate
(for example, `WorldRegionStream::isLoaded`) to exclude unloaded intermediate
regions. Its portal budget defaults to 128 and exhaustion is a query error.
An optional pure `PortalCost` callback supplies a finite nonnegative equivalent
distance for each directed crossing. The shortest path includes that cost, so
the Host can represent an acoustically resistant door without changing its
movement connection. Invalid costs fail the query and do not consume events.
This is still a range approximation: opening area, walls, diffraction and
frequency-dependent attenuation are not modeled. Sound event production and NPC task policy remain
with the Host/game.

Run `AYNavigationSpatialAwarenessExample` for a small baked 3D room where one simulated NPC
follows chase, search, heard-location and home goals through the existing Navigator.
`AYNavigationSpatialAwarenessTest` covers obstacle detours, region load/unload/revision,
route invalidation, visual and audio memory, query failure and expiration.

This is a layered 2.5D rectangular static-geometry bake, not general NavMesh generation. Scene
geometry extraction, physics ray occlusion, sound emission and special region traversal are
Host adapters. The module does not infer static meshes or generate a polygon NavMesh.
The World adapter extracts one flat floor by default; its span overload accepts
an ordered list of up to 17 persistent static Box IDs, with the first horizontal
and later boxes optionally tilted into supported planar ramps.

Enable `AY_NAVIGATION_BUILD_WORLD_ADAPTER=ON` when `AYEntityCore` is present. At a
quiescent frame boundary **after** a Scene or `WorldRegionStream` has attached its
physics components, pass `scene.world()` to `WorldBakeAdapter` and publish a region
with authored persistent IDs for static Box floors and ramps. Every other static,
non-trigger 3D Box collider in that region is extracted as a conservative blocker
from its composed World matrix. An unsupported static solid shape fails extraction;
the prior graph stays live. The explicit `publishRegion` path requires a new
content version for static edits; `refreshRegion` detects geometry changes instead.
Call `Navigator::publish` after a changed bake, and call
`unloadRegion` after `WorldRegionStream::unload` before issuing new routes. Existing
active agents in the unloaded region must be detached or moved by their World owner.

`PhysicsRayVisibility` borrows a ready Host `IPhysicsQuery`, current World and
`SpacePhysicsRoutes`. It queries only targets in the observer's current region,
filters the ray to that region's layer, recognizes a hit on the target's current
body. When the observer has a current body, `raycastSyncIgnoringBody` excludes
exactly that body so the next blocker or target can be seen. Query backends that
cannot exclude it return an explicit failure. A failed ray is not treated as sight.
The adapter uses the current Scene World through `scene.world()` and never retains
Entity or Physics handles. `AYNavigationWorldAdapterTest` exercises real World
entities, static changes, region removal, and a fake `IPhysicsQuery`; a live Jolt
backend and Scene resource attachment remain Host integration and are not certified
by that test; the Jolt ignored-body behavior has its own AYPhysics test.
`AYNavigationWorldAdapterExample` runs the same World-backed room, updates its
route graph after removing a wall, and feeds `PerceptionMemory` through the
physics-query adapter. Its substitute query makes the executable runnable without
a PhysicsManager; an assembled Host passes its real `physicsQuery()` instead.

## Build and try

From a Windows MSVC developer shell at the engine root:

```powershell
cmake -S AYRuntime/AYNavigation -B out/build/navigation-core-debug -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Debug -DAY_NAVIGATION_BUILD_BEHAVIOR=OFF
cmake --build out/build/navigation-core-debug
ctest --test-dir out/build/navigation-core-debug --output-on-failure
./out/build/navigation-core-debug/AYNavigationDemo.exe
```

The Windows sample is an interactive in-memory path authoring laboratory:

- Mode1: hand-authored three-dimensional corridors and directed jump links.
- Click blank space to add a node; drag a node to move it.
- Select a node, Shift-click another to add a directed corridor; Ctrl+Shift adds a jump link.
- Up/Down changes selected node height; brackets change incident corridor widths.
- Right-click a node to navigate; S stops when the current traversal permits interruption.
- Mode2 tests a horizontal plane; Mode3 tests free volume motion and selectable target height.
- O toggles an obstacle; R resets. With Authoring enabled, F5 saves a .nvg graph and F9 loads it.

`--smoke` drives a hidden real HWND through editing, route following, jump cancellation denial,
planar movement and free 3D arrival. It renders the actual paint code to `AYNavigationDemo.bmp`
in the working directory. This is a GDI projection/sample motor, not an engine GPU scene or
manual desktop acceptance. Cross-platform Core has no Windows dependency.

## Saved graphs and multi-unit World workbench

`AY_NAVIGATION_BUILD_AUTHORING=ON` (default) adds the standard-library Authoring target.
`readGraphFile`/`decodeGraph` return an owned definition plus errors/warnings; inspect success before
compiling/publishing. `writeGraphFile(path,definition)` validates everything and creates fresh;
pass `replace=true` only for an explicitly requested replacement. Failed validation leaves the old
file intact. `.nvg` v1 has a16MiB limit, stable ID ordering and locale-independent numeric round-trip.
Unknown versions/trailing fields reject. No external resource references or running tasks are stored.
See [design.md](design.md) for grammar and publication details.

With Entity, Behavior, Authoring and examples enabled, build/run `AYNavigationWorldDemo`:

```powershell
cmake --build out/build/behavior-world-debug --target AYNavigationWorldDemo
./out/build/behavior-world-debug/AYRuntime/AYNavigation/AYNavigationWorldDemo.exe
```

This assembly owns a real process World with three Spatial entities, native fixed-step
BehaviorSystem, RootKinematicMotor and the actual CameraFrame/projectSelection pipeline.
The GDI viewport is a diagnostic view; no GPU renderer or `.scn` component is installed.

- Drag a box to select; Shift adds, Ctrl toggles. Capture/focus loss cancels the gesture.
- Right-click moves, Shift+right-click appends, Ctrl+right-click interrupts at priority100.
- W appends60ticks of work; S stops each selected unit and reports refusals separately.
- Nearby authored nodes/projected candidates provide distinct reachable goal slots. This is a
  bounded greedy example policy, not a general formation or optimal assignment service.
- The authored merge, narrow path and jump share separate capacity1 resources. Neighbor spheres
  participate in avoidance. O toggles a blocking obstacle, R resets units/queues on the current graph.
- E toggles edit/play when idle. Click adds/selects, drag moves, Shift-click connects,
  Ctrl+Shift-click creates a jump, Up/Down changes height, brackets change widths, Delete removes.
  Invalid edits and edits stranding attached units reject without publication.
- F5 saves as, Ctrl+S saves the opened path, F9 opens, F8 validates and shows warning count/first reason.
  A load rebuilds units/queues; loading/resetting rejects during locked special movement. Invalid
  files retain existing live state. [narrow-jump.nvg](examples/assets/narrow-jump.nvg) is a supplied graph.

Per-unit rows show planning/moving/link wait/blocked/jumping/arrival/failure, queue/suspended count
and latest submission result. The hidden native smoke saves/reloads an edited graph, box-selects,
routes three units, verifies capacity/locked stop and obstacle feedback, and paints the actual
viewport to `AYNavigationWorldDemo.bmp`. Automated rendering is separate from manual desktop/GPU QA.

## Core composition

Compile a `GraphDefinition` into an immutable `GraphSpace`. Nodes have stable ID and
`Location{region,position}`; links have direction, full corridor width, height clearance,
integer cost, required capability bits, optional traversal kind and shared resource/capacity.
Opposite directions require explicit links. Polylines are authored as connected straight links.

Create `Navigator(space,planner,avoidance,motor,obstacles,config,follower)`, then attach an agent profile,
register needed Traversal kinds, begin(agent,task,goal) and call update once per fixed step.
Check errors and use copied state for observation. The navigator owns no thread or clock.
Motor and ObstacleSource are borrowed and must outlive Navigator and behavior shutdown.
Declare services first, navigator next, behavior runtime last; a World facade must shut down
before these borrowed services disappear.
Cancellation preserves the agent's attachment, profile and diagnostics. After behavior cleanup,
the owner must call `detach` for retired agents to reclaim the configured agent capacity.

The boundaries are intentionally replaceable:

- `Space`: location membership, route-step legality and **whole swept motion** legality.
- `Planner::poll`: immediate or Pending results tagged with the exact request ID; cancellation
  and snapshot ownership are explicit. Custom solvers can ignore GraphSpace entirely.
- `Follower`: choose path guidance and intermediate progression. The default stops at nodes;
  a flight policy can pass ordinary nodes continuously, while final arrival remains independently checked.
  Special/capacity boundaries still require low speed; use the provided next-step context to brake.
- `Avoidance`: choose a local velocity from actual motion and an owned obstacle snapshot.
  Final space/speed/acceleration/plane/sphere-sweep checks still apply to custom outputs.
- `Traversal`: implement jump/vault/climb/portal/dock with progress memory, real motor commands,
  safe interrupt windows and mandatory cancellation cleanup.
- `Motor`: sample actual committed location/velocity, submit movement and acknowledge final
  arrival; override acceptArrival for heading, docking or other game conditions.

`GraphPlanner` is bounded Dijkstra with stable cost/ID order, interior corridor starts/goals,
clearance/capability filtering and cross-region special links. `VolumeSpace` is an explicitly
empty set of convex boxes; `DirectPlanner` proposes a straight route accepted by the space.
They are examples/default backends, not a universal NavMesh/voxel/grid solver.

## Avoidance, movement and topology

`ConservativeAvoidance` samples preferred/slower/lateral/spatial velocities. `feasible` checks
the configured speed and acceleration envelope, optional arbitrary-up planar constraint,
the space's complete movement sweep, and relative moving-sphere overlap over one fixed step.
Blocked narrow paths wait or replan within budgets; they cannot escape the authored corridor.
The obstacle owner must provide complete, region-isolated sphere candidates, including real
static geometry approximations and moving neighbors; failed queries fail explicitly.

This is a conservative local sampled policy, not ORCA, a crowd deadlock solution or a long-term
collision guarantee. Infeasible output invalidates the prior motor drive; the real motor owns
emergency braking/physical collision. Global obstacle-aware detours require a suitable custom
planner or an updated graph; replanning the unchanged graph may select the same blocked path.

Arrival requires actual region, position, speed and the motor's optional final condition.
Movement is acceleration/braking-aware; no global up or instantaneous turn is assumed by the
session. Real flight motors supply thrust/rotation/dynamics and may converge over later samples.

Publish newer immutable revisions of the same space. Ordinary routes replan from actual
position; started special movement uses its frozen step to finish, then replans. Planner
requests are unique; mismatched old results never replace a newer route. Stable resource IDs
keep capacity shared across directions/revisions; they must not be reused for another meaning.

## Behavior and native Entity

With both root flags enabled, link `AYNavigation::Behavior`. Register `NavigationExecutor`
under an unused kind>=3 (`kNavigate` defaults to3), attach matching agent IDs, then submit
`navigationOrder(Location,source,priority)`. The owned region payload is an in-process recipe,
not a versioned network/save format. StepResult propagates dynamic interruption into
AYBehavior v0.2. An unsafe jump rejects ordinary replacement/stopping; caller may append or
retry. Mandatory unit deletion still cancels and the traversal owns emergency cleanup.
Resume replans from the actual location. `NavigationMotionPort` also supports existing
MoveExecutor with an explicit destination region.
`begin` retains a task high-water mark. Explicit owner-authorized `resume` can restore an earlier
suspended task only when current navigation is terminal, allocating a fresh planning request.
The behavior adapter uses this for navigation interrupting navigation. Do not route stale async
messages through resume. Cancellation of already finished tasks retains terminal diagnostics.
Queue acceptance uses `canQueue` so a unit can append navigation while in air; actual execution
checks the then-current start. In the workbench, Append candidates predict prior queue goals,
without replacing the real start/route checks when the queued task runs.

`RootKinematicMotor` is an opt-in native Spatial-root mover with exclusive pose ownership.
It uses checked per-step world pose writes without smoothing, physics or character authority.
Legacy region0 maps to a configured nonzero navigation region; actual Entity regions remain
authoritative. Cross-region kinematic pose changes are rejected. Real portals must compose
the engine's connected-space authority in a custom motor/traversal; no duplicate portal
topology or fake dynamic-body migration is installed here.

## Physical movement and GPU scene

Enable `AY_NAVIGATION_BUILD_PHYSICS=ON` with AYPhysics/Jolt and link `AYNavigation::Physics`.
Create controllers through the normal physics facade, cross their creation barrier, construct
`CharacterMotor(physicsWorld)` and call `attach(agent,character,region)`, then use it in Navigator.
The motor borrows live handles; match Profile dimensions/up and use one exclusive input owner.
It samples real completed feet/velocity, removes only grounded inward support bias relative to
ground velocity, and requires physical grounded arrival. It never steps physics or writes Entity poses.
Copy completed feet to Entity after physics; keep the services alive through all cleanup.

`CharacterJumpTraversal` replaces the demo arc on explicit same-region links. It submits one
grounded jump, uses actual airborne state for tangent steering/braking, and aligns only within
a bounded landing error. Real collision, unsupported/wrong landing, queue refusal and timeout can
fail. Forced cancel clears tangent input but leaves airborne gravity/collision running; the host
owns emergency recovery. No pose teleport, animation, portal transfer or variable-gravity route is installed.

[NavigationPlayground](../../Samples/NavigationPlayground/README.md) is a normal GameProject with
a real `.scn`, Jolt controllers, GPU geometry, AYUI HUD/selection box and the existing `.nvg`
authoring/order loop. Build `NavigationPlaygroundApp` using `AY_BUILD_NAVIGATION_PLAYGROUND=ON`
in a3D client profile. This is separate from the GDI diagnostic examples above.

## Evidence and limits

Tests: `AYNavigationCoreTest` (23 cases), `AYNavigationBehaviorTest` (6),
`AYNavigationEntityTest` (3 real World cases), `AYNavigationAssetTest` (4),
`AYNavigationWorldLabTest` (4) and both Windows native smoke tests.
With Jolt enabled, `AYNavigationPhysicsTest` adds4 actual physics cases. The GPU sample has
separate `navigation-playground-core`, `navigation-playground-host` and optional strict D3D11
`navigation-playground-render` gates, with an actual screenshot and grounded-arrival evidence.
The custom-space/custom-planner/custom-avoidance positive fixture uses a sphere domain and
dogleg solver, independently of the graph implementation. Negative fixtures reject forged
routes, stale results, unsafe avoidance, bad snapshots and invalid capacities.

No arbitrary NavMesh/voxel backend, path spline sampler, flight autopilot or animation traversal,
general shape/crowd avoidance, runtime task/save codec, Resource/Cook/Pak registration, network protocol, checkpoint or deterministic
simulation adapter is implemented. Native doubles and World motor are not lockstep authority.
Numbers are finite/bounded; see [design.md](design.md), [engine integration](../../AYDocs/NAVIGATION-SYSTEM.md)
and [SDK contract](../../AYDocs/sdk/contracts/navigation.md) for exact ownership and budgets.
