# AYNavigation

**Version:** 0.1.0

**Date:** 2026-10-23

**Status:** Active — experimental public API

**Owner:** Runtime gameplay

可替换导航空间、规划算法、路径跟随、特殊通行、局部避障与实际运动控制。
独立本地 Git 子仓库，无远端；主仓记录 gitlink，跨机器需单独携带仓库或 bundle。
根构建默认 `AY_ENABLE_NAVIGATION=OFF`。Core 仅依赖 C++23 标准库。

| Target | Scope |
|---|---|
| `AYNavigation::Core` | Immutable spaces, default planners, navigation session, traversal and avoidance contracts |
| `AYNavigation::Behavior` | Optional AYBehavior v0.2 orders/executor and MotionPort compatibility |
| `AYNavigation::Entity` | Optional explicit native root kinematic motor over AYBehavior::Entity / AYEntityCore |

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
- O toggles an obstacle; R resets. Edits are session-only, without an asset codec/editor project.

`--smoke` drives a hidden real HWND through editing, route following, jump cancellation denial,
planar movement and free 3D arrival. It renders the actual paint code to `AYNavigationDemo.bmp`
in the working directory. This is a GDI projection/sample motor, not an engine GPU scene or
manual desktop acceptance. Cross-platform Core has no Windows dependency.

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

`RootKinematicMotor` is an opt-in native Spatial-root mover with exclusive pose ownership.
It uses checked per-step world pose writes without smoothing, physics or character authority.
Legacy region0 maps to a configured nonzero navigation region; actual Entity regions remain
authoritative. Cross-region kinematic pose changes are rejected. Real portals must compose
the engine's connected-space authority in a custom motor/traversal; no duplicate portal
topology or fake dynamic-body migration is installed here.

## Evidence and limits

Tests: `AYNavigationCoreTest` (22 cases), `AYNavigationBehaviorTest` (3),
`AYNavigationEntityTest` (3 real World cases), and Windows `AYNavigationDemoSmoke`.
The custom-space/custom-planner/custom-avoidance positive fixture uses a sphere domain and
dogleg solver, independently of the graph implementation. Negative fixtures reject forged
routes, stale results, unsafe avoidance, bad snapshots and invalid capacities.

No NavMesh bake, grid/voxel backend, path spline sampler, physics jump/flight autopilot,
general shape/crowd avoidance, asset/save codec, network protocol, checkpoint or deterministic
simulation adapter is implemented. Native doubles and World motor are not lockstep authority.
Numbers are finite/bounded; see [design.md](design.md), [engine integration](../../AYDocs/NAVIGATION-SYSTEM.md)
and [SDK contract](../../AYDocs/sdk/contracts/navigation.md) for exact ownership and budgets.
