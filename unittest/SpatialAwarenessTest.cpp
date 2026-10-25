#include <AYNavigation/Perception.h>
#include <AYNavigation/StaticBake.h>

#include <array>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace ayt::navigation;

namespace {
void check(bool value, const char* label)
{
    if (!value) throw std::runtime_error(label);
}

BakeRegion room(RegionId id, double minX, double maxX)
{
    return {id, 1, {{minX, -1.0, 0.0}, {maxX, 0.0, 6.0}}, {}};
}

PlanResult route(std::shared_ptr<const GraphSpace> graph,
                 Location from, Location to,
                 std::vector<TraversalKind> traversals = {},
                 Steering steering = Steering::Planar)
{
    PlanRequest request;
    request.id = 1;
    request.agent = 1;
    request.task = 1;
    request.from = from;
    request.goal = to;
    request.profile.steering = steering;
    request.space = std::move(graph);
    request.traversals = std::move(traversals);
    return GraphPlanner{}.poll(request);
}

struct StationaryMotor final : Motor {
    MotionState motion{{1, {0.5, 0.0, 2.5}}, {}};
    std::optional<MotionState> sample(AgentId) const override { return motion; }
    bool drive(AgentId, TaskId, Vec3, Location, double) override { return true; }
    void stop(AgentId, TaskId) noexcept override {}
};
struct NoObstacles final : ObstacleSource {
    ObstacleSnapshot query(AgentId, const MotionState&, double) const override
    { return {true, {}}; }
};

void bakeAndUpdate()
{
    StaticBakeWorld bake(42);
    const auto emptyAtStart = bake.navigationSpace();
    check(emptyAtStart && emptyAtStart->stamp().version == 1
          && !emptyAtStart->contains({1, {0.5, 0.0, 0.5}}, {}),
          "empty bake has a revisioned denying navigation authority");
    auto first = room(1, 0.0, 6.0);
    first.blockers.push_back({{2.0, 0.0, 2.0}, {4.0, 2.0, 4.0}});
    auto compiled = bake.publishRegion(first);
    check(bool(compiled) && compiled.value != nullptr, "first bake");
    check(bake.regionContentVersion(1) == first.contentVersion
          && !bake.regionContentVersion(2),
          "committed region token is readable without changing graph");
    const auto firstNodeId = compiled.value->definition().nodes.front().id;
    const auto firstLink = compiled.value->definition().links.front();
    check(bake.report().nodes > 0 && bake.report().blockedCells > 0
          && bake.report().geometryRebakes == 1,
          "bake diagnostic counts");
    const Location start{1, {0.5, 0.0, 2.5}};
    const Location goal{1, {5.5, 0.0, 2.5}};
    auto detour = route(compiled.value, start, goal);
    check(detour.status == PlanStatus::Found && detour.route.steps.size() > 5,
          "route detours around static geometry");
    auto oldStamp = bake.report().stamp;
    check(bake.publishRegion(first).value == compiled.value
          && bake.report().stamp == oldStamp, "same content idempotent");
    auto sameVersionChanged = first;
    sameVersionChanged.blockers.clear();
    check(bake.publishRegion(sameVersionChanged).error == Error::Stale,
          "changed content needs new version");
    auto invalid = first;
    invalid.contentVersion = 2;
    invalid.blockers.front().minimum.x = 5.0;
    check(bake.publishRegion(invalid).error == Error::Invalid
          && bake.report().stamp == oldStamp,
          "failed bake retains published graph");

    auto second = room(2, 10.0, 16.0);
    auto two = bake.publishRegion(second);
    check(bool(two) && two.value != nullptr
          && two.value->stamp().version > oldStamp.version
          && bake.report().geometryRebakes == 1,
          "second region revision reuses first geometry");
    check(two.value->node(firstNodeId) != nullptr
          && two.value->node(firstNodeId)->location.region == 1
          && two.value->link(firstLink.id) != nullptr
          && two.value->link(firstLink.id)->from == firstLink.from
          && two.value->link(firstLink.id)->to == firstLink.to,
          "unchanged resident region retains graph IDs");
    NodeId secondNodeId = 0;
    for (const auto& node : two.value->definition().nodes)
        if (node.location.region == 2) { secondNodeId = node.id; break; }
    check(secondNodeId != 0, "second region has node ID");
    const BakeConnection portal{1, 2, {5.5, 0.0, 0.5},
                                {10.5, 0.0, 0.5}, 17, 88, 1};
    auto connected = bake.setConnections({portal});
    check(bool(connected) && connected.value != nullptr,
          "cross-region connection compiled");
    check(bake.report().geometryRebakes == 0,
          "connection-only revision reuses all region geometry");
    auto across = route(connected.value, {1, {5.5, 0.0, 0.5}},
                        {2, {10.5, 0.0, 0.5}}, {17});
    check(across.status == PlanStatus::Found, "route crosses connection");
    bool foundPortal = false;
    for (const auto& step : across.route.steps)
        foundPortal |= step.traversal == 17;
    check(foundPortal, "route contains portal traversal step");
    auto unloaded = bake.unloadRegion(2);
    check(!bake.regionContentVersion(2)
          && bake.regionContentVersion(1) == first.contentVersion,
          "unload clears only the removed region token");
    check(bool(unloaded) && unloaded.value != nullptr
          && bake.report().disabledConnections == 1
          && bake.report().geometryRebakes == 0,
          "unload removes topology but retains dormant connection");
    check(!unloaded.value->contains({2, {10.5, 0.0, 0.5}}, {}),
          "unloaded region is not navigable");
    auto reloaded = bake.publishRegion(second);
    check(bool(reloaded) && reloaded.value != nullptr
          && bake.report().disabledConnections == 0
          && bake.report().geometryRebakes == 1,
          "reload restores connection with new stamp");
    check(route(reloaded.value, {1, {5.5, 0.0, 0.5}},
                {2, {10.5, 0.0, 0.5}}, {17}).status == PlanStatus::Found,
          "restored cross-region route");
    for (const auto& node : reloaded.value->definition().nodes)
        if (node.location.region == 2) { secondNodeId = node.id; break; }

    first.contentVersion = 2;
    first.blockers = {{{2.0, 0.0, 0.0}, {4.0, 2.0, 6.0}}};
    auto blocked = bake.publishRegion(first);
    check(bool(blocked) && blocked.value != nullptr
          && bake.report().geometryRebakes == 1,
          "obstacle revision rebakes only changed region");
    check(blocked.value->node(secondNodeId) != nullptr
          && blocked.value->node(secondNodeId)->location.region == 2,
          "editing first region retains second region node ID");
    check(route(blocked.value, start, goal).status == PlanStatus::Unreachable,
          "new static barrier invalidates old route");
    StationaryMotor motor;
    NoObstacles obstacles;
    Navigator navigator(reloaded.value, std::make_shared<GraphPlanner>(),
                        std::make_shared<ConservativeAvoidance>(),
                        motor, obstacles);
    Profile profile;
    profile.steering = Steering::Planar;
    check(navigator.attach(1, profile) == Error::None, "attach agent");
    check(navigator.begin(1, 1, goal) == Error::None, "begin old route");
    bool followingOldRoute = false;
    for (int i = 0; i < 4; ++i) {
        (void)navigator.update(1, 1);
        followingOldRoute = navigator.state(1)->phase == Phase::Following;
        if (followingOldRoute) break;
    }
    check(followingOldRoute, "old route was active before revision");
    check(navigator.publish(blocked.value) == Error::None,
          "publish updated bake to running navigator");
    bool failedAfterReplan = false;
    for (int i = 0; i < 4; ++i) {
        const auto updated = navigator.update(1, 1);
        if (updated.status == TravelStatus::Failed) {
            failedAfterReplan = true;
            break;
        }
    }
    check(failedAfterReplan, "running route invalidated by new static barrier");
    check(bake.unloadRegion(1) && bake.unloadRegion(2),
          "remove final static regions");
    const auto emptyAfterUnload = bake.navigationSpace();
    check(!bake.current() && emptyAfterUnload
          && emptyAfterUnload->stamp().version > blocked.value->stamp().version
          && !emptyAfterUnload->contains(start, profile),
          "last unload retains a newer denying authority");
    check(navigator.publish(emptyAfterUnload) == Error::None
          && !navigator.canBegin(1, goal),
          "live navigator rejects new travel through unloaded content");
}

struct Visibility final : VisibilityQuery {
    bool open = true;
    bool fail = false;
    Result<bool> clear(Location, const SightTarget&) override
    {
        return fail ? Result<bool>{Error::QueryFailed}
                    : Result<bool>{Error::None, open};
    }
};

void visualAudioMemory()
{
    PerceptionMemory memory({10.0, 0.8, 3, 4});
    Visibility visibility;
    const Location eye{1, {0.5, 1.7, 0.5}};
    const Location home{1, {0.5, 0.0, 0.5}};
    const std::array targets{SightTarget{"target-1", {1, {4.5, 1.7, 0.5}}}};
    auto seen = memory.update(1, eye, {1, 0, 0}, targets, {}, home, visibility);
    check(bool(seen) && seen.value.mode == PursuitMode::Chase
          && seen.value.targetPersistentId == "target-1", "visual chase");
    visibility.fail = true;
    check(memory.update(2, eye, {1, 0, 0}, targets, {}, home, visibility).error
              == Error::QueryFailed
          && memory.rememberedCount() == 1,
          "occlusion query failure is atomic");
    visibility.fail = false;
    visibility.open = false;
    auto lost = memory.update(2, eye, {1, 0, 0}, targets, {}, home, visibility);
    check(bool(lost) && lost.value.mode == PursuitMode::Search
          && lost.value.goal == targets.front().location,
          "lost sight searches last known location");
    const std::array sounds{SoundStimulus{100, "target-1",
                                           {1, {5.5, 1.7, 0.5}}, 8.0}};
    auto heard = memory.update(3, eye, {1, 0, 0}, {}, sounds, home, visibility);
    check(bool(heard) && heard.value.mode == PursuitMode::Search
          && heard.value.goal == sounds.front().location,
          "sound refreshes search goal");
    auto duplicate = memory.update(4, eye, {1, 0, 0}, {}, sounds, home, visibility);
    check(bool(duplicate) && duplicate.value.mode == PursuitMode::Search,
          "duplicate sound does not become a new event");
    auto expired = memory.update(7, eye, {1, 0, 0}, {}, {}, home, visibility);
    check(bool(expired) && expired.value.mode == PursuitMode::Return
          && expired.value.goal == home, "memory expiry returns home");
    auto heardAgain = memory.update(8, eye, {1, 0, 0}, {},
        std::array{SoundStimulus{101, "target-1",
                                 {1, {3.5, 1.7, 0.5}}, 8.0}}, home, visibility);
    check(bool(heardAgain) && memory.rememberedCount() == 1,
          "new sound remembered");
    memory.forgetRegion(1);
    check(memory.rememberedCount() == 0, "region unload drops transient memory");
    auto replayed = memory.update(9, eye, {1, 0, 0}, {},
        std::array{SoundStimulus{101, "target-1",
                                 {1, {2.5, 1.7, 0.5}}, 8.0}}, home, visibility);
    check(bool(replayed) && replayed.value.mode == PursuitMode::Search
          && replayed.value.goal.position.x == 2.5,
          "reloaded region may reuse a previously seen sound event ID");
    memory.forgetRegion(1);
    auto afterUnload = memory.update(10, eye, {1, 0, 0}, {}, {}, home, visibility);
    check(bool(afterUnload) && afterUnload.value.mode == PursuitMode::Return,
          "unloaded target cannot be pursued");

    PerceptionMemory approaching({10.0, 0.8, 3, 4});
    const std::array persistentSound{SoundStimulus{102, "distant-target",
        {1, {5.5, 1.7, 0.5}}, 2.0}};
    auto outside = approaching.update(1, eye, {1, 0, 0}, {}, persistentSound,
                                      home, visibility);
    check(bool(outside) && outside.value.mode == PursuitMode::Return,
          "inaudible sound does not enter memory");
    const Location nearEye{1, {4.5, 1.7, 0.5}};
    auto entered = approaching.update(2, nearEye, {1, 0, 0}, {}, persistentSound,
                                      home, visibility);
    check(bool(entered) && entered.value.mode == PursuitMode::Search
          && entered.value.targetPersistentId == "distant-target",
          "same event can be heard after entering its radius");
    auto repeated = approaching.update(3, eye, {1, 0, 0}, {}, persistentSound,
                                       home, visibility);
    check(bool(repeated) && repeated.value.mode == PursuitMode::Search
          && repeated.value.goal == persistentSound.front().location,
          "heard event stays deduplicated");
}
void stackedFloorsNeedExplicitTraversal()
{
    StaticBakeWorld bake(91);
    auto layered = room(1, 0.0, 3.0);
    layered.additionalFloors.push_back({{0.0, 3.0, 0.0}, {3.0, 4.0, 6.0}});
    auto published = bake.publishRegion(layered);
    check(bool(published) && bake.report().nodes == 36,
          "two independent floor grids are baked");
    const Location lower{1, {0.5, 0.0, 0.5}};
    const Location upper{1, {0.5, 4.0, 0.5}};
    check(route(published.value, lower, upper).status == PlanStatus::Unreachable,
          "stacked floors do not connect by projection");
    const BakeConnection stairs{1, 1, lower.position, upper.position, 23, 0, 0};
    auto connected = bake.setConnections({stairs});
    check(bool(connected) && connected.value,
          "same-region authored traversal connects floors");
    const auto planned = route(connected.value, lower, upper, {23});
    check(planned.status == PlanStatus::Found,
          "route reaches upper floor through traversal");
    bool usedStairs = false;
    for (const auto& step : planned.route.steps)
        usedStairs |= step.traversal == 23;
    check(usedStairs, "route records authored stairs traversal");
    const auto before = bake.report().stamp;
    auto duplicate = layered;
    duplicate.contentVersion = 2;
    duplicate.additionalFloors.front() = layered.floor;
    check(bake.publishRegion(duplicate).error == Error::Invalid
          && bake.report().stamp == before,
          "duplicate same-height floor rejects without publication");
}

void planarRampRespectsGradeAndAtomicRevision()
{
    auto region = room(1, 0.0, 2.0);
    region.floor.maximum.z = 2.0;
    region.additionalFloors.push_back({{6.0, 1.0, 0.0}, {8.0, 2.0, 2.0}});
    region.ramps.push_back({{{2.0, -0.5, 0.0}, {6.0, 2.0, 2.0}},
                            0.0, 2.0, 0.0, 2.0});
    const BakeConnection enter{1, 1, {1.5, 0.0, 0.5},
        {2.5, 0.25, 0.5}, 24, 0, 0};
    const BakeConnection leave{1, 1, {5.5, 1.75, 0.5},
        {6.5, 2.0, 0.5}, 25, 0, 0};
    StaticBakeWorld bake(92);
    auto published = bake.publishRegion(region);
    check(bool(published) && bake.report().nodes == 16,
          "flat and sloped surfaces create separate cells");
    auto connected = bake.setConnections({enter, leave});
    const auto start = Location{1, {0.5, 0.0, 0.5}};
    const auto goal = Location{1, {7.5, 2.0, 0.5}};
    check(bool(connected) && route(connected.value, start, goal,
          {24, 25}, Steering::Spatial).status
          == PlanStatus::Found, "route ascends planar ramp");
    const auto before = bake.report().stamp;
    auto nonplanar = region;
    nonplanar.contentVersion = 2;
    nonplanar.ramps.front().y11 = 1.5;
    check(bake.publishRegion(nonplanar).error == Error::Invalid
          && bake.report().stamp == before,
          "nonplanar ramp leaves prior graph intact");
    auto overlap = region;
    overlap.contentVersion = 2;
    overlap.ramps.front().footprint.minimum.x = 1.0;
    check(bake.publishRegion(overlap).error == Error::Invalid
          && bake.report().stamp == before,
          "overlapping surface leaves prior graph intact");
    auto blocked = region;
    blocked.contentVersion = 2;
    blocked.blockers.push_back({{3.0, 0.0, 0.0}, {4.0, 3.0, 2.0}});
    auto blockedGraph = bake.publishRegion(blocked);
    check(bool(blockedGraph) && bake.report().blockedCells > 0
          && route(blockedGraph.value, start, goal,
             {24, 25}, Steering::Spatial).status == PlanStatus::Unreachable,
          "ramp headroom blocker closes the route");
    auto under = region;
    under.contentVersion = 3;
    under.blockers.push_back({{3.0, -2.0, 0.0}, {4.0, -1.0, 2.0}});
    auto underGraph = bake.publishRegion(under);
    check(bool(underGraph) && route(underGraph.value, start, goal,
          {24, 25}, Steering::Spatial).status == PlanStatus::Found,
          "blocker below the ramp leaves headroom clear");

    BakeConfig restricted;
    restricted.maxRisePerHorizontal = 0.25;
    StaticBakeWorld steep(93, restricted);
    check(bool(steep.publishRegion(region)), "restricted bake accepts authored ramp");
    auto restrictedGraph = steep.setConnections({enter, leave});
    check(bool(restrictedGraph) && route(restrictedGraph.value, start, goal,
          {24, 25}, Steering::Spatial).status == PlanStatus::Unreachable,
          "ordinary links reject an excessive grade");
}

void nonplanarHeightfieldBakesAndRejectsHiddenCliffs()
{
    auto region = room(1, 0.0, 4.0);
    region.floor.minimum.y = 0.0;
    region.floor.maximum.y = 2.0;
    region.floor.maximum.z = 1.0;
    region.primaryHeightfield = BakeHeightfield{5, 2,
        {0.0, 0.5, 1.0, 0.5, 0.0,
         0.0, 0.5, 1.0, 0.5, 0.0}};
    StaticBakeWorld bake(94);
    auto graph = bake.publishRegion(region);
    check(bool(graph) && graph.value->definition().nodes.size() == 4
          && graph.value->definition().nodes[1].location.position.y > 0.0,
          "nonplanar samples set primary floor node heights");
    const Location start{1, {0.5, 0.25, 0.5}};
    const Location goal{1, {3.5, 0.25, 0.5}};
    check(route(graph.value, start, goal, {}, Steering::Spatial).status
          == PlanStatus::Found,
          "spatial route crosses a gentle nonplanar heightfield");
    const auto oldStamp = bake.report().stamp;
    auto invalid = region;
    invalid.contentVersion = 2;
    invalid.primaryHeightfield->heights[2] =
        std::numeric_limits<double>::quiet_NaN();
    check(bake.publishRegion(invalid).error == Error::Invalid
          && bake.report().stamp == oldStamp,
          "nonfinite terrain sample rejects without graph publication");
    invalid = region;
    invalid.contentVersion = 2;
    invalid.primaryHeightfield->heights.pop_back();
    check(bake.publishRegion(invalid).error == Error::Invalid
          && bake.report().stamp == oldStamp,
          "incomplete terrain grid rejects without graph publication");
    invalid = region;
    invalid.contentVersion = 2;
    invalid.primaryHeightfield = BakeHeightfield{64, 2,
        std::vector<double>(128, 0.0)};
    check(bake.publishRegion(invalid).error == Error::Invalid
          && bake.report().stamp == oldStamp,
          "terrain detail finer than bake budget rejects atomically");
    auto spike = region;
    spike.contentVersion = 2;
    spike.primaryHeightfield = BakeHeightfield{9, 2,
        {0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    auto blocked = bake.publishRegion(spike);
    check(bool(blocked) && route(blocked.value,
          {1, {0.5, 0.0, 0.5}}, {1, {3.5, 0.0, 0.5}}, {},
          Steering::Spatial).status == PlanStatus::Unreachable,
          "subcell ridge blocks a link with equal endpoint heights");
    auto unchanged = bake.refreshRegion(spike);
    check(bool(unchanged) && bake.report().stamp == blocked.value->stamp(),
          "unchanged terrain refresh retains navigation revision");
    auto restored = bake.refreshRegion(region);
    check(bool(restored) && bake.report().geometryRebakes == 1
          && route(restored.value, start, goal, {}, Steering::Spatial).status
              == PlanStatus::Found,
          "changed terrain refresh republishes local geometry");
}
void retiredTargetDoesNotReplayOldSound()
{
    PerceptionMemory memory({10.0, 0.8, 10, 4});
    Visibility visibility;
    const Location eye{1, {0.5, 1.7, 0.5}};
    const Location home{1, {0.5, 0.0, 0.5}};
    const std::array targets{
        SightTarget{"retired", {1, {4.5, 1.7, 0.5}}},
        SightTarget{"survivor", {1, {5.5, 1.7, 0.5}}},
    };
    const std::array sound{SoundStimulus{300, "retired",
        {1, {3.5, 1.7, 0.5}}, 8.0}};
    check(bool(memory.update(1, eye, {1, 0, 0}, targets, {}, home, visibility)),
          "both targets seen before retirement");
    check(bool(memory.update(2, eye, {1, 0, 0}, {}, sound, home, visibility)),
          "sound receipt exists before retirement");
    check(memory.forgetTarget("retired") && !memory.forgetTarget("retired"),
          "retirement reports whether the target was remembered");
    const auto afterRetirement = memory.snapshot();
    check(afterRetirement.targets.size() == 1
          && afterRetirement.targets.front().persistentId == "survivor"
          && afterRetirement.heardSounds.size() == 1,
          "retirement leaves other targets and heard sound receipt intact");
    auto movedSound = sound;
    movedSound.front().location.position.x = 2.5;
    const auto next = memory.update(3, eye, {1, 0, 0}, {}, movedSound,
                                    home, visibility);
    check(bool(next) && next.value.mode == PursuitMode::Search
          && next.value.targetPersistentId == "survivor",
          "old heard event does not recreate a retired target");
    check(memory.forgetTarget("survivor"), "remaining target can retire");
    const auto empty = memory.update(4, eye, {1, 0, 0}, {}, movedSound,
                                     home, visibility);
    check(bool(empty) && empty.value.mode == PursuitMode::Return,
          "retired targets return to home without replaying sound");
}

void perceptionSnapshotRoundTrip()
{
    const PerceptionConfig config{10.0, 0.8, 5, 4};
    PerceptionMemory original(config);
    Visibility visibility;
    const Location eye{1, {0, 1, 0}};
    const Location home{1, {0, 0, 0}};
    const SoundStimulus sound{41, "visitor", {1, {2, 0, 0}}, 4.0};
    check(bool(original.update(1, eye, {1, 0, 0}, {},
                               std::array{sound}, home, visibility)),
          "snapshot source hears sound");
    const auto saved = original.snapshot();
    check(saved.lastTick == 1 && saved.targets.size() == 1
          && saved.heardSounds.size() == 1,
          "snapshot owns target and sound receipt");

    PerceptionMemory restored(config);
    check(restored.restore(saved) == Error::None,
          "restore matching perception state");
    check(restored.snapshot().targets.front().lastKnown == sound.location,
          "restore last known location");
    auto staleTick = restored.update(1, eye, {1, 0, 0}, {}, {}, home, visibility);
    check(staleTick.error == Error::Invalid, "restored tick rejects replay");
    SoundStimulus sameIdMoved = sound;
    sameIdMoved.location.position.x = 3;
    auto replay = restored.update(2, eye, {1, 0, 0}, {},
                                  std::array{sameIdMoved}, home, visibility);
    check(bool(replay) && replay.value.mode == PursuitMode::Search
          && replay.value.goal == sound.location,
          "restored sound receipt prevents memory refresh");

    auto malformed = saved;
    malformed.heardSounds.push_back(saved.heardSounds.front());
    check(restored.restore(malformed) == Error::Invalid
          && restored.snapshot().lastTick == 2,
          "duplicate receipt restore is atomic");
    malformed = saved;
    malformed.targets.front().lastKnown.position.x =
        std::numeric_limits<double>::infinity();
    check(restored.restore(malformed) == Error::Invalid
          && restored.snapshot().lastTick == 2,
          "nonfinite target restore is atomic");
    malformed = saved;
    malformed.schema++;
    check(restored.restore(malformed) == Error::Invalid,
          "unknown schema rejected");
    PerceptionMemory changedConfig({10.0, 0.8, 6, 4});
    check(changedConfig.restore(saved) == Error::Invalid,
          "config mismatch rejected");
    restored.forgetRegion(1);
    check(restored.snapshot().targets.empty()
          && restored.snapshot().heardSounds.empty(),
          "region unload clears restored state");
}
} // namespace

int main()
{
    try {
        bakeAndUpdate();
        visualAudioMemory();
        retiredTargetDoesNotReplayOldSound();
        perceptionSnapshotRoundTrip();
        stackedFloorsNeedExplicitTraversal();
        planarRampRespectsGradeAndAtomicRevision();
        nonplanarHeightfieldBakesAndRejectsHiddenCliffs();
        std::cout << "AYNavigation: spatial awareness passed\n";
    } catch (const std::exception& error) {
        std::cerr << "AYNavigation: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
