#include <AYNavigation/Perception.h>
#include <AYNavigation/StaticBake.h>

#include <array>
#include <iostream>
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
                 std::vector<TraversalKind> traversals = {})
{
    PlanRequest request;
    request.id = 1;
    request.agent = 1;
    request.task = 1;
    request.from = from;
    request.goal = to;
    request.profile.steering = Steering::Planar;
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
    auto first = room(1, 0.0, 6.0);
    first.blockers.push_back({{2.0, 0.0, 2.0}, {4.0, 2.0, 4.0}});
    auto compiled = bake.publishRegion(first);
    check(bool(compiled) && compiled.value != nullptr, "first bake");
    check(bake.report().nodes > 0 && bake.report().blockedCells > 0,
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
          && two.value->stamp().version > oldStamp.version, "second region revision");
    const BakeConnection portal{1, 2, {5.5, 0.0, 0.5},
                                {10.5, 0.0, 0.5}, 17, 88, 1};
    auto connected = bake.setConnections({portal});
    check(bool(connected) && connected.value != nullptr,
          "cross-region connection compiled");
    auto across = route(connected.value, {1, {5.5, 0.0, 0.5}},
                        {2, {10.5, 0.0, 0.5}}, {17});
    check(across.status == PlanStatus::Found, "route crosses connection");
    bool foundPortal = false;
    for (const auto& step : across.route.steps)
        foundPortal |= step.traversal == 17;
    check(foundPortal, "route contains portal traversal step");
    auto unloaded = bake.unloadRegion(2);
    check(bool(unloaded) && unloaded.value != nullptr
          && bake.report().disabledConnections == 1,
          "unload removes topology but retains dormant connection");
    check(!unloaded.value->contains({2, {10.5, 0.0, 0.5}}, {}),
          "unloaded region is not navigable");
    auto reloaded = bake.publishRegion(second);
    check(bool(reloaded) && reloaded.value != nullptr
          && bake.report().disabledConnections == 0,
          "reload restores connection with new stamp");
    check(route(reloaded.value, {1, {5.5, 0.0, 0.5}},
                {2, {10.5, 0.0, 0.5}}, {17}).status == PlanStatus::Found,
          "restored cross-region route");

    first.contentVersion = 2;
    first.blockers = {{{2.0, 0.0, 0.0}, {4.0, 2.0, 6.0}}};
    auto blocked = bake.publishRegion(first);
    check(bool(blocked) && blocked.value != nullptr, "obstacle revision published");
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
} // namespace

int main()
{
    try {
        bakeAndUpdate();
        visualAudioMemory();
        stackedFloorsNeedExplicitTraversal();
        std::cout << "AYNavigation: spatial awareness passed\n";
    } catch (const std::exception& error) {
        std::cerr << "AYNavigation: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
