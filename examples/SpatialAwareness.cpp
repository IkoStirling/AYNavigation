#include <AYNavigation/Perception.h>
#include <AYNavigation/StaticBake.h>

#include <array>
#include <iostream>

using namespace ayt::navigation;

namespace {
struct DemoMotor final : Motor {
    MotionState state{{1, {1.5, 0.0, 1.5}}, {}};
    std::optional<MotionState> sample(AgentId agent) const override
    { return agent == 1 ? std::optional{state} : std::nullopt; }
    bool drive(AgentId agent, TaskId, Vec3 velocity, Location,
               double seconds) override
    {
        if (agent != 1) return false;
        state.velocity = velocity;
        state.location.position = state.location.position + velocity * seconds;
        return true;
    }
    void stop(AgentId, TaskId) noexcept override { state.velocity = {}; }
};
struct EmptyObstacles final : ObstacleSource {
    ObstacleSnapshot query(AgentId, const MotionState&, double) const override
    { return {true, {}}; }
};
struct ClearView final : VisibilityQuery {
    Result<bool> clear(Location, const SightTarget&) override { return {Error::None, true}; }
};

bool follow(Navigator& navigator, TaskId task, const PursuitDecision& decision)
{
    if (navigator.begin(1, task, decision.goal) != Error::None) return false;
    for (int step = 0; step < 240; ++step) {
        const auto result = navigator.update(1, task);
        if (result.status == TravelStatus::Succeeded) return true;
        if (result.status == TravelStatus::Failed) return false;
    }
    return false;
}
} // namespace

int main()
{
    StaticBakeWorld bake(900);
    BakeRegion room{1, 1,
        {{0.0, -1.0, 0.0}, {8.0, 0.0, 6.0}},
        {{{3.0, 0.0, 2.0}, {5.0, 2.0, 4.0}}}};
    auto graph = bake.publishRegion(room);
    if (!graph || !graph.value) return 1;
    DemoMotor motor;
    EmptyObstacles obstacles;
    Navigator navigator(graph.value, std::make_shared<GraphPlanner>(),
        std::make_shared<ConservativeAvoidance>(), motor, obstacles);
    Profile profile;
    profile.steering = Steering::Planar;
    if (navigator.attach(1, profile) != Error::None) return 1;
    PerceptionMemory senses({20.0, 1.5, 3, 8});
    ClearView visibility;
    const Location home{1, {1.5, 0.0, 1.5}};
    const auto eye = [&] {
        auto result = motor.state.location;
        result.position.y += 1.6;
        return result;
    };
    const std::array target{SightTarget{"visitor",
        {1, {6.5, 0.0, 1.5}}, {1, {6.5, 1.6, 1.5}}}};
    const auto chase = senses.update(1, eye(), {1, 0, 0}, target, {},
                                     home, visibility);
    if (!chase || chase.value.mode != PursuitMode::Chase
        || !follow(navigator, 1, chase.value)) return 1;
    const auto search = senses.update(2, eye(), {1, 0, 0}, {}, {},
                                      home, visibility);
    if (!search || search.value.mode != PursuitMode::Search
        || !follow(navigator, 2, search.value)) return 1;
    const std::array noise{SoundStimulus{100, "visitor",
        {1, {6.5, 0.0, 4.5}}, 5.0}};
    const auto heard = senses.update(3, eye(), {1, 0, 0}, {}, noise,
                                     home, visibility);
    if (!heard || heard.value.mode != PursuitMode::Search
        || !follow(navigator, 3, heard.value)) return 1;
    const auto goHome = senses.update(8, eye(), {1, 0, 0}, {}, {},
                                      home, visibility);
    if (!goHome || goHome.value.mode != PursuitMode::Return
        || !follow(navigator, 4, goHome.value)) return 1;
    std::cout << "3D baked room: chase -> search -> sound search -> return; "
              << bake.report().nodes << " walkable cells\n";
    return 0;
}
