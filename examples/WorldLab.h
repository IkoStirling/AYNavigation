#pragma once
// Example-owned assembly, not an automatic engine Host or persistent Scene component.
#include <AYNavigation/EntityMotor.h>
#include <AYNavigation/GraphAsset.h>
#include <AYBehavior/Executors.h>
#include <AYEntity/World.h>
#include <string>

namespace navlab {
using namespace ayt::navigation;
namespace behavior=ayt::behavior;
GraphDefinition initialGraph();
struct CommandReport {
    AgentId unit=0;behavior::Error behaviorError=behavior::Error::None;
    Error navigationError=Error::None;Location goal;TaskId task=0;
};
class WorldLab {
public:
    WorldLab();~WorldLab();
    WorldLab(const WorldLab&)=delete;
    bool install(GraphDefinition); // validation first; rejects replacement during locked traversal
    bool edit(GraphDefinition); // only while idle, publishes a newer immutable revision
    bool load(const std::filesystem::path&);
    bool save(const std::filesystem::path&,bool replace);
    void tick();
    void issue(Location,behavior::SubmitMode=behavior::SubmitMode::Replace);
    void stop();void wait();
    bool idle() const;
    void refreshCamera(float width,float height);
    behavior::Point2 screen(Vec3) const;
    std::optional<Vec3> ground(behavior::Point2,double height) const;
    std::vector<behavior::ScreenCandidate> candidates() const;
    std::optional<MotionState> motion(AgentId id) const {return motor->sample(id);}
    std::optional<AgentView> navigation(AgentId id) const {return navigator->state(id);}
    std::optional<behavior::UnitView> unit(AgentId id) const {return system->runtime().state(id);}
    bool owns(AgentId) const;
    std::string status(AgentId) const;
    GraphDefinition definition;
    std::shared_ptr<const GraphSpace> graph;
    std::vector<ayt::entity::EntityHandle> units;
    behavior::SelectionSet selection;
    behavior::SelectionController gesture;
    std::vector<CommandReport> reports;
    std::vector<behavior::Event> events;
    std::string message;
    bool obstacle=false;
    std::size_t ticks=0;
    const ayt::entity::CameraFrame& cameraFrame() const {return camera;}
    const ayt::entity::CameraViewport& cameraViewport() const {return viewport;}
private:
    struct Obstacles final:ObstacleSource {
        const WorldLab* lab=nullptr;
        ObstacleSnapshot query(AgentId,const MotionState&,double) const override;
    } obstacles;
    ayt::entity::World& world=ayt::entity::World::processWorld();
    bool initialized=false;
    Profile profile;
    std::unique_ptr<RootKinematicMotor> motor;
    std::unique_ptr<Navigator> navigator;
    behavior::BehaviorSystem* system=nullptr;
    ayt::entity::CameraFrame camera;
    ayt::entity::CameraViewport viewport;
    std::vector<Location> goals(Location) const;
    void shutdown();
};
} // namespace navlab
