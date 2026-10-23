#pragma once
#include <AYNavigation/BehaviorIntegration.h>
#include <AYBehavior/EntityIntegration.h>

namespace ayt::navigation {
/// Explicit native root-only motor for simple kinematic scenes. Borrows World, uses checked pose
/// writes per fixed step, and deliberately has no interpolation/physics/character ownership.
/// Legacy World region0 is exposed under the configured nonzero navigation region. Actual nonzero
/// Entity regions are preserved; cross-region traversal requires a custom authoritative motor.
class RootKinematicMotor final:public Motor {
public:
    explicit RootKinematicMotor(ayt::entity::World&,RegionId legacyRegion=1);
    std::optional<MotionState> sample(AgentId) const override;
    bool drive(AgentId,TaskId,Vec3,Location target,double seconds) override;
    void stop(AgentId,TaskId) noexcept override;
    bool traversalPose(AgentId,TaskId,Location) override;
private:
    struct Record {TaskId task=0;Vec3 velocity;};
    ayt::entity::World& _world;RegionId _legacy;
    std::map<AgentId,Record> _records;
};
} // namespace ayt::navigation
