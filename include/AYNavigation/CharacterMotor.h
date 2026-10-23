#pragma once
#include <AYNavigation/Navigation.h>
#include <AYPhysics/CharacterController3D.h>
#include <AYPhysics/PhysicsWorld3D.h>

namespace ayt::navigation {
/// Optional physics-owned capsule motor. Borrowed PhysicsWorld3D and bound character handles must
/// remain live through navigation cleanup. Attach only after completed creation; caller owns destroy.
/// Behavior/drive runs before FixedPhysics, sample reads the latest completed barrier. Never steps
/// physics, teleports a pose or writes an Entity. Copy completed feet to presentation separately.
/// Grounded samples remove only into-support velocity bias relative to completed ground velocity;
/// airborne velocity is unchanged. Use matching capsule/profile dimensions and exclusive input ownership.
class CharacterMotor final:public Motor {
public:
    explicit CharacterMotor(ayt::physics::PhysicsWorld3D&,Vec3 gravity={0,-9.81,0});
    ~CharacterMotor();
    Error attach(AgentId,ayt::physics::CharacterHandle,RegionId);
    Error detach(AgentId);
    std::optional<ayt::physics::CharacterController3DState> character(AgentId) const;
    std::optional<MotionState> sample(AgentId) const override;
    bool drive(AgentId,TaskId,Vec3,Location,double) override;
    void stop(AgentId,TaskId) noexcept override;
    bool acceptArrival(AgentId,TaskId,Location,const Profile&) const override;
    /// Submit a one-shot grounded jump. Later airborne updates use drive (jumpSpeed zero).
    bool launch(AgentId,TaskId,Vec3 tangentVelocity,double jumpSpeed);
    Vec3 gravity() const noexcept {return _gravity;}
private:
    struct Binding {ayt::physics::CharacterHandle character;RegionId region;TaskId task=0;};
    ayt::physics::PhysicsWorld3D& _physics;Vec3 _gravity;
    std::map<AgentId,Binding> _bindings;
    bool input(AgentId,TaskId,Vec3,double jump);
};
struct JumpSettings {
    double apexHeight=1.6,maxHorizontalSpeed=6,maxJumpSpeed=9,maxSeconds=4,maxLandingError=.65;
};
/// Physics capsule jump over one explicit same-region link, aligned to the motor's constant gravity.
/// Ballistic launch plus bounded-speed airborne/landing steering is not a clearance proof. Real snapshots
/// control completion; blocked launch, wrong landing and timeout fail without pose correction.
/// The owner must provide supported landing geometry and emergency recovery for forced cancellation.
class CharacterJumpTraversal final:public Traversal {
public:
    CharacterJumpTraversal(CharacterMotor&,JumpSettings={});
    bool accepts(const Step&,const Profile&) const override;
    TravelResult update(TraversalContext&,bool starting) override;
    void cancel(TraversalContext&) noexcept override;
private:
    CharacterMotor& _motor;JumpSettings _settings;
    bool flight(const Step&,const Profile&,Vec3& tangent,double& speed,double& seconds) const;
};
} // namespace ayt::navigation
