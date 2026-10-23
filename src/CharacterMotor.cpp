#include <AYNavigation/CharacterMotor.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ayt::navigation {
namespace {
Vec3 native(ayt::math::FVector3 p){return {p.x,p.y,p.z};}
ayt::math::FVector3 native(Vec3 p){return {float(p.x),float(p.y),float(p.z)};}
}
CharacterMotor::CharacterMotor(ayt::physics::PhysicsWorld3D& physics,Vec3 gravity):_physics(physics),_gravity(gravity) {
    if(!finite(gravity)||length(gravity)<1e-3||length(gravity)>1e4)throw std::invalid_argument("Character motor requires finite nonzero gravity");
}
CharacterMotor::~CharacterMotor(){while(!_bindings.empty())(void)detach(_bindings.begin()->first);}
Error CharacterMotor::attach(AgentId id,ayt::physics::CharacterHandle handle,RegionId region) {
    if(!id||!handle||!region)return Error::Invalid;if(_bindings.contains(id))return Error::Duplicate;
    for(const auto& [_,binding]:_bindings)if(binding.character==handle)return Error::Duplicate;
    ayt::physics::CharacterController3DState state;
    if(_physics.getCharacterState(handle,state)!=ayt::physics::PhysResult::Ok||!finite(native(state.position))||!finite(native(state.linearVelocity)))return Error::Missing;
    if(dot(normalized(native(state.up)),normalized(_gravity)*-1)<.999)return Error::Unsupported;
    _bindings.emplace(id,Binding{handle,region});return Error::None;
}
Error CharacterMotor::detach(AgentId id) {
    const auto found=_bindings.find(id);if(found==_bindings.end())return Error::Missing;
    // Also clear a controller's pre-existing persistent input when a binding is retired.
    ayt::physics::CharacterController3DInput idle;idle.useGravityOverride=true;idle.gravity=native(_gravity);
    const auto error=_physics.setCharacterInput(found->second.character,idle);_bindings.erase(found);
    return error==ayt::physics::PhysResult::Ok?Error::None:Error::MotionFailed;
}
std::optional<ayt::physics::CharacterController3DState> CharacterMotor::character(AgentId id) const {
    const auto found=_bindings.find(id);if(found==_bindings.end())return {};
    ayt::physics::CharacterController3DState result;
    if(_physics.getCharacterState(found->second.character,result)!=ayt::physics::PhysResult::Ok)return {};
    if(!finite(native(result.position))||!finite(native(result.linearVelocity)))return {};
    return result;
}
std::optional<MotionState> CharacterMotor::sample(AgentId id) const {
    const auto state=character(id);if(!state)return {};
    auto velocity=native(state->linearVelocity);
    // Jolt keeps a downward support bias in controller velocity even on a stationary floor.
    // Remove only the into-support component, relative to the completed ground velocity.
    if(state->isGrounded()) {
        const auto normal=normalized(native(state->groundNormal));const auto relative=velocity-native(state->groundVelocity);
        const auto inward=dot(relative,normal);if(inward<0)velocity=velocity-normal*inward;
    }
    return MotionState{{_bindings.at(id).region,native(state->position)},velocity};
}
bool CharacterMotor::input(AgentId id,TaskId task,Vec3 velocity,double jump) {
    const auto found=_bindings.find(id);
    if(found==_bindings.end()||!task||!finite(velocity)||length(velocity)>1e4||!std::isfinite(jump)||jump<0||jump>1e4)return false;
    if(found->second.task&&found->second.task!=task)return false;
    ayt::physics::CharacterController3DInput command;command.horizontalVelocity=native(velocity);
    command.jumpSpeed=float(jump);command.gravity=native(_gravity);command.useGravityOverride=true;
    if(_physics.setCharacterInput(found->second.character,command)!=ayt::physics::PhysResult::Ok)return false;
    found->second.task=task;return true;
}
bool CharacterMotor::drive(AgentId id,TaskId task,Vec3 velocity,Location target,double dt) {
    const auto found=_bindings.find(id);
    if(found==_bindings.end()||target.region!=found->second.region||!finite(target.position)||!std::isfinite(dt)||dt<=0||dt>1)return false;
    return input(id,task,velocity,0);
}
bool CharacterMotor::launch(AgentId id,TaskId task,Vec3 tangent,double speed) {
    const auto state=character(id);return state&&state->isGrounded()&&speed>0&&input(id,task,tangent,speed);
}
void CharacterMotor::stop(AgentId id,TaskId task) noexcept {
    const auto found=_bindings.find(id);if(found==_bindings.end()||found->second.task!=task)return;
    (void)input(id,task,{},0);found->second.task=0;
}
bool CharacterMotor::acceptArrival(AgentId id,TaskId,Location goal,const Profile& profile) const {
    const auto state=character(id);const auto motion=sample(id);
    return state&&motion&&state->isGrounded()&&motion->location.region==goal.region
        &&length(motion->location.position-goal.position)<=profile.arrivalDistance&&length(motion->velocity)<=profile.arrivalSpeed;
}
CharacterJumpTraversal::CharacterJumpTraversal(CharacterMotor& motor,JumpSettings settings):_motor(motor),_settings(settings) {
    if(!std::isfinite(settings.apexHeight)||settings.apexHeight<=0||settings.apexHeight>100||
       !std::isfinite(settings.maxHorizontalSpeed)||settings.maxHorizontalSpeed<=0||settings.maxHorizontalSpeed>1000||
       !std::isfinite(settings.maxJumpSpeed)||settings.maxJumpSpeed<=0||settings.maxJumpSpeed>1000||
       !std::isfinite(settings.maxSeconds)||settings.maxSeconds<=0||settings.maxSeconds>60||
       !std::isfinite(settings.maxLandingError)||settings.maxLandingError<=0||settings.maxLandingError>10)
        throw std::invalid_argument("Invalid physical jump settings");
}
bool CharacterJumpTraversal::flight(const Step& step,const Profile& profile,Vec3& tangent,double& speed,double& seconds) const {
    if(!step.traversal||step.from.region!=step.to.region||!valid(profile))return false;
    const auto up=normalized(_motor.gravity())*-1;
    if(dot(normalized(profile.up),up)<.999)return false;
    const auto delta=step.to.position-step.from.position;const double rise=dot(delta,up),gravity=length(_motor.gravity());
    const double apex=std::max(_settings.apexHeight,rise+.5);speed=std::sqrt(2*gravity*apex);
    seconds=(speed+std::sqrt(std::max(0.0,speed*speed-2*gravity*rise)))/gravity;
    tangent=(delta-up*rise)*(1/seconds);
    return speed<=_settings.maxJumpSpeed&&seconds>0&&seconds<_settings.maxSeconds&&length(tangent)<=_settings.maxHorizontalSpeed;
}
bool CharacterJumpTraversal::accepts(const Step& step,const Profile& profile) const {
    Vec3 tangent;double speed=0,seconds=0;return flight(step,profile,tangent,speed,seconds);
}
TravelResult CharacterJumpTraversal::update(TraversalContext& context,bool starting) {
    const auto state=_motor.character(context.agent);if(!state)return {TravelStatus::Failed,true};
    const auto up=normalized(_motor.gravity())*-1;const double elapsed=context.elapsedSteps*context.seconds;
    if(starting) {
        Vec3 tangent;double speed=0,seconds=0;
        if(!state->isGrounded()||!flight(context.step,context.profile,tangent,speed,seconds)||!_motor.launch(context.agent,context.task,tangent,speed))return {TravelStatus::Failed,true};
        context.memory.values[0]=tangent.x;context.memory.values[1]=tangent.y;context.memory.values[2]=tangent.z;
        return {TravelStatus::Running,false};
    }
    if(!state->isGrounded())context.memory.words[0]=1;
    if(elapsed>_settings.maxSeconds || (!context.memory.words[0]&&elapsed>.35))return {TravelStatus::Failed,true};
    if(context.memory.words[0]&&state->isGrounded()) {
        const auto delta=context.step.to.position-context.motion.location.position;
        if(length(delta)>_settings.maxLandingError)return {TravelStatus::Failed,true};
        if(length(delta)<=context.profile.arrivalDistance&&length(context.motion.velocity)<=context.profile.arrivalSpeed) {
            _motor.stop(context.agent,context.task);return {TravelStatus::Succeeded,true};
        }
        const auto tangent=delta-up*dot(delta,up);const double distance=length(tangent);
        auto velocity=normalized(tangent)*std::min({context.profile.maxSpeed,std::sqrt(2*context.profile.maxAcceleration*distance),distance/context.seconds});
        const auto previous=context.motion.velocity-up*dot(context.motion.velocity,up);
        const auto change=velocity-previous;const double limit=context.profile.maxAcceleration*context.seconds;
        if(length(change)>limit)velocity=previous+normalized(change)*limit;
        if(!_motor.drive(context.agent,context.task,velocity,context.step.to,context.seconds))return {TravelStatus::Failed,true};
        return {TravelStatus::Running,true};
    }
    Vec3 tangent{context.memory.values[0],context.memory.values[1],context.memory.values[2]};
    if(context.memory.words[0]) {
        const auto delta=context.step.to.position-context.motion.location.position;
        const double verticalSpeed=dot(context.motion.velocity,up),gravity=length(_motor.gravity());
        const double remaining=(verticalSpeed+std::sqrt(std::max(0.0,verticalSpeed*verticalSpeed-2*gravity*dot(delta,up))))/gravity;
        tangent=(delta-up*dot(delta,up))*(1/std::max(context.seconds,remaining));
        const double stopSpeed=std::sqrt(2*context.profile.maxAcceleration*length(delta-up*dot(delta,up)));
        const double limit=std::min(_settings.maxHorizontalSpeed,stopSpeed);
        if(length(tangent)>limit)tangent=normalized(tangent)*limit;
    }
    if(!_motor.drive(context.agent,context.task,tangent,context.step.to,context.seconds))return {TravelStatus::Failed,true};
    return {TravelStatus::Running,false};
}
void CharacterJumpTraversal::cancel(TraversalContext& context) noexcept {_motor.stop(context.agent,context.task);}
} // namespace ayt::navigation
