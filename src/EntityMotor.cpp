#include <AYNavigation/EntityMotor.h>
#include <AYEntity/World.h>
#include <AYEntity/EntityImpl.h>
#include <cmath>
#include <stdexcept>

namespace ayt::navigation {
RootKinematicMotor::RootKinematicMotor(ayt::entity::World& world,RegionId legacyRegion):_world(world),_legacy(legacyRegion) {
    if(!legacyRegion) throw std::invalid_argument("Legacy navigation region must be explicit");
}
std::optional<MotionState> RootKinematicMotor::sample(AgentId id) const {
    if(!_world.isInitialized()) return {};
    auto* entity=_world.getEntityByHandle(ayt::behavior::entityHandle(id));
    if(!entity || !entity->isSpatial() || entity->spatialParent()) return {};
    ayt::math::FVector3 position;ayt::math::FQuaternion rotation;
    if(!entity->worldPose(position,rotation)) return {};
    const auto region=entity->effectiveSpaceRegion();const auto record=_records.find(id);
    return MotionState{{region?region:_legacy,{position.x,position.y,position.z}},record==_records.end()?Vec3{}:record->second.velocity};
}
bool RootKinematicMotor::drive(AgentId id,TaskId task,Vec3 velocity,Location target,double seconds) {
    const auto state=sample(id);
    if(!task || !state || state->location.region!=target.region || !finite(target.position) || !finite(velocity)
        || !std::isfinite(seconds) || seconds<=0 || seconds>1) return false;
    auto& record=_records[id];if(record.task && task<record.task) return false;
    auto* entity=_world.getEntityByHandle(ayt::behavior::entityHandle(id));
    ayt::math::FVector3 position;ayt::math::FQuaternion rotation;
    if(!entity->worldPose(position,rotation)) return false;
    const auto next=state->location.position+velocity*seconds;if(!finite(next)) return false;
    if(!entity->setWorldPose({float(next.x),float(next.y),float(next.z)},rotation)) return false;
    record={task,velocity};return true;
}
void RootKinematicMotor::stop(AgentId id,TaskId task) noexcept {
    const auto record=_records.find(id);if(record!=_records.end() && record->second.task==task) _records.erase(record);
}
bool RootKinematicMotor::traversalPose(AgentId id,TaskId task,Location target) {
    const auto state=sample(id);if(!state || state->location.region!=target.region || !finite(target.position) || !task) return false;
    auto& record=_records[id];if(record.task && task<record.task) return false;
    auto* entity=_world.getEntityByHandle(ayt::behavior::entityHandle(id));
    ayt::math::FVector3 position;ayt::math::FQuaternion rotation;
    if(!entity->worldPose(position,rotation) || !entity->setWorldPose({float(target.position.x),float(target.position.y),float(target.position.z)},rotation)) return false;
    record={task,{}};return true;
}
} // namespace ayt::navigation
