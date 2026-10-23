#include <AYNavigation/BehaviorIntegration.h>
#include <stdexcept>

namespace ayt::navigation {
namespace {
std::optional<Location> goal(const ayt::behavior::Order& order) {
    const auto* point=std::get_if<ayt::behavior::Point3>(&order.target);
    if(!point || order.payload.size()!=8) return {};
    RegionId region=0;for(unsigned i=0;i<8;++i) region|=RegionId(order.payload[i])<<(i*8);
    const Vec3 position{point->x,point->y,point->z};
    return region && finite(position)?std::optional<Location>{{region,position}}:std::nullopt;
}
ayt::behavior::StepResult translated(Update result) {
    using Status=ayt::behavior::StepStatus;
    return {result.status==TravelStatus::Succeeded?Status::Succeeded:result.status==TravelStatus::Failed?Status::Failed
        :result.status==TravelStatus::Waiting?Status::Waiting:Status::Running,std::uint32_t(result.error),result.interruptible};
}
}
ayt::behavior::Order navigationOrder(Location target,ayt::behavior::SourceId source,std::int32_t priority,ayt::behavior::KindId kind) {
    ayt::behavior::Order order;order.kind=kind;order.source=source;order.priority=priority;
    order.target=ayt::behavior::Point3{target.position.x,target.position.y,target.position.z};order.payload.resize(8);
    for(unsigned i=0;i<8;++i) order.payload[i]=std::uint8_t(target.region>>(i*8));return order;
}
NavigationExecutor::NavigationExecutor(Navigator& navigator,ayt::behavior::KindId kind):_navigator(navigator),_kind(kind) {
    if(kind<3) throw std::invalid_argument("Navigation kind must not replace Wait/Move");
}
bool NavigationExecutor::accepts(ayt::behavior::UnitId unit,const ayt::behavior::Order& order) const {
    const auto target=goal(order);return order.kind==_kind && target && _navigator.canBegin(unit,*target);
}
ayt::behavior::StepResult NavigationExecutor::step(ayt::behavior::TaskContext& context,ayt::behavior::StepPhase phase) {
    if(phase!=ayt::behavior::StepPhase::Tick) {
        const auto target=goal(context.order);
        if(!target) return {ayt::behavior::StepStatus::Failed,std::uint32_t(Error::Invalid)};
        const auto error=_navigator.begin(context.unit,context.task,*target);
        if(error!=Error::None) return {ayt::behavior::StepStatus::Failed,std::uint32_t(error)};
    }
    return translated(_navigator.update(context.unit,context.task));
}
void NavigationExecutor::control(ayt::behavior::TaskContext& context,ayt::behavior::Control) noexcept {
    (void)_navigator.cancel(context.unit,context.task);
}
NavigationMotionPort::NavigationMotionPort(Navigator& navigator,RegionId region):_navigator(navigator),_region(region) {
    if(!region) throw std::invalid_argument("Destination region must be explicit");
}
bool NavigationMotionPort::available(ayt::behavior::UnitId unit) const {return _navigator.available(unit);}
bool NavigationMotionPort::begin(ayt::behavior::UnitId unit,ayt::behavior::TaskId task,ayt::behavior::Point3 point) {
    return _navigator.begin(unit,task,{_region,{point.x,point.y,point.z}})==Error::None;
}
ayt::behavior::StepResult NavigationMotionPort::poll(ayt::behavior::UnitId unit,ayt::behavior::TaskId task) {
    return translated(_navigator.update(unit,task));
}
void NavigationMotionPort::stop(ayt::behavior::UnitId unit,ayt::behavior::TaskId task) noexcept {(void)_navigator.cancel(unit,task);}
} // namespace ayt::navigation
