#pragma once
#include <AYNavigation/Navigation.h>
#include <AYBehavior/Executors.h>

namespace ayt::navigation {
inline constexpr ayt::behavior::KindId kNavigate=3;
/// Region-aware owned order: Point3 target plus eight little-endian region bytes. This is an
/// in-process composition recipe, not a persistence/network format. Custom kind must be >=3.
ayt::behavior::Order navigationOrder(Location goal,ayt::behavior::SourceId source,
    std::int32_t priority=0,ayt::behavior::KindId kind=kNavigate);
/// Borrowed Navigator must outlive the behavior runtime/World shutdown. Special traversal results
/// propagate dynamic interrupt windows into StepResult; resumption replans from actual position.
class NavigationExecutor final:public ayt::behavior::Executor {
public:
    explicit NavigationExecutor(Navigator&,ayt::behavior::KindId kind=kNavigate);
    bool accepts(ayt::behavior::UnitId,const ayt::behavior::Order&) const override;
    ayt::behavior::StepResult step(ayt::behavior::TaskContext&,ayt::behavior::StepPhase) override;
    void control(ayt::behavior::TaskContext&,ayt::behavior::Control) noexcept override;
private:
    Navigator& _navigator;ayt::behavior::KindId _kind;
};
/// Compatibility adapter for existing MoveExecutor/MotionPort users with an explicit destination
/// region. Use NavigationExecutor when each command can target a different region.
class NavigationMotionPort final:public ayt::behavior::MotionPort {
public:
    NavigationMotionPort(Navigator&,RegionId destinationRegion);
    bool available(ayt::behavior::UnitId) const override;
    bool begin(ayt::behavior::UnitId,ayt::behavior::TaskId,ayt::behavior::Point3) override;
    ayt::behavior::StepResult poll(ayt::behavior::UnitId,ayt::behavior::TaskId) override;
    void stop(ayt::behavior::UnitId,ayt::behavior::TaskId) noexcept override;
private:
    Navigator& _navigator;RegionId _region;
};
} // namespace ayt::navigation
