#include "Test.h"
#include "WorldLab.h"
#include <array>
#include <fstream>
#include <chrono>
using namespace navlab;
namespace {
void selectAll(WorldLab& lab) {
    lab.refreshCamera(1280,800);const auto& viewport=lab.cameraViewport();
    CHECK(lab.gesture.begin({viewport.x,viewport.y})==behavior::Error::None);
    CHECK(lab.gesture.move({viewport.x+viewport.width,viewport.y+viewport.height})==behavior::Error::None);
    CHECK(lab.gesture.end({viewport.x+viewport.width,viewport.y+viewport.height},lab.candidates(),lab.selection));
    CHECK(lab.selection.selected().size()==3);
}
void actual_world_camera_box_selection_routes_three_units_to_separate_goals() {
    WorldLab lab;selectAll(lab);const auto ids=std::vector<AgentId>(lab.selection.selected().begin(),lab.selection.selected().end());
    lab.issue({1,{8,1,0}});CHECK(lab.reports.size()==3);
    for(const auto& report:lab.reports)CHECK(report.behaviorError==behavior::Error::None&&report.navigationError==Error::None&&report.task);
    for(std::size_t i=0;i<3;++i)for(std::size_t j=i+1;j<3;++j)CHECK(length(lab.reports[i].goal.position-lab.reports[j].goal.position)>.6);
    bool waiting=false,jumping=false;
    for(int tick=0;tick<5000&&!lab.idle();++tick) {
        lab.tick();unsigned inAir=0;
        for(const auto id:ids) {
            const auto phase=lab.navigation(id)->phase;waiting|=phase==Phase::WaitingForLink;
            if(phase==Phase::Traversing){jumping=true;++inAir;}
        }
        CHECK(inAir<=1);
        for(std::size_t i=0;i<ids.size();++i)for(std::size_t j=i+1;j<ids.size();++j)
            CHECK(length(lab.motion(ids[i])->location.position-lab.motion(ids[j])->location.position)>=.359);
    }
    for(const auto id:ids){if(lab.navigation(id)->phase!=Phase::Succeeded){const auto p=lab.motion(id)->location.position;std::fprintf(stderr,"unit %llu %s error=%d p=%g,%g,%g step=%zu\n",id,lab.status(id).c_str(),int(lab.navigation(id)->error),p.x,p.y,p.z,lab.navigation(id)->step);}CHECK(lab.navigation(id)->phase==Phase::Succeeded);}
    CHECK(lab.idle()&&waiting&&jumping);
}
void append_interrupt_resume_and_locked_stop_keep_individual_results() {
    WorldLab lab;selectAll(lab);const auto id=lab.selection.selected()[0];
    CHECK(lab.selection.apply(std::array<AgentId,1>{id}));lab.issue({1,{8,1,0}});const auto originalGoal=lab.reports[0].goal;
    lab.wait();CHECK(lab.unit(id)->queued.size()==1);
    lab.tick();lab.issue({1,{-4,0,0}},behavior::SubmitMode::Interrupt);CHECK(lab.reports[0].behaviorError==behavior::Error::None);
    CHECK(lab.unit(id)->suspended.size()==1);
    bool jumping=false;
    for(int i=0;i<2000;++i) {lab.tick();if(lab.navigation(id)->phase==Phase::Traversing){jumping=true;break;}}
    CHECK(jumping);const auto task=lab.unit(id)->active->id;lab.stop();CHECK(lab.reports[0].behaviorError==behavior::Error::NotInterruptible);
    CHECK(lab.unit(id)->active->id==task);CHECK(!lab.install(initialGraph()));
    for(int i=0;i<2000&&!lab.idle();++i)lab.tick();CHECK(lab.idle());
    CHECK(lab.navigation(id)->phase==Phase::Succeeded);CHECK(length(lab.motion(id)->location.position-originalGoal.position)<.04);
    CHECK(std::any_of(lab.events.begin(),lab.events.end(),[](auto& event){return event.event==behavior::EventKind::Succeeded;}));
}
void blocked_corridor_feedback_stop_and_invalid_goal_preserve_previous_work() {
    WorldLab lab;selectAll(lab);const auto id=lab.selection.selected()[1];CHECK(lab.selection.apply(std::array<AgentId,1>{id}));
    lab.obstacle=true;lab.issue({1,{8,1,0}});const auto task=lab.reports[0].task;
    lab.issue({2,{100,0,0}});CHECK(lab.reports[0].navigationError==Error::NoPath);CHECK(lab.unit(id)->active->id==task);
    bool blocked=false;
    for(int i=0;i<800;++i){lab.tick();if(lab.navigation(id)->phase==Phase::Blocked){blocked=true;break;}}
    CHECK(blocked);CHECK(lab.status(id).find("Blocked")!=std::string::npos);lab.stop();CHECK(lab.reports[0].behaviorError==behavior::Error::None);
    CHECK(lab.idle());
}
void file_reload_resets_handles_and_invalid_load_retains_running_state() {
    WorldLab lab;CHECK(lab.load(AY_NAVIGATION_EXAMPLE_GRAPH));selectAll(lab);const auto old=lab.selection.selected()[0];
    const auto path=std::filesystem::temp_directory_path()/("navlab-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".nvg");
    struct Cleanup{std::filesystem::path path;~Cleanup(){std::error_code error;std::filesystem::remove(path,error);}} cleanup{path};
    auto edited=lab.definition;edited.nodes.push_back({99,{1,{9,1,3}}});CHECK(lab.edit(edited));CHECK(lab.save(path,false));
    lab.issue({1,{8,1,0}});lab.tick();const auto before=lab.unit(old)->active->id;
    {std::ofstream file(path,std::ios::trunc);file<<"AYNavigation.Graph 999\n";}
    CHECK(!lab.load(path));CHECK(lab.unit(old)->active->id==before);CHECK(lab.definition.nodes.size()==11);
    CHECK(lab.save(path,true));CHECK(lab.load(path));CHECK(lab.definition.nodes.size()==11);CHECK(lab.selection.selected().empty());
    CHECK(!lab.owns(old));selectAll(lab);lab.issue({1,{8,1,0}});
    for(int i=0;i<5000&&!lab.idle();++i)lab.tick();CHECK(lab.idle());
}
}
TEST_MAIN(
    RUN(actual_world_camera_box_selection_routes_three_units_to_separate_goals);
    RUN(append_interrupt_resume_and_locked_stop_keep_individual_results);
    RUN(blocked_corridor_feedback_stop_and_invalid_goal_preserve_previous_work);
    RUN(file_reload_resets_handles_and_invalid_load_retains_running_state);
)
