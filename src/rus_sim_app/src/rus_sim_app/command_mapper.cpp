#include "rus_sim_app/command_mapper.hpp"

#include <variant>

#include "rus_sim_utils/types.hpp"

namespace RusSimApp {

    std::vector<DownstreamCmd> CommandMapper::Map(const RusUtils::HighLevelCommand& cmd) const
    {
        return std::visit(RusUtils::Overloaded{
            [](const RusUtils::PreScanStartCmd&) {
                return std::vector<DownstreamCmd>{{Module::PLANNING, "pre_scan_start", {}}};
            },
            [](const RusUtils::SetStartPoseCmd& c) {
                return std::vector<DownstreamCmd>{{Module::PLANNING, "set_start_pose", c.pose}};
            },
            [](const RusUtils::SetEndPoseCmd& c) {
                return std::vector<DownstreamCmd>{{Module::PLANNING, "set_end_pose", c.pose}};
            },
            [](const RusUtils::PlanCmd&) {
                return std::vector<DownstreamCmd>{{Module::PLANNING, "plan", {}}};
            },
            [](const RusUtils::ExecuteCmd&) {
                return std::vector<DownstreamCmd>{{Module::PLANNING, "execute", {}}};
            },
            [](const RusUtils::StopCmd&) {
                return std::vector<DownstreamCmd>{
                    {Module::PLANNING, "stop", {}},
                    {Module::DRIVER, "stop", {}},
                };
            },
            [](const RusUtils::PauseCmd&) {
                return std::vector<DownstreamCmd>{
                    {Module::PLANNING, "pause", {}},
                    {Module::DRIVER, "pause", {}},
                };
            },
            [](const RusUtils::ResumeCmd&) {
                return std::vector<DownstreamCmd>{
                    {Module::PLANNING, "resume", {}},
                    {Module::DRIVER, "resume", {}},
                };
            },
            [](const RusUtils::ResetCmd&) {
                return std::vector<DownstreamCmd>{{Module::PLANNING, "reset", {}}};
            },
            [](const RusUtils::ConnectCmd&) {
                return std::vector<DownstreamCmd>{{Module::DRIVER, "connect", {}}};
            },
            [](const RusUtils::ShutdownCmd&) {
                return std::vector<DownstreamCmd>{{Module::PLANNING, "shutdown", {}}};
            },
            // 查询：也映射为下游指令，统一走路由转发
            [](const RusUtils::QueryPreScanDone&) {
                return std::vector<DownstreamCmd>{{Module::PLANNING, "query_prescan_done", {}}};
            },
            [](const RusUtils::QueryMotionDone&) {
                return std::vector<DownstreamCmd>{{Module::DRIVER, "is_motion_done", {}}};
            },
            // 录制 / 回放：路由到数据模块
            [](const RusUtils::RecordStartCmd&) {
                return std::vector<DownstreamCmd>{{Module::DATA, "record_start", {}}};
            },
            [](const RusUtils::RecordStopCmd&) {
                return std::vector<DownstreamCmd>{{Module::DATA, "record_stop", {}}};
            },
            [](const RusUtils::PlaybackStartCmd&) {
                return std::vector<DownstreamCmd>{{Module::DATA, "playback_start", {}}};
            },
            [](const RusUtils::PlaybackStopCmd&) {
                return std::vector<DownstreamCmd>{{Module::DATA, "playback_stop", {}}};
            },
            [](const RusUtils::PlaybackPauseCmd&) {
                return std::vector<DownstreamCmd>{{Module::DATA, "playback_pause", {}}};
            },
            [](const RusUtils::PlaybackResumeCmd&) {
                return std::vector<DownstreamCmd>{{Module::DATA, "playback_resume", {}}};
            },
            [](const RusUtils::PlaybackSetSpeedCmd& c) {
                return std::vector<DownstreamCmd>{{Module::DATA, "playback_set_speed", c.args}};
            },
            [](const RusUtils::PlaybackSeekCmd& c) {
                return std::vector<DownstreamCmd>{{Module::DATA, "playback_seek", c.args}};
            },
            [](const RusUtils::PlaybackStepCmd& c) {
                return std::vector<DownstreamCmd>{{Module::DATA, "playback_step", c.args}};
            },
            [](const RusUtils::PlaybackSetLoopCmd& c) {
                return std::vector<DownstreamCmd>{{Module::DATA, "playback_set_loop", c.args}};
            },
            [](const RusUtils::PlaybackGetInfoCmd&) {
                return std::vector<DownstreamCmd>{{Module::DATA, "playback_get_info", {}}};
            },
        }, cmd);
    }

}  // namespace RusSimApp
