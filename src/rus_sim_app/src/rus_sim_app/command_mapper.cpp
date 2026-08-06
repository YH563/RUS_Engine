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
        }, cmd);
    }

}  // namespace RusSimApp
