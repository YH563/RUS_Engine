#include "components/types.hpp"
#include "components/command_defs.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace RusRobotDriver {

    RobotCommand ParseCommand(std::string_view name, const std::vector<double>& args)
    {
        using namespace RusRobotDriver::Cmd;

        // ── 运动指令 ──
        if (name == kMoveJ) {
            MotionCommand cmd;
            cmd.type = MOTION_TYPE_JOINT;
            if (!args.empty())
                cmd.target = Eigen::Map<const VectorXd>(args.data(), std::min<size_t>(args.size(), 6));
            if (args.size() > 6) cmd.speed = args[6];        // 比例 [0~1]，缺省 0.5；<0.01 由驱动钳到 0.01
            if (args.size() > 7) cmd.acceleration = args[7];
            return cmd;
        }

        if (name == kMoveL) {
            MotionCommand cmd;
            cmd.type = MOTION_TYPE_CART;
            if (!args.empty())
                cmd.target = Eigen::Map<const VectorXd>(args.data(), std::min<size_t>(args.size(), 6));
            if (args.size() > 6) cmd.speed = args[6];        // 比例 [0~1]，缺省 0.5（≥6 时 args[0..5] 为 x,y,z,rx,ry,rz）
            if (args.size() > 7) cmd.acceleration = args[7];
            return cmd;
        }

        if (name == kServoJ) {
            MotionCommand cmd;
            cmd.type = MOTION_TYPE_SERVOJ;
            if (!args.empty())
                cmd.target = Eigen::Map<const VectorXd>(args.data(), std::min<size_t>(args.size(), 6));
            return cmd;
        }

        if (name == kServoCart) {
            MotionCommand cmd;
            cmd.type = MOTION_TYPE_SERVOC;
            if (!args.empty())
                cmd.target = Eigen::Map<const VectorXd>(args.data(), std::min<size_t>(args.size(), 6));
            return cmd;
        }

        if (name == kStartJog) {
            MotionCommand cmd;
            // ref → type: 0→JOG_0(关节点动)、2→JOG_1(基坐标系点动)、4→JOG_2(工具坐标系点动)
            // 其他值（含 8=工件坐标系，暂未实现）→ 回退 JOG_0
            uint8_t ref = args.size() > 0 ? static_cast<uint8_t>(args[0]) : 0;
            switch (ref) {
                case 0:  cmd.type = MOTION_TYPE_JOG_0; break;
                case 2:  cmd.type = MOTION_TYPE_JOG_1; break;
                case 4:  cmd.type = MOTION_TYPE_JOG_2; break;
                default: cmd.type = MOTION_TYPE_JOG_0; break;
            }
            if (args.size() > 1) cmd.jog_axis    = static_cast<uint8_t>(args[1]);
            if (args.size() > 2) cmd.jog_dir     = static_cast<uint8_t>(args[2]);
            if (args.size() > 3) cmd.speed       = std::clamp(args[3] / 100.0, 0.0, 1.0);  // 百分比 → 比例
            if (args.size() > 4) cmd.acceleration = std::clamp(args[4] / 100.0, 0.0, 1.0);
            if (args.size() > 5) cmd.jog_max_dis = args[5];
            // jog_max_dis（第 6 个参数）仅作协议兼容解析，**不参与运动控制**：
            // 上限单位 rad（关节点动 / 笛卡尔旋转轴 4~6）/ m（笛卡尔平移轴 1~3），
            // 实际生效值由 driver_node 按 driver_params.yaml 的 jog_max_dis_joint/trans/rot 覆盖；
            // 真实驱动再把该值换算为 SDK 的 °/mm（换算只在驱动实现内部）。
            return cmd;
        }

        // ── 伺服模式 ──
        if (name == kServoStart) return ServoStartCmd{};
        if (name == kServoEnd)   return ServoEndCmd{};

        // ── 运动控制 ──
        if (name == kStop)            return StopCmd{};
        if (name == kPause)           return PauseCmd{};
        if (name == kResume)          return ResumeCmd{};
        if (name == kReset) {
            ResetCmd cmd;
            if (!args.empty())          cmd.mode   = static_cast<uint8_t>(args[0]);
            if (args.size() > 1)        cmd.enable = static_cast<uint8_t>(args[1]);
            return cmd;
        }
        if (name == kStopJOGDecel)    return StopJOGDecelCmd{};
        if (name == kStopJOGImmediate) return StopJOGImmediateCmd{};

        // ── 驱动控制 ──
        if (name == kConnect)         return ConnectCmd{};
        if (name == kDisconnect)      return DisconnectCmd{};
        if (name == kIsConnected)    return IsConnectedCmd{};
        if (name == kIsInDragTeach) return IsInDragTeachCmd{};
        if (name == kRobotEnable)    return RobotEnableCmd{args.empty() ? uint8_t{1} : static_cast<uint8_t>(args[0])};
        if (name == kGetState)       return GetStateCmd{args.empty() ? uint8_t{1} : static_cast<uint8_t>(args[0])};
        if (name == kIsMotionDone)   return IsMotionDoneCmd{};
        if (name == kGetDriverType)  return GetDriverTypeCmd{};
        // 扇出指令 query_motion_done：bridge 同时发给 planning + driver，等价 is_motion_done
        if (name == kQueryMotionDone) return IsMotionDoneCmd{};

        // ── 文件执行（path 由调用方设置） ──
        if (name == kRunFile)        return RunFileCmd{};

        // ── 工具坐标系 / 标定 ──
        if (name == kSetToolCalibPoint) {
            SetToolCalibPointCmd cmd;
            cmd.point_num = args.empty() ? 1 : static_cast<int>(args[0]);
            return cmd;
        }
        if (name == kComputeToolCalib) return ComputeToolCalibCmd{};
        if (name == kSetToolCoord) {
            SetToolCoordCmd cmd;
            cmd.id = args.empty() ? 0 : static_cast<int>(args[0]);
            // 第 0 位为 id，其余 6 位为 [x,y,z,rx,ry,rz]（m/rad）
            for (size_t i = 1; i < args.size() && cmd.coord.size() < 6; ++i)
                cmd.coord.push_back(args[i]);
            return cmd;
        }
        if (name == kSetToolIndex) {
            SetToolIndexCmd cmd;
            cmd.id = args.empty() ? 0 : static_cast<int>(args[0]);
            return cmd;
        }
        if (name == kGetToolCoords) return GetToolCoordsCmd{};

        // ── 仿真控制（仅 Sim 驱动） ──
        if (name == kSetTimeSpeed)   return SetTimeSpeedCmd{args.empty() ? 1.0 : args[0]};
        if (name == kGetTimeSpeed)   return GetTimeSpeedCmd{};
        if (name == kGetSimTime)     return GetSimTimeCmd{};
        if (name == kStepOnce)       return StepOnceCmd{};
        if (name == kGetFrameRate)     return GetFrameRateCmd{};

        // 未知指令
        return StopCmd{};  // 返回安全指令
    }

}  // namespace RusRobotDriver
