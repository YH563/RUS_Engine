#pragma once

// ════════════════════════════════════════════════════════════════════
//  指令定义（结构体绑定）
//  ────────────────────────────────────────────────────────────────────
//  指令 = 结构体，静态声明 kName（引用 command_defs.hpp 常量）。
//  结构体只承载指令名 + 参数，不含任何路由 / 模块信息 / 层级划分。
//  "指令名 → 目标模块" 的对应关系由 bridge 侧 CommandRegistry 注册，
//  与指令定义解耦。
//  依赖：command_defs.hpp
// ════════════════════════════════════════════════════════════════════

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "rus_sim_utils/command_defs.hpp"

namespace RusUtils {

    namespace Cmd {

        struct Connect {
            static constexpr std::string_view kName = CmdName::kConnect;
        };

        /// 关闭系统（bridge 本地处理：注册表空模块列表）
        struct Shutdown {
            static constexpr std::string_view kName = CmdName::kShutdown;
        };

        /// 模式切换（bridge 本地处理：修改 pause/resume/reset 等指令的扇出目标）
        /// 0=手动（直控 driver），1=自动（planning 协调）
        struct SetMode {
            static constexpr std::string_view kName = CmdName::kSetMode;
            std::vector<double> mode;                     // [0=手动, 1=自动]
            static bool ParseArgs(const std::vector<double>& a, SetMode& o) {
                if (a.size() < 1) return false;
                o.mode = a;
                return true;
            }
        };

        struct PreScanStart {
            static constexpr std::string_view kName = CmdName::kPreScanStart;
        };

        struct PreScanEnd {
            static constexpr std::string_view kName = CmdName::kPreScanEnd;
        };

        struct SetStartPose {
            static constexpr std::string_view kName = CmdName::kSetStartPose;
            std::vector<double> pose;                     // [x,y,z]
            static bool ParseArgs(const std::vector<double>& a, SetStartPose& o) {
                if (a.size() < 3) return false;
                o.pose = a;
                return true;
            }
        };

        struct SetEndPose {
            static constexpr std::string_view kName = CmdName::kSetEndPose;
            std::vector<double> pose;                     // [x,y,z]
            static bool ParseArgs(const std::vector<double>& a, SetEndPose& o) {
                if (a.size() < 3) return false;
                o.pose = a;
                return true;
            }
        };

        struct Plan {
            static constexpr std::string_view kName = CmdName::kPlan;
        };

        struct Execute {
            static constexpr std::string_view kName = CmdName::kExecute;
        };

        struct Stop {
            static constexpr std::string_view kName = CmdName::kStop;
        };

        struct Pause {
            static constexpr std::string_view kName = CmdName::kPause;
        };

        struct Resume {
            static constexpr std::string_view kName = CmdName::kResume;
        };

        struct Reset {
            static constexpr std::string_view kName = CmdName::kReset;
            double mode = 1;    // 复位模式：0=仅软复位（清运动状态），1=完整复位（清错误+重新使能）
            double enable = 1;  // 完整复位后是否重新上使能（mode=1 时有效）
            static bool ParseArgs(const std::vector<double>& a, Reset& o) {
                if (a.size() > 2) return false;
                if (!a.empty())    o.mode   = a[0];
                if (a.size() > 1)  o.enable = a[1];
                return true;
            }
        };

        struct QueryPreScanDone {
            static constexpr std::string_view kName = CmdName::kQueryPreScanDone;
        };

        struct QueryMotionDone {
            static constexpr std::string_view kName = CmdName::kQueryMotionDone;
        };

        struct MoveJ {
            static constexpr std::string_view kName = CmdName::kMoveJ;
            std::vector<double> q;                        // [q1..q6, speed?, acc?]
            static bool ParseArgs(const std::vector<double>& a, MoveJ& o) {
                if (a.size() < 6) return false;
                o.q = a;
                return true;
            }
        };

        struct MoveL {
            static constexpr std::string_view kName = CmdName::kMoveL;
            std::vector<double> pose;                     // [x,y,z,rx,ry,rz]
            static bool ParseArgs(const std::vector<double>& a, MoveL& o) {
                // 支持 [x,y,z]（姿态保持当前）或 [x,y,z,rx,ry,rz]（显式姿态）
                if (a.size() < 3) return false;
                o.pose = a;
                return true;
            }
        };

        struct ServoJ {
            static constexpr std::string_view kName = CmdName::kServoJ;
            std::vector<double> q;                        // [q1..q6]
            static bool ParseArgs(const std::vector<double>& a, ServoJ& o) {
                if (a.size() < 6) return false;
                o.q = a;
                return true;
            }
        };

        struct ServoCart {
            static constexpr std::string_view kName = CmdName::kServoCart;
            std::vector<double> pose;                     // [x,y,z,rx,ry,rz]
            static bool ParseArgs(const std::vector<double>& a, ServoCart& o) {
                if (a.size() < 6) return false;
                o.pose = a;
                return true;
            }
        };

        struct StartJog {
            static constexpr std::string_view kName = CmdName::kStartJog;
            std::vector<double> params;                   // [ref, axis, dir, speed%, acc%, max_dis?]
            static bool ParseArgs(const std::vector<double>& a, StartJog& o) {
                if (a.size() < 5) return false;
                o.params = a;
                return true;
            }
        };

        struct StopJogDecel {
            static constexpr std::string_view kName = CmdName::kStopJogDecel;
        };

        struct StopJogImmediate {
            static constexpr std::string_view kName = CmdName::kStopJogImmediate;
        };

        struct ServoStart {
            static constexpr std::string_view kName = CmdName::kServoStart;
        };

        struct ServoEnd {
            static constexpr std::string_view kName = CmdName::kServoEnd;
        };

        struct Disconnect {
            static constexpr std::string_view kName = CmdName::kDisconnect;
        };

        struct IsConnected {
            static constexpr std::string_view kName = CmdName::kIsConnected;
        };

        struct IsInDragTeach {
            static constexpr std::string_view kName = CmdName::kIsInDragTeach;
        };

        struct RobotEnable {
            static constexpr std::string_view kName = CmdName::kRobotEnable;
            std::vector<double> state;                    // [state]
            static bool ParseArgs(const std::vector<double>& a, RobotEnable& o) {
                if (a.size() < 1) return false;
                o.state = a;
                return true;
            }
        };

        struct GetState {
            static constexpr std::string_view kName = CmdName::kGetState;
        };

        struct IsMotionDone {
            static constexpr std::string_view kName = CmdName::kIsMotionDone;
        };

        struct RunFile {
            static constexpr std::string_view kName = CmdName::kRunFile;
        };

        struct SwitchDriver {
            static constexpr std::string_view kName = CmdName::kSwitchDriver;
            std::vector<double> params;                   // [type, ip1..ip4]
            static bool ParseArgs(const std::vector<double>& a, SwitchDriver& o) {
                if (a.size() < 1) return false;
                o.params = a;
                return true;
            }
        };

        /// 查询当前驱动类型（result: [0=仿真, 1=真实]，与 SwitchDriver::params[0] 同编码）
        struct GetDriverType {
            static constexpr std::string_view kName = CmdName::kGetDriverType;
        };

        struct SetTimeSpeed {
            static constexpr std::string_view kName = CmdName::kSetTimeSpeed;
            std::vector<double> speed;                    // [speed]
            static bool ParseArgs(const std::vector<double>& a, SetTimeSpeed& o) {
                if (a.size() < 1) return false;
                o.speed = a;
                return true;
            }
        };

        struct GetTimeSpeed {
            static constexpr std::string_view kName = CmdName::kGetTimeSpeed;
        };

        struct GetSimTime {
            static constexpr std::string_view kName = CmdName::kGetSimTime;
        };

        struct StepOnce {
            static constexpr std::string_view kName = CmdName::kStepOnce;
        };

        struct GetFrameRate {
            static constexpr std::string_view kName = CmdName::kGetFrameRate;
        };

        // ────────────────────────────────────────────────────────────
        //  回放指令（rus_sim_recorder_replay；路由到 Module::REPLAYER）
        //  单位 / 编码口径见 docs/Protocol/WsProtocol.md §4.7
        // ────────────────────────────────────────────────────────────

        /// 载入待回放文件：args = [目录清单序号?]（缺省 = 保持当前序号）
        struct ReplayLoad {
            static constexpr std::string_view kName = CmdName::kReplayLoad;
            double index = 0;
            static bool ParseArgs(const std::vector<double>& a, ReplayLoad& o) {
                if (a.size() > 1) return false;
                if (!a.empty()) o.index = a[0];
                return true;
            }
        };

        /// 列出录音清单（result = [文件数, 当前序号]；文件名在 reply.message 里）
        struct ReplayList {
            static constexpr std::string_view kName = CmdName::kReplayList;
        };

        /// 开始播放：args = [倍速?]（缺省 = 当前倍速；0.05 ~ 20，越界钳位）
        struct ReplayStart {
            static constexpr std::string_view kName = CmdName::kReplayStart;
            double speed = 0.0;   // 0 = 不改，沿用当前倍速
            static bool ParseArgs(const std::vector<double>& a, ReplayStart& o) {
                if (a.size() > 1) return false;
                if (!a.empty()) o.speed = a[0];
                return true;
            }
        };

        struct ReplayPause {
            static constexpr std::string_view kName = CmdName::kReplayPause;
        };

        struct ReplayResume {
            static constexpr std::string_view kName = CmdName::kReplayResume;
        };

        /// 停止并复位到起点（保留已载入文件，可直接再 start）
        struct ReplayStop {
            static constexpr std::string_view kName = CmdName::kReplayStop;
        };

        /// 跳转：args = [t_s]（相对文件起点，按时间轴定位到第一条 ≥ t 的记录）
        struct ReplaySeek {
            static constexpr std::string_view kName = CmdName::kReplaySeek;
            double t = 0.0;
            static bool ParseArgs(const std::vector<double>& a, ReplaySeek& o) {
                if (a.size() != 1) return false;
                o.t = a[0];
                return true;
            }
        };

        /// 设置倍速：args = [speed]（0.05 ~ 20）
        struct ReplaySetSpeed {
            static constexpr std::string_view kName = CmdName::kReplaySetSpeed;
            double speed = 1.0;
            static bool ParseArgs(const std::vector<double>& a, ReplaySetSpeed& o) {
                if (a.size() != 1) return false;
                o.speed = a[0];
                return true;
            }
        };

        /// 单步：args = [n?]（暂停 / 空闲态下顺序发布 n 条，缺省 1）
        struct ReplayStep {
            static constexpr std::string_view kName = CmdName::kReplayStep;
            double count = 1;
            static bool ParseArgs(const std::vector<double>& a, ReplayStep& o) {
                if (a.size() > 1) return false;
                if (!a.empty()) o.count = a[0];
                return true;
            }
        };

        /// 查询回放状态：result = [state, 进度s, 时长s, 倍速, 游标, 记录数, 已载入, 文件序号, 文件数]
        struct ReplayStatus {
            static constexpr std::string_view kName = CmdName::kReplayStatus;
        };

        // ════════════════════════════════════════════════════════════
        //  录制控制（recorder_node；服务 /recorder/command）
        //  节点默认启动即录（参数 autostart），这三条用于运行期开关落盘。
        // ════════════════════════════════════════════════════════════

        /// 开始录制（失败＝已在录制 / 不可用：未启用、无通道、写失败熔断）
        struct RecorderStart {
            static constexpr std::string_view kName = CmdName::kRecorderStart;
        };

        /// 停止录制：把已入队数据写完 → 封存（写尾索引）→ 停录（文件保留，可再 start）
        struct RecorderStop {
            static constexpr std::string_view kName = CmdName::kRecorderStop;
        };

        /// 查询录制状态：result = [state, 记录数, payload MiB, 当前文件 MiB, 丢弃, 限流, 文件数]
        struct RecorderStatus {
            static constexpr std::string_view kName = CmdName::kRecorderStatus;
        };

        // ════════════════════════════════════════════════════════════
        //  指令类型清单（唯一真源：所有指令类型的并集）
        //  查找表由本清单编译期自动展开生成，加新指令只改这里。
        // ════════════════════════════════════════════════════════════

        using CommandVariant = std::variant<
            Connect, Shutdown, SetMode,
            PreScanStart, PreScanEnd,
            SetStartPose, SetEndPose, Plan, Execute,
            Stop, Pause, Resume, Reset,
            QueryPreScanDone, QueryMotionDone,
            MoveJ, MoveL, ServoJ, ServoCart, StartJog,
            StopJogDecel, StopJogImmediate, ServoStart, ServoEnd,
            Disconnect, IsConnected, IsInDragTeach, RobotEnable,
            GetState, IsMotionDone, RunFile, SwitchDriver, GetDriverType,
            SetTimeSpeed, GetTimeSpeed, GetSimTime, StepOnce, GetFrameRate,
            ReplayLoad, ReplayList, ReplayStart, ReplayPause, ReplayResume,
            ReplayStop, ReplaySeek, ReplaySetSpeed, ReplayStep, ReplayStatus,
            RecorderStart, RecorderStop, RecorderStatus>;

        // ────────────────────────────────────────────────────────────
        //  解析：指令名 + args → 类型化结构体
        // ────────────────────────────────────────────────────────────

        namespace detail {

            struct Entry {
                std::string_view name;
                bool (*parse)(const std::vector<double>&, CommandVariant&);
            };

            // 检测结构体是否声明了 ParseArgs(args, out) → bool
            template <typename T, typename = void>
            struct has_parse_args : std::false_type {};
            template <typename T>
            struct has_parse_args<T, std::void_t<decltype(
                T::ParseArgs(std::declval<const std::vector<double>&>(),
                             std::declval<T&>()))>> : std::true_type {};

            // 统一解析入口：构造结构体实例 → 写入 variant
            template <typename T>
            bool parse_into(const std::vector<double>& args, CommandVariant& out) {
                T value;
                if constexpr (has_parse_args<T>::value) {
                    if (!T::ParseArgs(args, value)) return false;
                } else {
                    if (!args.empty()) return false;  // 无参指令不接受参数
                }
                out = std::move(value);
                return true;
            }

            // 从结构体类型生成表项（自动提取 kName + 解析函数）
            template <typename T>
            constexpr Entry make_entry() {
                return {T::kName, &parse_into<T>};
            }

            // 编译期根据 CommandVariant 的类型清单逐项生成查找表。
            // 表由 variant 自动展开而来，二者不可能不一致——加新指令
            // 只需在 CommandVariant 中登记，无需（也无法）单独维护表。
            template <typename Variant, std::size_t... I>
            constexpr std::array<Entry, sizeof...(I)> build_table(std::index_sequence<I...>) {
                return {{ make_entry<std::variant_alternative_t<I, Variant>>()... }};
            }

            inline constexpr auto CommandTable = build_table<CommandVariant>(
                std::make_index_sequence<std::variant_size_v<CommandVariant>>{});

        }  // namespace detail

        /// 解析指令名 + 参数为类型化结构体。
        /// @return nullopt = 未知指令或参数非法（错误描述写入 err）
        inline std::optional<CommandVariant> ParseCommand(
            const std::string& name, const std::vector<double>& args,
            std::string& err) {
            for (const auto& e : detail::CommandTable) {
                if (e.name != name) continue;
                CommandVariant out;
                if (e.parse(args, out)) return out;
                err = "invalid args for command: " + name;
                return std::nullopt;
            }
            err = "unknown command: " + name;
            return std::nullopt;
        }

        /// 指令名访问器（std::visit 返回统一类型）
        inline std::string_view NameOf(const CommandVariant& v) {
            return std::visit([](const auto& c) { return c.kName; }, v);
        }

    }  // namespace Cmd

}  // namespace RusUtils
