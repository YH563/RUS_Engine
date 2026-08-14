#pragma once

// ════════════════════════════════════════════════════════════════════
//  指令 → 模块注册表（bridge 侧配置）
//  ────────────────────────────────────────────────────────────────────
//  指令名与指令类型解耦。bridge 启动时通过 Register() 声明
//  "指令名 → 目标模块" 的对应关系，加新指令只改注册配置，不碰协议。
//
//  注意：本组件属于「路由域」，只在 bridge 节点使用。
//  后端子模块间通信（如 planning → driver）是直连的，不经过 bridge。
//  依赖：command_defs.hpp
// ════════════════════════════════════════════════════════════════════

#include <array>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "rus_sim_utils/command_defs.hpp"

namespace RusUtils {

    /// 后端目标模块
    enum class Module {
        PLANNING,    // 运动规划 / 执行
        DRIVER,      // 驱动
        PERCEPTION,  // 感知（点云 / 图像）
    };

    /// 模块 → 下游 CommandService 服务名
    constexpr const char* module_service_name(Module m) {
        switch (m) {
            case Module::PLANNING:   return "/planning/command";
            case Module::DRIVER:     return "/driver/command";
            case Module::PERCEPTION: return "/perception/command";
        }
        return "";
    }

    /// 模块 → 名称（日志 / 调试）
    constexpr std::string_view module_name(Module m) {
        switch (m) {
            case Module::PLANNING:   return "planning";
            case Module::DRIVER:     return "driver";
            case Module::PERCEPTION: return "perception";
        }
        return "?";
    }

    /**
     * @brief 指令 → 模块注册表
     *
     * 用法（bridge 启动时配置）：
     *   CommandRegistry reg;
     *   reg.Register(CmdName::kConnect, {Module::DRIVER});
     *   reg.Register(CmdName::kStop, {Module::PLANNING, Module::DRIVER});  // 扇出
     *   reg.Register(CmdName::kShutdown, {});                              // 空列表 = bridge 本地处理
     */
    class CommandRegistry {
    public:
        /// 注册指令的目标模块列表；空列表 = bridge 本地处理（不转发下游）
        void Register(std::string_view name, std::vector<Module> modules) {
            table_[std::string(name)] = Entry{std::move(modules)};
        }

        /// 运行时修改指令的扇出目标（模式切换等场景用：手动 ↔ 自动 切换 pause/resume 等去向）
        void SetTargets(std::string_view name, std::vector<Module> modules) {
            auto it = table_.find(std::string(name));
            if (it != table_.end()) it->second.targets = std::move(modules);
        }

        /// 指令是否已注册
        bool IsRegistered(std::string_view name) const {
            return table_.count(std::string(name)) > 0;
        }

        /// 是否 bridge 本地处理（目标模块列表为空）
        bool IsLocal(std::string_view name) const {
            return TargetsOf(name).empty();
        }

        /// 目标模块列表（多目标扇出；本地指令 / 未注册指令返回空）
        std::vector<Module> TargetsOf(std::string_view name) const {
            auto it = table_.find(std::string(name));
            if (it == table_.end()) return {};
            return it->second.targets;
        }

    private:
        struct Entry {
            std::vector<Module> targets;
        };

        std::unordered_map<std::string, Entry> table_;
    };

}  // namespace RusUtils
