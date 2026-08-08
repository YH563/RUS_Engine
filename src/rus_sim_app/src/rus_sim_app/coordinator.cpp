#include "rus_sim_app/coordinator.hpp"

#include <chrono>
#include <thread>

namespace RusSimApp {

    // ── 单例与构造 ──

    Coordinator::SharedPtr Coordinator::get_instance()
    {
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
        static SharedPtr instance(new Coordinator());
        return instance;
    }

    Coordinator::Coordinator()
        : rclcpp::Node("coordinator")
    {
        mapper_ = std::make_unique<CommandMapper>();
        router_ = std::make_unique<CommandRouter>(*this);

        // 注册各模块通道（统一 CommandService 服务）
        router_->RegisterModule(Module::DRIVER, "/driver/command");
        router_->RegisterModule(Module::PLANNING, "/planning/command");
        router_->RegisterModule(Module::PERCEPTION, "/perception/command");
        router_->RegisterModule(Module::DATA, "/data/command");

        // TODO: 创建并启动前端 WsServer
    }

    // ── 抽象任务接口 ──

    TaskHandle Coordinator::SubmitTask(const AppRequest& req)
    {
        TaskHandle handle = next_handle_.fetch_add(1);
        auto result = std::make_shared<std::promise<TaskResult>>();
        {
            std::lock_guard lock(tasks_mutex_);
            tasks_[handle] = result;
        }

        // 后台线程执行请求，完成时兑现结果
        // 统一流程：映射 → 路由转发 → 汇总（查询与操作同一条链）
        std::thread([this, req, result]() {
            auto cmds = mapper_->Map(req);
            auto results = router_->RouteAll(cmds);

            // 汇总：全部成功 → SUCCESS，携带查询数据（如 is_motion_done 0/1）
            bool ok = true;
            std::vector<double> data;
            for (const auto& r : results) {
                if (!r.success) { ok = false; break; }
                if (!r.data.empty())
                    data = r.data;
            }

            if (ok)
                result->set_value({TaskStatus::SUCCESS, "", std::move(data)});
            else
                result->set_value({TaskStatus::FAILED, "downstream failed", {}});
        }).detach();

        return handle;
    }

    TaskResult Coordinator::WaitTask(const TaskHandle& handle, double timeout)
    {
        std::shared_ptr<std::promise<TaskResult>> p;
        {
            std::lock_guard lock(tasks_mutex_);
            auto it = tasks_.find(handle);
            if (it != tasks_.end())
                p = it->second;
        }
        if (!p)
            return {TaskStatus::FAILED, "unknown task handle", {}};

        auto fut = p->get_future();
        if (fut.wait_for(std::chrono::duration<double>(timeout)) == std::future_status::ready)
            return fut.get();
        return {TaskStatus::TIMEOUT, "task timeout", {}};
    }

    void Coordinator::CancelTask(const TaskHandle& handle)
    {
        std::shared_ptr<std::promise<TaskResult>> p;
        {
            std::lock_guard lock(tasks_mutex_);
            auto it = tasks_.find(handle);
            if (it != tasks_.end()) {
                p = it->second;
                tasks_.erase(it);
            }
        }
        if (p) {
            try {
                p->set_value({TaskStatus::CANCELLED, "cancelled", {}});
            } catch (...) {}
        }
    }

    // ── 意图输入 ──

    std::string Coordinator::WaitForUserCommand(double timeout)
    {
        // TODO: 等待前端经 WsServer 注入业务指令（pre_scan_start / execute / shutdown ...）
        (void)timeout;
        return "timeout";
    }

    // ── 用户指令入口 ──

    void Coordinator::SetUserCommandSink(UserCommandSink sink)
    {
        user_sink_ = std::move(sink);
    }

    bool Coordinator::HandleUserCommand(const std::string& name, const std::vector<double>& args)
    {
        RusUtils::HighLevelCommand cmd;
        try {
            cmd = RusUtils::ParseHighLevelCommand(name, args);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "未知指令: %s (%s)", name.c_str(), e.what());
            return false;
        }
        SubmitTask(cmd);
        return true;
    }

}  // namespace RusSimApp
