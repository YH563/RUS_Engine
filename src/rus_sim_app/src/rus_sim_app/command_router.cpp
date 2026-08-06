#include "rus_sim_app/command_router.hpp"


namespace RusSimApp {

    CommandRouter::CommandRouter(rclcpp::Node& node)
        : node_(node)
    {}

    void CommandRouter::RegisterModule(Module module, const std::string& service_name)
    {
        std::lock_guard lock(clients_mutex_);
        service_names_[module] = service_name;
    }

    std::future<ModuleResult> CommandRouter::RouteAsync(const DownstreamCmd& cmd) const
    {
        // 获取 / 惰性创建客户端
        rclcpp::Client<rus_sim_interfaces::srv::CommandService>::SharedPtr client;
        {
            std::lock_guard lock(clients_mutex_);
            auto it = clients_.find(cmd.target);
            if (it != clients_.end()) {
                client = it->second;
            } else {
                auto sit = service_names_.find(cmd.target);
                if (sit == service_names_.end()) {
                    std::promise<ModuleResult> p;
                    p.set_value({false, "module not registered", {}});
                    return p.get_future();
                }
                client = node_.create_client<rus_sim_interfaces::srv::CommandService>(sit->second);
                clients_[cmd.target] = client;
            }
        }

        // 构造请求
        auto request = std::make_shared<rus_sim_interfaces::srv::CommandService::Request>();
        request->command = cmd.name;
        request->args = cmd.args;

        // 异步发送，后台线程等待响应并兑现结果
        auto shared_fut = client->async_send_request(request);
        std::promise<ModuleResult> p;
        auto result_fut = p.get_future();
        std::thread([shared_fut = std::move(shared_fut), p = std::move(p)]() mutable {
            auto resp = shared_fut.get();
            ModuleResult r;
            r.success = resp->success;
            r.message = resp->message;
            r.data = resp->result;
            p.set_value(r);
        }).detach();
        return result_fut;
    }

    std::vector<ModuleResult> CommandRouter::RouteAll(
        const std::vector<DownstreamCmd>& cmds) const
    {
        std::vector<std::future<ModuleResult>> futures;
        futures.reserve(cmds.size());
        for (const auto& c : cmds)
            futures.push_back(RouteAsync(c));

        std::vector<ModuleResult> results;
        results.reserve(cmds.size());
        for (auto& f : futures)
            results.push_back(f.get());
        return results;
    }

}  // namespace RusSimApp
