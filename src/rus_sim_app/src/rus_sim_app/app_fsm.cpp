#include "rus_sim_app/app_fsm.hpp"

#include <chrono>
#include <utility>

namespace RusSimApp {

    namespace {

        // ── 业务超时 [s] ──

        /** 预扫查任务超时 */
        constexpr double kPreScanTimeout = 300.0;
        /** 正式扫查任务超时 */
        constexpr double kScanTimeout = 600.0;

        /** TaskResult → 业务事件 */
        Event TaskResultToEvent(const TaskResult& r)
        {
            switch (r.status) {
                case TaskStatus::SUCCESS:     return Event::TASK_DONE;
                case TaskStatus::TIMEOUT:     return Event::TASK_TIMEOUT;
                case TaskStatus::CANCELLED:   return Event::TASK_CANCELLED;
                case TaskStatus::INTERRUPTED: return Event::TASK_CANCELLED;
                default:                      return Event::TASK_FAILED;
            }
        }

    }  // namespace

    // ── 构造与生命周期 ──

    AppFsm::AppFsm(Coordinator::SharedPtr coord)
        : coord_(std::move(coord))
        , fsm_(std::make_unique<Fsm<State, Event>>())
    {
        configure();
    }

    void AppFsm::Start()
    {
        // 注入用户指令入口（协调器 WS 指令 → 本状态机）
        coord_->SetUserCommandSink(
            [this](const std::string& name, const std::vector<double>& args) {
                return HandleUserCommand(name, args);
            });

        // 启动即进入待机（INIT 阶段的连接驱动 / 上使能由协调器启动时完成）
        fsm_->SetInitial(State::IDLE);

        running_.store(true);
        fsm_thread_ = std::thread([this]() { fsm_->Run(); });
        RCLCPP_INFO(coord_->get_logger(), "AppFsm 状态机已启动（初始状态 IDLE）");
    }

    void AppFsm::Stop()
    {
        if (!running_.exchange(false))
            return;

        fsm_->Stop();
        if (fsm_thread_.joinable())
            fsm_thread_.join();

        cancel_main_task();
        RCLCPP_INFO(coord_->get_logger(), "AppFsm 状态机已停止");
    }

    // ── 事件入口 ──

    bool AppFsm::HandleUserCommand(const std::string& name, const std::vector<double>& args)
    {
        // ── 操作指令 → 业务事件（STOP / SHUTDOWN 为抢占事件，其余入队） ──
        if (auto evt = StringToEvent(name)) {
            if (*evt == Event::STOP || *evt == Event::SHUTDOWN)
                HandlePreempt(*evt);
            else
                HandleEvent(*evt);
            return true;
        }

        // ── 查询指令（不过状态机，直接经协调器转发） ──
        namespace Cmd = RusUtils::Cmd;
        if (name == Cmd::kQueryPreScanDone || name == Cmd::kQueryMotionDone) {
            coord_->SubmitTask(RusUtils::ParseHighLevelCommand(name, args));
            return true;
        }

        return false;
    }

    void AppFsm::HandleEvent(Event evt)
    {
        fsm_->Post(evt);
    }

    void AppFsm::HandlePreempt(Event evt)
    {
        if (evt == Event::SHUTDOWN) {
            // 关闭：下发 ShutdownCmd + 停止状态机线程（不做转移）
            // 注意：须由外部线程（WS / 主线程）调用，勿在状态机线程内触发
            send_control(RusUtils::ShutdownCmd{});
            Stop();
            return;
        }
        fsm_->Preempt(evt);
    }

    // ── 查询 ──

    State AppFsm::Current() const
    {
        return fsm_->Current();
    }

    bool AppFsm::IsRunning() const
    {
        return running_.load();
    }

    // ── 配置 ──

    void AppFsm::configure()
    {
        // ── 转移表 ──

        // 待机
        fsm_->AddTransition(State::IDLE, Event::START_PRE_SCAN, State::PRE_SCAN_RUNNING);
        fsm_->AddTransition(State::IDLE, Event::START_SCAN, State::SCANNING_RUNNING);

        // 预扫查
        fsm_->AddTransition(State::PRE_SCAN_RUNNING, Event::PAUSE, State::PRE_SCAN_PAUSED,
                            [this] { send_control(RusUtils::PauseCmd{}); });
        fsm_->AddTransition(State::PRE_SCAN_RUNNING, Event::TASK_DONE, State::IDLE);
        fsm_->AddTransition(State::PRE_SCAN_RUNNING, Event::TASK_FAILED, State::ERROR);
        fsm_->AddTransition(State::PRE_SCAN_RUNNING, Event::TASK_TIMEOUT, State::ERROR);
        fsm_->AddTransition(State::PRE_SCAN_RUNNING, Event::TASK_CANCELLED, State::IDLE);
        fsm_->AddTransition(State::PRE_SCAN_RUNNING, Event::STOP, State::IDLE);
        fsm_->AddTransition(State::PRE_SCAN_PAUSED, Event::RESUME, State::PRE_SCAN_RUNNING);
        fsm_->AddTransition(State::PRE_SCAN_PAUSED, Event::STOP, State::IDLE);

        // 正式扫查
        fsm_->AddTransition(State::SCANNING_RUNNING, Event::PAUSE, State::SCANNING_PAUSED,
                            [this] { send_control(RusUtils::PauseCmd{}); });
        fsm_->AddTransition(State::SCANNING_RUNNING, Event::TASK_DONE, State::IDLE);
        fsm_->AddTransition(State::SCANNING_RUNNING, Event::TASK_FAILED, State::ERROR);
        fsm_->AddTransition(State::SCANNING_RUNNING, Event::TASK_TIMEOUT, State::ERROR);
        fsm_->AddTransition(State::SCANNING_RUNNING, Event::TASK_CANCELLED, State::IDLE);
        fsm_->AddTransition(State::SCANNING_RUNNING, Event::STOP, State::IDLE);
        fsm_->AddTransition(State::SCANNING_PAUSED, Event::RESUME, State::SCANNING_RUNNING);
        fsm_->AddTransition(State::SCANNING_PAUSED, Event::STOP, State::IDLE);

        // 错误复位
        fsm_->AddTransition(State::ERROR, Event::RESET, State::IDLE);
        fsm_->AddTransition(State::ERROR, Event::STOP, State::IDLE);

        // ── 生命周期回调 ──

        // 进入执行态：提交主任务（预扫查 / 正式扫查）
        fsm_->SetOnEnter(State::PRE_SCAN_RUNNING,
                         [this] { start_main_task(RusUtils::PreScanStartCmd{}, kPreScanTimeout); });
        fsm_->SetOnEnter(State::SCANNING_RUNNING,
                         [this] { start_main_task(RusUtils::ExecuteCmd{}, kScanTimeout); });

        // 离开执行态：取消主任务等待（暂停 / 停止 / 完成）
        fsm_->SetOnExit(State::PRE_SCAN_RUNNING, [this] { cancel_main_task(); });
        fsm_->SetOnExit(State::SCANNING_RUNNING, [this] { cancel_main_task(); });

        // ── 抢占回调 ──

        // 急停：立即下发 StopCmd（planning stop + driver stop）
        fsm_->SetOnPreempt(Event::STOP,
                           [this] { send_control(RusUtils::StopCmd{}); });
    }

    // ── 任务执行 ──

    void AppFsm::start_main_task(const RusUtils::HighLevelCommand& cmd, double timeout)
    {
        TaskHandle handle = coord_->SubmitTask(cmd);
        {
            std::lock_guard lock(task_mutex_);
            active_handle_ = handle;
        }

        // 监视线程：等待结果 → 业务事件
        std::thread([this, handle, timeout]() {
            TaskResult r = coord_->WaitTask(handle, timeout);
            fsm_->Post(TaskResultToEvent(r));
        }).detach();
    }

    void AppFsm::send_control(const RusUtils::HighLevelCommand& cmd)
    {
        // 控制指令（Pause/Stop/Shutdown）只转发，不参与主任务管理
        coord_->SubmitTask(cmd);
    }

    void AppFsm::cancel_main_task()
    {
        TaskHandle handle = 0;
        {
            std::lock_guard lock(task_mutex_);
            handle = active_handle_;
            active_handle_ = 0;
        }
        if (handle != 0)
            coord_->CancelTask(handle);
    }

}  // namespace RusSimApp
