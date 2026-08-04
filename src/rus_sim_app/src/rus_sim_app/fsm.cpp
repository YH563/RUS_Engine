#include "rus_sim_app/fsm.hpp"
#include "rus_sim_app/states.hpp"

#include <string>

namespace RusSimApp {

    AppFsm::AppFsm(std::shared_ptr<Coordinator> coord)
        : coord_(std::move(coord))
    {
        // ── 顶层状态机 ──
        root_ = std::make_shared<yasmin::StateMachine>(
            "app_fsm",
            yasmin::Outcomes{std::string(outcome::kDone), std::string(outcome::kError),
                             std::string(outcome::kCanceled)});

        // 初始化：连接驱动、上使能
        root_->add_state(
            "INIT", std::make_shared<InitState>(coord_),
            {{std::string(outcome::kReady), "IDLE"}, {std::string(outcome::kError), "ERROR"}});

        // 待机：等待用户业务指令
        root_->add_state(
            "IDLE", std::make_shared<IdleState>(coord_),
            {{std::string(IdleState::kStartPreScan), "PRE_SCAN"},
             {std::string(IdleState::kStartScan), "SCANNING"},
             {std::string(IdleState::kShutdown), std::string(outcome::kDone)},
             {std::string(IdleState::kNoPreScan), "IDLE"},
             {std::string(outcome::kError), "ERROR"},
             {std::string(outcome::kTimeout), "IDLE"}});

        // 预扫查：触发 planning，完成后标记 prescan_done
        root_->add_state(
            "PRE_SCAN", std::make_shared<PreScanState>(coord_),
            {{std::string(outcome::kDone), "IDLE"},
             {std::string(outcome::kTimeout), "IDLE"},
             {std::string(outcome::kError), "ERROR"}});

        // 正式扫查：触发 planning 执行
        root_->add_state(
            "SCANNING", std::make_shared<ScanState>(coord_),
            {{std::string(outcome::kDone), "IDLE"},
             {std::string(outcome::kError), "ERROR"}});

        // 错误处理：复位
        root_->add_state(
            "ERROR", std::make_shared<ErrorState>(),
            {{std::string(outcome::kReset), "IDLE"}, {std::string(outcome::kRetry), "IDLE"}});

        root_->set_start_state("INIT");
    }

    std::string AppFsm::Run()
    {
        return root_->execute();
    }

    void AppFsm::Cancel()
    {
        root_->cancel_state();
    }

}  // namespace RusSimApp
