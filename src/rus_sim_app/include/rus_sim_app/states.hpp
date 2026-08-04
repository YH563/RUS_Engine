#pragma once

#include <initializer_list>
#include <memory>
#include <string>
#include <string_view>

#include "yasmin/state.hpp"
#include "yasmin/blackboard.hpp"

#include "rus_sim_utils/command_definitions.hpp"
#include "coordinator.hpp"

namespace RusSimApp {

    // ── 通用 outcome 常量 ──

    /** @brief 通用 outcome 常量 */
    namespace outcome {
        inline constexpr std::string_view kDone     {"done"};
        inline constexpr std::string_view kError    {"error"};
        inline constexpr std::string_view kTimeout  {"timeout"};
        inline constexpr std::string_view kCanceled {"canceled"};
        inline constexpr std::string_view kReady    {"ready"};
        inline constexpr std::string_view kReset    {"reset"};
        inline constexpr std::string_view kRetry    {"retry"};
    }

    /** @brief 将 string_view 列表转换为 yasmin::Outcomes（std::set<std::string>） */
    inline yasmin::Outcomes MakeOutcomes(std::initializer_list<std::string_view> svs) {
        yasmin::Outcomes out;
        for (std::string_view sv : svs)
            out.emplace(sv);
        return out;
    }

    // ── 业务状态：预扫查 / 正式扫查 ──

    /**
     * @brief 预扫查
     *
     * 协调器触发 planning 执行（点云建模 + 前端起终点 + 路径生成），
     * 阻塞等待完成。执行细节全部下沉 planning，本状态只做"请求 + 等待"。
     */
    class PreScanState : public yasmin::State {
    public:
        /**
         * @param coord 协调器
         */
        explicit PreScanState(std::shared_ptr<Coordinator> coord)
            : yasmin::State(MakeOutcomes({outcome::kDone, outcome::kError, outcome::kTimeout}))
            , coord_(std::move(coord)) {}

        /**
         * @brief 触发预扫查并等待完成
         *
         * @param bb 黑板
         * @return outcome::kDone / kError / kTimeout
         */
        std::string execute(yasmin::Blackboard::SharedPtr bb) override {
            (void)bb;
            if (!coord_->ExecutePreScan(kTimeout))
                return std::string(outcome::kError);
            return std::string(outcome::kDone);
        }

    private:
        std::shared_ptr<Coordinator> coord_;
        static constexpr double kTimeout = 300.0;  // [s]
    };

    /**
     * @brief 正式扫查
     *
     * 协调器触发 planning 按路径对 driver 下发执行，阻塞等待完成。
     */
    class ScanState : public yasmin::State {
    public:
        /**
         * @param coord 协调器
         */
        explicit ScanState(std::shared_ptr<Coordinator> coord)
            : yasmin::State(MakeOutcomes({outcome::kDone, outcome::kError}))
            , coord_(std::move(coord)) {}

        /**
         * @brief 触发正式扫查并等待完成
         *
         * @param bb 黑板
         * @return outcome::kDone / kError
         */
        std::string execute(yasmin::Blackboard::SharedPtr bb) override {
            (void)bb;
            if (!coord_->ExecuteScan(kTimeout))
                return std::string(outcome::kError);
            return std::string(outcome::kDone);
        }

    private:
        std::shared_ptr<Coordinator> coord_;
        static constexpr double kTimeout = 600.0;  // [s]
    };

    // ── 顶层状态：初始化 / 待机 / 错误 ──

    /**
     * @brief 初始化
     *
     * 连接驱动、上使能由协调器在启动时完成；本状态简单返回就绪。
     */
    class InitState : public yasmin::State {
    public:
        /**
         * @param coord 协调器
         */
        explicit InitState(std::shared_ptr<Coordinator> coord)
            : yasmin::State(MakeOutcomes({outcome::kReady, outcome::kError}))
            , coord_(std::move(coord)) {}

        /**
         * @brief 返回就绪
         *
         * @param bb 黑板
         * @return outcome::kReady / kError
         */
        std::string execute(yasmin::Blackboard::SharedPtr bb) override {
            (void)bb;
            return std::string(outcome::kReady);
        }

    private:
        std::shared_ptr<Coordinator> coord_;
    };

    /**
     * @brief 待机
     *
     * 阻塞等待用户业务指令（由前端经 user_interface 注入）。
     * 收到 start_scan 时经协调器校验是否已完成预扫查。
     */
    class IdleState : public yasmin::State {
    public:
        /**
         * @param coord 协调器
         */
        explicit IdleState(std::shared_ptr<Coordinator> coord)
            : yasmin::State(MakeOutcomes({kStartPreScan, kStartScan, kShutdown,
                                          kNoPreScan, outcome::kError, outcome::kTimeout}))
            , coord_(std::move(coord)) {}

        /**
         * @brief 等待用户业务指令
         *
         * @param bb 黑板
         * @return 业务指令字符串，或 kNoPreScan / kTimeout / kError
         */
        std::string execute(yasmin::Blackboard::SharedPtr bb) override {
            (void)bb;
            std::string cmd = coord_->WaitForUserCommand(kWaitTimeout);

            // 正式扫查前必须已完成预扫查
            if (cmd == kStartScan && !coord_->IsPreScanDone())
                return std::string(kNoPreScan);
            return cmd;
        }

        static constexpr std::string_view kStartPreScan {RusUtils::Cmd::kPreScanStart};
        static constexpr std::string_view kStartScan    {RusUtils::Cmd::kExecute};
        static constexpr std::string_view kShutdown     {RusUtils::Cmd::kShutdown};
        static constexpr std::string_view kNoPreScan    {"no_prescan"};

    private:
        std::shared_ptr<Coordinator> coord_;
        static constexpr double kWaitTimeout = 3600.0;  // [s]
    };

    /**
     * @brief 错误处理
     *
     * 简化实现：一律复位，回到待机。
     */
    class ErrorState : public yasmin::State {
    public:
        ErrorState() : yasmin::State(MakeOutcomes({outcome::kReset, outcome::kRetry})) {}

        /**
         * @brief 返回复位
         *
         * @param bb 黑板
         * @return outcome::kReset
         */
        std::string execute(yasmin::Blackboard::SharedPtr bb) override {
            (void)bb;
            return std::string(outcome::kReset);
        }
    };

}  // namespace RusSimApp
