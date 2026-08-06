#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <utility>

namespace RusSimApp {

    /** @brief pair 哈希（std::hash 未提供 pair 特化，C++17 需自行实现） */
    struct PairHash {
        template<typename A, typename B>
        std::size_t operator()(const std::pair<A, B>& p) const
        {
            std::size_t h1 = std::hash<A>{}(p.first);
            std::size_t h2 = std::hash<B>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };

    /**
     * @brief 事件驱动状态机核心
     *
     * 模型：外部事件入队 → 查转移表 → on_exit / 切换 / on_enter。
     *
     * 特性：
     *   1. 事件驱动：外部指令任意线程可 Post，状态机线程处理
     *   2. 抢占事件：Preempt 立即执行抢占回调（如取消当前任务）并优先转移
     *   3. 生命周期回调：on_enter（提交任务）/ on_exit（暂停/取消任务）
     *   4. 无转移的事件自动忽略（转移表即合法性约束）
     *
     * @tparam State 状态类型（业务层枚举）
     * @tparam Event 事件类型（业务层枚举）
     */
    template<typename State, typename Event>
    class Fsm {
    public:
        /** 生命周期 / 抢占回调 */
        using Callback = std::function<void()>;

        // ── 配置（启动前注册） ──

        /**
         * @brief 注册转移：当前状态 + 事件 → 下一状态
         */
        void AddTransition(State from, Event evt, State to);

        /**
         * @brief 注册转移并绑定转移动作（切换前执行，如暂停时下发 PauseCmd）
         */
        void AddTransition(State from, Event evt, State to, Callback action);

        /**
         * @brief 设置初始状态（Run 前调用）
         */
        void SetInitial(State s);

        /**
         * @brief 注册状态进入回调（通常用于提交任务）
         */
        void SetOnEnter(State s, Callback cb);

        /**
         * @brief 注册状态离开回调（通常用于暂停 / 取消任务）
         */
        void SetOnExit(State s, Callback cb);

        /**
         * @brief 注册抢占事件回调（Preempt 时立即执行，如取消当前任务）
         */
        void SetOnPreempt(Event evt, Callback cb);

        // ── 事件入口（任意线程可调用） ──

        /**
         * @brief 投递普通事件（入队，状态机循环处理）
         */
        void Post(Event evt);

        /**
         * @brief 投递抢占事件（立即执行抢占回调 + 优先转移）
         */
        void Preempt(Event evt);

        // ── 运行 ──

        /**
         * @brief 阻塞运行事件循环（通常主线程）
         */
        void Run();

        /**
         * @brief 停止事件循环
         */
        void Stop();

        // ── 查询 ──

        /**
         * @brief 当前状态
         */
        State Current() const;

    private:
        /** @brief 处理一次事件：查表 → on_exit → 切换 → on_enter */
        void handle(Event evt);

        // 转移表：{状态, 事件} → 下一状态
        std::unordered_map<std::pair<State, Event>, State, PairHash> transitions_;

        // 转移动作：{状态, 事件} → 切换前执行（如暂停时下发 PauseCmd）
        std::unordered_map<std::pair<State, Event>, Callback, PairHash> transition_actions_;

        // 生命周期回调
        std::unordered_map<State, Callback> on_enter_;
        std::unordered_map<State, Callback> on_exit_;

        // 抢占事件回调
        std::unordered_map<Event, Callback> on_preempt_;

        // 事件队列（线程安全）
        std::queue<Event> queue_;
        std::mutex mtx_;
        std::condition_variable cv_;

        // 抢占标志
        std::atomic<bool> preempt_pending_{false};
        Event preempt_evt_{};

        // 运行标志 / 当前状态
        std::atomic<bool> running_{true};
        std::atomic<State> current_{};
    };

    // ============================================================
    //  模板实现
    // ============================================================

    template<typename State, typename Event>
    void Fsm<State, Event>::AddTransition(State from, Event evt, State to)
    {
        transitions_[{from, evt}] = to;
    }

    template<typename State, typename Event>
    void Fsm<State, Event>::AddTransition(State from, Event evt, State to, Callback action)
    {
        transitions_[{from, evt}] = to;
        transition_actions_[{from, evt}] = std::move(action);
    }

    template<typename State, typename Event>
    void Fsm<State, Event>::SetInitial(State s)
    {
        current_.store(s);
    }

    template<typename State, typename Event>
    void Fsm<State, Event>::SetOnEnter(State s, Callback cb)
    {
        on_enter_[s] = std::move(cb);
    }

    template<typename State, typename Event>
    void Fsm<State, Event>::SetOnExit(State s, Callback cb)
    {
        on_exit_[s] = std::move(cb);
    }

    template<typename State, typename Event>
    void Fsm<State, Event>::SetOnPreempt(Event evt, Callback cb)
    {
        on_preempt_[evt] = std::move(cb);
    }

    template<typename State, typename Event>
    void Fsm<State, Event>::Post(Event evt)
    {
        {
            std::lock_guard lock(mtx_);
            queue_.push(evt);
        }
        cv_.notify_one();
    }

    template<typename State, typename Event>
    void Fsm<State, Event>::Preempt(Event evt)
    {
        // 立即执行抢占回调（取消当前任务，打断阻塞）
        auto it = on_preempt_.find(evt);
        if (it != on_preempt_.end())
            it->second();

        // 标记抢占事件，Run 循环优先处理
        {
            std::lock_guard lock(mtx_);
            preempt_evt_ = evt;
            preempt_pending_ = true;
        }
        cv_.notify_one();
    }

    template<typename State, typename Event>
    void Fsm<State, Event>::Run()
    {
        while (running_.load()) {
            // ① 抢占事件优先
            if (preempt_pending_.exchange(false)) {
                handle(preempt_evt_);
                continue;
            }

            // ② 普通事件队列
            Event evt{};
            {
                std::unique_lock lock(mtx_);
                cv_.wait(lock, [this] {
                    return !queue_.empty() || preempt_pending_.load() || !running_.load();
                });
                if (!running_.load()) break;
                if (preempt_pending_.load()) continue;  // 抢占优先，回到循环头
                if (queue_.empty()) continue;
                evt = queue_.front();
                queue_.pop();
            }
            handle(evt);
        }
    }

    template<typename State, typename Event>
    void Fsm<State, Event>::Stop()
    {
        running_.store(false);
        cv_.notify_all();
    }

    template<typename State, typename Event>
    State Fsm<State, Event>::Current() const
    {
        return current_.load();
    }

    template<typename State, typename Event>
    void Fsm<State, Event>::handle(Event evt)
    {
        State cur = current_.load();
        auto it = transitions_.find({cur, evt});
        if (it == transitions_.end())
            return;  // 无转移：忽略（转移表即合法性）

        // 离开当前状态
        auto ex = on_exit_.find(cur);
        if (ex != on_exit_.end())
            ex->second();

        // 转移动作（切换前，如暂停时下发 PauseCmd）
        auto ta = transition_actions_.find({cur, evt});
        if (ta != transition_actions_.end())
            ta->second();

        // 切换状态
        current_.store(it->second);

        // 进入新状态
        auto en = on_enter_.find(it->second);
        if (en != on_enter_.end())
            en->second();
    }

}  // namespace RusSimApp
