#pragma once

// ════════════════════════════════════════════════════════════════════
//  轻量线程池（reconstruction：并行融合用）
//  ────────────────────────────────────────────────────────────────────
//  固定工作线程 + 任务队列；ParallelFor 把 [0,count) 切成 chunk 提交并等待。
//  纯 std::thread，无第三方依赖。
// ════════════════════════════════════════════════════════════════════

#include <algorithm>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace RusReconstruction {

    class ThreadPool {
    public:
        /// @param threads 0 = 硬件并发数
        explicit ThreadPool(unsigned threads = 0)
        {
            if (threads == 0) threads = std::thread::hardware_concurrency();
            if (threads == 0) threads = 1;
            workers_.reserve(threads);
            for (unsigned i = 0; i < threads; ++i) {
                workers_.emplace_back([this] { worker_loop(); });
            }
        }

        ~ThreadPool()
        {
            {
                std::lock_guard<std::mutex> lk(m_);
                stop_ = true;
            }
            cv_.notify_all();
            for (auto& t : workers_) {
                if (t.joinable()) t.join();
            }
        }

        ThreadPool(const ThreadPool&) = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;

        unsigned Size() const { return static_cast<unsigned>(workers_.size()); }

        void Submit(std::function<void()> job)
        {
            {
                std::lock_guard<std::mutex> lk(m_);
                jobs_.push(std::move(job));
            }
            cv_.notify_one();
        }

        void WaitIdle()
        {
            std::unique_lock<std::mutex> lk(m_);
            done_cv_.wait(lk, [this] { return jobs_.empty() && active_ == 0; });
        }

        /// 把 [0,count) 按 chunk 切块并行执行 f(begin,end)，等全部完成
        template <typename F>
        void ParallelFor(size_t count, size_t chunk, F&& f)
        {
            if (count == 0) return;
            if (chunk == 0) chunk = 1;
            const size_t njobs = (count + chunk - 1) / chunk;
            for (size_t j = 0; j < njobs; ++j) {
                const size_t b = j * chunk;
                const size_t e = std::min(count, b + chunk);
                Submit([f, b, e] { f(b, e); });
            }
            WaitIdle();
        }

    private:
        void worker_loop()
        {
            for (;;) {
                std::function<void()> job;
                {
                    std::unique_lock<std::mutex> lk(m_);
                    cv_.wait(lk, [this] { return stop_ || !jobs_.empty(); });
                    if (stop_ && jobs_.empty()) return;
                    job = std::move(jobs_.front());
                    jobs_.pop();
                    ++active_;
                }
                job();
                {
                    std::lock_guard<std::mutex> lk(m_);
                    --active_;
                }
                done_cv_.notify_all();
            }
        }

        std::vector<std::thread> workers_;
        std::queue<std::function<void()>> jobs_;
        std::mutex m_;
        std::condition_variable cv_;
        std::condition_variable done_cv_;
        unsigned active_ = 0;
        bool stop_ = false;
    };

}  // namespace RusReconstruction
