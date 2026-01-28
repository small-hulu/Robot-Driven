#ifndef THREAD_POOL_H_
#define THREAD_POOL_H_
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>
#include <stdexcept>
#include <future>
#include <memory>

class ThreadPool {
public:
    using Task = std::function<void()>;
    explicit ThreadPool(size_t thread_num = 4) : is_running_(true){
        for (size_t i = 0; i < thread_num; ++i) {
            workers_.emplace_back([this]() {
                while (is_running_) {
                    Task task;
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex_);
                        cv_.wait(lock, [this]() {
                            return !is_running_ || !tasks_.empty();
                        });

                        if (!is_running_ && tasks_.empty()) {
                            return;
                        }

                        task = std::move(tasks_.front());
                        tasks_.pop();
                    }

                    task();
                }
            });
        }
    }

    ~ThreadPool() {
        stop();
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;
    ThreadPool(ThreadPool&&) = delete;
    ThreadPool& operator=(ThreadPool&&) = delete;

    template<typename F, typename... Args>
    void enqueue(F&& f, Args&&... args) {
        if (!is_running_) {
            throw std::runtime_error("线程池已停止，无法添加任务！");
        }

        auto task = std::bind(std::forward<F>(f), std::forward<Args>(args)...);

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (tasks_.size() > 2000) {
                fprintf(stderr, "线程池任务队列满，丢弃旧任务！\n");
                tasks_.pop();
            }
            tasks_.emplace(std::move(task));
        }
        cv_.notify_one();
    }

    void stop(){
        if(!is_running_.exchange(false)){
            return;
        }
        cv_.notify_all();
        for(auto& worker : workers_){
            if(worker.joinable()){
                worker.join();
            }
        }

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            while(!tasks_.empty()){
                tasks_.pop();
            }
        }
    }

private:
    std::vector<std::thread> workers_; // 工作线程列表
    std::queue<Task> tasks_;   // 任务队列
    mutable std::mutex queue_mutex_;  // 保护队列的锁
    std::condition_variable cv_; // 条件变量
    std::atomic<bool> is_running_; // 运行状态
};



#endif  // THREAD_POOL_H_