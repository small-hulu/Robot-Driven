#ifndef ICOMTHREAD_HPP_1724744170
#define ICOMTHREAD_HPP_1724744170

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <functional>

class ICOMThread {
public:
    std::atomic<bool> th_isRunning;       
    mutable std::mutex th_taskMutex;      
    std::condition_variable th_taskCV;    

public:
    ICOMThread() : th_isRunning(false) {}

    virtual ~ICOMThread() {
        Stop_thread();
        Clear_threadResource();
    }

    /**
     * 启动线程循环
     * 具体的线程任务由派生类实现
     */
    virtual void Start_thread() = 0;

    /**
     * 停止线程循环
     */
    virtual void Stop_thread() = 0;

    /**
     * 清理线程资源
     * 这里一般用来关闭文件、释放资源等
     */
    virtual void Clear_threadResource() = 0;

protected:
    /**
     * 线程任务的执行体
     * 派生类可以实现具体的任务逻辑
     */
    virtual void Run() = 0;

    /**
     * 锁定任务，防止共享资源的并发访问
     */
    void LockTask() const {
        std::lock_guard<std::mutex> lock(th_taskMutex);
    }

    /**
     * 使用条件变量等待任务，直到被通知
     */
    void WaitForTask() {
        std::unique_lock<std::mutex> lock(th_taskMutex);
        th_taskCV.wait(lock, [this] { return !th_isRunning; });
    }

    /**
     * 通知等待的线程继续工作
     */
    void NotifyTask() {
        th_taskCV.notify_all();
    }
};

#endif  // ICOMTHREAD_HPP_1724744170
