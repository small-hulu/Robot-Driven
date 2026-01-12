#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>
#include <cassert>
#include "cpp_pkg/CRetCode.h"
#include "cpp_pkg/SerialPortImpl.h"
#include "rclcpp/rclcpp.hpp"
class IDeviceThread {
public:
    using AtomicTask = std::function<RetCode(void)>;

protected:
    static constexpr int PeriodMSInterval_Task = 20;

protected:
    std::atomic<int>  m_modeIndex{0};
    std::atomic<bool> m_isLooping{false};
    std::atomic<bool> m_isRunning{false};

    std::vector<AtomicTask> execSetFuncList;

    std::thread              m_thread;
    std::mutex               m_taskMutex;
    std::condition_variable  m_taskCV;

public:
    IDeviceThread();
    virtual ~IDeviceThread();

    void start();
    void stop();

    void set_loopFun(std::vector<AtomicTask> setFun);

    int get_currentMode() const {
        return m_modeIndex.load();
    }
    void log(std::string head,std::string data);
public:
    /// 并行执行（唤醒线程）
    void Exec_parallelTask();

    /// 串行执行（当前线程）
    RetCode Exec_sequencedTask();

protected:
    RetCode operator_task(std::unique_lock<std::mutex>& lock);

    void thread_loop();
};

