#include "cpp_pkg/IDeviceThread.h"
#include <chrono>
#include <thread>

IDeviceThread::IDeviceThread() {}

IDeviceThread::~IDeviceThread() {
    stop();
}

void IDeviceThread::start() {
    if (m_isRunning.exchange(true)) {
        return;
    }
    m_thread = std::thread(&IDeviceThread::thread_loop, this);
}

void IDeviceThread::stop() {
    if (!m_isRunning.exchange(false)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lk(m_taskMutex);
        m_isLooping = false;
    }
    m_taskCV.notify_all();
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void IDeviceThread::set_loopFun(std::vector<AtomicTask> setFun) {
    execSetFuncList   = std::move(setFun);
    assert(execSetFuncList.size());
}

void IDeviceThread::Exec_parallelTask() {
    m_isLooping = false;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(PeriodMSInterval_Task * 2.5)));
    m_isLooping = true;
    m_taskCV.notify_one();
}

RetCode IDeviceThread::Exec_sequencedTask() {
    static std::unique_lock<std::mutex> dummy_lock;
    return operator_task(dummy_lock);
}

RetCode IDeviceThread::operator_task(std::unique_lock<std::mutex>& lock) {
    assert(m_modeIndex < static_cast<int>(execSetFuncList.size()));

    auto& setFunc   = execSetFuncList[m_modeIndex];
    RetCode success = RetCode::Running;
    auto& serial = SerialPortImpl::instance();
    if (!serial.isOpen()) {
        return RetCode::SerialError;
    }

    // ---------- set ----------
    if (setFunc) {
        if (!lock.owns_lock() || m_isLooping) {
            success = setFunc();
        }
    }
    return success;
}
void IDeviceThread::log(std::string head,std::string data) 
{ 
    RCLCPP_INFO_STREAM(rclcpp::get_logger(head), data); 
}
void IDeviceThread::thread_loop() {
    log("IDeviceThread thread", "IDeviceThread thread");
    while (m_isRunning) {
        std::unique_lock<std::mutex> lock(m_taskMutex);
        m_taskCV.wait(lock, [&]() {
            return !m_isRunning || m_isLooping;
        });

        if (!m_isRunning) {
            break;
        }

        operator_task(lock);

        m_isLooping = false;
    }
}
