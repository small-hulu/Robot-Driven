#ifndef SERIAL_PORT_IMPL_H_
#define SERIAL_PORT_IMPL_H_

#include <serial/serial.h>
#include <queue>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <future>
#include <string>

struct SerialReplyWrapper {
    std::string data;
    bool success{false};
    std::string error_msg;
};

struct Info {
    std::string port;
    uint32_t baud_rate{115200};
    serial::Timeout timeout{serial::Timeout::simpleTimeout(100)};
};

class SerialPortImpl {
public:
    static SerialPortImpl& instance();

    SerialPortImpl(const SerialPortImpl&) = delete;
    SerialPortImpl& operator=(const SerialPortImpl&) = delete;

    void Set_info(const Info& info);

    bool open();
    void close();
    bool isOpen();

    SerialReplyWrapper write(const std::string& request_data);
    SerialReplyWrapper write_bytes(const uint8_t* data, size_t len);
    void start_recv();
    void stop_recv();

    using RecvCallback = std::function<void(const std::string&)>;
    void set_recv_callback(RecvCallback cb);
    void log(std::string head,std::string data);
private:
    SerialPortImpl();
    ~SerialPortImpl();

    using task_type = std::function<void()>;
    std::thread th_worker;
    std::atomic<bool> th_isRunning{true};
    std::queue<task_type> m_taskQueue;
    std::mutex th_taskMutex;
    std::condition_variable th_taskCV;

    template <typename Fun, typename... Args>
    auto Append_Task(Fun&& fun, Args&&... args)
        -> std::shared_ptr<std::future<std::invoke_result_t<Fun, Args...>>>;

    void run();
    void recv_loop();
    void parse_frame(std::string& buffer);

private:
    serial::Serial m_serial;
    Info m_info;

    std::mutex serial_mutex_;

    std::thread recv_thread_;
    std::atomic<bool> recv_running_{false};
    RecvCallback recv_cb_;

    static constexpr uint8_t FRAME_HEAD = 0x7B;
    static constexpr size_t FRAME_LEN = 11;
};

#endif
