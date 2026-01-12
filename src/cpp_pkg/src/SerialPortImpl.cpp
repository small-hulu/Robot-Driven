#include "cpp_pkg/SerialPortImpl.h"
#include "rclcpp/rclcpp.hpp"
#include <sstream>
#include <iomanip>
using namespace std::chrono_literals;

SerialPortImpl& SerialPortImpl::instance()
{
    static SerialPortImpl inst;
    return inst;
}

SerialPortImpl::SerialPortImpl()
{
    th_worker = std::thread(&SerialPortImpl::run, this);
}
static std::string to_hex(const std::string& data)
{
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');

    for (unsigned char c : data)
    {
        oss << std::setw(2) << static_cast<int>(c) << " ";
    }
    return oss.str();
}

SerialPortImpl::~SerialPortImpl()
{
    stop_recv();

    th_isRunning = false;
    th_taskCV.notify_all();

    if (th_worker.joinable()) {
        th_worker.join();
    }

    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (m_serial.isOpen()) {
        m_serial.close();
    }
}
void SerialPortImpl::log(std::string head,std::string data) 
{ 
    RCLCPP_INFO_STREAM(rclcpp::get_logger(head), data); 
}

void SerialPortImpl::Set_info(const Info& info)
{
    m_info = info;
}

bool SerialPortImpl::open()
{
    auto task = [this]() {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        m_serial.setPort(m_info.port);
        m_serial.setBaudrate(m_info.baud_rate);
        m_serial.setTimeout(m_info.timeout);
        m_serial.open();
    };

    auto fut = Append_Task(task);
    fut->get();
    return m_serial.isOpen();
}

void SerialPortImpl::close()
{
    stop_recv();

    auto task = [this]() {
        std::lock_guard<std::mutex> lock(serial_mutex_);
        if (m_serial.isOpen()) {
            m_serial.close();
        }
    };

    auto fut = Append_Task(task);
    fut->get();
}

bool SerialPortImpl::isOpen()
{
    std::lock_guard<std::mutex> lock(serial_mutex_);
    return m_serial.isOpen();
}

SerialReplyWrapper SerialPortImpl::write_bytes(
    const uint8_t* data, size_t len)
{
    auto task = [this, data, len]() -> SerialReplyWrapper {
        SerialReplyWrapper reply;

        try {
            std::ostringstream oss;
            for (size_t i = 0; i < len; ++i) {
                oss << std::uppercase
                    << std::hex
                    << std::setw(2)
                    << std::setfill('0')
                    << static_cast<int>(data[i])
                    << " ";
            }

            RCLCPP_INFO(
                rclcpp::get_logger("serial_tx"),
                "TX: %s",
                oss.str().c_str()
            );
            std::lock_guard<std::mutex> lock(serial_mutex_);
            m_serial.write(data, len);

            reply.success = true;
        }
        catch (const serial::IOException& e) {
            reply.error_msg = e.what();
        }

        return reply;
    };

    auto fut = Append_Task(task);
    return fut->get();
}

SerialReplyWrapper SerialPortImpl::write(const std::string& request_data)
{
    auto task = [this, request_data]() -> SerialReplyWrapper {
        SerialReplyWrapper reply;
        try {
            log("send msg:",request_data);
            //log(request_data);
            std::lock_guard<std::mutex> lock(serial_mutex_);
            m_serial.write(request_data);
            reply.success = true;
        } catch (const serial::IOException& e) {
            reply.error_msg = e.what();
        }
        return reply;
    };

    auto fut = Append_Task(task);
    return fut->get();
}


void SerialPortImpl::start_recv()
{
    if (recv_running_) return;

    recv_running_ = true;
    recv_thread_ = std::thread(&SerialPortImpl::recv_loop, this);
}

void SerialPortImpl::stop_recv()
{
    recv_running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

void SerialPortImpl::set_recv_callback(RecvCallback cb)
{
    recv_cb_ = cb;
}
// void SerialPortImpl::recv_loop()
// {
//     std::string buffer;
//     log("recv_loop thread","recv_loop thread");

//     while (recv_running_) {
//         try {
//             size_t n = 0;
//             {
//                 std::lock_guard<std::mutex> lock(serial_mutex_);
//                 if (!m_serial.isOpen()) {
//                     std::this_thread::sleep_for(50ms);
//                     continue;
//                 }
//                 n = m_serial.available();
//             }

//             if (n > 0) {
//                 std::string data;
//                 {
//                     std::lock_guard<std::mutex> lock(serial_mutex_);
//                     data = m_serial.read(n);
//                 }
//                 buffer += data;
//                 parse_frame(buffer);
//             } else {
//                 std::this_thread::sleep_for(5ms);
//             }

//         } catch (const serial::IOException& e) {
//             RCLCPP_ERROR(rclcpp::get_logger("serial_rx"), "%s", e.what());
//             std::this_thread::sleep_for(50ms);
//         }
//     }
// }
// void SerialPortImpl::parse_frame(std::string& buffer)
// {
//     size_t pos;
//     while ((pos = buffer.find('\n')) != std::string::npos) {
//         std::string line = buffer.substr(0, pos);
//         buffer.erase(0, pos + 1); 

//         if (recv_cb_) {
//             recv_cb_(line);
//         }
//     }

//     if (buffer.size() > 4096) {
//         buffer.clear();
//     }
// }

void SerialPortImpl::recv_loop()
{
    std::string buffer;
    buffer.reserve(1024);  

    log("recv_loop thread", "recv_loop thread");

    while (recv_running_) {
        try {
            size_t n = 0;
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                if (!m_serial.isOpen()) {
                    std::this_thread::sleep_for(50ms);
                    continue;
                }
                n = m_serial.available();
            }

            if (n > 0) {
                std::string data;
                {
                    std::lock_guard<std::mutex> lock(serial_mutex_);
                    data = m_serial.read(n);   
                }

                buffer.append(data);           
                parse_frame(buffer);            
            } else {
                std::this_thread::sleep_for(5ms);
            }

        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(
                rclcpp::get_logger("serial_rx"),
                "%s",
                e.what()
            );
            std::this_thread::sleep_for(50ms);
        }
    }
}
void SerialPortImpl::parse_frame(std::string& buffer)
{   
    while (buffer.size() >= 5)
    {
        if (static_cast<uint8_t>(buffer[0]) != 0xAA)
        {
            buffer.erase(0, 1);
            continue;
        }
        uint8_t len = static_cast<uint8_t>(buffer[1]);

        if (len < 1)
        {
            buffer.erase(0, 1);
            continue;
        }

        size_t frame_len = len + 4; // AA + LEN + (CMD+DATA) + CHK + 0D

        if (buffer.size() < frame_len)
        {
            return; 
        }

        if (static_cast<uint8_t>(buffer[frame_len - 1]) != 0x0D)
        {
            buffer.erase(0, 1);
            continue;
        }

        uint8_t checksum = 0;
        for (size_t i = 2; i < 2 + len; ++i)
        {
            checksum += static_cast<uint8_t>(buffer[i]);
        }

        uint8_t recv_chk =
            static_cast<uint8_t>(buffer[2 + len]);

        if (checksum != recv_chk)
        {
            RCLCPP_WARN(
                rclcpp::get_logger("serial_rx"),
                "Checksum error, calc=0x%02X recv=0x%02X, raw frame: %s",
                checksum,
                recv_chk,
                to_hex(buffer.substr(0, frame_len)).c_str()
            );
            buffer.erase(0, frame_len);
            continue;
        }

        std::string frame = buffer.substr(0, frame_len);
        buffer.erase(0, frame_len);

        if (recv_cb_)
        {
            recv_cb_(frame);
        }
    }
}

void SerialPortImpl::run()
{
    while (th_isRunning) {
        task_type task;
        {
            std::unique_lock<std::mutex> lock(th_taskMutex);
            th_taskCV.wait(lock, [this] {
                return !m_taskQueue.empty() || !th_isRunning;
            });

            if (!th_isRunning) break;

            task = std::move(m_taskQueue.front());
            m_taskQueue.pop();
        }
        if (task) task();
    }
}

template <typename Fun, typename... Args>
auto SerialPortImpl::Append_Task(Fun&& fun, Args&&... args)
    -> std::shared_ptr<std::future<std::invoke_result_t<Fun, Args...>>>
{
    using return_type = std::invoke_result_t<Fun, Args...>;

    auto task = std::make_shared<std::packaged_task<return_type()>>(
        std::bind(std::forward<Fun>(fun), std::forward<Args>(args)...));

    auto res = std::make_shared<std::future<return_type>>(task->get_future());
    {
        std::lock_guard<std::mutex> lock(th_taskMutex);
        m_taskQueue.emplace([task]() { (*task)(); });
    }
    th_taskCV.notify_one();
    return res;
}
