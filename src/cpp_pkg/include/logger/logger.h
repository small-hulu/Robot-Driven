#ifndef LOGGER_H_
#define LOGGER_H_
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <sstream>

enum class LogLevel { //日志级别
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

struct LogEvent {             //日志时间结构体
    std::string msg;          // 日志内容
    LogLevel level;           // 日志级别
    std::string time_str;     // 格式化时间字符串
    std::string thread_id;    // 线程ID
    std::string file;         // 文件名
    int line;                 // 行号
};

using LogBuffer = std::vector<char>;
const size_t kBufferSize = 4 * 1024 * 1024;

class AsyncLogger{
public:
    static AsyncLogger& Instance(){
        static AsyncLogger logger;
        return logger;
    }

    void Flush();

    void Init(const std::string& log_dir, int flush_interval = 1);

    void Write(LogLevel level, const std::string& msg, const char* file, int line);

    void Stop();

private:
    AsyncLogger(); // 构造/析构
    ~AsyncLogger();

    AsyncLogger(const AsyncLogger&) = delete; // 禁止拷贝和赋值
    AsyncLogger& operator=(const AsyncLogger&) = delete;

    void CreateLogFile(); // 创建日志文件
    std::string FormatTime(); // 格式化时间
    std::string FormatLog(const LogEvent& event); //格式化日志内容

    void BackendThreadFunc(); // 后台消费者线程函数

private:
    std::string log_dir_ = ""; // 日志存储目录
    int flush_interval_ = 3; // 日志刷新时间
    std::atomic<bool> running_{false}; // 线程运行标志
    std::thread backend_thread_;  //后台线程
    std::mutex mutex_;   // 保护缓冲的互斥锁
    std::condition_variable cond_;   // 线程同步条件变量
    LogBuffer current_buf_;  // 当前写入缓冲
    LogBuffer next_buf_;     // 备用缓冲
    std::ofstream log_file_; // 日志文件流
};

// 日志宏定义
#define LOG_DEBUG(msg) AsyncLogger::Instance().Write(LogLevel::DEBUG, msg, __FILE__, __LINE__)
#define LOG_INFO(msg)  AsyncLogger::Instance().Write(LogLevel::INFO,  msg, __FILE__, __LINE__)
#define LOG_WARN(msg)  AsyncLogger::Instance().Write(LogLevel::WARN,  msg, __FILE__, __LINE__)
#define LOG_ERROR(msg) AsyncLogger::Instance().Write(LogLevel::ERROR, msg, __FILE__, __LINE__)
#define LOG_FATAL(msg) AsyncLogger::Instance().Write(LogLevel::FATAL, msg, __FILE__, __LINE__)

#endif //LOGGER_H_