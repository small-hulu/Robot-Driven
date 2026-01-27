#include "logger/logger.h"
AsyncLogger::AsyncLogger(){
    current_buf_.reserve(kBufferSize); 
    next_buf_.reserve(kBufferSize);
}

AsyncLogger::~AsyncLogger(){
    Stop();
}

void AsyncLogger::Flush() { //只会在捕获Ctrl+C信号之后执行
    LogBuffer write_buf;
    { //降低锁粒度
        std::lock_guard<std::mutex> lock(mutex_);
        write_buf.swap(current_buf_);
        if (!next_buf_.empty()) {
            LogBuffer temp_buf;
            temp_buf.swap(next_buf_);
            write_buf.insert(write_buf.end(), temp_buf.begin(), temp_buf.end());
        }
    }

    if (!write_buf.empty() && log_file_.is_open()) {
        log_file_.write(write_buf.data(), write_buf.size());
        log_file_.flush(); // 强制刷入磁盘
    }
}

void AsyncLogger::Init(const std::string& log_dir, int flush_interval){
    if (running_) {
        return;
    }

    log_dir_ = log_dir;
    flush_interval_ = flush_interval;

    CreateLogFile(); //创建日志文件

    running_ = true;
    backend_thread_ = std::thread(&AsyncLogger::BackendThreadFunc, this); // 启动后端写入线程
    LOG_INFO("AsyncLogger initialized successfully! Log dir: " + log_dir_);
}

void AsyncLogger::Write(LogLevel level, const std::string& msg, const char* file, int line) {
    // 构造日志事件
    LogEvent event;
    event.level = level;
    event.msg = msg;
    event.time_str = FormatTime();
    //event.thread_id = std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
    event.file = file;
    event.line = line;

    // 格式化日志字符串
    std::string log_str = FormatLog(event);
    const char* data = log_str.c_str();
    size_t len = log_str.size();

    // 加锁写入缓冲
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_buf_.size() + len < kBufferSize) {
        // 当前缓冲有足够空间，直接写入
        current_buf_.insert(current_buf_.end(), data, data + len);
    } else {
        // 当前缓冲满，切换双缓冲并唤醒后端线程
        current_buf_.swap(next_buf_);
        current_buf_.clear();
        current_buf_.insert(current_buf_.end(), data, data + len);
        cond_.notify_one();
    }
}

// 停止日志器
void AsyncLogger::Stop() {
    if (!running_) {
        return;
    }

    running_ = false;
    cond_.notify_one(); // 唤醒后端线程
    if (backend_thread_.joinable()) {
        backend_thread_.join();
    }
}

void AsyncLogger::CreateLogFile(){ // 创建日志文件
    struct stat st; //检查日志目录是否存在
    if (stat(log_dir_.c_str(), &st) != 0) { //目录不存在
        if (mkdir(log_dir_.c_str(), 0755) != 0) {
            fprintf(stderr, "Failed to create log dir: %s\n", log_dir_.c_str());
            return;
        }
    }

    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_ = *std::localtime(&t);
    std::ostringstream oss;
    oss << log_dir_ << "/log_" 
        << std::put_time(&tm_, "%Y%m%d") 
        << ".txt";
    
    log_file_.open(oss.str(), std::ios::app | std::ios::out);
    if (!log_file_.is_open()) {
        fprintf(stderr, "Failed to open log file: %s\n", oss.str().c_str());
    }
}

std::string AsyncLogger::FormatTime() { //格式化日志时间
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ) % 1000;

    std::tm tm_ = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm_, "%Y-%m-%d %H:%M:%S") 
        << "." << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

std::string AsyncLogger::FormatLog(const LogEvent& event) { //格式化日志内容
    const char* level_str[] = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
    std::ostringstream oss;
    oss << event.time_str << " [" << level_str[(int)event.level] << "] "
        << event.file.substr(event.file.find_last_of("/\\") + 1) << ":" << event.line << " - "
        << event.msg << "\n";
    return oss.str();
}

void AsyncLogger::BackendThreadFunc() {
    while (running_) {
        LogBuffer write_buf;

        {
            std::unique_lock<std::mutex> lock(mutex_);
            cond_.wait_for(lock, std::chrono::seconds(flush_interval_), [this]() {
                return !current_buf_.empty() || !running_;
            });

            if (!running_ && current_buf_.empty()) {
                break;
            }

            write_buf.swap(current_buf_);
            if (!next_buf_.empty()) {
                LogBuffer temp_buf;
                temp_buf.swap(next_buf_);
                write_buf.insert(write_buf.end(), temp_buf.begin(), temp_buf.end());
            }
        }

        if (!write_buf.empty() && log_file_.is_open()) {
            log_file_.write(write_buf.data(), write_buf.size());
            if(write_buf.size() >= 4 * 1024){
                log_file_.flush();
            }
            write_buf.clear();
        }
    }

    LogBuffer write_buf;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!current_buf_.empty()) {
            write_buf.insert(write_buf.end(), current_buf_.begin(), current_buf_.end());
            current_buf_.clear();
        }
        if (!next_buf_.empty()) {
            write_buf.insert(write_buf.end(), next_buf_.begin(), next_buf_.end());
            next_buf_.clear();
        }
    }

    // 写入最后剩余数据
    if (!write_buf.empty() && log_file_.is_open()) {
        log_file_.write(write_buf.data(), write_buf.size());
        log_file_.flush();
    }

    if (log_file_.is_open()) {
        log_file_.close();
    }
}
