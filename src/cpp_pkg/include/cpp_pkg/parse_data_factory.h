#ifndef PARSE_DATA_FACTORY_H_
#define PARSE_DATA_FACTORY_H_

#include "parse_imu_data.h"
#include "publisher.h"
#include <unordered_map>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <memory>

class ParseDataFactory
{
public:
    static ParseDataFactory& instance() //单例模式获取工厂实例
    {
        static ParseDataFactory ins;
        return ins;
    }

    ParseDataFactory(const ParseDataFactory&) = delete; //禁止拷贝构造
    ParseDataFactory& operator=(const ParseDataFactory&) = delete; 

    template <typename ProcessorType>
    void register_processor() //注册数据处理器
    {
        std::lock_guard<std::mutex> lock(mutex_);

        std::shared_ptr<ProcessorType> temp_processor = std::make_shared<ProcessorType>(); //临时实例化以获取数据标识
        uint8_t data_flag = temp_processor->get_data_flag();

        if (processor_map_.find(data_flag) != processor_map_.end()) { //已注册
            throw std::runtime_error("Processor with data flag " + std::to_string(data_flag) + " already registered!");
        }

        processor_map_[data_flag] = []() -> ParseData::Ptr { //返回处理器实例的函数
            return std::make_shared<ProcessorType>();
        };
    }

    ParseData::Ptr create_processor(uint8_t data_flag) //创建数据处理器实例
    {
        std::lock_guard<std::mutex> lock(mutex_); 

        auto it = processor_map_.find(data_flag); //查找对应处理器
        if (it == processor_map_.end()) {
            return nullptr;
        }

        return it->second(); //调用函数创建处理器实例
    }

    void dispatch_process(const std::string& frame, std::shared_ptr<Publisher> pub_node = nullptr) //分发处理数据
    {
        uint8_t data_flag = static_cast<uint8_t>(frame[2]);
        ParseData::Ptr processor = create_processor(data_flag);
        if (!processor) {
        return;
        }
        
        DataResult result = processor->parse_frame(frame); //解析数据帧
        if(!result.is_valid){
            return;
        }

        processor->process_data(result, pub_node); 
    }

private:
    ParseDataFactory() = default;
    ~ParseDataFactory() = default;
    std::mutex mutex_;
    std::unordered_map<uint8_t, std::function<ParseData::Ptr()>> processor_map_;
};

#endif // PARSE_DATA_FACTORY_H_