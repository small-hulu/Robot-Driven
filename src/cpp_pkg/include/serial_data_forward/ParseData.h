#ifndef PARSE_DATA_H_
#define PARSE_DATA_H_
#include "sensor_msgs/msg/imu.hpp"
#include "data_result.h"
#include "publisher.h"
#include <string>
#include <iostream>
#include <memory>

class ParseData
{
public:
    using Ptr = std::shared_ptr<ParseData>;
    virtual ~ParseData() = default;

    virtual std::shared_ptr<DataResult> decode_frame(const std::string& frame) = 0; //解析数据帧

    virtual void process_data(const std::shared_ptr<DataResult>& result, std::shared_ptr<PublisherNode> pub_node = nullptr) = 0; //处理数据

    virtual uint8_t get_data_flag() const = 0; //获取数据标识

    //辅助函数
    inline int16_t parse_int16_be(const std::string& data, size_t start){
    return (static_cast<int16_t>(static_cast<uint8_t>(data[start])) << 8) |
            static_cast<uint8_t>(data[start+1]);
    }
    inline uint32_t parse_uint32_be(const std::string& data, size_t start){
    return (static_cast<uint8_t>(data[start]) << 24) |
           (static_cast<uint8_t>(data[start+1]) << 16) |
           (static_cast<uint8_t>(data[start+2]) << 8) |
           static_cast<uint8_t>(data[start+3]);
    }
};

#endif // PARSE_DATA_H_