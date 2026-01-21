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

    virtual DataResult decode_frame(const std::string& frame) = 0; //解析数据帧

    virtual void process_data(const DataResult& result, std::shared_ptr<PublisherNode> pub_node = nullptr) = 0; //处理数据

    virtual uint8_t get_data_flag() const = 0; //获取数据标识
};

#endif // PARSE_DATA_H_