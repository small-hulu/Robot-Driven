#ifndef PARSE_DATA_H_
#define PARSE_DATA_H_
#include "publisher.h"
#include <string>
#include <iostream>
#include <memory>

struct ImuData                 
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    bool is_valid = false; 
    void print() const
    {
        if (is_valid) {
            std::cout << "IMU Data: ax=" << ax << ", ay=" << ay << ", az=" << az << ", gx=" << gx << ", gy=" << gy << ", gz=" << gz << std::endl;
        }
    }
};

enum class DataType // 数据类型
{
    NONE,       // 无数据
    IMU_DATA,   // IMU数据
};

struct DataResult
{
    DataType data_type = DataType::NONE; // 标记当前存储的数据类型
    bool is_valid = false;     // 标记数据是否有效
    union DataUnion
    {
        ImuData imu;   // IMU数据
        DataUnion() {}
        ~DataUnion() {} 
    } data;
};

class ParseData
{
public:
    using Ptr = std::shared_ptr<ParseData>;
    virtual ~ParseData() = default;

    virtual DataResult parse_frame(const std::string& frame) = 0; //解析数据帧

    virtual void process_data(const DataResult& result, std::shared_ptr<Publisher> pub_node = nullptr) = 0; //处理数据

    virtual uint8_t get_data_flag() const = 0; //获取数据标识
};

#endif // PARSE_DATA_H_