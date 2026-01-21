#ifndef DATA_RESULT_H_
#define DATA_RESULT_H_
#include "sensor_msgs/msg/imu.hpp"

enum class DataType // 数据类型
{
    NONE,       // 无数据
    IMU_DATA,   // IMU数据
};

struct DataResult
{
    DataType data_type = DataType::NONE; // 标记当前存储的数据类型
    bool is_valid = false;     // 标记数据是否有效

    struct DataStruct
    {
        sensor_msgs::msg::Imu imu;  //std::optional
        DataStruct() {}
        ~DataStruct() {} 
    } data;
};

#endif // DATA_RESULT_H_