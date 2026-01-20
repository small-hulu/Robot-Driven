#include "cpp_pkg/parse_imu_data.h"
#include <rclcpp/rclcpp.hpp>

DataResult ParseImuData::decode_frame(const std::string& frame)
{
    DataResult result;
    ImuData imu;

    size_t length = static_cast<uint8_t>(frame[1]) - 1;

    if (static_cast<uint8_t>(frame[2]) != IMU_DATA_FLAG || 
        (3 + length) > frame.size()) {
        return result;
    }

    // 提取IMU数据段
    std::string imu_data = frame.substr(3, length);
    if (imu_data.size() < 12) { 
        return result;
    }

    imu.ax = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[0])) << 8) 
           | static_cast<uint8_t>(imu_data[1]);

    imu.ay = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[2])) << 8) 
           | static_cast<uint8_t>(imu_data[3]);

    imu.az = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[4])) << 8) 
           | static_cast<uint8_t>(imu_data[5]);

    imu.gx = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[6])) << 8) 
           | static_cast<uint8_t>(imu_data[7]);

    imu.gy = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[8])) << 8) 
           | static_cast<uint8_t>(imu_data[9]);

    imu.gz = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[10])) << 8) 
           | static_cast<uint8_t>(imu_data[11]);

    // 标记数据有效
    imu.is_valid = true;
    result.is_valid = true;
    result.data.imu = imu;
    result.data_type = DataType::IMU_DATA;

    return result;
}

void ParseImuData::process_data(const DataResult& result, std::shared_ptr<Publisher> pub_node)
{
    if (!result.is_valid || result.data_type != DataType::IMU_DATA || !result.data.imu.is_valid) {
        return;
    }

    const ImuData& imu_data = result.data.imu; //获取imu数据
    imu_data.print();
    if(imu_data.az > 1050){
        RCLCPP_WARN(rclcpp::get_logger("ParseImuData"),"az > 1050! ********************** az = %d", imu_data.az);
        if(pub_node){
            pub_node->publish_data("up");
        }
    }
}

uint8_t ParseImuData::get_data_flag() const
{
    return IMU_DATA_FLAG;
}