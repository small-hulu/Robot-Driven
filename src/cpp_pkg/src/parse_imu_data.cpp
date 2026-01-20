#include "cpp_pkg/parse_imu_data.h"
#include <rclcpp/rclcpp.hpp>

DataResult ParseImuData::decode_frame(const std::string& frame)
{
    DataResult result;
    sensor_msgs::msg::Imu imu;

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

    imu.linear_acceleration.x = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[0])) << 8) 
           | static_cast<uint8_t>(imu_data[1]);

    imu.linear_acceleration.y = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[2])) << 8) 
           | static_cast<uint8_t>(imu_data[3]);

    imu.linear_acceleration.z = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[4])) << 8) 
           | static_cast<uint8_t>(imu_data[5]);

    imu.angular_velocity.x = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[6])) << 8) 
           | static_cast<uint8_t>(imu_data[7]);

    imu.angular_velocity.y = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[8])) << 8) 
           | static_cast<uint8_t>(imu_data[9]);

    imu.angular_velocity.z = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[10])) << 8) 
           | static_cast<uint8_t>(imu_data[11]);

    // 标记数据有效
    result.is_valid = true;
    result.data.imu = imu;
    result.data_type = DataType::IMU_DATA;

    return result;
}

void ParseImuData::process_data(const DataResult& result, std::shared_ptr<Publisher> pub_node)
{
    if (!result.is_valid || result.data_type != DataType::IMU_DATA) {
        return;
    }

    sensor_msgs::msg::Imu imu_data = result.data.imu; //获取imu数据
    print_imu_data(result);
    if(imu_data.linear_acceleration.z > 1050){
        RCLCPP_WARN(rclcpp::get_logger("ParseImuData"),"az > 1050! ********************** az = %f", imu_data.linear_acceleration.z);
        if(pub_node){
            pub_node->publish_data("up");
        }
    }
}

uint8_t ParseImuData::get_data_flag() const
{
    return IMU_DATA_FLAG;
}

void ParseImuData::print_imu_data(const DataResult& result) 
{
    if (result.is_valid && result.data_type == DataType::IMU_DATA) {
        std::cout << "IMU Data: ax=" << result.data.imu.linear_acceleration.x 
                    << ", ay=" << result.data.imu.linear_acceleration.y 
                    << ", az=" << result.data.imu.linear_acceleration.z 
                    << ", gx=" << result.data.imu.angular_velocity.x 
                    << ", gy=" << result.data.imu.angular_velocity.y 
                    << ", gz=" << result.data.imu.angular_velocity.z << std::endl;
    } else {
        std::cout << "IMU Data: invalid or wrong type" << std::endl;
    }
}