#include "cpp_pkg/publisher.h"

Publisher::Publisher()
    : Node("IMU_data_publisher")
{
    // 创建一个发布者，发布到话题 IMU_data_topic
    publisher_ = this->create_publisher<std_msgs::msg::String>("hug", 10);
}

Publisher::~Publisher()
{
}

// 发布数据
void Publisher::publish_data(const std::string& data)
{
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = data;  
    publisher_->publish(*msg);
}

ImuData Publisher::parse_imu_data(const std::string& frame)
{
    ImuData imu;
    uint8_t length = static_cast<uint8_t>(frame[1]) - 1; //减去命令字节的长度
    if (frame[2] != 0x11) {
        return imu; //返回空数据
    }
    std::string imu_data = frame.substr(3, length); //imu数据

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

    imu.is_valid = true; // 标记数据为有效
    return imu;
}



