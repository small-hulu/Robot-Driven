#include "cpp_pkg/turn_on_robot.h"
#include "cpp_pkg/SerialPortImpl.h"
#include "cpp_pkg/publisher.h"
#include <iostream>
#include <vector>
#include <cstdint>
#include <sstream>
#include <iomanip>

static std::string bytes_to_hex(const std::string& data)
{
    std::ostringstream oss;
    oss << std::uppercase << std::hex;

    for (uint8_t b : data) {
        oss << std::setw(2)
            << std::setfill('0')
            << static_cast<int>(b)
            << " ";
    }
    return oss.str();
}

std::vector<uint8_t> pack_frame(uint8_t cmd,const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> frame;

    uint8_t len = 1 + data.size(); 

    frame.push_back(0xAA);     // 帧头
    frame.push_back(len);      // 长度
    frame.push_back(cmd);      // 命令

    uint8_t checksum = cmd;
    for (auto b : data) {
        frame.push_back(b);
        checksum += b;
    }

    frame.push_back(checksum); // 校验
    frame.push_back(0x0D);     // 帧尾

    return frame;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    
    auto& serial = SerialPortImpl::instance();

    Info info;
    info.port = "/dev/ttyS1";
    info.baud_rate = 115200;

    serial.Set_info(info);
    
    auto IMU_node = std::make_shared<Publisher>();

    serial.set_recv_callback(
        [&IMU_node](const std::string& frame) {
            //处理下位机来的信息
            //RCLCPP_INFO(rclcpp::get_logger("rx"),"RX: %s", frame.c_str());
            RCLCPP_INFO(rclcpp::get_logger("serial_rx"),"RX: %s",bytes_to_hex(frame).c_str());
            RCLCPP_INFO(rclcpp::get_logger("serial_rx"),"---------------------------");
            IMU_node->publish_data(bytes_to_hex(frame));
        }
    );
    serial.open();
    serial.start_recv();

    auto robot_node = std::make_shared<turn_on_robot>();
    robot_node->start();
    
    rclcpp::spin(robot_node);
    rclcpp::spin(IMU_node);
    rclcpp::shutdown();
    return 0;
}