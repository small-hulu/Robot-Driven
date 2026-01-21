#include "cpp_pkg/turn_on_robot.h"
#include "cpp_pkg/SerialPortImpl.h"
#include "serial_data_forward/publisher.h"
#include "serial_data_forward/parse_data_factory.h"
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

    // 注册IMU数据处理器
    auto& factory = ParseDataFactory::instance();
    factory.register_processor<ParseImuData>();
    RCLCPP_INFO(rclcpp::get_logger("main"), "IMU processor registered successfully.");
    
    // 设置串口
    auto& serial = SerialPortImpl::instance();
    Info info;
    info.port = "/dev/ttyS1";
    info.baud_rate = 115200;
    serial.Set_info(info);
    

    auto publish_node = std::make_shared<PublisherNode>();

    // 设置串口数据处理回调函数
    serial.set_recv_callback(
        [&publish_node](const std::string& frame) {
            ParseDataFactory::instance().dispatch_process(frame, publish_node);
        }
    );
    
    serial.open();
    serial.start_recv();

    auto robot_node = std::make_shared<turn_on_robot>();
    robot_node->start();
    
    rclcpp::spin(robot_node);
    rclcpp::spin(publish_node);
    rclcpp::shutdown();
    return 0;
}