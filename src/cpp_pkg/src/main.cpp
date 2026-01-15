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

struct ImuData
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;

    void print() const {
        RCLCPP_INFO(rclcpp::get_logger("imu_data"),
                "IMU Data - ax: %d, ay: %d, az: %d, gx: %d, gy: %d, gz: %d",
                ax, ay, az, gx, gy, gz);
    }
};

ImuData parse_imu_data(const std::string& frame)
{
    ImuData imu;
    uint8_t length = static_cast<uint8_t>(frame[1]) - 1; //减去命令字节的长度
    if (length != 12) {
        throw std::runtime_error("Frame too short to contain IMU data");
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
    return imu;
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
            //RCLCPP_INFO(rclcpp::get_logger("serial_rx"),"RX: %s",bytes_to_hex(frame).c_str());
            //RCLCPP_INFO(rclcpp::get_logger("serial_rx"),"---------------------------");
            ImuData imu_data = parse_imu_data(frame);
            imu_data.print();
            if(imu_data.az > 1050){
                RCLCPP_WARN(rclcpp::get_logger("imu_data"),"az > 1050! ********************** az = %d", imu_data.az);
                IMU_node->publish_data("up");
            }
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