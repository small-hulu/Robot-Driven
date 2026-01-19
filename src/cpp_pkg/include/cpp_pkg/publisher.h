#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>

struct ImuData
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    bool is_valid = false; 

    void print() const {
        RCLCPP_INFO(rclcpp::get_logger("imu_data"),
                "IMU Data - ax: %d, ay: %d, az: %d, gx: %d, gy: %d, gz: %d",
                ax, ay, az, gx, gy, gz);
    }
};

class Publisher : public rclcpp::Node
{
public:
    Publisher();        
    void publish_data(const std::string& data);
    ImuData parse_imu_data(const std::string& frame);
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};


#endif  // PUBLISHER_H_