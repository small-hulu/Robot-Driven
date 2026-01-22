#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#include "data_result.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <string>

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode();
    void publish_string_data(const std::string& data);
    void publish_imu_data(const DataResult& result);
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
    
    //复用，避免不断的开辟/销毁智能指针
    std::shared_ptr<std_msgs::msg::String> string_msg_ptr_ = std::make_shared<std_msgs::msg::String>();
    std::shared_ptr<sensor_msgs::msg::Imu> imu_msg_ptr_ = std::make_shared<sensor_msgs::msg::Imu>();
};


#endif  // PUBLISHER_H_