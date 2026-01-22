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
    void publish_data(const std::string& data);
    void publish_imu_data(const DataResult& result);
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
};


#endif  // PUBLISHER_H_