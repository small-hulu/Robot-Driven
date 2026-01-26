#include "serial_data_forward/publisher.h"

PublisherNode::PublisherNode()
    : Node("data_publisher")
{
    // 创建一个发布者，发布到话题 hug
    string_publisher_ = this->create_publisher<std_msgs::msg::String>("hug", 10);
    imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    odom_data_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
}

// 发布数据
void PublisherNode::publish_string_data(const std::string& data)
{
    string_msg_ptr_->data = data;  
    string_publisher_->publish(*string_msg_ptr_);
}

void PublisherNode::publish_imu_data(const std::shared_ptr<DataResult>& result){
    imu_data_publisher_->publish(result->data.imu);
}

void PublisherNode::publish_odom_data(const std::shared_ptr<DataResult>& result){
    odom_data_publisher_->publish(result->data.odom);
}



