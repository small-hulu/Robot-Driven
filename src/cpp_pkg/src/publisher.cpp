#include "cpp_pkg/publisher.h"

PublisherNode::PublisherNode()
    : Node("data_publisher")
{
    // 创建一个发布者，发布到话题 hug
    publisher_ = this->create_publisher<std_msgs::msg::String>("hug", 10);
    imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

}

// 发布数据
void PublisherNode::publish_data(const std::string& data)
{
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = data;  
    publisher_->publish(*msg);
}

void PublisherNode::publish_imu_data(const DataResult& result){
    auto msg = std::make_shared<sensor_msgs::msg::Imu>();
    msg->orientation = result.data.imu.orientation;
    msg->angular_velocity = result.data.imu.angular_velocity;
    msg->linear_acceleration = result.data.imu.linear_acceleration;
    imu_data_publisher_->publish(*msg);
}



