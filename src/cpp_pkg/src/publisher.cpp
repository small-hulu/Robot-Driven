#include "cpp_pkg/publisher.h"

Publisher::Publisher()
    : Node("data_publisher")
{
    // 创建一个发布者，发布到话题 hug
    publisher_ = this->create_publisher<std_msgs::msg::String>("hug", 10);
}

// 发布数据
void Publisher::publish_data(const std::string& data)
{
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = data;  
    publisher_->publish(*msg);
}




