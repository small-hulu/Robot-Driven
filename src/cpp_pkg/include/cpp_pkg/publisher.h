#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>

class Publisher : public rclcpp::Node
{
public:
    Publisher();        
    void publish_data(const std::string& data);
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};


#endif  // PUBLISHER_H_