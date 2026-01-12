#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class RobotControl : public rclcpp::Node
{
public:
  RobotControl() : Node("robot_control")
  {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10
    );

    timer_ = this->create_wall_timer(
      1000ms,   // 10 Hz
      std::bind(&RobotControl::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "robot_control node started");
  }

private:
  void timerCallback()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x  = 0.1;
    msg.angular.z = 0.3;
    RCLCPP_INFO(this->get_logger(), "robot_control publish msg");
    cmd_pub_->publish(msg);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotControl>());
  rclcpp::shutdown();
  return 0;
}
