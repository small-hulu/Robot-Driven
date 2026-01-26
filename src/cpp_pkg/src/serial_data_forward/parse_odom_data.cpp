#include "serial_data_forward/parse_odom_data.h"
#include <rclcpp/rclcpp.hpp>

std::shared_ptr<DataResult> ParseOdomData::decode_frame(const std::string& frame)
{
    auto result = std::make_shared<DataResult>();
    nav_msgs::msg::Odometry& odom = result->data.odom;

    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    size_t length = static_cast<uint8_t>(frame[1]) - 1; // 校验
    if (static_cast<uint8_t>(frame[2]) != ODOM_DATA_FLAG || 
        (3 + length) > frame.size()) {
        return result;
    }

    // 提取里程计数据段
    std::string odom_data = frame.substr(3, length);
    if(odom_data.size() < 4){
        return result;
    }

    // uint32_t odom_ts = parse_uint32_be(odom_data, 0); // 提取时间戳
    // uint64_t hw_time_us = odom_ts * 25; // 转换为微秒

    // if(!offset_initialized_){
    //     init_time_offset(hw_time_us);
    // }

    // int64_t ros_time_us = static_cast<int64_t>(hw_time_us) + time_offset_us_; // 计算ROS时间
    // odom.header.stamp.sec = ros_time_us / 1000000;
    // odom.header.stamp.nanosec = (ros_time_us % 1000000) * 1000;

    double v_left = static_cast<double>(parse_int16_be(odom_data, 0)) * 0.001; // mm/s to m/s
    double v_right = static_cast<double>(parse_int16_be(odom_data, 2)) * 0.001; 

    RCLCPP_INFO(rclcpp::get_logger("ParseOdomData"),
            "- 左轮: %.3f m/s, 右轮: %.3f m/s", 
            v_left, v_right);

    calculate_odometry(v_left, v_right, odom);

    result->is_valid = true;
    result->data_type = DataType::ODOM_DATA;
    
    return result;
}

void ParseOdomData::process_data(const std::shared_ptr<DataResult>& result, std::shared_ptr<PublisherNode> pub_node)
{
    if (!result->is_valid || result->data_type != DataType::ODOM_DATA){
        return;
    }
    if(pub_node){
        pub_node->publish_odom_data(result);
    }
} 

uint8_t ParseOdomData::get_data_flag() const
{
    return ODOM_DATA_FLAG;
}

void ParseOdomData::init_time_offset(uint64_t hw_time_us) {
    rclcpp::Time ros_now = rclcpp::Clock().now();  // 获取ROS当前时间
    int64_t ros_time_us = ros_now.nanoseconds() / 1000;    
    time_offset_us_ = ros_time_us - static_cast<int64_t>(hw_time_us);  // 计算偏移量：ROS时间 - 下位机时间
    offset_initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ParseOdomData"),
                "Initialized time offset: %ld us", time_offset_us_);
}

void ParseOdomData::calculate_odometry(double v_left_mps, double v_right_mps, nav_msgs::msg::Odometry& odom) {
    double linear_x = (v_left_mps + v_right_mps) / 2.0;  // 前进线速度
    double angular_z = (v_right_mps - v_left_mps) / WHEEL_BASE; // 角速度

    double delta_yaw = angular_z * DT;                  // 角度变化量
    double delta_x = linear_x * cos(last_yaw_) * DT;    // x方向位移
    double delta_y = linear_x * sin(last_yaw_) * DT;    // y方向位移

    last_x_ += delta_x;
    last_y_ += delta_y;
    last_yaw_ += delta_yaw;

    last_yaw_ = std::fmod(last_yaw_ + M_PI, 2 * M_PI) - M_PI;  // 归一化偏航角

    // 位置
    odom.pose.pose.position.x = last_x_;
    odom.pose.pose.position.y = last_y_;
    odom.pose.pose.position.z = 0.0;

    // 将偏航角转换为四元数
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, last_yaw_); // 滚转/俯仰=0，仅偏航
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // 填充里程计的速度
    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = angular_z;

    // 填充协方差
    std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
    odom.pose.covariance[0] = 0.01;    // x位置误差
    odom.pose.covariance[7] = 0.01;    // y位置误差
    odom.pose.covariance[35] = 0.01;   // 偏航角误差
    // 无意义的维度设为大值，避免误导
    odom.pose.covariance[14] = 10000.0;
    odom.pose.covariance[21] = 10000.0;
    odom.pose.covariance[28] = 10000.0;

    std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
    odom.twist.covariance[0] = 0.001;   // 线速度x的误差
    odom.twist.covariance[35] = 0.001;  // 角速度z的误差
    // 无意义的维度设为大值
    odom.twist.covariance[7] = 10000.0;  // linear.y
    odom.twist.covariance[14] = 10000.0; // linear.z
    odom.twist.covariance[21] = 10000.0; // angular.x
    odom.twist.covariance[28] = 10000.0; // angular.y
}