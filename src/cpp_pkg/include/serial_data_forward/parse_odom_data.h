#ifndef PARSE_ODOM_DATA_H_
#define PARSE_ODOM_DATA_H_
#include "ParseData.h"
#include <memory>
#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

class ParseOdomData : public ParseData
{
public:
    std::shared_ptr<DataResult> decode_frame(const std::string& frame) override;
    void process_data(const std::shared_ptr<DataResult>& result, std::shared_ptr<PublisherNode> pub_node = nullptr) override;
    uint8_t get_data_flag() const override;

private:
    static const uint8_t ODOM_DATA_FLAG = 0x81; //标志位
    bool offset_initialized_ = false; //时间偏移
    int64_t time_offset_us_ = 0; // 偏移量：ROS时间(us) - 下位机时间(us)
    void init_time_offset(uint64_t hw_time_us);

    static constexpr double WHEEL_BASE = 0.3;         // 轮距
    static constexpr double UPDATE_FREQ = 100.0;      // 更新频率 HZ
    static constexpr double DT = 1.0 / UPDATE_FREQ;   // 时间间隔 0.01s

    double last_x_ = 0.0;    // 上一帧x坐标
    double last_y_ = 0.0;    // 上一帧y坐标
    double last_yaw_ = 0.0;  // 上一帧偏航角

    void calculate_odometry(double v_left_mps, double v_right_mps, nav_msgs::msg::Odometry& odom);
};
#endif // PARSE_ODOM_DATA_H_