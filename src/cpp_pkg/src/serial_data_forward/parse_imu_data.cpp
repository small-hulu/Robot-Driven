#include "serial_data_forward/parse_imu_data.h"
#include <rclcpp/rclcpp.hpp>

std::shared_ptr<DataResult> ParseImuData::decode_frame(const std::string& frame)
{
    auto result = std::make_shared<DataResult>();
    sensor_msgs::msg::Imu& imu = result->data.imu;

    imu.header.frame_id = "imu_link";  //frame_id设置成imu_link

    size_t length = static_cast<uint8_t>(frame[1]) - 1; // 校验
    if (static_cast<uint8_t>(frame[2]) != IMU_DATA_FLAG || 
        (3 + length) > frame.size()) {
        return result;
    }

    // 提取IMU数据段
    std::string imu_data = frame.substr(3, length);
    if (imu_data.size() < 16) { 
        return result;
    }

    uint32_t imu_ts = parse_uint32_be(imu_data, 0); // 提取时间戳
    uint64_t hw_time_us = imu_ts * 25; // 转换为微秒

    if(!offset_initialized_){
        init_time_offset(hw_time_us);
    }

    int64_t ros_time_us = static_cast<int64_t>(hw_time_us) + time_offset_us_; // 计算ROS时间
    imu.header.stamp.sec = ros_time_us / 1000000;
    imu.header.stamp.nanosec = (ros_time_us % 1000000) * 1000;

    // 解码
    imu.linear_acceleration.x = parse_int16_be(imu_data, 4);  
    imu.linear_acceleration.y = parse_int16_be(imu_data, 6);  
    imu.linear_acceleration.z = parse_int16_be(imu_data, 8); 
    imu.angular_velocity.x = parse_int16_be(imu_data, 10);     
    imu.angular_velocity.y = parse_int16_be(imu_data, 12);     
    imu.angular_velocity.z = parse_int16_be(imu_data, 14);    

    Quaternion_Solution(result); // 计算四元数

/************************************************************************************* */
    imu.linear_acceleration_covariance = { //设置协方差矩阵  ******待测试、标定******
        3.53e-5, 0.0, 0.0,
        0.0, 3.53e-5, 0.0,
        0.0, 0.0, 3.53e-5
    };
    imu.angular_velocity_covariance = {
        1.38e-6, 0.0, 0.0,
        0.0, 1.38e-6, 0.0,
        0.0, 0.0, 1.38e-6
    };
    imu.orientation_covariance = {
        1e-6, 0.0, 0.0,
        0.0, 1e-6, 0.0,
        0.0, 0.0, 1e-6
    };
/************************************************************************************* */

    // 标记数据有效
    result->is_valid = true;
    result->data_type = DataType::IMU_DATA;

    return result;
}

void ParseImuData::process_data(const std::shared_ptr<DataResult>& result, std::shared_ptr<PublisherNode> pub_node)
{
    if (!result->is_valid || result->data_type != DataType::IMU_DATA) {
        return;
    }

    print_imu_data(result);
    if (pub_node) {
        pub_node->publish_imu_data(result); // 发布IMU数据
    }
}

uint8_t ParseImuData::get_data_flag() const
{
    return IMU_DATA_FLAG;
}

void ParseImuData::print_imu_data(const std::shared_ptr<DataResult>& result) 
{
    if (result->is_valid && result->data_type == DataType::IMU_DATA) {
        std::cout << "IMU Data: ax=" << result->data.imu.linear_acceleration.x 
                    << ", ay=" << result->data.imu.linear_acceleration.y 
                    << ", az=" << result->data.imu.linear_acceleration.z 
                    << ", gx=" << result->data.imu.angular_velocity.x 
                    << ", gy=" << result->data.imu.angular_velocity.y 
                    << ", gz=" << result->data.imu.angular_velocity.z 
                    << ", frame_id=" << result->data.imu.header.frame_id << std::endl;
    } else {
        std::cout << "IMU Data: invalid or wrong type" << std::endl;
    }
}

float ParseImuData::InvSqrt(float number){ //平方根倒数，求四元数用到
    volatile float x, y;
    volatile const float f = 1.5F;
    volatile uint32_t i; 
    
    x = number * 0.5F;
    y = number;
    memcpy(const_cast<void*>(static_cast<volatile void*>(&i)), const_cast<const void*>(static_cast<volatile const void*>(&y)), sizeof(i));
    i = 0x5f375a86 - (i >> 1);
    memcpy(const_cast<void*>(static_cast<volatile void*>(&y)), const_cast<const void*>(static_cast<volatile const void*>(&i)), sizeof(y));
    y = y * (f - (x * y * y));
    return y;
}

void ParseImuData::Quaternion_Solution(std::shared_ptr<DataResult>& result){
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float gx = result->data.imu.angular_velocity.x, 
          gy = result->data.imu.angular_velocity.y, 
          gz = result->data.imu.angular_velocity.z;
    float ax = result->data.imu.linear_acceleration.x, 
          ay = result->data.imu.linear_acceleration.y, 
          az = result->data.imu.linear_acceleration.z;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) { // 有加速度数据
        recipNorm = InvSqrt(ax * ax + ay * ay + az * az); // 归一化加速度测量值
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm; 

        // 把四元数换算成方向余弦中的第三行的三个元素
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        //误差是估计的重力方向和测量的重力方向的交叉乘积之和
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 计算并应用积分反馈（如果启用）
        if(twoKi > 0.0f) {
        integralFBx += twoKi * halfex * (1.0f / SAMPLING_FREQ);  // integral error scaled by Ki
        integralFBy += twoKi * halfey * (1.0f / SAMPLING_FREQ);
        integralFBz += twoKi * halfez * (1.0f / SAMPLING_FREQ);
        gx += integralFBx;        // apply integral feedback
        gy += integralFBy;
        gz += integralFBz;
        }
        else {
        integralFBx = 0.0f;       // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
        }
        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / SAMPLING_FREQ));   // pre-multiply common factors
    gy *= (0.5f * (1.0f / SAMPLING_FREQ));
    gz *= (0.5f * (1.0f / SAMPLING_FREQ));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); 
    
    // Normalise quaternion
    recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    result->data.imu.orientation.w = q0;
    result->data.imu.orientation.x = q1;
    result->data.imu.orientation.y = q2;
    result->data.imu.orientation.z = q3;
}

void ParseImuData::init_time_offset(uint64_t hw_time_us) { //计算偏移量
    rclcpp::Time ros_now = rclcpp::Clock().now();  // 获取ROS当前时间
    int64_t ros_time_us = ros_now.nanoseconds() / 1000;    
    time_offset_us_ = ros_time_us - static_cast<int64_t>(hw_time_us);  // 计算偏移量：ROS时间 - 下位机时间
    offset_initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ParseImuData"), 
                "Time offset initialized: %ld us", time_offset_us_);
}
