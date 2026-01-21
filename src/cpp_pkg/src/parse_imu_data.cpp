#include "cpp_pkg/parse_imu_data.h"
#include <rclcpp/rclcpp.hpp>

DataResult ParseImuData::decode_frame(const std::string& frame)
{
    DataResult result;
    sensor_msgs::msg::Imu imu;

    size_t length = static_cast<uint8_t>(frame[1]) - 1;

    if (static_cast<uint8_t>(frame[2]) != IMU_DATA_FLAG || 
        (3 + length) > frame.size()) {
        return result;
    }

    // 提取IMU数据段
    std::string imu_data = frame.substr(3, length);
    if (imu_data.size() < 12) { 
        return result;
    }

    imu.linear_acceleration.x = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[0])) << 8) 
           | static_cast<uint8_t>(imu_data[1]);

    imu.linear_acceleration.y = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[2])) << 8) 
           | static_cast<uint8_t>(imu_data[3]);

    imu.linear_acceleration.z = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[4])) << 8) 
           | static_cast<uint8_t>(imu_data[5]);

    imu.angular_velocity.x = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[6])) << 8) 
           | static_cast<uint8_t>(imu_data[7]);

    imu.angular_velocity.y = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[8])) << 8) 
           | static_cast<uint8_t>(imu_data[9]);

    imu.angular_velocity.z = (static_cast<int16_t>(static_cast<uint8_t>(imu_data[10])) << 8) 
           | static_cast<uint8_t>(imu_data[11]);

    Quaternion_Solution(result); // 计算四元数

    // 标记数据有效
    result.is_valid = true;
    result.data.imu = imu;
    result.data_type = DataType::IMU_DATA;

    return result;
}

void ParseImuData::process_data(const DataResult& result, std::shared_ptr<PublisherNode> pub_node)
{
    if (!result.is_valid || result.data_type != DataType::IMU_DATA) {
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

void ParseImuData::print_imu_data(const DataResult& result) 
{
    if (result.is_valid && result.data_type == DataType::IMU_DATA) {
        std::cout << "IMU Data: ax=" << result.data.imu.linear_acceleration.x 
                    << ", ay=" << result.data.imu.linear_acceleration.y 
                    << ", az=" << result.data.imu.linear_acceleration.z 
                    << ", gx=" << result.data.imu.angular_velocity.x 
                    << ", gy=" << result.data.imu.angular_velocity.y 
                    << ", gz=" << result.data.imu.angular_velocity.z 
                    << ", qw=" << result.data.imu.orientation.w
                    << ", qx=" << result.data.imu.orientation.x
                    << ", qy=" << result.data.imu.orientation.y
                    << ", qz=" << result.data.imu.orientation.z<<std::endl;
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

void ParseImuData::Quaternion_Solution(DataResult& result){
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float gx = result.data.imu.angular_velocity.x, 
          gy = result.data.imu.angular_velocity.y, 
          gz = result.data.imu.angular_velocity.z;
    float ax = result.data.imu.linear_acceleration.x, 
          ay = result.data.imu.linear_acceleration.y, 
          az = result.data.imu.linear_acceleration.z;

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

    result.data.imu.orientation.w = q0;
    result.data.imu.orientation.x = q1;
    result.data.imu.orientation.y = q2;
    result.data.imu.orientation.z = q3;
}