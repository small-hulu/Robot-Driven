#ifndef PARSE_IMU_DATA_H_
#define PARSE_IMU_DATA_H_
#include "ParseData.h"
#include <cstring>
#include <cstdint> 

class ParseImuData : public ParseData
{
public:
    DataResult decode_frame(const std::string& frame) override;
    void process_data(const DataResult& result, std::shared_ptr<PublisherNode> pub_node = nullptr) override;
    uint8_t get_data_flag() const override;

private:
    void print_imu_data(const DataResult& result);
    static float InvSqrt(float number);
    void Quaternion_Solution(DataResult& result);

    static const uint8_t IMU_DATA_FLAG = 0x11;

    static constexpr float SAMPLING_FREQ = 20.0f; // 采样频率
    volatile float twoKp = 1.0f;    // 比例反馈系数
    volatile float twoKi = 0.0f;    // 积分反馈系数
    volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // 四元数初始值
    volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // 积分反馈初始值
};
#endif // PARSE_IMU_DATA_H_