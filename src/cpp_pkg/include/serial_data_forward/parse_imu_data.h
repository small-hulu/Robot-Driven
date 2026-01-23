#ifndef PARSE_IMU_DATA_H_
#define PARSE_IMU_DATA_H_
#include "ParseData.h"
#include <cstring>
#include <cstdint> 

class ParseImuData : public ParseData
{
public:
    std::shared_ptr<DataResult> decode_frame(const std::string& frame) override;
    void process_data(const std::shared_ptr<DataResult>& result, std::shared_ptr<PublisherNode> pub_node = nullptr) override;
    uint8_t get_data_flag() const override;

private:
    static const uint8_t IMU_DATA_FLAG = 0x11; //标志位

    void print_imu_data(const std::shared_ptr<DataResult>& result); // 打印

    static float InvSqrt(float number); 
    void Quaternion_Solution(std::shared_ptr<DataResult>& result); // 计算四元数
    static constexpr float SAMPLING_FREQ = 20.0f; // 采样频率
    volatile float twoKp = 1.0f;    // 比例反馈系数
    volatile float twoKi = 0.0f;    // 积分反馈系数
    volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // 四元数初始值
    volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // 积分反馈初始值

    bool offset_initialized_ = false; //时间偏移
    int64_t time_offset_us_ = 0; // 偏移量：ROS时间(us) - 下位机时间(us)

    //辅助函数
    inline int16_t parse_int16_be(const std::string& data, size_t start);
    inline uint32_t parse_uint32_be(const std::string& data, size_t start);

    void init_time_offset(uint64_t hw_time_us);
};
#endif // PARSE_IMU_DATA_H_