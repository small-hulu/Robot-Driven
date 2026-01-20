#ifndef PARSE_IMU_DATA_H_
#define PARSE_IMU_DATA_H_
#include "ParseData.h"

class ParseImuData : public ParseData
{
public:
    DataResult parse_frame(const std::string& frame) override;
    void process_data(const DataResult& result, std::shared_ptr<Publisher> pub_node = nullptr) override;
    uint8_t get_data_flag() const override;

private:
    static const uint8_t IMU_DATA_FLAG = 0x11;
};
#endif // PARSE_IMU_DATA_H_