#ifndef PARSE_ODOM_DATA_H_
#define PARSE_ODOM_DATA_H_
#include "ParseData.h"
#include <memory>
#include <string>


class ParseOdomData : public ParseData
{
public:
    std::shared_ptr<DataResult> decode_frame(const std::string& frame) override;
    void process_data(const std::shared_ptr<DataResult>& result, std::shared_ptr<PublisherNode> pub_node = nullptr) override;
    uint8_t get_data_flag() const override;

private:
    static const uint8_t ODOM_DATA_FLAG = 0x12; //标志位
private:

};
#endif // PARSE_ODOM_DATA_H_