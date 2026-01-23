#include "serial_data_forward/parse_odom_data.h"
#include <rclcpp/rclcpp.hpp>

std::shared_ptr<DataResult> ParseOdomData::decode_frame(const std::string& frame)
{

}

void ParseOdomData::process_data(const std::shared_ptr<DataResult>& result, std::shared_ptr<PublisherNode> pub_node)
{

} 

uint8_t ParseOdomData::get_data_flag() const
{
    return ODOM_DATA_FLAG;
}