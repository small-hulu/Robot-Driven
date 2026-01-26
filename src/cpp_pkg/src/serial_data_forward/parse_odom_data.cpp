#include "serial_data_forward/parse_odom_data.h"
#include <rclcpp/rclcpp.hpp>

std::shared_ptr<DataResult> ParseOdomData::decode_frame(const std::string& frame)
{
    auto result = std::make_shared<DataResult>();
    nav_msgs::msg::Odometry& odom = result->data.odom;

    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    (void)frame;

    result->is_valid = true;
    result->data_type = DataType::ODOM_DATA;
    
    return result;
}

void ParseOdomData::process_data(const std::shared_ptr<DataResult>& result, std::shared_ptr<PublisherNode> pub_node)
{
    (void)result;
    (void)pub_node;
} 

uint8_t ParseOdomData::get_data_flag() const
{
    return ODOM_DATA_FLAG;
}