#include "cpp_pkg/turn_on_robot.h"
using std::placeholders::_1;

turn_on_robot::turn_on_robot()
: rclcpp::Node ("wheeltec_robot")
{
  ///init funtion
  {
    std::vector<AtomicTask> setFun;
    setFun.push_back(std::bind(&turn_on_robot::execSet_None, this));
    setFun.push_back(std::bind(&turn_on_robot::execSet_XZMove, this));
    setFun.push_back(std::bind(&turn_on_robot::execSet_Reach_Fwd, this));
    setFun.push_back(std::bind(&turn_on_robot::execSet_Wave, this));
    setFun.push_back(std::bind(&turn_on_robot::execSet_Raise, this));
    setFun.push_back(std::bind(&turn_on_robot::execSet_RHead, this));

    assert(setFun.size() == Mode_Count);
    set_loopFun(setFun);
  }

  RCLCPP_INFO(this->get_logger(),"wheeltec_robot create\r\n");

  //Set the velocity control command callback function
  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 2, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));
  
  Cmd_Hed_Sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_hed", 2, std::bind(&turn_on_robot::Cmd_Hed_Callback, this, _1));
  
  // position 订阅
  Position_Sub = create_subscription<std_msgs::msg::String>(
      "position", 2, std::bind(&turn_on_robot::position_Callback, this, _1));
  RCLCPP_INFO(this->get_logger(),"turn_on_robot Data ready");   
}

RetCode turn_on_robot::execSet_None()
{
  RCLCPP_INFO(this->get_logger(),"execSet_None");   
  
  return RetCode::Success;
}

RetCode turn_on_robot::execSet_XZMove()
{
    RCLCPP_INFO(this->get_logger(), "execSet_XZMove");

    SerialFrame frame(Mode_XZMove);

    int16_t vx = static_cast<int16_t>(m_param.x_linear);
    int16_t vz = static_cast<int16_t>(m_param.z_angular);

    frame.set_data({
        static_cast<uint8_t>((vx >> 8) & 0xFF),
        static_cast<uint8_t>( vx       & 0xFF),
        static_cast<uint8_t>((vz >> 8) & 0xFF),
        static_cast<uint8_t>( vz       & 0xFF),
    });

    auto bytes = frame.to_bytes();
    SerialPortImpl::instance().write_bytes(bytes.data(), bytes.size());

    return RetCode::Success;
}

RetCode turn_on_robot::execSet_Reach_Fwd()
{
    RCLCPP_INFO(this->get_logger(),"execSet_Reach_Fwd");   
    SerialFrame frame(Mode_Reach_Fwd);

    // int16_t vx = static_cast<int16_t>(m_param.x_linear);
    // int16_t vz = static_cast<int16_t>(m_param.z_angular);

    // frame.set_data({
    //     static_cast<uint8_t>((vx >> 8) & 0xFF),
    //     static_cast<uint8_t>( vx       & 0xFF),
    //     static_cast<uint8_t>((vz >> 8) & 0xFF),
    //     static_cast<uint8_t>( vz       & 0xFF),
    // });

    auto bytes = frame.to_bytes();
    SerialPortImpl::instance().write_bytes(bytes.data(), bytes.size());
    return RetCode::Success;
}

RetCode turn_on_robot::execSet_Wave()
{
  RCLCPP_INFO(this->get_logger(),"execSet_Wave");   
  SerialFrame frame(Mode_Wave);

    // int16_t vx = static_cast<int16_t>(m_param.x_linear);
    // int16_t vz = static_cast<int16_t>(m_param.z_angular);

    // frame.set_data({
    //   static_cast<uint8_t>((vx >> 8) & 0xFF),
    //   static_cast<uint8_t>( vx       & 0xFF),
    //   static_cast<uint8_t>((vz >> 8) & 0xFF),
    //   static_cast<uint8_t>( vz       & 0xFF),
    // });

    auto bytes = frame.to_bytes();
    SerialPortImpl::instance().write_bytes(bytes.data(), bytes.size());
  return RetCode::Success;
}

RetCode turn_on_robot::execSet_Raise()
{
  RCLCPP_INFO(this->get_logger(),"execSet_Raise");  
  SerialFrame frame(Mode_Raise);

    // int16_t vx = static_cast<int16_t>(m_param.x_linear);
    // int16_t vz = static_cast<int16_t>(m_param.z_angular);

    // frame.set_data({
    //     static_cast<uint8_t>((vx >> 8) & 0xFF),
    //     static_cast<uint8_t>( vx       & 0xFF),
    //     static_cast<uint8_t>((vz >> 8) & 0xFF),
    //     static_cast<uint8_t>( vz       & 0xFF),
    // });

    auto bytes = frame.to_bytes();
    SerialPortImpl::instance().write_bytes(bytes.data(), bytes.size()); 
  return RetCode::Success;
}

RetCode turn_on_robot::execSet_RHead()
{
  RCLCPP_INFO(this->get_logger(),"execSet_RHead");  
  SerialFrame frame(Mode_RHead);

    int16_t angular_z = static_cast<int16_t>(m_param.z_angular); //水平角
    int16_t angular_y = static_cast<int16_t>(m_param.y_angular); //俯仰角
    

    frame.set_data({
        static_cast<uint8_t>((angular_z >> 8) & 0xFF),
        static_cast<uint8_t>( angular_z       & 0xFF),
        static_cast<uint8_t>((angular_y >> 8) & 0xFF),
        static_cast<uint8_t>( angular_y       & 0xFF),
    });

    auto bytes = frame.to_bytes();
    SerialPortImpl::instance().write_bytes(bytes.data(), bytes.size()); 
    return RetCode::Success;
}
/**************************************
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机 
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
   /*
      考虑做个滑动窗口滤波
      robot_node->Set_Config(turn_on_robot::Mode_XYZMove);
      robot_node->Exec_sequencedTask();
   */
    RCLCPP_INFO(this->get_logger(),"twist_aux");  
    static geometry_msgs::msg::Twist last_twist;

    if (twist_aux->linear.x  == last_twist.linear.x &&
        twist_aux->angular.z == last_twist.angular.z) {
        return;  
    }
    last_twist = *twist_aux;

    Params param;
    param.isValid   = true;
    param.x_linear  = static_cast<int32_t>(twist_aux->linear.x  * 1000);
    param.z_angular = static_cast<int32_t>(twist_aux->angular.z * 1000);

    Set_Config(Mode_XZMove,param);
    Exec_parallelTask();
}

/**************************************
Function: The Head topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: Head话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机 
***************************************/
void turn_on_robot::Cmd_Hed_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{

    static geometry_msgs::msg::Twist last_twist;

    if (twist_aux->angular.z  == last_twist.angular.z &&
        twist_aux->angular.y == last_twist.angular.y) {
        //return;  
    }
    last_twist = *twist_aux;
     RCLCPP_INFO(this->get_logger(),"twist_aux_head"); 
    Params param;
    param.isValid   = true;
    param.z_angular = static_cast<int32_t>(twist_aux->angular.z);  //水平
    param.y_angular = static_cast<int32_t>(twist_aux->angular.y);  //俯角

    Set_Config(Mode_RHead,param);
    Exec_parallelTask();
}

/**************************************
Function: 
功能：Position话题订阅回调函数Callback，订阅到done指令后，随机选择一个动作执行，发送对应的串口指令控制下位机
***************************************/
void turn_on_robot::position_Callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received position: '%s'", msg->data.c_str());
    if (msg->data != "done") return;
    
    RCLCPP_INFO(this->get_logger(), "Position command done received!");
    //三选一
    Params param;
    param.isValid   = true;

    std::random_device rd;
    static thread_local std::mt19937 rng{rd()};

    static constexpr std::array<uint16_t, 3> modes = {Mode_Reach_Fwd, Mode_Wave, Mode_Raise};
    std::uniform_int_distribution<std::size_t> dist(0, modes.size() - 1); 
    uint16_t Mode_Choice = modes[dist(rng)]; //随机选择一个动作

    Set_Config(Mode_Choice,param);
    Exec_parallelTask();
}