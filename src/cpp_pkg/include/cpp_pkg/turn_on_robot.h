#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cpp_pkg/SerialPortImpl.h>
#include "cpp_pkg/IDeviceThread.h"
#include "cpp_pkg/serial_frame.h"
#include "std_msgs/msg/string.hpp"
#include <random>
#include <array>
class turn_on_robot: 
    public rclcpp::Node,
    public IDeviceThread {
public:
    enum Mode : uint16_t {
        Mode_None        = 0x00,  /// 无(保留模式)
        Mode_XZMove      = 0x01,  /// xZ运动模式
        Mode_Reach_Fwd   = 0x02,  /// 向前伸手
        Mode_Wave        = 0x03,  /// 挥手
        Mode_Raise       = 0x04,  /// 举手
        Mode_RHead       = 0X05,  ///转动头部

        Mode_Count  /// 此处有两个作用，1)枚举计数 2)非法标志
    };
    
    struct Params {
        bool isValid = false;

        int32_t x_linear   = 0;  /// Vx
        int32_t x_angular  = 0;  /// Az
        int32_t y_angular  = 0;  /// Az
        int32_t z_angular  = 0;  /// Az

    };
public:
    turn_on_robot();
public:
    void Set_Config(int mode,Params param) {
        m_modeIndex = mode;
        m_param     = param;
    }
    Params    m_param;
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
    //The speed topic subscribes to the callback function
    void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);

    //创建一个与应用层交互的节点来跑业务 /position
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Position_Sub;
    void position_Callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Hed_Sub;
    //The Head topic subscribes to the callback function
    void Cmd_Hed_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
private:
    RetCode execSet_None();

    RetCode execSet_XZMove();

    RetCode execSet_Reach_Fwd();

    RetCode execSet_Wave();

    RetCode execSet_Raise();

    RetCode execSet_RHead();
};