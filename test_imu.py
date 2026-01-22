#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time

class IMUSubscriber(Node):
    def __init__(self):
        # 初始化节点，命名为"imu_test_subscriber"
        super().__init__("imu_test_subscriber")
        # 订阅IMU话题（默认话题名/imu/data，根据你的实际话题名修改！）
        self.subscription = self.create_subscription(
            Imu,
            "/imu",  # 重点：替换成你实际的IMU话题名（比如/imu、/imu_link/data）
            self.imu_callback,  # 回调函数：收到数据后执行
            10  # 队列大小
        )
        self.get_logger().info("IMU测试订阅器已启动，等待数据...")

    def imu_callback(self, msg: Imu):
        """IMU数据回调函数：完整打印所有字段"""
        self.get_logger().info("\n==================== IMU数据 ====================")
        
        # 1. 打印Header（时间戳+frame_id）
        self.get_logger().info(f"[Header]")
        self.get_logger().info(f"  frame_id: {msg.header.frame_id}")
        self.get_logger().info(f"  stamp: sec={msg.header.stamp.sec}, nanosec={msg.header.stamp.nanosec}")
        # 转换为可读时间（秒.纳秒）
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.get_logger().info(f"  可读时间戳: {stamp_sec:.9f} s")

        # 2. 打印姿态四元数（orientation）
        self.get_logger().info(f"\n[姿态四元数 (x/y/z/w)]")
        self.get_logger().info(f"  x: {msg.orientation.x:.6f}")
        self.get_logger().info(f"  y: {msg.orientation.y:.6f}")
        self.get_logger().info(f"  z: {msg.orientation.z:.6f}")
        self.get_logger().info(f"  w: {msg.orientation.w:.6f}")

        # 3. 打印姿态协方差矩阵（orientation_covariance）
        self.get_logger().info(f"\n[姿态协方差矩阵]")
        cov = msg.orientation_covariance
        self.get_logger().info(f"  [{cov[0]:.6f}, {cov[1]:.6f}, {cov[2]:.6f}]")
        self.get_logger().info(f"  [{cov[3]:.6f}, {cov[4]:.6f}, {cov[5]:.6f}]")
        self.get_logger().info(f"  [{cov[6]:.6f}, {cov[7]:.6f}, {cov[8]:.6f}]")

        # 4. 打印线加速度（linear_acceleration，单位m/s²）
        self.get_logger().info(f"\n[线加速度 (m/s²)]")
        self.get_logger().info(f"  x: {msg.linear_acceleration.x:.6f}")
        self.get_logger().info(f"  y: {msg.linear_acceleration.y:.6f}")
        self.get_logger().info(f"  z: {msg.linear_acceleration.z:.6f}")

        # 5. 打印加速度协方差矩阵
        self.get_logger().info(f"\n[加速度协方差矩阵]")
        cov = msg.linear_acceleration_covariance
        self.get_logger().info(f"  [{cov[0]:.6f}, {cov[1]:.6f}, {cov[2]:.6f}]")
        self.get_logger().info(f"  [{cov[3]:.6f}, {cov[4]:.6f}, {cov[5]:.6f}]")
        self.get_logger().info(f"  [{cov[6]:.6f}, {cov[7]:.6f}, {cov[8]:.6f}]")

        # 6. 打印角速度（angular_velocity，单位rad/s）
        self.get_logger().info(f"\n[角速度 (rad/s)]")
        self.get_logger().info(f"  x: {msg.angular_velocity.x:.6f}")
        self.get_logger().info(f"  y: {msg.angular_velocity.y:.6f}")
        self.get_logger().info(f"  z: {msg.angular_velocity.z:.6f}")

        # 7. 打印角速度协方差矩阵
        self.get_logger().info(f"\n[角速度协方差矩阵]")
        cov = msg.angular_velocity_covariance
        self.get_logger().info(f"  [{cov[0]:.6f}, {cov[1]:.6f}, {cov[2]:.6f}]")
        self.get_logger().info(f"  [{cov[3]:.6f}, {cov[4]:.6f}, {cov[5]:.6f}]")
        self.get_logger().info(f"  [{cov[6]:.6f}, {cov[7]:.6f}, {cov[8]:.6f}]")
        self.get_logger().info(f"===================================================\n")

def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    # 创建订阅节点
    imu_sub = IMUSubscriber()
    # 自旋节点（持续监听话题）
    try:
        rclpy.spin(imu_sub)
    except KeyboardInterrupt:
        imu_sub.get_logger().info("程序被手动终止")
    finally:
        # 销毁节点，关闭ROS2
        imu_sub.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()