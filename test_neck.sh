#!/bin/bash
source /opt/tros/humble/setup.sh 
# ROS 2 topic
TOPIC="/cmd_hed"

# 循环执行
while true; do
    # 1. 发布第一条指令
    ros2 topic pub -1 $TOPIC geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 60.0, z: 50.0}}"

    # 2. 发布第二条指令
    ros2 topic pub -1 $TOPIC geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 90.0}}"

    # 3. 发布第三条指令
    ros2 topic pub -1 $TOPIC geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 60.0, z: 130.0}}"

    # 4. 发布第四条指令
    ros2 topic pub -1 $TOPIC geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 90.0}}"
done
