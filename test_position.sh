#!/bin/bash
source /opt/tros/humble/setup.sh
TOPIC="/position"

while true; do
  ros2 topic pub -1 $TOPIC std_msgs/msg/String "data: 'done'"
  sleep 1
done