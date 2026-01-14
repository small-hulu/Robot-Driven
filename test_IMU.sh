#!/bin/bash
source /opt/tros/humble/setup.sh
TOPIC="/IMU_data_topic"

while true; do
  echo "$(date '+%Y-%m-%d %H:%M:%S') -----------------------------------"
  ros2 topic echo $TOPIC
  sleep 1
done
