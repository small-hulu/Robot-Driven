#!/bin/bash
source /opt/tros/humble/setup.sh 
#colcon build --packages-select cpp_pkg --executor parallel --parallel-workers $(nproc)
source ./install/setup.sh 
ros2 run cpp_pkg driver_node