# 加载 ROS2 的环境配置文件，初始化 ROS2 环境变量、路径及命令行工具
source /opt/tros/humble/setup.sh
# 限制当前终端进程最大虚拟内存为2G（限制内存资源）
ulimit -v 2097152
#设置make编译默认使用1个CPU核心（限制CPU资源）
export MAKEFLAGS="-j1" 
# 删除编译生成的build/install/log目录
rm -rf build/ install/ log/
# 低优先级单核心编译cpp_pkg包，编译模式为Release
nice -n 10 colcon build --packages-select cpp_pkg --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
# 加载当前工作空间编译产物的环境配置
source ./install/setup.sh
# 运行cpp_pkg包中的 driver_node 节点
ros2 run cpp_pkg driver_node