cmake_minimum_required(VERSION 3.8)
project(robot_app_pkg)

# 编译警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================== 依赖 ==================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ================== 可执行节点 ==================
add_executable(robot_control
  src/robot_control.cpp
)

# 头文件路径（如果以后有 include）
target_include_directories(robot_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# C++ 标准
target_compile_features(robot_control PUBLIC cxx_std_17)

# ROS2 依赖
ament_target_dependencies(
  robot_control
  rclcpp
  std_msgs
  geometry_msgs
)

# ================== 安装 ==================
install(TARGETS
  robot_control
  DESTINATION lib/${PROJECT_NAME}
)

# ================== 测试（可保留） ==================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
