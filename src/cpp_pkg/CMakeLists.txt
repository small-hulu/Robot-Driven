cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 设置 serial 库路径
set(SERIAL_LIB_PATH "/usr/local")

# 添加 serial 库的头文件目录
include_directories(${SERIAL_LIB_PATH}/include)

# 设置库路径
link_directories(${SERIAL_LIB_PATH}/lib)

# 设置安装路径
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 使用 file(GLOB ...) 动态获取所有源文件
file(GLOB SRC_FILES src/*.cpp src/serial_data_forward/*.cpp)

# 添加可执行文件
add_executable(driver_node ${SRC_FILES})

# 设置包含目录，添加 include/cpp_pkg 目录
target_include_directories(driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 编译时包含的目录
  $<INSTALL_INTERFACE:include>  # 安装时包含的目录
)

# 设置 C++ 标准
target_compile_features(driver_node PUBLIC cxx_std_17)

# 链接 ROS 2 依赖项
ament_target_dependencies(
  driver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# 链接 serial 库
target_link_libraries(driver_node serial)

# 安装目标
install(TARGETS driver_node
  DESTINATION lib/${PROJECT_NAME}  # 安装到指定目录
)
# ===== 安装 launch 文件=====
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# 如果需要，构建测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
