cmake_minimum_required(VERSION 3.8)
project(livox_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(livox_interfaces REQUIRED)

# 添加可执行文件
add_executable(livox_msg_converter src/livox_msg_converter.cpp)
ament_target_dependencies(livox_msg_converter rclcpp livox_ros_driver2 livox_interfaces)

# 安装可执行文件
install(TARGETS
  livox_msg_converter
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
    