cmake_minimum_required(VERSION 3.10)
project(hikrobot_camera)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 仅在Debug模式添加编译警告
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找ROS 2依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

# 根据处理器架构选择MVS库路径
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  link_directories(/opt/MVS/lib/aarch64/)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  link_directories(/opt/MVS/lib/64/)
else()
  message(FATAL_ERROR "Unsupported processor architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
endif()

# 添加Pylon库路径（根据架构自动选择）
link_directories(
  $<$<PLATFORM_ID:Linux>:/opt/pylon/lib64/>
  $<$<PLATFORM_ID:Linux>:/opt/pylon/lib/>
)

# 添加RKNN库路径
link_directories(
  /home/elf/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/aarch64
)

# 包含头文件路径
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/include/
  /opt/pylon/include
  /home/elf/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/include
)

# 添加可执行文件
add_executable(${PROJECT_NAME}
  src/hikrobot_camera.cpp
)

# 添加图像推理节点 (RKNN版本)
add_executable(image_inference_node
  src/image_inference_node_rknn.cpp
)

# 添加图像压缩节点
add_executable(image_compression_node
  src/image_compression_node.cpp
)

# 链接库文件
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  MvCameraControl
  MVGigEVisionSDK
  MVRender
  MvUsb3vTL
  MediaProcess
  X11
  
  # Basler Pylon库
  GCBase_gcc_v3_1_Basler_pylon
  GenApi_gcc_v3_1_Basler_pylon
  pylonbase
  pylon_TL_gige
)

# 链接图像推理节点的库
target_link_libraries(image_inference_node
  ${OpenCV_LIBS}
  rknnrt
)

# 链接图像压缩节点的库
target_link_libraries(image_compression_node
  ${OpenCV_LIBS}
)

# 设置安装规则
install(TARGETS ${PROJECT_NAME} image_inference_node image_compression_node
  DESTINATION lib/${PROJECT_NAME}
)

# 修正：安装实际的头文件
install(
  FILES include/hikrobot_camera.hpp
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz_config
  DESTINATION share/${PROJECT_NAME}
)

# 移除不存在的CMake配置文件安装
# install(FILES ${PROJECT_NAME}.cmake DESTINATION share/${PROJECT_NAME})

# 生成编译依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
)

# 为图像推理节点生成编译依赖
ament_target_dependencies(image_inference_node
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
)

# 为图像压缩节点生成编译依赖
ament_target_dependencies(image_compression_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

# 仅在明确要求时启用测试
if(DEFINED ENABLE_TESTING AND ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 注册ament包
ament_package()