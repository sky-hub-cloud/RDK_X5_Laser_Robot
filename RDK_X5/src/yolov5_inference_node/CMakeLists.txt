cmake_minimum_required(VERSION 3.8)
project(yolov5_inference_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# 地平线库路径
set(HB_DNN_LIB "/usr/lib/libdnn.so")
set(HBRT_LIB "/usr/lib/libhbrt_bayes_aarch64.so")

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  /usr/include
  /usr/include/dnn
)

add_executable(inference_node src/inference_node.cpp)

# 使用 ament_target_dependencies 添加 ROS 依赖
ament_target_dependencies(inference_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

# 链接 OpenCV 和地平线库
target_link_libraries(inference_node
  ${OpenCV_LIBS}
  ${HB_DNN_LIB}
  ${HBRT_LIB}
  pthread
)

# 安装目标
install(TARGETS
  inference_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
