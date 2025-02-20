cmake_minimum_required(VERSION 3.8)
project(vision_max)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)  # 如果使用Eigen库
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS
  core
  imgproc
  calib3d
  highgui
)


# 编译节点
add_executable(camera_node src/camera_node.cpp)
target_include_directories(camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(camera_node
  rclcpp
  OpenCV
  image_transport
  cv_bridge
  sensor_msgs
  camera_info_manager  # 添加 camera_info_manager 依赖
)

add_executable(detection_node src/detection_node.cpp)
target_include_directories(detection_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)
ament_target_dependencies(detection_node
  rclcpp
  image_transport
  OpenCV
  cv_bridge
  sensor_msgs
  geometry_msgs
  message_filters
  camera_info_manager  # 添加 camera_info_manager 依赖
)

add_executable(predictor_node src/predictor_node.cpp)
target_include_directories(predictor_node PRIVATE
  ${Eigen3_INCLUDE_DIRS}
)
ament_target_dependencies(predictor_node
  rclcpp
  geometry_msgs
  
  image_transport
  
  cv_bridge
  sensor_msgs
  
  message_filters
  camera_info_manager
  OpenCV
)

# 安装可执行文件和配置文件
install(TARGETS
  camera_node
  detection_node
  predictor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_package()