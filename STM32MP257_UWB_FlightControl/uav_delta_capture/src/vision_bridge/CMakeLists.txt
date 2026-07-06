cmake_minimum_required(VERSION 3.8)
project(vision_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# vision_transform_node
add_executable(vision_transform_node src/vision_transform_node.cpp)
ament_target_dependencies(vision_transform_node
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

install(TARGETS
  vision_transform_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ config/
  DESTINATION share/${PROJECT_NAME}
)

# Python 节点
install(PROGRAMS
  src/perception_node.py
  src/jetson_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
