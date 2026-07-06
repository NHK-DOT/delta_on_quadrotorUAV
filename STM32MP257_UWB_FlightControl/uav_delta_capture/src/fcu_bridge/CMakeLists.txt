cmake_minimum_required(VERSION 3.8)
project(fcu_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(uav_delta_msgs REQUIRED)

# fcu_state_node
add_executable(fcu_state_node src/fcu_state_node.cpp)
ament_target_dependencies(fcu_state_node
  rclcpp geometry_msgs sensor_msgs std_msgs tf2 mavros_msgs uav_delta_msgs
)

# fcu_link_monitor_node
add_executable(fcu_link_monitor_node src/fcu_link_monitor_node.cpp)
ament_target_dependencies(fcu_link_monitor_node
  rclcpp geometry_msgs std_msgs
)

# flight_commander_node
add_executable(flight_commander_node src/flight_commander_node.cpp)
ament_target_dependencies(flight_commander_node
  rclcpp geometry_msgs std_msgs mavros_msgs uav_delta_msgs
)

# flight_state_machine_node
add_executable(flight_state_machine_node src/flight_state_machine_node.cpp)
ament_target_dependencies(flight_state_machine_node
  rclcpp std_msgs
)

# attitude_publisher_node
add_executable(attitude_publisher_node src/attitude_publisher_node.cpp)
ament_target_dependencies(attitude_publisher_node
  rclcpp geometry_msgs sensor_msgs tf2 tf2_geometry_msgs
)

# mock_mavros_pose_node
add_executable(mock_mavros_pose_node src/mock_mavros_pose_node.cpp)
ament_target_dependencies(mock_mavros_pose_node
  rclcpp geometry_msgs tf2 tf2_geometry_msgs
)

install(TARGETS
  fcu_state_node
  fcu_link_monitor_node
  flight_commander_node
  flight_state_machine_node
  attitude_publisher_node
  mock_mavros_pose_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ config/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
