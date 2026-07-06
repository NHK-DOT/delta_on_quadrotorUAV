cmake_minimum_required(VERSION 3.8)
project(uwb_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(uav_delta_msgs REQUIRED)

# uwb_mission_planner_node
add_executable(uwb_mission_planner_node src/uwb_mission_planner_node.cpp)
ament_target_dependencies(uwb_mission_planner_node
  rclcpp geometry_msgs std_msgs uav_delta_msgs
)

install(TARGETS
  uwb_mission_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/test_mission_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ config/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
