cmake_minimum_required(VERSION 3.8)
project(safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# failsafe_manager_node
add_executable(failsafe_manager_node src/failsafe_manager_node.cpp)
ament_target_dependencies(failsafe_manager_node
  rclcpp geometry_msgs std_msgs
)

install(TARGETS
  failsafe_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ config/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
