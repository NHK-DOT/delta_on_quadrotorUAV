cmake_minimum_required(VERSION 3.8)
project(delta_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(delta_kinematics_core
  src/delta_kinematics.cpp
)

target_include_directories(delta_kinematics_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(delta_kinematics_core Eigen3::Eigen)
ament_target_dependencies(delta_kinematics_core rclcpp geometry_msgs std_msgs)

add_executable(delta_kinematics_node src/delta_kinematics_node.cpp)
target_link_libraries(delta_kinematics_node delta_kinematics_core)
ament_target_dependencies(delta_kinematics_node rclcpp geometry_msgs std_msgs)

install(TARGETS
  delta_kinematics_core
  delta_kinematics_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
