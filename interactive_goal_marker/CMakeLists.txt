cmake_minimum_required(VERSION 3.8)
project(interactive_goal_marker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(interactive_goal_marker
  src/interactive_goal_marker.cpp
  src/interactive_goal_marker_node.cpp
)

target_include_directories(interactive_goal_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(interactive_goal_marker
  geometry_msgs
  interactive_markers
  rclcpp
  visualization_msgs
)

install(TARGETS interactive_goal_marker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
