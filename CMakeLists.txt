cmake_minimum_required(VERSION 3.5)
project(motion_parallax)

set(CMAKE_CXX_STANDARD 17)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)



add_executable(motion_parallax_node
  src/main.cpp
  src/frame.cpp
  src/point.cpp
  src/utils.cpp
  src/objdetection.cpp
  src/hungarian.cpp
)


ament_target_dependencies(motion_parallax_node rclcpp std_msgs)

target_include_directories(motion_parallax_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  motion_parallax_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include
  DESTINATION include/${PROJECT_NAME})


ament_package()
