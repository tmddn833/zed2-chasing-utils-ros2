cmake_minimum_required(VERSION 3.5)
project(zed2_chasing_utils_ros2)

#set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(zed2_chasing_server src/zed2_chasing_server/zed2_chasing_server.cpp)
ament_target_dependencies(zed2_chasing_server rclcpp)
target_include_directories(zed2_chasing_server PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
install(FILES include/${PROJECT_NAME}/zed2_chasing_server.h
DESTINATION ${CMAKE_ISNTALL_PREFIX}/include/${PROJECT_NAME}/zed2_chasing_server)

ament_package()
