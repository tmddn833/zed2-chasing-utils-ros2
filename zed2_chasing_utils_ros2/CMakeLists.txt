cmake_minimum_required(VERSION 3.5)
project(zed2_chasing_utils_ros2)

#set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(zed2_chasing_utils REQUIRED)

add_library(zed2_chasing_server src/zed2_chasing_server/zed2_chasing_server.cpp)
ament_target_dependencies(zed2_chasing_server rclcpp zed2_chasing_utils)
target_include_directories(zed2_chasing_server PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_link_libraries(zed2_chasing_server zed2_chasing_utils)
install(FILES include/${PROJECT_NAME}/zed2_chasing_server/zed2_chasing_server.h
DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/zed2_chasing_server)

add_executable(zed2_chasing_server_node src/zed2_chasing_server/zed2_chasing_server_node.cpp)
target_link_libraries(zed2_chasing_server_node zed2_chasing_server)
install(TARGETS zed2_chasing_server_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY param DESTINATION share/${PROJECT_NAME})

ament_package()
