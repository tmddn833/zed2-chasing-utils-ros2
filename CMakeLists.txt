cmake_minimum_required(VERSION 3.5)
project(zed2_chasing_utils)

#set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")
set(CMAKE_BUILD_TYPE Release)

if(OpenCV_VERSION VERSION_LESS "4.0")
    add_definitions(-DOPENCV_VERSION_3_ZED)
else()
    add_definitions(-DOPENCV_VERSION_4_ZED)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(compressed_depth_image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(zed2_chasing_info_manager src/zed2_chasing_info_manager/zed2_chasing_info_manager.cpp)
ament_target_dependencies(zed2_chasing_info_manager image_transport image_geometry compressed_depth_image_transport compressed_image_transport zed_interfaces sensor_msgs pcl_conversions)
target_include_directories(zed2_chasing_info_manager PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(zed2_chasing_info_manager)
install(FILES include/${PROJECT_NAME}/zed2_chasing_info_manager/zed2_chasing_info_manager.h
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/zed2_chasing_info_manager)

add_library(zed2_chasing_server src/zed2_chasing_server/zed2_chasing_server.cpp)
ament_target_dependencies(zed2_chasing_server rclcpp message_filters image_transport image_geometry sensor_msgs tf2_ros zed_interfaces)
target_include_directories(zed2_chasing_server PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_link_libraries(zed2_chasing_server zed2_chasing_info_manager)
install(FILES include/${PROJECT_NAME}/zed2_chasing_server/zed2_chasing_server.h
DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/zed2_chasing_server)



add_executable(zed2_chasing_server_node src/zed2_chasing_server/zed2_chasing_server_node.cpp)
target_link_libraries(zed2_chasing_server_node zed2_chasing_server)
install(TARGETS zed2_chasing_server_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY param DESTINATION share/${PROJECT_NAME})

ament_package()
