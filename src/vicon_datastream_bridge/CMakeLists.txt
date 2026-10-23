cmake_minimum_required(VERSION 3.8)
project(vicon_datastream_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(VICON_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../output/vicon-sdk-linux64/Linux64"
    CACHE PATH "Vicon DataStream SDK Linux64 directory")
if(NOT EXISTS "${VICON_SDK_DIR}/DataStreamClient.h" OR
   NOT EXISTS "${VICON_SDK_DIR}/libViconDataStreamSDK_CPP.so")
  message(FATAL_ERROR "Vicon SDK not found at ${VICON_SDK_DIR}")
endif()

add_executable(vicon_datastream_bridge src/vicon_datastream_bridge.cpp)
target_include_directories(vicon_datastream_bridge PRIVATE "${VICON_SDK_DIR}")
target_link_libraries(vicon_datastream_bridge "${VICON_SDK_DIR}/libViconDataStreamSDK_CPP.so")
ament_target_dependencies(vicon_datastream_bridge rclcpp geometry_msgs)
set_target_properties(vicon_datastream_bridge PROPERTIES INSTALL_RPATH "$ORIGIN")

install(TARGETS vicon_datastream_bridge DESTINATION lib/${PROJECT_NAME})
install(FILES "${VICON_SDK_DIR}/libViconDataStreamSDK_CPP.so"
        DESTINATION lib/${PROJECT_NAME})
ament_package()
