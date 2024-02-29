cmake_minimum_required(VERSION 3.12.1)

project(rviz_bridge)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
    tf
    tf2_ros
    rviz
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf tf2_ros message_generation rviz_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${CATKIN_DEVEL_PREFIX}/include
)

add_executable(rviz_bridge
    src/rviz_bridge.cc
    src/type_converter.cc
)

target_link_libraries(rviz_bridge
    adsfi_proto
    globalproto
    util
    yaml-cpp
    gflags
    glog
    protobuf
    ${catkin_LIBRARIES}
)

install(
    TARGETS rviz_bridge
    RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY conf launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
