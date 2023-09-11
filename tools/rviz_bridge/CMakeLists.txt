cmake_minimum_required(VERSION 3.12.1)

project(rviz_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(CMAKE_INSTALL_PREFIX ${REPO_ROOT}/release/ros)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
    tf
    tf2_ros
)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES fusion-localization-internal
    CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf tf2_ros
    #  DEPENDS system_lib
)

include_directories(
    ${REPO_ROOT}/release/x86/include/mapping/interface
    ${REPO_ROOT}/release/x86/include/mapping/modules/util/include
    ${REPO_ROOT}/depend/third_party/x86/protobuf/include
    ${REPO_ROOT}/depend/third_party/x86/zmq/include
    ${REPO_ROOT}/depend/third_party/x86/gflags/include
    ${REPO_ROOT}/depend/third_party/x86/glog/include
    ${catkin_INCLUDE_DIRS}
)

link_directories(
    ${REPO_ROOT}/release/x86/lib
    ${REPO_ROOT}/depend/third_party/x86/protobuf/lib
    ${REPO_ROOT}/depend/third_party/x86/zmq/lib
    ${REPO_ROOT}/depend/third_party/x86/gflags/lib
    ${REPO_ROOT}/depend/third_party/x86/glog/lib
)

add_executable(rviz_bridge
    src/rviz_bridge.cc
    src/type_converter.cc
)

target_link_libraries(rviz_bridge
    mapping_adsfi_proto
    mapping_util
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

set(PREPARE_FOR_ROS_SHELL "
    mkdir -p ros
    cp -r -d x86/lib/libmapping_* ros/lib
    cp -r -d ../depend/third_party/x86/protobuf/lib/lib* ros/lib
    cp -r -d ../depend/third_party/x86/gflags/lib/lib* ros/lib
    cp -r -d ../depend/third_party/x86/glog/lib/lib* ros/lib
")
file(WRITE "${CMAKE_INSTALL_PREFIX}/../prepare_for_ros.sh" "${PREPARE_FOR_ROS_SHELL}")
install(CODE "execute_process(COMMAND bash prepare_for_ros.sh WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/..)")