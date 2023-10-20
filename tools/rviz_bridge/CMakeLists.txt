cmake_minimum_required(VERSION 3.12.1)

project(rviz_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
get_filename_component(REPO_ROOT ${REPO_ROOT} ABSOLUTE)
set(CMAKE_INSTALL_PREFIX ${REPO_ROOT}/release/ros)
# for install adsfi_proto and util
set(CMAKE_INSTALL_PREFIX_RS ${CMAKE_INSTALL_PREFIX})
# required in FindProtobuf.cmake
set(3RD_ROOT ${REPO_ROOT}/depend/third_party/x86_2004)
set(3RD_ROOT_PROTOC ${3RD_ROOT}/protobuf)
# for using protoc
set(ENV{LD_LIBRARY_PATH} "$ENV{LD_LIBRARY_PATH}:${3RD_ROOT_PROTOC}/lib")
# for using protobuf_generate_cpp() in adsfi_proto
include(${3RD_ROOT}/../cmake/FindProtobuf.cmake)

# required in mapping.cmake
set(MAPPING_LIB_PREFIX mapping_)
# for using add_mapping_library()
include(${REPO_ROOT}/cmake/mapping.cmake)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
    tf
    tf2_ros
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf tf2_ros
)

include_directories(
    ${REPO_ROOT}
    ${REPO_ROOT}/interface
    ${REPO_ROOT}/depend/third_party/x86_2004/protobuf/include
    ${REPO_ROOT}/depend/third_party/x86_2004/zmq/include
    ${REPO_ROOT}/depend/third_party/x86_2004/gflags/include
    ${REPO_ROOT}/depend/third_party/x86_2004/glog/include
    ${REPO_ROOT}/depend/third_party/x86_2004/eigen3/include
    ${REPO_ROOT}/depend/third_party/x86_2004/yaml-cpp/include
    ${catkin_INCLUDE_DIRS}
)

link_directories(
    ${REPO_ROOT}/depend/third_party/x86_2004/protobuf/lib
    ${REPO_ROOT}/depend/third_party/x86_2004/zmq/lib
    ${REPO_ROOT}/depend/third_party/x86_2004/gflags/lib
    ${REPO_ROOT}/depend/third_party/x86_2004/glog/lib
    ${REPO_ROOT}/depend/third_party/x86_2004/yaml-cpp/lib
)

add_subdirectory(${REPO_ROOT}/interface interface)
add_subdirectory(${REPO_ROOT}/modules/util util)

add_executable(rviz_bridge
    src/rviz_bridge.cc
    src/type_converter.cc
)

target_link_libraries(rviz_bridge
    adsfi_proto
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

set(PREPARE_FOR_ROS_SHELL "
    mkdir -p ros
    cp -r -d ../depend/third_party/x86_2004/protobuf/lib/lib* ros/lib
    cp -r -d ../depend/third_party/x86_2004/yaml-cpp/lib/lib* ros/lib
    cp -r -d ../depend/third_party/x86_2004/gflags/lib/lib* ros/lib
    cp -r -d ../depend/third_party/x86_2004/glog/lib/lib* ros/lib
")
file(WRITE "${CMAKE_INSTALL_PREFIX}/../prepare_for_ros.sh" "${PREPARE_FOR_ROS_SHELL}")
install(CODE "execute_process(COMMAND bash prepare_for_ros.sh WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/..)")