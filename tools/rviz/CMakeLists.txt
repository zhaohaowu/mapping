cmake_minimum_required(VERSION 3.12.1)

project(mapping_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# for switch in mapping_log.h
add_definitions(-DNOT_USE_PCP_BASE_LOG)

set(PLATFORM "x86")
set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
get_filename_component(REPO_ROOT ${REPO_ROOT} ABSOLUTE)
set(CMAKE_INSTALL_PREFIX ${REPO_ROOT}/release/ros)
# for install adsfi_proto and util
set(CMAKE_INSTALL_PREFIX_RS ${CMAKE_INSTALL_PREFIX})
# required in FindProtobuf.cmake
set(3RD_ROOT ${REPO_ROOT}/depend/third_party/x86_2004)
set(3RD_ROOT_PROTOC ${3RD_ROOT}/protobuf)
# for global proto
set(PLATFORM_3RD ${3RD_ROOT})
# for using protoc
set(ENV{LD_LIBRARY_PATH} "$ENV{LD_LIBRARY_PATH}:${3RD_ROOT_PROTOC}/lib")
# for using protobuf_generate_cpp() in adsfi_proto
include(${3RD_ROOT}/../cmake/FindProtobuf.cmake)

# required in mapping.cmake
set(MAPPING_LIB_PREFIX mapping_)
# for using add_mapping_library()
include(${REPO_ROOT}/cmake/mapping.cmake)

include_directories(
    ${REPO_ROOT}
    ${REPO_ROOT}/depend
    ${REPO_ROOT}/interface
    ${REPO_ROOT}/depend/third_party/x86_2004/protobuf/include
    ${REPO_ROOT}/depend/third_party/x86_2004/zmq/include
    ${REPO_ROOT}/depend/third_party/x86_2004/gflags/include
    ${REPO_ROOT}/depend/third_party/x86_2004/glog/include
    ${REPO_ROOT}/depend/third_party/x86_2004/eigen3/include
    ${REPO_ROOT}/depend/third_party/x86_2004/yaml-cpp/include
    ${REPO_ROOT}/depend/third_party/x86_2004/opencv/include
    ${REPO_ROOT}/depend/third_party/x86_2004/opencv/include/opencv4
)

link_directories(
    ${REPO_ROOT}/depend/third_party/x86_2004/protobuf/lib
    ${REPO_ROOT}/depend/third_party/x86_2004/zmq/lib
    ${REPO_ROOT}/depend/third_party/x86_2004/gflags/lib
    ${REPO_ROOT}/depend/third_party/x86_2004/glog/lib
    ${REPO_ROOT}/depend/third_party/x86_2004/yaml-cpp/lib
    ${REPO_ROOT}/depend/third_party/x86_2004/opencv/lib
)

add_subdirectory(${REPO_ROOT}/depend/proto proto)
add_subdirectory(${REPO_ROOT}/interface interface)
add_subdirectory(${REPO_ROOT}/modules/util util)

add_subdirectory(rviz_msgs)
add_subdirectory(rviz_bridge)

set(PREPARE_FOR_ROS_SHELL "
if [ ! -d ros/lib ]; then
  mkdir -p ros/lib
fi
cp -r -d ../depend/third_party/x86_2004/protobuf/lib/lib* ros/lib
cp -r -d ../depend/third_party/x86_2004/yaml-cpp/lib/lib* ros/lib
cp -r -d ../depend/third_party/x86_2004/gflags/lib/lib* ros/lib
cp -r -d ../depend/third_party/x86_2004/glog/lib/lib* ros/lib
cp -r -d ../depend/third_party/x86_2004/opencv/lib/lib* ros/lib
")
file(WRITE "${CMAKE_INSTALL_PREFIX}/../prepare_for_ros.sh" "${PREPARE_FOR_ROS_SHELL}")
install(CODE "execute_process(COMMAND bash prepare_for_ros.sh WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/..)")
