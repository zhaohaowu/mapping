cmake_minimum_required(VERSION 3.16)

project(pose_estimation_core_ut)

include_directories(
    ${CMAKE_SOURCE_DIR}/modules/location/pose_estimation/lib/factor
    ${CMAKE_SOURCE_DIR}/modules/location/pose_estimation/lib/pose_estimate
    ${CMAKE_SOURCE_DIR}/modules/location/pose_estimation/lib/tracking
    ${CMAKE_SOURCE_DIR}/modules/location/pose_estimation/lib/util
    ${CMAKE_SOURCE_DIR}/modules/location/pose_estimation/lib/hd_map
    ${CMAKE_SOURCE_DIR}/modules/location/pose_estimation/lib/perception
    ${CMAKE_SOURCE_DIR}/modules/location/pose_estimation/lib/reloc
    ${CMAKE_SOURCE_DIR}/modules/location/pose_estimation/lib/fault
    ${CMAKE_SOURCE_DIR}/modules/location/pose_estimation/lib
    ${CMAKE_SOURCE_DIR}/modules/location/
    ${CMAKE_SOURCE_DIR}/modules/
    ${CMAKE_SOURCE_DIR}/modules/util/include/
    ${CMAKE_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB_RECURSE POSE_ESTIMATION_SRCS "${CMAKE_SOURCE_DIR}/modules/location/pose_estimation/lib/*.cc")
file(GLOB_RECURSE POSE_ESTIMATION_HEADERS "${CMAKE_SOURCE_DIR}/modules/location/pose_estimation/lib/*.h")


set(LIBS
    globalproto
    protobuf
    ceres
    adsfi_proto
    util
    yaml-cpp
    mf
    glog
)

file(GLOB _SRC "*.cc")
add_library(${PROJECT_NAME} STATIC ${POSE_ESTIMATION_SRCS} ${_SRC} ${POSE_ESTIMATION_HEADERS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX_UT}/lib)

