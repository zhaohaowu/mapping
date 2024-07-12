cmake_minimum_required(VERSION 3.16)

project(fusion_center_ut)

include_directories(
    ${CMAKE_SOURCE_DIR}/modules/location/fusion_center/
    ${CMAKE_SOURCE_DIR}/modules/location/
    ${CMAKE_SOURCE_DIR}/modules/
    ${CMAKE_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(FUSION_CENTER_SRCS
  ${CMAKE_SOURCE_DIR}/modules/location/fusion_center/lib/fusion_center.cc
  ${CMAKE_SOURCE_DIR}/modules/location/fusion_center/lib/eskf.cc
  ${CMAKE_SOURCE_DIR}/modules/location/fusion_center/lib/kalman_filter.cc
  ${CMAKE_SOURCE_DIR}/modules/location/fusion_center/lib/monitor.cc
)

set(LIBS
    gtest
    util
    mf
    globalproto

    protobuf
    yaml-cpp
    boost_filesystem
    ${USED_LOG}
)

file(GLOB _SRC "*.cc")
add_library(${PROJECT_NAME} STATIC ${FUSION_CENTER_SRCS} ${_SRC})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX_UT}/lib)

