cmake_minimum_required(VERSION 3.16)

project(ins_fusion_ut)

include_directories(
    ${CMAKE_SOURCE_DIR}/modules/location/ins_fusion/
    ${CMAKE_SOURCE_DIR}/modules/location/
    ${CMAKE_SOURCE_DIR}/modules/
    ${CMAKE_SOURCE_DIR}/modules/util/include/
    ${CMAKE_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(INS_FUSION_SRCS
  ${CMAKE_SOURCE_DIR}/modules/location/ins_fusion/lib/ins_fusion.cc
  ${CMAKE_SOURCE_DIR}/modules/location/ins_fusion/lib/smoother.cc
)

set(LIBS
  protobuf
  yaml-cpp
  util
  glog
  boost_filesystem
  globalproto
)

file(GLOB _SRC "*.cc")
add_library(${PROJECT_NAME} SHARED ${INS_FUSION_SRCS} ${_SRC})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX_UT}/lib)

