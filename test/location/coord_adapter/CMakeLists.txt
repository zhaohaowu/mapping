cmake_minimum_required(VERSION 3.16)

project(coord_adapter_core_ut)

include_directories(
    ${CMAKE_SOURCE_DIR}/modules/location/coord_adapter/lib/
    ${CMAKE_SOURCE_DIR}/modules/location/coord_adapter/
    ${CMAKE_SOURCE_DIR}/modules/location/
    ${CMAKE_SOURCE_DIR}/modules/
    ${CMAKE_SOURCE_DIR}
)

set(COORD_ADAPTER_SRCS ${CMAKE_SOURCE_DIR}/modules/location/coord_adapter/lib/coord_adapter.cc)

set(LIBS
  globalproto

  protobuf
  yaml-cpp
  boost_filesystem
  ${USED_LOG}
)

file(GLOB _SRC "*.cc")
add_library(${PROJECT_NAME} SHARED ${COORD_ADAPTER_SRCS} ${_SRC})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX_UT}/lib)

