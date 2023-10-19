project(coord_adapter_core)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(COORD_ADAPTER_SRCS
  coord_adapter.cc
)

add_mapping_library(location_coord_adapter_core SHARED ${COORD_ADAPTER_SRCS})

target_link_libraries(location_coord_adapter_core
  globalproto

  protobuf
  yaml-cpp
  boost_filesystem
  ${USED_LOG}
)

set_target_properties(location_coord_adapter_core PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS location_coord_adapter_core
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
