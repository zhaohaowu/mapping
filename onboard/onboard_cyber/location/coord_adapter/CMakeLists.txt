project(coord_adapter_cyber)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(COORD_ADAPTER_CYBER_SRCS
  coord_adapter_component.cc
)

add_mapping_library(location_coord_adapter_cyber SHARED ${COORD_ADAPTER_CYBER_SRCS})

target_link_libraries(location_coord_adapter_cyber
  location_coord_adapter_core
  gflags
  cyberrt
  fastrtps
  fastcdr
)

set_target_properties(location_coord_adapter_cyber PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS location_coord_adapter_cyber
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
