project(fusion_center_cyber)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(FUSION_CENTER_CYBER_SRCS
  fc_component.cc
)

add_mapping_library(location_fusion_center_cyber SHARED ${FUSION_CENTER_CYBER_SRCS})

target_link_libraries(location_fusion_center_cyber
  util
  location_fusion_center_core
  gflags
  cyberrt
  fastrtps
  fastcdr
)

set_target_properties(location_fusion_center_cyber PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS location_fusion_center_cyber
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
