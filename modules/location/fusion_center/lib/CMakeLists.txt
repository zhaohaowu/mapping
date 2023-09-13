project(fusion_center_core)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(FUSION_CENTER_SRCS
    fusion_center.cc
)

add_mapping_library(location_fusion_center_core SHARED ${FUSION_CENTER_SRCS})

target_link_libraries(location_fusion_center_core
  util

  protobuf
  yaml-cpp
  ${USED_LOG}
)

set_target_properties(location_fusion_center_core PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS location_fusion_center_core
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
