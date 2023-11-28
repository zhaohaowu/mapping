project(mapping_location_fc_lite)
set(MAPPING_ONBOARD_LITE_FC onboard_lite_fc)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(LOCATION_LITE_FC_SRCS
  fusion_center_lite.cc
)

add_mapping_library(${MAPPING_ONBOARD_LITE_FC}
  ${LOCATION_LITE_FC_SRCS}
)

target_link_libraries(${MAPPING_ONBOARD_LITE_FC}
  gflags
  ${DEPENDS_LIBS}
  perception-onboard-common
  perception-lib
  location_fusion_center_core
  location_coord_adapter_core
  lmap_core
)

install(
  TARGETS ${MAPPING_ONBOARD_LITE_FC}
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
