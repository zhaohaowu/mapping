set(MAP_FUSION_LITE_SRCS
  map_fusion_lite.cc
  map_fusion_config_lite.cc
  tlr_fusion_lite.cc
)

add_mapping_library(onboard_lite_mf
  ${MAP_FUSION_LITE_SRCS}
)

target_link_libraries(onboard_lite_mf
  mf
  ${DEPENDS_LIBS}
  # fastrtps
  perception-base-mal
  perception-lib-mal
  gflags
)

install(
  TARGETS onboard_lite_mf
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
