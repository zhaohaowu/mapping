set(MAP_FUSION_LITE_SRCS
  prior_provider_lite.cc
)

add_mapping_library(mf_lite
  ${MAP_FUSION_LITE_SRCS}
)

target_link_libraries(mf_lite
  mf
  perception-onboard-common
  ${DEPENDS_LIBS}
  fastrtps
  gflags
)

install(
  TARGETS mf_lite
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
