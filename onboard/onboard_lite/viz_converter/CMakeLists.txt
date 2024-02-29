add_mapping_library(onboard_lite_viz_cvt
  viz_converter_lite.cc
  )

target_link_libraries(onboard_lite_viz_cvt
  util
  europa_common
  )

install(
  TARGETS onboard_lite_viz_cvt
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
