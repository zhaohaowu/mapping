add_mapping_library(onboard_global_lite
  global_lite.cc
)

target_link_libraries(onboard_global_lite
  ${DEPENDS_LIBS}
  # fastrtps
  util
  perception-base-mal
  perception-lib-mal
  gflags
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
)

install(
  TARGETS onboard_global_lite
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
