# set(LANELINE_POSTPROCESS_LITE_SRCS
# env_bevfusion_onboard_lite.cc
# env_bevfusion_onboard_lite.h
# )

file(GLOB_RECURSE LANELINE_POSTPROCESS_LITE_SRCS "*.cc" "*.h")

add_library(onboard_lite_laneline_postprocess
  ${LANELINE_POSTPROCESS_LITE_SRCS}
)

include_directories(
  ${CMAKE_SOURCE_DIR}/modules
  # ${CMAKE_SOURCE_DIR}/depend/perception-common-onboard
  ${CMAKE_SOURCE_DIR}/depend/perception-base
  ${CMAKE_SOURCE_DIR}/depend
)

# target_include_directories(onboard_lite_laneline_postprocess PRIVATE ${CMAKE_SOURCE_DIR}/modules)

target_link_libraries(onboard_lite_laneline_postprocess
   
  ${DEPENDS_LIBS}
  lmap_core
  opencv_core
  opencv_imgproc
  opencv_imgcodecs
  perception-base-mal
  perception-lib-mal
  laneline_postprocess_core
  # perception-common-onboard
  # perception-common

)

install(
  TARGETS onboard_lite_laneline_postprocess
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
