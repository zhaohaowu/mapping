set(LOCAL_MAPPING_LITE_SRCS
local_mapping_lite.cc
local_mapping_lite.h
)

add_library(onboard_lite_localmapping
  ${LOCAL_MAPPING_LITE_SRCS}
)

target_link_libraries(onboard_lite_localmapping
   
  ${DEPENDS_LIBS}
  # fastrtps
  # gflags
  lmap_core
  opencv_core
  opencv_imgproc
  opencv_imgcodecs

)

install(
  TARGETS onboard_lite_localmapping
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
