project(mapping_lmap_core)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB_RECURSE LM_SRCS "*.cpp" "*.cc")

add_mapping_library(lmap_core SHARED ${LM_SRCS})

target_link_libraries(lmap_core
  adsfi_proto
  util
  -lopencv_calib3d
  -lopencv_gapi 
  -lopencv_objdetect
  -lopencv_core  
  -lopencv_highgui 
  -lopencv_photo
  -lopencv_dnn 
  -lopencv_imgcodecs
  -lopencv_stitching
  -lopencv_features2d  
  -lopencv_imgproc 
  -lopencv_videoio
  -lopencv_flann
  -lopencv_ml 
  -lopencv_video
)

set_target_properties(lmap_core PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS lmap_core
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
