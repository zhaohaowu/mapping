project(local_mapping)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../
)

file(GLOB_RECURSE LM_SRCS "*.cc" "*.h")

file(GLOB_RECURSE PROTO_FILES
    "lib/laneline/proto/*.proto"
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HEADERS ${PROTO_FILES})


#add_mapping_library 加前缀mapping_
add_mapping_library(lmap_core SHARED ${LM_SRCS} ${PROTO_SRCS})

target_link_libraries(lmap_core
  globalproto
  util
  europa_hdmap
  adsfi_proto
  gflags
  glog
  opencv_core  
  opencv_highgui 
  opencv_imgproc 
  opencv_flann
  mf
  perception-base-mal
  perception-lib-mal
)

set_target_properties(lmap_core PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS lmap_core
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
