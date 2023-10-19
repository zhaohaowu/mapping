project(ins_fusion_core)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(INS_FUSION_SRCS
  ins_fusion.cc
  smoother.cc
)

add_mapping_library(localization_ins_fusion_core SHARED ${INS_FUSION_SRCS})

target_link_libraries(localization_ins_fusion_core
  protobuf
  yaml-cpp
  util
  glog
  boost_filesystem
  globalproto
)

set_target_properties(localization_ins_fusion_core PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS localization_ins_fusion_core
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
