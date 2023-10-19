project(dr_fusion_core)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(DR_FUSION_SRCS
  dr_fusion.cc
)

add_mapping_library(localization_dr_fusion_core SHARED ${DR_FUSION_SRCS})

target_link_libraries(localization_dr_fusion_core
  protobuf
  yaml-cpp
  util
  boost_filesystem
  localization_ins_fusion_core
)

set_target_properties(localization_dr_fusion_core PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS localization_dr_fusion_core
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
