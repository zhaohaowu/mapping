project(ins_fusion_cyber)

add_mapping_library(localization_ins_fusion_cyber SHARED
  ins_fusion_component.cc
)

target_link_libraries(localization_ins_fusion_cyber
  localization_ins_fusion_core
  cyberrt
  fastrtps
  glog
)

set_target_properties(localization_ins_fusion_cyber PROPERTIES LINKER_LANGUAGE CXX)
install(
  TARGETS localization_ins_fusion_cyber
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
