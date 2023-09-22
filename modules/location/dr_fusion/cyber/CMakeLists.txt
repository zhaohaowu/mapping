project(dr_fusion_cyber)

add_mapping_library(localization_dr_fusion_cyber SHARED
  dr_fusion_component.cc
)

target_link_libraries(localization_dr_fusion_cyber
  localization_dr_fusion_core
  cyberrt
  fastrtps
  globalproto
)

set_target_properties(localization_dr_fusion_cyber PROPERTIES LINKER_LANGUAGE CXX)
install(
  TARGETS localization_dr_fusion_cyber
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)