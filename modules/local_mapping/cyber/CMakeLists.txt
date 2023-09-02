project(mapping_lmap_cyber)

add_mapping_library(lmap_cyber SHARED
  local_mapping_component.cpp
)

target_link_libraries(lmap_cyber
  lmap_core
  gflags
  glog
)

set_target_properties(lmap_cyber PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS lmap_cyber
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)