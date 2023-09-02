project(mapping_lmap_core)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB_RECURSE LM_SRCS "*.cpp" "*.cc")

add_mapping_library(lmap_core SHARED ${LM_SRCS})

target_link_libraries(lmap_core
  adsfi_proto
)

set_target_properties(lmap_core PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS lmap_core
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
