project(location)
add_subdirectory(ins_fusion)
set(INSFUSION "${CMAKE_CURRENT_LIST_DIR}/ins_fusion/lib")
add_subdirectory(fusion_center)
add_subdirectory(pose_estimation)
add_subdirectory(coord_adapter)

set(LOC_ASBL_SRCS
  location.cc
)

add_mapping_library(location_asbl_core SHARED ${LOC_ASBL_SRCS})

target_link_libraries(location_asbl_core
  globalproto

  location_coord_adapter_core
  location_fusion_center_core
  localization_ins_fusion_core

  protobuf
  yaml-cpp
  boost_filesystem
  ${USED_LOG}
)

set_target_properties(location_asbl_core PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS location_asbl_core
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
