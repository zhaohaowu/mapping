project(pose_estimation_core)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)
aux_source_directory(factor/point_factor_2d POINT_FACTOR_SRCS)
aux_source_directory(pose_estimate POSE_ESTIMATE)
aux_source_directory(tracking TRACKING_SRCS)
aux_source_directory(util UTIL_SRCS)
aux_source_directory(hd_map HD_MAP_SRCS)
aux_source_directory(perception PERCEPTION_SRCS)
aux_source_directory(../../util/include UTIL_INCLUDE_SRCS)
aux_source_directory(reloc RELOC_SRCS)
aux_source_directory(fault FAULT_SRCS)

set(POSE_ESTIMATION_SRCS
  pose_estimation.cc
  map_matching.cc
  ${UTIL_INCLUDE_SRCS}
  ${POINT_FACTOR_SRCS}
  ${POSE_ESTIMATE}
  ${TRACKING_SRCS}
  ${UTIL_SRCS}
  ${HD_MAP_SRCS}
  ${PERCEPTION_SRCS}
  ${RELOC_SRCS}
  ${FAULT_SRCS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../rviz/location_rviz.cc
)

add_mapping_library(location_pose_estimation_core SHARED ${POSE_ESTIMATION_SRCS})

target_link_libraries(location_pose_estimation_core
  globalproto
  protobuf
  ceres
  adsfi_proto
  util
  yaml-cpp
  mf
  glog
)

set_target_properties(location_pose_estimation_core PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS location_pose_estimation_core
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
