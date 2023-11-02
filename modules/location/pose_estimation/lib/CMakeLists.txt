project(pose_estimation_core)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# aux_source_directory(preprogress PREPROGRESS_SRCS)
# aux_source_directory(factor FACTOR_SRCS)
# aux_source_directory(factor/line_factor LINE_FACTOR_SRCS)
# aux_source_directory(factor/line_factor/factor LINE_FACTOR_FACTOR_SRCS)
# aux_source_directory(factor/depth_factor DEPTH_FACTOR_SRCS)
# aux_source_directory(factor/plane_factor PLANE_FACTOR_SRCS)
# aux_source_directory(factor/plane_factor/task PLANE_FACTOR_TASK_SRCS)
aux_source_directory(factor/point_factor_2d POINT_FACTOR_SRCS)
# aux_source_directory(factor/test TEST_SRCS)

# aux_source_directory(optimization OPTIMIZATION_SRCS)
aux_source_directory(pose_estimate POSE_ESTIMATE)
aux_source_directory(tracking TRACKING_SRCS)
# aux_source_directory(kdtree KDTREE_SRCS)
aux_source_directory(util UTIL_SRCS)
# aux_source_directory(km KM_SRCS)
aux_source_directory(hd_map HD_MAP_SRCS)
aux_source_directory(perception PERCEPTION_SRCS)


# aux_source_directory(../../wheel_odom/lib/filter FILTER_SRCS)
aux_source_directory(../../util/include UTIL_INCLUDE_SRCS)

set(POSE_ESTIMATION_SRCS
  pose_estimation.cc
  # map_match_solver.cc
  # ${PREPROGRESS_SRCS}
#   ${FILTER_SRCS}
  ${UTIL_INCLUDE_SRCS}
#   ${FACTOR_SRCS}
#   ${LINE_FACTOR_SRCS}
#   ${LINE_FACTOR_FACTOR_SRCS}
#   ${DEPTH_FACTOR_SRCS}
#   ${PLANE_FACTOR_SRCS}
  ${POINT_FACTOR_SRCS}
#   ${TEST_SRCS}
#   ${PLANE_FACTOR_TASK_SRCS}
#   ${OPTIMIZATION_SRCS}
  ${POSE_ESTIMATE}
  ${TRACKING_SRCS}
#   ${KDTREE_SRCS}
  ${UTIL_SRCS}
#   ${KM_SRCS}
  ${HD_MAP_SRCS}
  ${PERCEPTION_SRCS}
)


add_mapping_library(location_pose_estimation_core SHARED ${POSE_ESTIMATION_SRCS})


target_link_libraries(location_pose_estimation_core
 globalproto
#   localization_minieye_proto
#   localization_rviz_agent
  protobuf
  ceres
  adsfi_proto
  util
  yaml-cpp
  mf
)

target_link_libraries(location_pose_estimation_core
  glog
)
set_target_properties(location_pose_estimation_core PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS location_pose_estimation_core
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
