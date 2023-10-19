project(pose_estimation_cyber)

set(POSE_ESTIMATION ${MAPPING}/modules/location/pose_estimation)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${POSE_ESTIMATION}/lib
  ${POSE_ESTIMATION}/lib/factor
#   ${CMAKE_CURRENT_SOURCE_DIR}/lib/optimization
  ${POSE_ESTIMATION}/lib/pose_estimate
  ${POSE_ESTIMATION}/lib/tracking
#   ${CMAKE_CURRENT_SOURCE_DIR}/lib/kdtree
  ${POSE_ESTIMATION}/lib/util
#   ${CMAKE_CURRENT_SOURCE_DIR}/lib/km
  ${POSE_ESTIMATION}/lib/hd_map
  ${POSE_ESTIMATION}/lib/perception
  ${MAPPING}/modules/util/include
)

add_mapping_library(location_pose_estimation_cyber SHARED
  pose_estimation_component.cc
)

target_link_libraries(location_pose_estimation_cyber
  location_pose_estimation_core
  adsfi_proto
  cyberrt
  fastrtps
  yaml-cpp
  protobuf
)

set_target_properties(location_pose_estimation_cyber PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS location_pose_estimation_cyber
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
