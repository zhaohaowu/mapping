project(pose_estimation_cyber)

add_library(location_pose_estimation_cyber SHARED
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
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
