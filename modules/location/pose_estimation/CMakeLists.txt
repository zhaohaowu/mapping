project(pose_estimation)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/factor
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/pose_estimate
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/tracking
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/util
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/hd_map
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/perception 
)

add_subdirectory(lib)
