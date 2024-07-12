
project(mapping_ut)


set(CMAKE_INSTALL_PREFIX_UT ${CMAKE_INSTALL_PREFIX}/test)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")

include_directories(
 ${CMAKE_SOURCE_DIR}
 ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(dr)
add_subdirectory(phm_comment_lite)
add_subdirectory(laneline_postprocess)
add_subdirectory(local_mapping)
add_subdirectory(location)
add_subdirectory(map_fusion)

file(GLOB SRC *.cc)
add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(
  ${PROJECT_NAME} PRIVATE
  gflags
  mapping_fusion_test_ut
  local_mapping_ut
  phm_comment_lite_ut
  dr_ut
  laneline_postprocess_test_ut
  fusion_center_ut
  ins_fusion_ut
  pose_estimation_core_ut
  coord_adapter_core_ut
  gtest
  neta_log
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX_UT}/bin)
install(DIRECTORY production/scripts DESTINATION ${CMAKE_INSTALL_PREFIX_UT}/)
