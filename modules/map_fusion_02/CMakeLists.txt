project(map_fusion2)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB_RECURSE MAP_APP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/app/*.cc)
file(GLOB_RECURSE MAP_BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/base/*.cc)
file(GLOB_RECURSE MAP_COMMON_SRC ${CMAKE_CURRENT_SOURCE_DIR}/common/*.cc)
file(GLOB_RECURSE MAP_DATA_CONVERT_SRC ${CMAKE_CURRENT_SOURCE_DIR}/data_convert/*.cc)
file(GLOB_RECURSE MAP_DATA_MANAGER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/data_manager/*.cc)
file(GLOB_RECURSE MAP_MAP_SERVICE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/map_service/*.cc)
file(GLOB_RECURSE MAP_MOUDLES_SRC ${CMAKE_CURRENT_SOURCE_DIR}/modules/*.cc)
file(GLOB_RECURSE MAP_PIPELINES_SRC ${CMAKE_CURRENT_SOURCE_DIR}/pipelines/*.cc)
file(GLOB_RECURSE MAP_RVIZ_SRC ${CMAKE_CURRENT_SOURCE_DIR}/rviz/*.cc)

set(MAP_FUSION_SRCS
  ${MAP_APP_SRC}
  ${MAP_BASE_SRC}
  ${MAP_COMMON_SRC}
  ${MAP_DATA_CONVERT_SRC}
  ${MAP_DATA_MANAGER_SRC}
  ${MAP_MAP_SERVICE_SRC}
  ${MAP_MOUDLES_SRC}
  ${MAP_PIPELINES_SRC}
  ${MAP_RVIZ_SRC}
  )

add_mapping_library(mf ${MAP_FUSION_SRCS})
target_link_libraries(mf
  protobuf
  util
  adsfi_proto
  globalproto
  europa_hdmap
  opencv_flann
  opencv_core
  pcl_segmentation
  neta_https

  perception-base-mal
  perception-lib-mal
  )

install(
  TARGETS mf
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
