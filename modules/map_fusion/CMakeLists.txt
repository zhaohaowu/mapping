project(map_fusion)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(FUSION_COMMON_SRCS
  src/fusion_common/calc_util.cc
  src/fusion_common/viz_util.cc
  src/fusion_common/elemap2proto.cc
  src/fusion_common/proto2elemap.cc
  )

set(TOPO_SRCS
  src/topo_assignment/topo_assignment.cc
  )

set(PREDICTION_SRCS
  src/map_prediction/map_prediction.cc
  src/map_prediction/viz_map.cc
  )

set(RECOGNITION_SRC
  src/road_recognition/road_recognition.cc
  src/road_recognition/geo_optimization.cc
  src/road_recognition/topo_generation.cc
  src/road_recognition/routing_generation.cc
  src/road_recognition/group_map.cc
  src/road_recognition/path_manager.cc
  src/road_recognition/occ_guideline_manager.cc
  )

set(MAPSERVICE_SRCS
  src/map_service/ehp/amap_core.cc
  src/map_service/ehp/amap_data.cc
  src/map_service/ehp/amap_diagnose.cc
  src/map_service/ehp/amap_ota.cc
  src/map_service/ehp/amap_system.cc
  src/map_service/map_proto_marker.cc
  src/map_service/map_service.cc
  src/map_service/global_hd_map.cc
  src/map_service/map_table.cc
  )
set(AMAP_LIBS
  base_utils
  GAdasData
  GAdasSDK
  GNet
  OTAHDiff
  GAdasEngine
  GAdasUtils
  GPlatformInterface
  third_party_libs
  )
set(MAPSELECT_SRCS
  src/map_select/map_select_lite.cc
  src/map_select/lane_change_observer.cc
  src/map_select/double_rise_decider.cc
  src/map_select/data_decider_lite.cc
  )


set(MAP_FUSION_SRCS
  src/map_fusion.cc
  src/tlr_fusion.cc
  ${FUSION_COMMON_SRCS}
  ${TOPO_SRCS}
  ${PREDICTION_SRCS}
  ${MAPSERVICE_SRCS}
  ${RECOGNITION_SRC}
  ${MAPSELECT_SRCS}
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
  ${AMAP_LIBS}
  neta_https
  )

install(
  TARGETS mf
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
