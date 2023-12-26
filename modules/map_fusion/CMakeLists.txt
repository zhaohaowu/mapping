project(map_fusion)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(TOPO_SRCS
  src/topo_assignment/topo_assignment.cc
  )

set(PREDICTION_SRCS
  src/map_prediction/map_prediction.cc
  src/map_prediction/viz_map.cc
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


set(MAP_FUSION_SRCS
  src/map_fusion.cc
  src/tlr_fusion.cc
  ${TOPO_SRCS}
  ${PREDICTION_SRCS}
  ${MAPSERVICE_SRCS}
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
  ${AMAP_LIBS}
  neta_https
  )

install(
  TARGETS mf
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
