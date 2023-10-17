project(map_fusion)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(TOPO_SRCS
  src/local_map_provider/local_map_provider.cc
  src/topo_assignment/topo_assignment.cc
  )

set(PREDICTION_SRCS
  src/map_prediction/map_prediction.cc
  src/map_prediction/viz_map.cc
  )

if(${PLATFORM} STREQUAL "orin")
  message(STATUS "currently MapService is not supported under orin")
  set(MAPSERVICE_SRCS)
  set(AMAP_LIBS)
else()
  set(MAPSERVICE_SRCS
    src/map_service/ehp/amap_core.cc
    src/map_service/ehp/amap_data.cc
    src/map_service/ehp/amap_diagnose.cc
    src/map_service/ehp/amap_ota.cc
    src/map_service/ehp/amap_system.cc
    src/map_service/map_proto_marker.cc
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
endif()

set(MAP_FUSION_SRCS
  src/map_fusion.cc
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
  ${AMAP_LIBS}
  )

install(
  TARGETS mf
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
