cmake_minimum_required(VERSION 3.16)

project(mapping_fusion_test_ut)

include_directories(
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/include/map_fusion
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/include/
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/
    ${CMAKE_SOURCE_DIR}/modules/util/include/util
    ${CMAKE_SOURCE_DIR}/modules/util/include/
    ${CMAKE_SOURCE_DIR}/modules/util/
    ${CMAKE_SOURCE_DIR}/modules/
    ${CMAKE_SOURCE_DIR}
)

set(FUSION_COMMON_SRCS
  ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/fusion_common/calc_util.cc
  ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/fusion_common/viz_util.cc
)

set(TOPO_SRCS
  ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/topo_assignment/topo_assignment.cc
)

set(PREDICTION_SRCS
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_prediction/map_prediction.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_prediction/viz_map.cc
)

set(RECOGNITION_SRC
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/road_recognition/road_recognition.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/road_recognition/geo_optimization.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/road_recognition/topo_generation.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/road_recognition/routing_generation.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/road_recognition/group_map.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/road_recognition/path_manager.cc
)

set(MAPSERVICE_SRCS
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_service/ehp/amap_core.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_service/ehp/amap_data.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_service/ehp/amap_diagnose.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_service/ehp/amap_ota.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_service/ehp/amap_system.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_service/map_proto_marker.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_service/map_service.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_service/global_hd_map.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_service/map_table.cc
)

set(MAPSELECT_SRCS
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_select/map_select_lite.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_select/lane_change_observer.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_select/double_rise_decider.cc
    ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_select/data_decider_lite.cc
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
  ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/map_fusion.cc
  ${CMAKE_SOURCE_DIR}/modules/map_fusion/src/tlr_fusion.cc
  ${TOPO_SRCS}
  ${FUSION_COMMON_SRCS}
  ${PREDICTION_SRCS}
  ${MAPSERVICE_SRCS}
  ${RECOGNITION_SRC}
  ${MAPSELECT_SRCS}
)

set(LIBS
    gtest
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
    ${DEPENDS_LIBS}
    gflags
)

file(GLOB _SRC "*.cc")

add_library(${PROJECT_NAME} STATIC ${_SRC} ${MAP_FUSION_SRCS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX_UT}/lib)

