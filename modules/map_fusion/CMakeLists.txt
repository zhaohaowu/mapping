project(map_fusion)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(PROVIDER_SRCS
  src/prior_provider/prior_provider.cc
  )

set(TOPO_SRCS
  src/local_map_provider/local_map_provider.cc
  )

set(PREDICTION_SRCS
  src/map_prediction/map_prediction.cc
  )

set(MAP_FUSION_SRCS
  src/map_fusion.cc
  ${PROVIDER_SRCS}
  ${TOPO_SRCS}
  ${PREDICTION_SRCS}
  )

add_mapping_library(mf ${MAP_FUSION_SRCS})
target_link_libraries(mf
  util
  adsfi_proto
  europa_common_proto
  )

install(
  TARGETS mf
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

if (MIDDLEWARE MATCHES CYBER)
  set(PROVIDER_CYBER_SRCS
    cyber/prior_provider_component.cc
    )

  set(TOPO_CYBER_SRCS
    cyber/local_map_provider_component.cc
    )

  set(PREDICTION_CYBER_SRCS
    cyber/map_prediction_component.cc
    )

  set(MAP_FUSION_CYBER_SRCS
    ${PROVIDER_CYBER_SRCS}
    ${TOPO_CYBER_SRCS}
    ${PREDICTION_CYBER_SRCS}
    )

  add_mapping_library(mf_cyber
    cyber/map_fusion_component.cc
    ${MAP_FUSION_CYBER_SRCS}
    )
  target_link_libraries(mf_cyber
    mf
    cyberrt
    fastrtps
    fastcdr
    )
  install(
    TARGETS mf_cyber
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  )
endif ()