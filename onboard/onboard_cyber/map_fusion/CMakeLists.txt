project(map_fusion_cyber)

include_directories(
    ${MAPPING}/modules/map_fusion/include
    ${MAPPING}/modules/util/include
)

set(MAPSERVICE_CYBER_SRCS
    map_service_component.cc
)

set(TOPO_CYBER_SRCS
    local_map_provider_component.cc
    topo_assignment_component.cc
)

set(PREDICTION_CYBER_SRCS
    map_prediction_component.cc
)

set(MAP_FUSION_CYBER_SRCS
    ${TOPO_CYBER_SRCS}
    ${PREDICTION_CYBER_SRCS}
    ${MAPSERVICE_CYBER_SRCS}
)

add_mapping_library(mf_cyber
    map_fusion_component.cc
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
    DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
