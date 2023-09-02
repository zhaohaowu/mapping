project(map_fusion)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(MAP_FUSION_SRCS
    src/map_fusion.cc
)

add_mapping_library(mf ${MAP_FUSION_SRCS})
target_link_libraries(mf
    util
)

install(
    TARGETS mf
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

if(MIDDLEWARE MATCHES CYBER)
    add_mapping_library(mf_cyber cyber/map_fusion_component.cc)
    target_link_libraries(mf_cyber
        mf
        adsfi_proto
        cyberrt
        fastrtps
        fastcdr
    )
    install(
        TARGETS mf_cyber
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    )
endif()