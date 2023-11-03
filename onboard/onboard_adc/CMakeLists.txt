project(mapping_onboard_adc)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/map_fusion/include
)

# add_library(mapping_onboard_adc
#     ${PROTO_SRCS}
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/data_board.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/mapping_onboard.cc
# )

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../depend/ap-release/lib
)
message("------------${CMAKE_CURRENT_SOURCE_DIR}/../../depend/ap-release/lib")

add_executable(hz_mapping
    ${PROTO_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data_board.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mapping_onboard.cc
)

target_link_libraries(hz_mapping
    globalproto
    hz_adsfi
    dr
    util
    adsfi_proto
    lmap_core
    location_asbl_core
    ${DEPENDS_LIBS}
)

add_subdirectory(production)

set_target_properties(hz_mapping PROPERTIES LINK_FLAGS "-Wl,--as-needed,--disable-new-dtags")

install(
    TARGETS hz_mapping
    DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/bin
)
