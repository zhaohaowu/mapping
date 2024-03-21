project(mapping_location_dr_fusion_lite)
set(MAPPING_ONBOARD_LITE_DR_FUSION onboard_lite_dr_fusion)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(LOCATION_DR_LITE_SRCS
    dr_fusion_lite.cc)

add_mapping_library(${MAPPING_ONBOARD_LITE_DR_FUSION}
    ${LOCATION_DR_LITE_SRCS})

target_link_libraries(${MAPPING_ONBOARD_LITE_DR_FUSION}
    gtest
    ${DEPENDS_LIBS}
    localization_dr_fusion_core
    perception-base-mal
    perception-lib-mal
    gflags)

install(
    TARGETS ${MAPPING_ONBOARD_LITE_DR_FUSION}
    DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib)
