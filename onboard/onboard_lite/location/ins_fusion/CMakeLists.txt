project(mapping_location_ins_fusion_lite)
set(MAPPING_ONBOARD_LITE_INS_FUSION onboard_lite_ins_fusion)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(LOCATION_INS_LITE_SRCS
    ins_fusion_lite.cc)

add_mapping_library(${MAPPING_ONBOARD_LITE_INS_FUSION}
    ${LOCATION_INS_LITE_SRCS})

target_link_libraries(${MAPPING_ONBOARD_LITE_INS_FUSION}
    gtest
    ${DEPENDS_LIBS}
    localization_ins_fusion_core
    perception-lib-mal
    perception-base-mal
    gflags)

install(
    TARGETS ${MAPPING_ONBOARD_LITE_INS_FUSION}
    DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib)
