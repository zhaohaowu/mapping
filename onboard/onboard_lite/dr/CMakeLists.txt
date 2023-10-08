set(MAPPING_ONBOARD_LITE_DR onboard_lite_dr)

# include_directories(
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/dr/include
# )
add_library(${MAPPING_ONBOARD_LITE_DR} ${PROTO_SRCS}
    dr_lite.cpp 
)
target_link_libraries(${MAPPING_ONBOARD_LITE_DR}
    util
    perception-onboard-common
    gtest
    dr
    ${DEPENDS_LIBS}
)

install(
    TARGETS ${MAPPING_ONBOARD_LITE_DR}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
