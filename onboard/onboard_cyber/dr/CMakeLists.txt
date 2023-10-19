set(MAPPING_ONBOARD_LITE_DR onboard_cyber_dr)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(${MAPPING_ONBOARD_LITE_DR} ${PROTO_SRCS}
    dr_cyber.cpp 
)
target_link_libraries(${MAPPING_ONBOARD_LITE_DR}
    # util
    # perception-onboard-common
    gtest
    dr
    ${DEPENDS_LIBS}
)

install(TARGETS ${MAPPING_ONBOARD_LITE_DR}
        RUNTIME
        DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib/)
