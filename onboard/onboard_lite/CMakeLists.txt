project(mapping_onboard_lite)

#file(GLOB_RECURSE PROTO_FILES "*.proto")
#PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HEADERS ${PROTO_FILES})

set(MAPPING_ONBOARD_LITE_DR onboard_lite_dr)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/dr/include
)
add_library(${MAPPING_ONBOARD_LITE_DR} ${PROTO_SRCS}
    dr/dr_lite.cpp 
)
target_link_libraries(${MAPPING_ONBOARD_LITE_DR}
    util
    perception-onboard-common
    gtest
    dr
    ${DEPENDS_LIBS}
)

install(TARGETS ${MAPPING_ONBOARD_LITE_DR} RUNTIME DESTINATION lib/)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/production DESTINATION  ${CMAKE_CURRENT_SOURCE_DIR}/../../release/x86/)
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/production DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../release/x86/)