cmake_minimum_required(VERSION 3.16)

project(laneline_postprocess_test_ut)

include_directories(
    ${CMAKE_SOURCE_DIR}/modules/laneline_postprocess
    ${CMAKE_SOURCE_DIR}/modules
)

file(GLOB _SRC "*.cc")
file(GLOB_RECURSE _SRCS "${CMAKE_SOURCE_DIR}/modules/laneline_postprocess/*.cc")

# file(GLOB_RECURSE LANELINE_PROTO_FILES "${CMAKE_SOURCE_DIR}/modules/laneline_postprocess/lib/laneline/proto/*.cc")


set(LIBS
    gtest
    perception-base-mal
    perception-lib-mal
    opencv_flann
    ceres
    lane_post_inner_proto
    ${DEPENDS_LIBS}
)

add_library(${PROJECT_NAME} SHARED ${_SRC} ${_SRCS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})
add_dependencies(laneline_postprocess_test_ut lane_post_inner_proto)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX_UT}/lib)

