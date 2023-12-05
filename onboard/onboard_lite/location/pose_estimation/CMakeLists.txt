project(mapping_location_pose_estimation_lite)
set(MAPPING_ONBOARD_LITE_POSE_ESTIMATION onboard_lite_pose_estimation)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(LOCATION_POSE_ESTIMATION_LITE_SRCS
    pose_estimation_lite.cc)

add_mapping_library(${MAPPING_ONBOARD_LITE_POSE_ESTIMATION}
    ${LOCATION_POSE_ESTIMATION_LITE_SRCS})

target_link_libraries(${MAPPING_ONBOARD_LITE_POSE_ESTIMATION}
    gtest
    ${DEPENDS_LIBS}
    location_pose_estimation_core
    gflags)

install(
    TARGETS ${MAPPING_ONBOARD_LITE_POSE_ESTIMATION}
    DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib)
