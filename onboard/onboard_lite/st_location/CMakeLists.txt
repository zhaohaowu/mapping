project(mapping_location_st_location)#libst_location
set(ST_LOCATION st_location)


set(ST_INCLUDE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/st_localization/include/)
message("==============wangmeng:ST_INCLUDE_PATH:${ST_INCLUDE_PATH}")
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ST_INCLUDE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/st_localization/src/util/modules/3rdparty/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/st_localization/src/util/modules/ad_log/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/st_localization/src/util/modules/ad_scm/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/st_localization/src/util//modules/ad_common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/st_localization/src/util//modules/ad_diagmanager/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/st_localization/src/util/modules/ad_time/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/st_localization/src/util/modules/ad_utils/include

)

set(LOCATION_POSE_ESTIMATION_LITE_SRCS
    st_location.cc)

add_mapping_library(${ST_LOCATION}
    ${LOCATION_POSE_ESTIMATION_LITE_SRCS})

target_link_libraries(${ST_LOCATION}
    gtest
    ${DEPENDS_LIBS}
    #location_pose_estimation_core#==todo:咨询浩武这个库到底从哪里编译
    perception-base-mal
    perception-lib-mal
    gflags
    localization
    )

install(
    TARGETS ${ST_LOCATION}
    DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib)
