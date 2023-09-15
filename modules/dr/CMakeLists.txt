project(dr)

#file(GLOB_RECURSE PROTO_FILES "*.proto")
#PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HEADERS ${PROTO_FILES})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(dr
    ${PROTO_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dr.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/odometry_base.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/wheel_odom.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/odometry2D.cc
)
target_link_libraries(dr
    europa_common_proto
    gtest
    
    opencv_calib3d
    opencv_features2d
    opencv_imgcodecs
    opencv_imgproc
    opencv_highgui
    opencv_core
    opencv_flann
    opencv_videoio

    pcl_common
    pcl_io
    pcl_kdtree
    pcl_filters
    pcl_search
    pcl_segmentation
    ${DEPENDS_LIBS}
)

install(
    TARGETS dr
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
