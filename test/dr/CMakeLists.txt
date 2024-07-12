cmake_minimum_required(VERSION 3.16)

project(dr_ut)

include_directories(
    ${CMAKE_SOURCE_DIR}/modules/dr/include
)

file(GLOB _SRC "*.cc")
file(GLOB SOURCE_FILES "${CMAKE_SOURCE_DIR}/modules/dr/src/*.cc")


set(LIBS
    globalproto
    gtest
    perception-lib-mal
    opencv_calib3d
    opencv_features2d
    opencv_imgcodecs
    opencv_imgproc
    opencv_highgui
    opencv_core
    opencv_flann
    opencv_videoio
    ${DEPENDS_LIBS}
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${_SRC})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX_UT}/lib)

