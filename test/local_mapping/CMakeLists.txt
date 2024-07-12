cmake_minimum_required(VERSION 3.16)

project(local_mapping_ut)

include_directories(
    ${CMAKE_SOURCE_DIR}/modules/local_mapping/
    ${CMAKE_SOURCE_DIR}/modules/
    ${CMAKE_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

file(GLOB _SRC "*.cc")
set(LM_SOURCES_DIR "${CMAKE_SOURCE_DIR}/modules/local_mapping/")
file(GLOB_RECURSE LM_SRCS "${LM_SOURCES_DIR}/*.cc" "${LM_SOURCES_DIR}/*.h")

set(LIBS
    gtest
    globalproto
    util
    europa_hdmap
    adsfi_proto
    gflags
    glog
    opencv_core  
    opencv_highgui 
    opencv_imgproc 
    opencv_flann
    mf
    perception-base-mal
    perception-lib-mal
)


add_library(${PROJECT_NAME} STATIC ${_SRC} ${LM_SRCS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX_UT}/lib)

