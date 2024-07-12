cmake_minimum_required(VERSION 3.16)

project(phm_comment_lite_ut)

file(GLOB SOURCE_FILES "*.cc")
file(GLOB PROTO_SOURCE_FILES "${CMAKE_SOURCE_DIR}/onboard/onboard_lite/phm_comment_lite/proto/*.cc")
file(GLOB PHM_SOURCE_FILES "${CMAKE_SOURCE_DIR}/onboard/onboard_lite/phm_comment_lite/*.cc")
list(REMOVE_ITEM PHM_SOURCE_FILES "${CMAKE_SOURCE_DIR}/onboard/onboard_lite/phm_comment_lite/comment_lite.cc")

set(LIBS
    ${DEPENDS_LIBS}
    perception-base-mal
    perception-lib-mal
    neta_phm
    mal_gflags
    glog
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${PROTO_SOURCE_FILES} ${PHM_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX_UT}/lib)

