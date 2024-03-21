cmake_minimum_required(VERSION 3.16)

project(phm_component_lite)

file(GLOB_RECURSE PROTO_FILES "proto/*.proto")
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HEADERS ${PROTO_FILES})

file(GLOB _SRCS "*.cc")
message(status " ==================== ${_SRCS} ")

set(LIBS
    ${DEPENDS_LIBS}
    perception-base-mal
    perception-lib-mal
    neta_phm
    mal_gflags
    glog
)

add_library(${PROJECT_NAME} SHARED ${_SRCS} ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib)
