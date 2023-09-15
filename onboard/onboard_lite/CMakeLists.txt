project(mapping_onboard_lite)

#file(GLOB_RECURSE PROTO_FILES "*.proto")
#PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HEADERS ${PROTO_FILES})


file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/production DESTINATION  ${CMAKE_CURRENT_SOURCE_DIR}/../../release/x86/)
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/production DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../release/x86/)

add_subdirectory(map_fusion)
add_subdirectory(dr)