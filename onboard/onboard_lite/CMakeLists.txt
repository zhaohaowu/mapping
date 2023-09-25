project(mapping_onboard_lite)

#file(GLOB_RECURSE PROTO_FILES "*.proto")
#PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HEADERS ${PROTO_FILES})

# add_subdirectory(map_fusion)
# add_subdirectory(dr)
add_subdirectory(location)
