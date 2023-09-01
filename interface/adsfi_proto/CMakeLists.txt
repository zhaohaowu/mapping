project(adsfi_proto)

if(NOT COMMAND protobuf_generate_cpp)
    message(FATAL_ERROR "protobuf_generate_cpp not defined, should include FindProtobuf.cmake first")
endif()

file(
  GLOB
  ADSFI_PROTOS
  "common/*.proto"
  "image/*.proto"
  "internal/*.proto"
  "location/*.proto"
  "map/*.proto"
  "perception/*.proto"
  "sensors/*.proto"
  "vehicle/*.proto"
  "viz/*.proto"
)

protobuf_generate_cpp(ADSFI_PROTO_SRCS ADSFI_PROTO_HDRS ${ADSFI_PROTOS})

add_mapping_library(adsfi_proto SHARED ${ADSFI_PROTO_SRCS})
set_target_properties(adsfi_proto PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(adsfi_proto protobuf)

install(
    TARGETS adsfi_proto
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
