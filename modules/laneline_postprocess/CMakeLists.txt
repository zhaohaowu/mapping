
project(laneline_postprocess)


message("modules_path: ${CMAKE_SOURCE_DIR}/modules")
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../
)

file(GLOB_RECURSE _SRCS "*.cc" "*.h")

file(GLOB_RECURSE PROTO_FILES
    "lib/laneline/proto/*.proto"
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HEADERS ${PROTO_FILES})

# add_mapping_library(laneline_postprocess_core SHARED ${_SRCS} ${PROTO_SRCS})

add_library(laneline_postprocess_core SHARED ${_SRCS} ${PROTO_SRCS})

target_link_libraries(laneline_postprocess_core PRIVATE
    ${DEPENDS_LIBS}
    perception-base-mal
    perception-lib-mal
    opencv_flann
    ceres
)

set_target_properties(laneline_postprocess_core PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS laneline_postprocess_core
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)