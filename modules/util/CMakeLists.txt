project(util)

include_directories(include)

set(NODELINK_SRCS
    src/nodelink/core/linker.cc
    src/nodelink/core/node.cc
    src/nodelink/core/pub_worker.cc
    src/nodelink/core/publisher.cc
    src/nodelink/core/sub_worker.cc
    src/nodelink/deploy/deployer.cc
    src/nodelink/deploy/loader.cc
    src/nodelink/sample/sample_bar.cc
    src/nodelink/sample/sample_foo.cc
)

set(RVIZ_AGENT_SRCS
    src/rviz_agent/rviz_agent_client.cc
    src/rviz_agent/rviz_agent.cc
)

set(GEO_SRCS
   src/geo.cc
)

set(UTIL_SRCS
    ${NODELINK_SRCS}
    ${RVIZ_AGENT_SRCS}
    ${GEO_SRCS}
    src/rate.cc
)

add_mapping_library(util ${UTIL_SRCS})

target_link_libraries(util
    adsfi_proto
    pthread
    zmq
    glog
    yaml-cpp
    ${CMAKE_DL_LIBS}
)
install(
    TARGETS util
    DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)
