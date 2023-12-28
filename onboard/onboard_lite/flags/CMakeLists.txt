cmake_minimum_required(VERSION 3.16)

project(mal_gflags)

file(GLOB _SRCS "*.cc")

set(LIBS
    ${DEPENDS_LIBS}
    gflags
)

add_library(${PROJECT_NAME} SHARED ${_SRCS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib)
