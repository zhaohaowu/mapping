cmake_minimum_required(VERSION 3.0.2)
project(ad_log)

add_definitions(-std=c++14)

set(MAJOR_VERSION 1)
set(MINOR_VERSION 0)
set(PATCH_VERSION 1)

#############
##  Build  ##
#############

file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

include_directories(
    include
    # ${ADFramework_INCLUDE_DIRS}
    ${PATH_SOURCE_3RDPARTY}
    ${PATH_AD_INCLUDE}/ad_common/include
)

add_library(${PROJECT_NAME} SHARED ${SRCS})
target_include_directories(${PROJECT_NAME}
    PUBLIC
        include
        ${PATH_SOURCE_3RDPARTY}
        ${PATH_AD_INCLUDE}
        )
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION} SOVERSION ${MAJOR_VERSION})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
)

install(DIRECTORY include/spdlog
    DESTINATION include
)
