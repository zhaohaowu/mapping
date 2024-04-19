cmake_minimum_required(VERSION 3.0.2)
project(ad_diagmanager)

add_definitions(-std=c++11)

set(MAJOR_VERSION 1)
set(MINOR_VERSION 0)
set(PATCH_VERSION 0)

# find_package(ADFramework REQUIRED COMPONENTS ad_log)

include_directories(
    include
    # ${ADFramework_INCLUDE_DIRS}
    ${PATH_SOURCE_3RDPARTY}
    ${PATH_AD_INCLUDE}/ad_log/include
)

#############
##  Build  ##
#############

file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SRCS})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION} SOVERSION ${MAJOR_VERSION})

IF(WITH_TDA)
    remove_linked_libraries(${ADFramework_LIBRARIES} pthread)
ENDIF()

target_link_libraries(${PROJECT_NAME})


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
