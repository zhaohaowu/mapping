cmake_minimum_required(VERSION 3.0.2)
project(ad_utils)

add_definitions(-std=c++11)

set(MAJOR_VERSION 1)
set(MINOR_VERSION 0)
set(PATCH_VERSION 0)

# find_package(ADFramework REQUIRED COMPONENTS ad_common ad_log)

include_directories(
    include
    ${PATH_SOURCE_3RDPARTY}
    ${PATH_AD_INCLUDE}/ad_log/include
    ${PATH_AD_INCLUDE}/ad_common/include
)

#############
##  Build  ##
#############

file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SRCS})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION} SOVERSION ${MAJOR_VERSION})
target_link_libraries(${PROJECT_NAME} 
                    #   ${ADFramework_LIBRARIES}
                    #   ${QNX_BACKTRACE_LIB}
                      )

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