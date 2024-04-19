cmake_minimum_required(VERSION 3.0.2)
project(ad_scm)

add_definitions(-std=c++11)

set(MAJOR_VERSION 1)
set(MINOR_VERSION 0)
set(PATCH_VERSION 1)

if(WITH_TDA)
    find_package(OpenCV REQUIRED PATHS /usr/local/opencv-3.4.12/share NO_DEFAULT_PATH)
else()
    find_package(OpenCV 2 REQUIRED)
endif()
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# find_package(Eigen3 REQUIRED)
# message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

# find_package(ADFramework REQUIRED COMPONENTS ad_common ad_log)

#############
##  Build  ##
#############

file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

include_directories(
    include
    # ${ADFramework_INCLUDE_DIRS}
    ${PATH_SOURCE_3RDPARTY}
    ${PATH_AD_INCLUDE}/ad_common/include
    ${PATH_AD_INCLUDE}/ad_log/include
)


add_library(${PROJECT_NAME} SHARED ${SRCS})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION} SOVERSION ${MAJOR_VERSION})
target_include_directories(${PROJECT_NAME}
    PUBLIC
        include
    PRIVATE
        # ${OpenCV_INCLUDE_DIRS}
        # ${EIGEN3_INCLUDE_DIR}
        # ${ADFramework_INCLUDE_DIRS}
        # ${PATH_SOURCE_3RDPARTY}
        # ${PATH_AD_INCLUDE}/ad_common/include
)
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
