cmake_minimum_required(VERSION 3.0.2)
project(ad_common)

set(MAJOR_VERSION 1)
set(MINOR_VERSION 0)
set(PATCH_VERSION 1)

#############
##  Build  ##
#############
include_directories(
    include
    ${PATH_SOURCE_3RDPARTY}
    ${PATH_AD_INCLUDE}/ad_log/include
    ${PATH_AD_INCLUDE}/ad_common/include
)
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
    INTERFACE
        include
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
)
