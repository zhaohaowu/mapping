cmake_minimum_required(VERSION 3.12.1)

project(perception-lib)

unset(BUILD_SHARED_LIBS CACHE)
option(BUILD_SHARED_LIBS "" ON)

if(${PLATFORM} STREQUAL "MDC")
    set(TARGET_PLATFORM arm)
    set(LOCAL_TARGET_PLATFORM mdc)
elseif(${PLATFORM} STREQUAL "ORIN")
    set(TARGET_PLATFORM orin)
    set(LOCAL_TARGET_PLATFORM ${TARGET_PLATFORM})
elseif(${PLATFORM} STREQUAL "X86")
    set(TARGET_PLATFORM x86)
    set(LOCAL_TARGET_PLATFORM ${TARGET_PLATFORM})
endif()

unset(BUILD_BY_PERCEPTION CACHE)
option(BUILD_BY_PERCEPTION "enable orin env" ON)
set(THIRDPARTY_ROOT ${CMAKE_SOURCE_DIR}/third_party/third_party)

if(MAPPING_SINGLE_MODULE_COMPILE)
    if(CMAKE_BUILD_TYPE MATCHES Debug)
        set(CMAKE_CXX_FLAGS "-std=c++17 -O0 -Wall -pthread -fopenmp -fPIE -fPIC -Wno-deprecated -pipe -fprofile-generate" CACHE STRING "" FORCE)
    elseif(CMAKE_BUILD_TYPE MATCHES Release)
        set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -g3 -Wall -pthread -fopenmp -fPIE -fPIC -Wno-deprecated -pipe -fprofile-generate" CACHE STRING "" FORCE)
    else()
        set(DEFAULT_COMPILE -std=c++17 -O3 -Wall -pthread -fopenmp -fPIE -fPIC -Wno-deprecated -pipe -fprofile-generate)

        # set(CMAKE_BUILD_TYPE RelWithDebInfo)
    endif()

    add_definitions("-Werror=return-type -W -Wall -Wno-unused-variable -Wno-unused-parameter -Wattributes")

    set(3RD_ROOT "${CMAKE_CURRENT_LIST_DIR}/third_party/third_party/${TARGET_PLATFORM}")

    set(THIRDPARTY_ROOT_PATH ${3RD_ROOT}/../../)
    message(STATUS "3RD_ROOT: ${3RD_ROOT}")

    set(3RD_ROOT_PROTOC ${THIRDPARTY_ROOT_PATH}/third_party/x86/protobuf)

    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${LOCAL_TARGET_PLATFORM}/config.cmake)

    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindEigen3.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindGLOG.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindGTEST.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindGflags.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindBoost.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindCUDA.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindProtobuf.cmake)
endif()

if (ENABLE_COMPILE_BASE)
    add_subdirectory(third_party/perception-base)
endif()

# file(GLOB_RECURSE _SRCS "mapping/*.cc" "mapping*.h")
# # # 排除名为 "test" 的路径
# # list(FILTER _SRCS EXCLUDE REGEX "/test/.*")
# add_library(${PROJECT_NAME} SHARED ${_SRCS})
# target_link_libraries(${PROJECT_NAME} PRIVATE
#     ${DEPENDS_LIBS}
# )
# install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

INSTALL(FILES version.json DESTINATION ${CMAKE_SOURCE_DIR}/release)

if(${PLATFORM} STREQUAL "X86")
    add_subdirectory(test)
endif()
