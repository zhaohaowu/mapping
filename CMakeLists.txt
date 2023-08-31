cmake_minimum_required(VERSION 3.12.1)

project(mapping)

unset(BUILD_SHARED_LIBS CACHE)
option(BUILD_SHARED_LIBS "" ON)

# PLATFORM is set in compile.py
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
set(THIRDPARTY_ROOT ${CMAKE_SOURCE_DIR}/depend/third_party)

# MAPPING_SINGLE_MODULE_COMPILE is defined in compile.py
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

    set(3RD_ROOT "${CMAKE_CURRENT_LIST_DIR}/depend/third_party/${TARGET_PLATFORM}")

    set(THIRDPARTY_ROOT_PATH ${3RD_ROOT}/../..)
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
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindFastRTPS.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindHzAvpMap.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindOpenCV.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindYAMLCPP.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindZMQ.cmake)

    # These enable variables are defined in compile.py
    if (MAPPING_ENABLE_COMPILE_BASE)
        add_subdirectory(depend/perception-base)
    endif()
    if (MAPPING_ENABLE_COMPILE_COMBOARD)
        add_subdirectory(depend/perception-common-onboard)
    endif()
    if (MAPPING_ENABLE_COMPILE_LIB)
        add_subdirectory(depend/perception-lib)
    endif()
    
    # MIDDLEWARE is mandatory for europa repos
    if(${PLATFORM} STREQUAL "X86")
        set(MIDDLEWARE "CYBER")
    elseif(${PLATFORM} STREQUAL "MDC")
        set(MIDDLEWARE "MDC")
    endif()
    # add_subdirectory(depend/proto)
    # add_subdirectory(depend/common)
    # add_subdirectory(depend/map)
endif()

add_subdirectory(modules)

INSTALL(FILES version.json DESTINATION ${CMAKE_SOURCE_DIR}/release)

if(${PLATFORM} STREQUAL "X86")
    add_subdirectory(test)
endif()
