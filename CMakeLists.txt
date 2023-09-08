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

# MAPPING_LIB_PREFIX is defined in compile.py
macro(add_mapping_library mlib)
    add_library(${ARGV})
    set_target_properties(${mlib} PROPERTIES OUTPUT_NAME ${MAPPING_LIB_PREFIX}${mlib})
endmacro()

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

    # MIDDLEWARE and LLVM are mandatory for europa repos and third_party
    if(${PLATFORM} STREQUAL "X86")
        set(MIDDLEWARE "CYBER")
        set(LLVM 0)
    elseif(${PLATFORM} STREQUAL "MDC")
        set(MIDDLEWARE "MDC")
        set(LLVM 1)
    endif()

    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindEigen3.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindGLOG.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindGTEST.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindGflags.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindBoost.cmake)
    # include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindCUDA.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindProtobuf.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindCyber.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindFastRTPS.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindHzAvpMap.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindOpenCV.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindYAMLCPP.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindZMQ.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindSophus.cmake)

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
    
    add_subdirectory(depend/proto)
    add_subdirectory(depend/common)
    add_subdirectory(depend/map)
endif()

include_directories(interface)

add_subdirectory(interface)
add_subdirectory(modules)

if(${PLATFORM} STREQUAL "X86")
    add_subdirectory(test)
endif()

install(FILES version.json DESTINATION ${CMAKE_SOURCE_DIR}/release)

install(
    DIRECTORY dag launch conf
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)

# 打包cyber环境
if(MIDDLEWARE MATCHES CYBER)
    install(
        # 注意结尾的斜杠/，这可以使目录重命名到目标路径
        DIRECTORY ${THIRDPARTY_ROOT_PATH}/third_party/x86/cyber/
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        USE_SOURCE_PERMISSIONS
    )
    # 为了能运行cyber，以后看有没有更好的办法
    set(PREPARE_FOR_CYBER_SHELL "
        mkdir -p x86/data/log
        ln -s x86 cyber
        mkdir -p x86/lib
        cp -r -d ../depend/third_party/x86/fastrtps/lib/lib* x86/lib
        cp -r -d ../depend/third_party/x86/protobuf/lib/lib* x86/lib
        cp -r -d ../depend/third_party/x86/yaml-cpp/lib/lib* x86/lib
    ")
    file(WRITE "${CMAKE_INSTALL_PREFIX}/../prepare_for_cyber.sh" "${PREPARE_FOR_CYBER_SHELL}")
    install(CODE "execute_process(COMMAND bash prepare_for_cyber.sh WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/..)")
endif()