cmake_minimum_required(VERSION 3.12.1)

project(mapping)

unset(BUILD_SHARED_LIBS CACHE)
option(BUILD_SHARED_LIBS "" ON)

# PLATFORM is set in compile.py, TARGET_PLATFORM for third_party
# LOCAL_TARGET_PLATFORM for local output
if(${PLATFORM} STREQUAL "mdc")
    set(TARGET_PLATFORM arm)
    set(LOCAL_TARGET_PLATFORM mdc)
elseif(${PLATFORM} STREQUAL "orin")
    set(TARGET_PLATFORM orin)
    set(LOCAL_TARGET_PLATFORM ${TARGET_PLATFORM})
elseif((${PLATFORM} STREQUAL "x86") OR (${PLATFORM} STREQUAL "x86_2004"))
    set(TARGET_PLATFORM x86_2004)
    set(LOCAL_TARGET_PLATFORM x86)
endif()

unset(BUILD_BY_PERCEPTION CACHE)
option(BUILD_BY_PERCEPTION "enable orin env" ON)
set(MAPPING ${CMAKE_CURRENT_SOURCE_DIR})
set(DEPEND ${CMAKE_CURRENT_SOURCE_DIR}/depend)
set(THIRDPARTY_ROOT ${DEPEND}/third_party)

# MAPPING_LIB_PREFIX is defined in compile.py
macro(add_mapping_library mlib)
    add_library(${ARGV})
    set_target_properties(${mlib} PROPERTIES OUTPUT_NAME ${MAPPING_LIB_PREFIX}${mlib})
endmacro()
macro(add_mapping_executable mexe)
    add_executable(${ARGV})
    set_target_properties(${mexe} PROPERTIES OUTPUT_NAME ${MAPPING_LIB_PREFIX}${mexe})
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

    # for global proto
    set(PLATFORM_3RD "${CMAKE_CURRENT_LIST_DIR}/depend/third_party/${TARGET_PLATFORM}")

    set(3RD_ROOT_PROTOC ${THIRDPARTY_ROOT_PATH}/third_party/x86/protobuf)

    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${LOCAL_TARGET_PLATFORM}/config.cmake)

    # MIDDLEWARE and LLVM are mandatory for europa repos and third_party
    if(${LOCAL_TARGET_PLATFORM} STREQUAL "x86")
        set(LLVM 0)
    elseif(${LOCAL_TARGET_PLATFORM} STREQUAL "mdc")
        set(LLVM 1)
        set(MDC_SDK "/usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler" CACHE STRING "Path MDC SDK")
        set(MDC_TOOLCHAIN ${MDC_SDK}/../toolchain.cmake CACHE STRING "Path to MDC toolchain.cmake")
        include(${MDC_TOOLCHAIN})
        add_definitions(-DISMDC="is mdc")
        add_definitions(-DAP_RELEASE_IS_ADSFI_2_0)
    elseif(${LOCAL_TARGET_PLATFORM} STREQUAL "orin")
        add_definitions(-DISORIN=1)
    endif()

    list(APPEND CMAKE_MODULE_PATH ${THIRDPARTY_ROOT_PATH}/third_party/cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindEigen3.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindGLOG.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindGTEST.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindGflags.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindBoost.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindCUDA.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindProtobuf.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindCyber.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindHzAvpMap.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindOpenCV.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindYAMLCPP.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindZMQ.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindSophus.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindSqlite3.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindAmap.cmake)
    include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindAbsl.cmake)
    if(EXISTS ${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/nlohmann_json)
        include_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/nlohmann_json/include)
    elseif(EXISTS ${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/json)
        include_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/json/include)
    else()
        message(WARNING "neither nlohmann_json nor json exists in ${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}")
    endif()
    include_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/pcl/include/pcl-1.11)
    link_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/pcl/lib)
    include_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/ceres/include)
    link_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/ceres/lib)

    include_directories(${DEPEND})
    include_directories(depend/ap-release/include/adsfi/include)
    include_directories(depend/ap-release/include/generated/includes)

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

    set(FROM_TOP ON)
    add_subdirectory(depend/proto)
    add_subdirectory(depend/common)
    add_subdirectory(depend/map)
endif()

set(TARGET_DESTINATION ${MAPPING}/release/${LOCAL_TARGET_PLATFORM})
set(TARGET_DESTINATION_LIB ${TARGET_DESTINATION}/lib)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(interface)

add_subdirectory(interface)
add_subdirectory(modules)
add_subdirectory(onboard)

if(${LOCAL_TARGET_PLATFORM} STREQUAL "x86")
    add_subdirectory(test)
endif()

install(FILES version.json DESTINATION ${CMAKE_SOURCE_DIR}/release)

install(
    DIRECTORY conf
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)

# 打包third party环境

if(IND)
    set(PREPARE_FOR_CYBER_SHELL "
        mkdir -p ${LOCAL_TARGET_PLATFORM}/data/log
        mkdir -p ${TARGET_DESTINATION_LIB}
        cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/fast-dds/lib/lib* ${TARGET_DESTINATION_LIB}
        cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/yaml-cpp/lib/lib* ${TARGET_DESTINATION_LIB}
        cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/glog/lib/lib* ${TARGET_DESTINATION_LIB}
        cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/gflags/lib/lib* ${TARGET_DESTINATION_LIB}
        cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/protobuf/lib/lib* ${TARGET_DESTINATION_LIB}
        cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/ceres/lib/lib* ${TARGET_DESTINATION_LIB}
        cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/boost/lib/lib* ${TARGET_DESTINATION_LIB}
        cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/opencv/lib/lib* ${TARGET_DESTINATION_LIB}
        cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/fastrtps/lib/lib* ${TARGET_DESTINATION_LIB}
        cp -r -d ${DEPEND}/perception-common-onboard/release/${LOCAL_TARGET_PLATFORM}/lib/lib* ${TARGET_DESTINATION_LIB}
        cp -r -d ${DEPEND}/perception-lib/release/${LOCAL_TARGET_PLATFORM}/lib/lib* ${TARGET_DESTINATION_LIB}")

    file(WRITE "${CMAKE_INSTALL_PREFIX}/../PREPARE_FOR_CYBER_SHELL.sh" "${PREPARE_FOR_CYBER_SHELL}")
    install(CODE "execute_process(COMMAND bash PREPARE_FOR_CYBER_SHELL.sh WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/..)")

    set(PERCEPTIONBASELIB ${DEPEND}/perception-base/release/${LOCAL_TARGET_PLATFORM}/lib)
    install(DIRECTORY ${PERCEPTIONBASELIB} DESTINATION ${TARGET_DESTINATION})
endif()
