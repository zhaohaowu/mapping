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

include(cmake/mapping.cmake)

set(CMAKE_INSTALL_PREFIX_RS ${CMAKE_INSTALL_PREFIX}/runtime_service/mapping)
set(EXTERNAL_LIB_PATH ${CMAKE_INSTALL_PREFIX}/lib)

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


    if(${LOCAL_TARGET_PLATFORM} STREQUAL "orin")
        include_directories(${THIRDPARTY_ROOT_PATH}/third_party/${LOCAL_TARGET_PLATFORM}/eigen/include/eigen3)
    elseif(${LOCAL_TARGET_PLATFORM} STREQUAL "x86") 
        include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindEigen3.cmake)
    elseif(${LOCAL_TARGET_PLATFORM} STREQUAL "mdc")
        include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindEigen3.cmake)
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
    include_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/cuda/targets/x86_64-linux/include)

    message("cudaheader====" ${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/cuda/targets/x86_64-linux/include)    

    link_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/pcl/lib)
    include_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/ceres/include)
    link_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/ceres/lib)

    include_directories(${DEPEND})
    include_directories(depend/ap-release/include/adsfi/include)
    include_directories(depend/ap-release/include/generated/includes)

    include_directories(depend/perception-lib/release/${LOCAL_TARGET_PLATFORM}/include)
    link_directories(depend/perception-lib/release/${LOCAL_TARGET_PLATFORM}/lib)

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
    # add_subdirectory(depend/proto)
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
add_subdirectory(depend/proto) # 必须是这个顺序，否则libglobalproto.so会被third_party覆盖

if(${LOCAL_TARGET_PLATFORM} STREQUAL "x86")
    add_subdirectory(test)
endif()

install(FILES version.json DESTINATION ${CMAKE_INSTALL_PREFIX} RENAME mal_version.json)

install(
    DIRECTORY conf
    DESTINATION ${CMAKE_INSTALL_PREFIX_RS}
)

# 打包third party环境
if(IND)
    install(DIRECTORY 
        ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/yaml-cpp/lib/
        ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/glog/lib/
        ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/gflags/lib/
        ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/protobuf/lib/
        ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/ceres/lib/
        ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/boost/lib/
        ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/opencv/lib/
        ${DEPEND}/perception-common-onboard/release/${LOCAL_TARGET_PLATFORM}/lib/
        ${DEPEND}/perception-lib/release/${LOCAL_TARGET_PLATFORM}/lib/
        ${DEPEND}/perception-base/release/${LOCAL_TARGET_PLATFORM}/lib/
        DESTINATION ${EXTERNAL_LIB_PATH}
    )

    if (NOT (${PLATFORM} STREQUAL "mdc"))
        install(
            DIRECTORY ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/fast-dds/lib/
            DESTINATION ${EXTERNAL_LIB_PATH}
        )
    endif()

    if(NOT (MIDDLEWARE MATCHES CYBER))
        install(DIRECTORY
            ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/cuda/targets/x86_64-linux/lib/
            DESTINATION ${EXTERNAL_LIB_PATH}
            FILES_MATCHING
            PATTERN "libcudart*"
        ) 
    endif()

    if(${PLATFORM} STREQUAL "x86_2004")
        install(DIRECTORY 
            ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/fastrtps/lib/ #x86平台需要
            DESTINATION ${EXTERNAL_LIB_PATH}
        )
    endif()

    if(${PLATFORM} STREQUAL "orin")
        install(DIRECTORY 
            ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/absl/lib/  #orin平台需要
            ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/osqp/lib/  #orin平台需要
            DESTINATION ${EXTERNAL_LIB_PATH}
        )
    endif()
endif()
