add_definitions(-DUSE_PLATFORM_ORIN)
add_definitions(-DUSE_GPU=1)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/orin/toolchain.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/orin/dependence.cmake)

set(CMAKE_FIND_ROOT_PATH "${THIRDPARTY_ROOT_PATH}/nos/orin/lib/cmake;${CMAKE_FIND_ROOT_PATH}")
# 设置变量NETA_THIRDPARTY_DIR为third_party的路径。若不设置，则默认为output的同级目录下的third_party。
set(NETA_THIRDPARTY_DIR ${THIRDPARTY_ROOT_PATH}/third_party)
# CONFIG模式 查找 netaos
find_package(netaos CONFIG REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/orin_sdk/usr/local/cuda-11.4/targets/aarch64-linux/include
    ${THIRDPARTY_ROOT_PATH}/nos/orin/include
    # ${THIRDPARTY_ROOT_PATH}/perception-common-onboard/release/${LOCAL_TARGET_PLATFORM}/include/perception-common-onboard
    # ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/include/perception-base
    ${THIRDPARTY_ROOT_PATH}/nos/${LOCAL_TARGET_PLATFORM}/include/fast-dds
    ${THIRDPARTY_ROOT_PATH}/nos/${LOCAL_TARGET_PLATFORM}/include/fast-dds/include
    ${THIRDPARTY_ROOT_PATH}/perception-base
    ${THIRDPARTY_ROOT_PATH}/perception-lib
    ${CMAKE_CURRENT_SOURCE_DIR}/onboard/onboard_lite/
)

link_directories(
    ${THIRDPARTY_ROOT_PATH}/nos/${LOCAL_TARGET_PLATFORM}/lib
    /usr/local/orin_sdk/usr/local/cuda-11.4/targets/aarch64-linux/lib
    # ${THIRDPARTY_ROOT_PATH}/perception-common-onboard/release/${LOCAL_TARGET_PLATFORM}/lib
    # ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/lib
    ${THIRDPARTY_ROOT_PATH}/third_party/${LOCAL_TARGET_PLATFORM}/fast-dds/lib
    ${THIRDPARTY_ROOT_PATH}/perception-lib/release/${LOCAL_TARGET_PLATFORM}/lib
)
