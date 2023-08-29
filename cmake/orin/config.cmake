
add_definitions(-DUSE_PLATFORM_ORIN)
add_definitions(-DUSE_GPU=1)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/orin/toolchain.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/orin/dependence.cmake)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/local/orin_sdk/usr/local/cuda-11.4/targets/aarch64-linux/include
    ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/include/perception-base
)

link_directories(
    ${THIRDPARTY_ROOT_PATH}/nos/${LOCAL_TARGET_PLATFORM}/lib
    /usr/local/orin_sdk/usr/local/cuda-11.4/targets/aarch64-linux/lib
    ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/lib
)