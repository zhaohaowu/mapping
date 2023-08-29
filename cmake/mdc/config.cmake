
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/mdc/toolchain.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/mdc/dependence.cmake)

add_definitions(-DUSE_PLATFORM_MDC)
add_definitions(-DMDC_PRODUCTION_MDC610)

set(MDC_ACLLIB_SDK ${MDC_SDK}/sysroot/usr/local/Ascend/runtime)
set(CROSS_SDK_ROOT ${MDC_SDK}/sysroot)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CROSS_SDK_ROOT}/usr/include/adsfi/adb/include
    ${MDC_ACLLIB_SDK}/include
    ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/include/perception-base
)

link_directories(
    ${CROSS_SDK_ROOT}/usr/local/Ascend/runtime/lib64/
    ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/lib
)