##
# Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
# Song Longjun <songlongjun@senseauto.com>
##

## Begin of System variable definition
set(CMAKE_SYSTEM_NAME       Linux)
set(CMAKE_C_COMPILER        /usr/local/gcc-arm/bin/aarch64-none-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER      /usr/local/gcc-arm/bin/aarch64-none-linux-gnu-g++)
set(TARGET_OS linux)
set(PLATFORM_NAME aarch64)
set(CMAKE_SYSTEM_PROCESSOR  aarch64)
## End of System variable definition

## Begin of Compilation variable definition
add_definitions(-DWITH_TDA)
## End of Compilation variable definition

## Begin of 3rd party path definition
set(BOOST_INC_DIR "/usr/local/boost-1.58/${TARGET_OS}_${PLATFORM_NAME}/include")
set(PATH_SOURCE_3RDPARTY "/usr/local/senseauto_local/3rdparty")
set (TIDL_ROOT "${PATH_SOURCE_3RDPARTY}/tidl")
set (TIDL_INCLUDE_DIRS
    "${TIDL_ROOT}/include"
    "${TIDL_ROOT}/include/ti"
    "${TIDL_ROOT}/include/TI"
    "${TIDL_ROOT}/include/VX"
)

IF(EXISTS "${TIDL_ROOT}/lib/libti_rpmsg_char.so")
    SET(TI_RPMSG_LIB 
    "${TIDL_ROOT}/lib/libti_rpmsg_char.so"
    "${TIDL_ROOT}/lib/libti_rpmsg_char.so.0"
    "${TIDL_ROOT}/lib/libti_rpmsg_char.so.0.3.0")
ENDIF()

IF(EXISTS "${TIDL_ROOT}/lib/libvx_target_kernels_img_proc_a72.a")
    SET(IMG_PROC_A72_LIB
        "${TIDL_ROOT}/lib/libvx_target_kernels_img_proc_a72.a")
ENDIF()

set (TI_LIBRARIES
    "${TIDL_ROOT}/lib/libvx_kernels_hwa.a"
    # for below two static libraries, we list one of the two twice for solving loop reference
    "${TIDL_ROOT}/lib/libvx_platform_psdk_j7_linux.a"
    "${TIDL_ROOT}/lib/libvx_framework.a"
    "${TIDL_ROOT}/lib/libvx_platform_psdk_j7_linux.a"
    "${TIDL_ROOT}/lib/libvx_kernels_tidl.a"
    "${TIDL_ROOT}/lib/libvx_kernels_openvx_core.a"
    "${TIDL_ROOT}/lib/libvx_kernels_host_utils.a"
    "${TIDL_ROOT}/lib/libvx_kernels_imaging.a"
    "${TIDL_ROOT}/lib/libvx_target_kernels_imaging_aewb.a"
    "${TIDL_ROOT}/lib/libapp_utils_iss.a"
    "${TIDL_ROOT}/lib/libapp_utils_mem.a"
    "${TIDL_ROOT}/lib/libapp_utils_ipc.a"
    "${TIDL_ROOT}/lib/libapp_utils_console_io.a"
    "${TIDL_ROOT}/lib/libvx_kernels_common.a"
    "${IMG_PROC_A72_LIB}"
    "${TIDL_ROOT}/lib/libvx_kernels_img_proc.a"
    "${TIDL_ROOT}/lib/libvx_kernels_target_utils.a"
    "${TIDL_ROOT}/lib/libapp_tirtos_linux_mpu1_common.a"
    "${TIDL_ROOT}/lib/libapp_utils_mem.a"
    "${TIDL_ROOT}/lib/libapp_utils_remote_service.a"
    "${TIDL_ROOT}/lib/libapp_utils_perf_stats.a"
    "${TI_RPMSG_LIB}"
)
## End of 3rd party path definition