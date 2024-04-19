##
# Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
# Xu Minhao <xuminhao@senseauto.com>
##

## Begin of System variable definition
set(CMAKE_SYSTEM_NAME QNX)
set(CMAKE_C_COMPILER        "$ENV{QNX_HOST}/usr/bin/aarch64-unknown-nto-qnx7.1.0-gcc")
set(CMAKE_CXX_COMPILER      "$ENV{QNX_HOST}/usr/bin/aarch64-unknown-nto-qnx7.1.0-g++")
set(TARGET_OS qnx)
set(PLATFORM_NAME aarch64)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_SYSROOT $ENV{QNX_TARGET})
## End of System variable definition

## Begin of Compilation variable definition
add_definitions(-D_QNX_SOURCE)
add_definitions(-DWITH_TDA_QNX)
add_definitions(-DWITH_TDA)
## End of Compilation variable definition

## Begin of 3rd party path definition
set(PATH_SOURCE_3RDPARTY "/usr/local/senseauto_local/3rdparty")

# PCL will use this variable to find boost path
set(BOOST_ROOT_DIR "/usr/local/boost-1.58/${TARGET_OS}_${PLATFORM_NAME}/")
set(BOOST_INC_DIR "/usr/local/boost-1.58/${TARGET_OS}_${PLATFORM_NAME}/include")

if (WITH_ADCU)
    set(TIDL_ROOT "${PATH_SOURCE_3RDPARTY}/tidl/ADCU")
elseif(WITH_PDCU)
    set(TIDL_ROOT "${PATH_SOURCE_3RDPARTY}/tidl/PDCU")
endif()
set(TI_DR_LIB_ROOT "${PATH_SOURCE_3RDPARTY}/tidr/")
set(TI_DR_LIBS  
   "${TI_DR_LIB_ROOT}/lib/ti.osal.aa72fg"
   )

SET (TIDL_INCLUDE_DIRS
    "${TIDL_ROOT}/include"
    "${TIDL_ROOT}/include/ti"
    "${TIDL_ROOT}/include/TI"
    "${TIDL_ROOT}/include/VX"
)

SET(TI_SHM_LIB
    "${TIDL_ROOT}/lib/libsharedmemallocator.so"
)      
    
SET(TI_RPMSG_LIB 
    "${TIDL_ROOT}/lib/libtiipc-usr.so"
)

SET(QNX_BACKTRACE_LIB
    "backtrace")

IF(EXISTS "${TIDL_ROOT}/lib/libvx_target_kernels_img_proc_a72.a")
    SET(IMG_PROC_A72_LIB
        "${TIDL_ROOT}/lib/libvx_target_kernels_img_proc_a72.a")
ENDIF()

set (TI_LIBRARIES
    "${TIDL_ROOT}/lib/libvx_kernels_hwa.a"
    # for below two static libraries, we list one of the two twice for solving loop reference
    "${TIDL_ROOT}/lib/libvx_platform_psdk_j7_qnx.a"
    "${TIDL_ROOT}/lib/libvx_framework.a"
    "${TIDL_ROOT}/lib/libvx_platform_psdk_j7_qnx.a"
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
    "${TIDL_ROOT}/lib/libapp_tirtos_qnx_mpu1_common.a"
    "${TIDL_ROOT}/lib/libapp_utils_mem.a"
    "${TIDL_ROOT}/lib/libapp_utils_remote_service.a"
    "${TIDL_ROOT}/lib/libapp_utils_perf_stats.a"
    "${TI_CPSW_LIB}"
    "${TI_SHM_LIB}" 
    "${TI_RPMSG_LIB}"
    "${TI_DR_LIBS}"
)