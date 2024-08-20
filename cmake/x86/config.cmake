
add_definitions(-DUSE_PLATFORM_X86)
add_definitions(-DUSE_GPU=1)

# 设置find路径。假设TARGET_PLATFORM变量为平台orin/x86/mdc-llvm/j5等
set(CMAKE_FIND_ROOT_PATH "${THIRDPARTY_ROOT_PATH}/nos/x86_2004/lib/cmake;${CMAKE_FIND_ROOT_PATH}")
# 设置变量NETA_THIRDPARTY_DIR为third_party的路径。若不设置，则默认为output的同级目录下的third_party。
set(NETA_THIRDPARTY_DIR ${THIRDPARTY_ROOT_PATH}/third_party)
# CONFIG模式 查找 netaos
find_package(netaos CONFIG REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/x86/dependence.cmake)

if(MIDDLEWARE MATCHES CYBER) # using cyber
include_directories(
    # 为了使用adf/include/log.h
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include
    ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/include/perception-base
)

link_directories(
    ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/lib
)
# using fastrtps in cyber, fast-dds in adf-lite
include(${THIRDPARTY_ROOT_PATH}/third_party/cmake/FindFastRTPS.cmake)
else() # using adf-lite
# 设置find路径。假设TARGET_PLATFORM变量为平台orin/x86/mdc-llvm/j5等
set(CMAKE_FIND_ROOT_PATH "${THIRDPARTY_ROOT_PATH}/nos/x86_2004/lib/cmake;${CMAKE_FIND_ROOT_PATH}")
# 设置变量NETA_THIRDPARTY_DIR为third_party的路径。若不设置，则默认为output的同级目录下的third_party。
set(NETA_THIRDPARTY_DIR ${THIRDPARTY_ROOT_PATH}/third_party)
# CONFIG模式 查找 netaos
find_package(netaos CONFIG REQUIRED)

include_directories(
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include/adf-lite/include
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include/cfg
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include/per
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include/https/include
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include/crypto/include
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include/crypto/include/common
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include/crypto/include/cryp
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include/crypto/include/keys
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include/crypto/include/utility
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include/crypto/include/x509
    # ${THIRDPARTY_ROOT_PATH}/perception-common-onboard/release/x86/include/perception-common-onboard
    # ${THIRDPARTY_ROOT_PATH}/perception-lib/release/${LOCAL_TARGET_PLATFORM}/include/perception-lib
    # ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/include/perception-base
    ${THIRDPARTY_ROOT_PATH}/third_party/x86/fast-dds/include
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include/fast-dds
    ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/yaml-cpp/include
    ${THIRDPARTY_ROOT_PATH}/perception-base
    ${THIRDPARTY_ROOT_PATH}/perception-lib
    ${CMAKE_CURRENT_SOURCE_DIR}/onboard/onboard_lite/
)

link_directories(
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/lib
    # ${THIRDPARTY_ROOT_PATH}/perception-common-onboard/release/x86/lib
    # ${THIRDPARTY_ROOT_PATH}/perception-lib/release/${LOCAL_TARGET_PLATFORM}/lib
    # ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/lib
    ${THIRDPARTY_ROOT_PATH}/third_party/x86/fast-dds/lib
    ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/gflags/lib
    ${THIRDPARTY_ROOT_PATH}/perception-lib/release/${LOCAL_TARGET_PLATFORM}/lib
)
endif()