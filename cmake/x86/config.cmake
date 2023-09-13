
add_definitions(-DUSE_PLATFORM_X86)
add_definitions(-DUSE_GPU=1)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/x86/dependence.cmake)
# 设置find路径。假设TARGET_PLATFORM变量为平台orin/x86/mdc-llvm/j5等
set(CMAKE_FIND_ROOT_PATH "${THIRDPARTY_ROOT_PATH}/nos/x86_2004/lib/cmake;${CMAKE_FIND_ROOT_PATH}")
# 设置变量NETA_THIRDPARTY_DIR为third_party的路径。若不设置，则默认为output的同级目录下的third_party。
set(NETA_THIRDPARTY_DIR ${THIRDPARTY_ROOT_PATH}/third_party)
# CONFIG模式 查找 netaos
find_package(netaos CONFIG REQUIRED)

set(OPENGV_INCLUDE_DIR ${THIRDPARTY_ROOT_PATH}/third_party/x86/opengv/include)
set(OPENCV_INCLUDE_DIR ${THIRDPARTY_ROOT_PATH}/third_party/x86/opencv/include)
set(G2O_INCLUDE_DIR ${THIRDPARTY_ROOT_PATH}/third_party/x86/g2o/include)
set(PCL_INCLUDE_DIR ${THIRDPARTY_ROOT_PATH}/third_party/x86/pcl/include/pcl-1.11)
set(JSON_INCLUDE_DIR ${THIRDPARTY_ROOT_PATH}/third_party/x86/nlohmann_json/include)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}

    ${PCL_INCLUDE_DIR}

    ${THIRDPARTY_ROOT_PATH}
    ${THIRDPARTY_ROOT_PATH}/nos/x86_2004/include
    ${THIRDPARTY_ROOT_PATH}/ap-release/include/
    ${THIRDPARTY_ROOT_PATH}/perception-common-onboard/release/x86/include/perception-common-onboard

    ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/include/perception-base
    # ${THIRDPARTY_ROOT_PATH}/nos/${LOCAL_TARGET_PLATFORM}_2004/include
    ${THIRDPARTY_ROOT_PATH}/third_party/x86/fast-dds/include
    ${THIRDPARTY_ROOT_PATH}/third_party/x86/json/include
    ${THIRDPARTY_ROOT_PATH}/third_party/x86/hzavpmap/include
)

link_directories(
    ${THIRDPARTY_ROOT_PATH}/perception-common-onboard/release/x86/lib

    ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/lib
    # ${THIRDPARTY_ROOT_PATH}/nos/${LOCAL_TARGET_PLATFORM}_2004/lib
    ${THIRDPARTY_ROOT_PATH}/third_party/x86/fast-dds/lib
    ${THIRDPARTY_ROOT_PATH}/third_party/x8/pcl/lib
)
