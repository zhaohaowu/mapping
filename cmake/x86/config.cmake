
add_definitions(-DUSE_PLATFORM_X86)
add_definitions(-DUSE_GPU=1)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/x86/dependence.cmake)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/include/perception-base
    # ${THIRDPARTY_ROOT_PATH}/nos/${LOCAL_TARGET_PLATFORM}_2004/include
)

link_directories(
    ${THIRDPARTY_ROOT_PATH}/perception-base/release/${LOCAL_TARGET_PLATFORM}/lib
    # ${THIRDPARTY_ROOT_PATH}/nos/${LOCAL_TARGET_PLATFORM}_2004/lib
)
