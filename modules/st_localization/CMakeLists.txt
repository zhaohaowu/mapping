
project(localization)

# add_subdirectory(src/util)

if(${LOCAL_TARGET_PLATFORM} STREQUAL "x86")
message("1CMAKE_INCLUDE_PATH: ${CMAKE_INCLUDE_PATH}")
list(REMOVE_ITEM CMAKE_INCLUDE_PATH ${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/ceres/include)
message("CMAKE_INCLUDE_PATH: ${CMAKE_INCLUDE_PATH}")
# 获取全局的include_directories属性
get_property(GLOBAL_INCLUDE_DIRECTORIES DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/.. PROPERTY INCLUDE_DIRECTORIES)

# 移除特定目录
# message("fdslkfjl:  ${GLOBAL_INCLUDE_DIRECTORIES}")
# list(REMOVE_ITEM GLOBAL_INCLUDE_DIRECTORIES ${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/ceres/include)
# message("11fdslkfjl:  ${CMAKE_BINARY_DIR}")

message("22fdslkfjl:  ${PROJECT_SOURCE_DIR}/include")

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util/modules/3rdparty/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util/modules/ad_log/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util/modules/ad_scm/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util//modules/ad_common/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util//modules/ad_diagmanager/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util/modules/ad_time/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util/modules/ad_utils/include
  # ${CMAKE_SOURCE_DIR}/depend/hozon_3rdparty/linux_x86_64/include
)
endif()

if(${LOCAL_TARGET_PLATFORM} STREQUAL "orin")
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util/modules/3rdparty/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util/modules/ad_log/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util/modules/ad_scm/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util//modules/ad_common/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util//modules/ad_diagmanager/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util/modules/ad_time/include
  ${CMAKE_SOURCE_DIR}/modules/st_localization/src/util/modules/ad_utils/include
  # ${CMAKE_SOURCE_DIR}/depend/hozon_3rdparty/qnx_aarch64/include
)
endif()

file(GLOB_RECURSE _SRCS "*.cpp" "*.hpp")


if(${LOCAL_TARGET_PLATFORM} STREQUAL "x86")

link_directories(
  # ${CMAKE_SOURCE_DIR}/depend/hozon_3rdparty/linux_x86_64/lib
)

endif()

if(${LOCAL_TARGET_PLATFORM} STREQUAL "orin")

link_directories(
  # ${CMAKE_SOURCE_DIR}/depend/hozon_3rdparty/qnx_aarch64/lib
)

endif()


add_library(localization SHARED ${_SRCS})

target_link_libraries(localization PRIVATE
    ${DEPENDS_LIBS}
    # perception-base-mal
    # perception-lib-mal
    opencv_flann
    libceres.a
    boost_filesystem
    # pcl
)

set_target_properties(localization PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS localization
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/lib
)