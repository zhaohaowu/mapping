project(mapping_onboard_cyber)

execute_process(COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX})
execute_process(
  COMMAND ln -s x86 cyber
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/../
)

add_subdirectory(dr)
add_subdirectory(location)
add_subdirectory(local_mapping)
add_subdirectory(map_fusion)

install(
  DIRECTORY production/dag production/launch
  DESTINATION ${CMAKE_INSTALL_PREFIX}
)

install(
  # 注意结尾的斜杠/，这可以使目录重命名到目标路径
  DIRECTORY ${THIRDPARTY_ROOT_PATH}/third_party/x86/cyber/
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  USE_SOURCE_PERMISSIONS
)

# 为了能运行cyber，以后看有没有更好的办法
set(PREPARE_FOR_CYBER_SHELL "
    mkdir -p x86/data/log
    if [ ! -d cyber ]; then
      ln -s x86 cyber
    fi
    mkdir -p x86/lib
    cp -r -d ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/fastrtps/lib/lib* x86/lib
    cp -r -d ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/protobuf/lib/lib* x86/lib
    cp -r -d ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/yaml-cpp/lib/lib* x86/lib
    cp -r -d ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/glog/lib/lib* x86/lib
    cp -r -d ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/gflags/lib/lib* x86/lib
    cp -r -d ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/ceres/lib/lib* x86/lib
    cp -r -d ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/boost/lib/lib* x86/lib
    cp -r -d ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/opencv/lib/lib* x86/lib
    cp -r -d ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/absl/lib/lib* x86/lib
    cp -r -d ${THIRDPARTY_ROOT_PATH}/third_party/x86_2004/hzavpmap/lib/lib* x86/lib
    cp -r -d ${THIRDPARTY_ROOT_PATH}/third_party/x86/boost/lib/lib* x86/lib
")
file(WRITE "${CMAKE_INSTALL_PREFIX}/../prepare_for_cyber.sh" "${PREPARE_FOR_CYBER_SHELL}")
install(CODE "execute_process(COMMAND bash prepare_for_cyber.sh WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/..)")
