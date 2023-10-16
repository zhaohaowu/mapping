project(mapping_onboard_cyber)

execute_process(COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX})
execute_process(
  COMMAND ln -s x86 cyber
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/../
)

add_subdirectory(dr)
add_subdirectory(location)

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
