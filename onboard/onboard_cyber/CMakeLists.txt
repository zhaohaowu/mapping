project(mapping_onboard_cyber)

execute_process(COMMAND mkdir -p ${CMAKE_INSTALL_PREFIX_RS}/data/log)
execute_process(
  COMMAND ln -s mapping cyber
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX_RS}/../
)

add_subdirectory(dr)
add_subdirectory(location)
add_subdirectory(local_mapping)
add_subdirectory(map_fusion)

install(DIRECTORY production/scripts DESTINATION ${CMAKE_INSTALL_PREFIX})

install(
  DIRECTORY production/dag production/launch
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}
)

install(
  # 注意结尾的斜杠/，这可以使目录重命名到目标路径
  DIRECTORY ${THIRDPARTY_ROOT_PATH}/third_party/x86/cyber/
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}
  USE_SOURCE_PERMISSIONS
)
