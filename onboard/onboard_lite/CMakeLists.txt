project(mapping_onboard_lite)

include_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/fast-dds/include)
include_directories(${MAPPING}/modules/map_fusion/include)

if(${LOCAL_TARGET_PLATFORM} MATCHES x86)
  set(NOS_PLATFORM x86_2004)
else()
  set(NOS_PLATFORM ${LOCAL_TARGET_PLATFORM})
endif()

add_subdirectory(flags)
add_subdirectory(phm_comment_lite)
add_subdirectory(map_fusion)
add_subdirectory(dr)
add_subdirectory(location)
add_subdirectory(laneline_postprocess) # add by wenhai.zhang
add_subdirectory(local_mapping)
add_subdirectory(viz_converter)

install(
  DIRECTORY production/conf/
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/conf/lite
  PATTERN "production/conf/phm_config/phm_params" EXCLUDE
  PATTERN "production/conf/config_manager.config" EXCLUDE
  PATTERN "production/conf/phm_params_config.prototxt" EXCLUDE
)
install(
  FILES production/conf/config_manager.config
  DESTINATION ${CMAKE_INSTALL_PREFIX}/conf/mapping
)
install(
  DIRECTORY production/conf/phm_config
  DESTINATION ${CMAKE_INSTALL_PREFIX}/conf/mapping
  PATTERN "production/conf/phm_config/phm_common_onboard_config.yaml" EXCLUDE
)

install(
  FILES production/conf/config_manager.config
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/conf/mapping
)
install(
  DIRECTORY production/conf/phm_config
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/conf/mapping
  PATTERN "production/conf/phm_config/phm_common_onboard_config.yaml" EXCLUDE
)

if(${LOCAL_TARGET_PLATFORM} MATCHES orin)
  install(DIRECTORY ${DEPEND}/nos/nos_orin/runtime_service/mapping/etc DESTINATION ${CMAKE_INSTALL_PREFIX_RS})
endif()

install(DIRECTORY production/scripts DESTINATION ${CMAKE_INSTALL_PREFIX})

if(IND)
  set(NOSLIB ${DEPEND}/nos/nos_${NOS_PLATFORM}/lib)
  install(DIRECTORY ${NOSLIB}/ DESTINATION ${EXTERNAL_LIB_PATH}
  FILES_MATCHING
  PATTERN "*.so*"
  )
endif()

set(NOSBIN ${DEPEND}/nos/${NOS_PLATFORM}/bin/adf-lite-process)
install(
  FILES ${NOSBIN}
  DESTINATION ${CMAKE_INSTALL_PREFIX_RS}/bin
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  RENAME mapping
)
