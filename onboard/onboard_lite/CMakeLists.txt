project(mapping_onboard_lite)

include_directories(${THIRDPARTY_ROOT_PATH}/third_party/${TARGET_PLATFORM}/fast-dds/include)

if(${LOCAL_TARGET_PLATFORM} MATCHES x86)
  set(NOS_PLATFORM x86_2004)
else()
  set(NOS_PLATFORM ${LOCAL_TARGET_PLATFORM})
endif()

# add_subdirectory(map_fusion)
add_subdirectory(dr)
add_subdirectory(location)

install(
  DIRECTORY production
  DESTINATION ${TARGET_DESTINATION}
)

set(LITETEST ${MAPPING}/test)
install(DIRECTORY ${LITETEST} DESTINATION ${TARGET_DESTINATION})

set(NOSLIB ${DEPEND}/nos/${NOS_PLATFORM}/lib)
install(DIRECTORY ${NOSLIB} DESTINATION ${TARGET_DESTINATION})

set(NOSBIN ${DEPEND}/nos/${NOS_PLATFORM}/bin)
install(DIRECTORY ${NOSBIN} DESTINATION ${TARGET_DESTINATION})
