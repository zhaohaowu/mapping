project(mapping_onboard_lite)

# add_subdirectory(map_fusion)
add_subdirectory(dr)
add_subdirectory(location)

install(
  DIRECTORY production
  DESTINATION ${TARGET_DESTINATION}
)

set(LITETEST ${MAPPING}/test)
install(DIRECTORY ${LITETEST} DESTINATION ${TARGET_DESTINATION})

if(${TARGET_PLATFORM} MATCHES x86)
  set(NOS_PLATFORM x86_2004)
else()
  set(NOS_PLATFORM ${TARGET_PLATFORM})
endif()

set(NOSLIB ${DEPEND}/nos/${NOS_PLATFORM}/lib)
install(DIRECTORY ${NOSLIB} DESTINATION ${TARGET_DESTINATION})

set(NOSBIN ${DEPEND}/nos/${NOS_PLATFORM}/bin)
install(DIRECTORY ${NOSBIN} DESTINATION ${TARGET_DESTINATION})
