project(mapping_onboard_lite)

#file(GLOB_RECURSE PROTO_FILES "*.proto")
#PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HEADERS ${PROTO_FILES})

# add_subdirectory(map_fusion)
# add_subdirectory(dr)
add_subdirectory(location)

if(IND)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/production DESTINATION  ${MAPPING}/release/${TARGET_PLATFORM}/)
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/production DESTINATION ${MAPPING}/release/${TARGET_PLATFORM}/)

    if(${TARGET_PLATFORM} MATCHES x86)
    set(CURRENT_PLATFORM x86_2004)
    else()
        set(CURRENT_PLATFORM ${TARGET_PLATFORM})
    endif()

    set(NOSLIB ${DEPEND}/nos/${CURRENT_PLATFORM}/lib)
    file(COPY ${NOSLIB} DESTINATION  ${MAPPING}/release/${TARGET_PLATFORM}/)
    install(DIRECTORY ${NOSLIB} DESTINATION ${MAPPING}/release/${TARGET_PLATFORM}/)

    set(NOSBIN ${DEPEND}/nos/${CURRENT_PLATFORM}/bin)
    file(COPY ${NOSBIN} DESTINATION  ${MAPPING}/release/${TARGET_PLATFORM}/)
    install(DIRECTORY ${NOSBIN} DESTINATION ${MAPPING}/release/${TARGET_PLATFORM}/)

    set(LITETEST ${MAPPING}/test)
    file(COPY ${LITETEST} DESTINATION  ${MAPPING}/release/${TARGET_PLATFORM}/)
    install(DIRECTORY ${LITETEST} DESTINATION ${MAPPING}/release/${TARGET_PLATFORM}/)
endif()