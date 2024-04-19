## LIB_SUFFIX
if(NOT DEFINED TARGET_OS)
    if(CMAKE_HOST_SYSTEM_NAME MATCHES Linux*)
        set(TARGET_OS linux)
    elseif(CMAKE_HOST_SYSTEM_NAME MATCHES Darwin*)
        set(TARGET_OS mac)
    elseif(CMAKE_HOST_SYSTEM_NAME MATCHES Windows*)
        set(TARGET_OS windows)
    endif()
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES i.86 OR BUILD_I386)
    set(PLATFORM_NAME x86)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES x86*)
    set(PLATFORM_NAME x86_64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES armv7*)
    set(PLATFORM_NAME armv7)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES aarch64)
    set(PLATFORM_NAME aarch64)
endif()

set(PATH_SOURCE_3RDPARTY /usr/local/senseauto_local/3rdparty)
function(find_3rdparty_package)
    cmake_parse_arguments(ARG
        "HAS_ROOT_INCLUDE"
        "HEADER;LIBRARIES;STATIC_DEPS;SHARED_DEPS;LIBRARIES_DIR"
        ""
        ${ARGN}
        )
    set(_target_includes)
    list(APPEND _target_includes ${PATH_SOURCE_3RDPARTY}/include)
    set(_target_libs_fullpath)
    set(_libs_root ${PATH_SOURCE_3RDPARTY}/lib/${TARGET_OS}_${PLATFORM_NAME})
    if(IS_DIRECTORY ${_libs_root})
        # Search for libraries
        file(GLOB_RECURSE _static_libs "${_libs_root}/*.a")
        foreach(_dep ${${ARG_STATIC_DEPS}})
            # Specifiy dep name
            string(REGEX REPLACE "(.*)[.]a$" "\\1" _temp ${_dep})
            string(REGEX REPLACE "^lib(.*)" "\\1" _dep_name ${_temp})
            set(_include_seperate ${PATH_SOURCE_3RDPARTY}/${_dep_name}/include)
            set(_include_merged ${PATH_SOURCE_3RDPARTY}/include/${_dep_name})
            if(IS_DIRECTORY ${_include_seperate})
                list(APPEND _target_includes ${_include_seperate})
            elseif(IS_DIRECTORY ${_include_merged})
                list(APPEND _target_includes ${_include_merged})
            endif()
            # Fetch 3rdparty static libraries fullpath
            foreach(_lib_fullpath ${_static_libs})
                string(REGEX MATCH "3rdparty/.*$" _lib_strip_path ${_lib_fullpath})
                if("${_lib_strip_path}" MATCHES "${_dep}")
                    list(APPEND _target_libs_fullpath "${_lib_fullpath}")
                endif()
            endforeach()
        endforeach()
        # Search for libraries
        file(GLOB _shared_libs "${_libs_root}/*.so")
        file(GLOB _shared_cuda_dependent_libs "${_libs_root}/${TARGET_OS}_${PLATFORM_NAME}-CUDA${CUDA_VERSION_MAJOR}.0/*.so")
        list(APPEND _shared_libs ${_shared_cuda_dependent_libs})
        foreach(_dep ${${ARG_SHARED_DEPS}})
            # Specifiy dep name
            string(REGEX REPLACE "(.*)[.]so.*$" "\\1" _temp ${_dep})
            string(REGEX REPLACE "^lib(.*)" "\\1" _dep_name ${_temp})
            set(_include_seperate ${PATH_SOURCE_3RDPARTY}/${_dep_name}/include)
            set(_include_merged ${PATH_SOURCE_3RDPARTY}/include/${_dep_name})
            if(IS_DIRECTORY ${_include_seperate})
                list(APPEND _target_includes ${_include_seperate})
            elseif(IS_DIRECTORY ${_include_merged})
                list(APPEND _target_includes ${_include_merged})
            endif()
            # Fetch 3rdparty static libraries fullpath
            foreach(_lib_fullpath ${_shared_libs})
                string(REGEX MATCH "3rdparty/.*$" _lib_strip_path ${_lib_fullpath})
                if("${_lib_strip_path}" MATCHES "${_dep}")
                    list(APPEND _target_libs_fullpath "${_lib_fullpath}")
                endif()
            endforeach()
        endforeach()
    else()
        message(FATAL_ERROR "${BoldRed}3rdparty libraries root: ${_libs_root} not found.${ColourReset}")
    endif()
    if(ARG_HAS_ROOT_INCLUDE)
        list(APPEND _target_includes ${PATH_SOURCE_3RDPARTY}/include)
    endif()
    if(_target_includes)
        list(REMOVE_DUPLICATES _target_includes)
    endif()
    if(_target_libs_fullpath)
        list(REMOVE_DUPLICATES _target_libs_fullpath)
    endif()
    if(ARG_HEADER)
        set(${ARG_HEADER} ${_target_includes} PARENT_SCOPE)
    endif()
    if(ARG_LIBRARIES)
        set(${ARG_LIBRARIES} ${_target_libs_fullpath} PARENT_SCOPE)
    endif()
    if(ARG_LIBRARIES_DIR)
        set(_target_lib_dirs)
        foreach(_target_lib_fullpath ${_target_libs_fullpath})
            get_filename_component(_target_lib_dir ${_target_lib_fullpath} DIRECTORY)
            list(APPEND _target_lib_dirs ${_target_lib_dir})
        endforeach()
        set(${ARG_LIBRARIES_DIR} ${_target_lib_dirs} PARENT_SCOPE)
    endif()
endfunction()

function(generate_idl)
    cmake_parse_arguments(ARG
        ""
        "IDL_DEPS;IDL_PATH;IDL_SRC;IDL_EXTERN_SRC;IDL_INCLUDE"
        ""
        ${ARGN}
        )
    set(FASTDDSGEN_SCRIPT ${PATH_SOURCE_3RDPARTY}/bin/fastddsgen/fastddsgen)
    set(IDL_EXTERN_SCRIPT ${ROOT_DIR}/modules/ad_dds_monitor/script/idl_extern_script.py)
    ## create temp path
    execute_process(
        COMMAND
            mkdir -p auto_generate/data_type_tmp
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_BINARY_DIR}
    )
    
    foreach(_dep ${${ARG_IDL_DEPS}})
        execute_process(
            COMMAND
                ${FASTDDSGEN_SCRIPT} -d ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/data_type_tmp/ ${${ARG_IDL_PATH}}/${_dep}.idl
            COMMAND
                ${IDL_EXTERN_SCRIPT} -i ${${ARG_IDL_PATH}}/${_dep}.idl -d ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/data_type_tmp
            WORKING_DIRECTORY
                ${CMAKE_CURRENT_BINARY_DIR}
        )
        execute_process(
            COMMAND
            ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/data_type_tmp/${_dep}.cxx ${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/cm_fastdds/data_type/${_dep}.cxx
            COMMAND
            ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/data_type_tmp/${_dep}.h ${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/cm_fastdds/data_type/${_dep}.h
            COMMAND
            ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/data_type_tmp/${_dep}PubSubTypes.cxx ${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/cm_fastdds/data_type/${_dep}PubSubTypes.cxx
            COMMAND
            ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/data_type_tmp/${_dep}PubSubTypes.h ${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/cm_fastdds/data_type/${_dep}PubSubTypes.h
            COMMAND
            ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/data_type_tmp/${_dep}Extern.cpp ${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/cm_fastdds/data_type/${_dep}Extern.cpp
            COMMAND
            ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/data_type_tmp/${_dep}Extern.h ${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/cm_fastdds/data_type/${_dep}Extern.h
        )
    endforeach()
    execute_process(
        COMMAND
            rm -rf auto_generate/data_type_tmp
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_BINARY_DIR}
    )
    set(idl_includes ${CMAKE_BINARY_DIR}/auto_generate/)
    file(GLOB idl_src_file "${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/cm_fastdds/data_type/*.cxx")
    file(GLOB idl_extern_src_file "${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/cm_fastdds/data_type/*.cpp")
    file(GLOB idl_extern_src_file_b "${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/cm_fastdds/data_type/Header*")
    if(ARG_IDL_INCLUDE)
        set(${ARG_IDL_INCLUDE} ${idl_includes} PARENT_SCOPE)
    endif()
    if(ARG_IDL_SRC)
        set(${ARG_IDL_SRC} ${idl_src_file} PARENT_SCOPE)
    endif()
    if(ARG_IDL_EXTERN_SRC)
        set(${ARG_IDL_EXTERN_SRC} ${idl_extern_src_file} ${idl_extern_src_file_b} PARENT_SCOPE)
    endif()
endfunction()

function(generate_proto)
    cmake_parse_arguments(ARG
        ""
        "PROTO_NAME;PROTO_PATH;PROTO_DEP;PROTO_SRC;PROTO_INCLUDE"
        ""
        ${ARGN}
        )
    ## handle protobuf
    set(PROTOC_ENV LD_LIBRARY_PATH=${PATH_SOURCE_3RDPARTY}/bin/protoc/linux_x86_64/)
    set(PROTOC_BIN ${PATH_SOURCE_3RDPARTY}/bin/protoc/linux_x86_64/protoc)
    # to prevent recompile after cmake
    execute_process(
        COMMAND
            mkdir -p auto_generate/proto_tmp
        COMMAND
            mkdir -p auto_generate/proto
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_BINARY_DIR}
    )
    execute_process(
        COMMAND
            ${CMAKE_COMMAND} -E env ${PROTOC_ENV}
            ${PROTOC_BIN} -I=${${ARG_PROTO_PATH}}  --cpp_out=${CMAKE_CURRENT_BINARY_DIR}/auto_generate/proto_tmp/ ${${ARG_PROTO_PATH}}/${ARG_PROTO_NAME}.proto ${${ARG_PROTO_DEP}}
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_BINARY_DIR}
    )

    execute_process(
        COMMAND
            ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/proto_tmp/${ARG_PROTO_NAME}.pb.cc ${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/proto/${ARG_PROTO_NAME}.pb.cc
        COMMAND
            ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/proto_tmp/${ARG_PROTO_NAME}.pb.h ${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/proto/${ARG_PROTO_NAME}.pb.h
    )
    execute_process(
        COMMAND
            rm -rf auto_generate/proto_tmp
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_BINARY_DIR}
    )
    set(proto_includes ${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/
                       ${CMAKE_BINARY_DIR}/auto_generate/perception_common/
                       ### For Module Proto
                       ${CMAKE_BINARY_DIR}/auto_generate/perception_common/proto/
    )
    file(GLOB proto_src_file "${CMAKE_BINARY_DIR}/auto_generate/${PROJECT_NAME}/proto/*.pb.cc")
    file(GLOB common_proto_src "${CMAKE_BINARY_DIR}/auto_generate/perception_common/proto/*.pb.cc")
    if(ARG_PROTO_INCLUDE)
        set(${ARG_PROTO_INCLUDE} ${proto_includes} PARENT_SCOPE)
    endif()
    if(ARG_PROTO_SRC)
        set(${ARG_PROTO_SRC} ${proto_src_file} ${common_proto_src} PARENT_SCOPE)
    endif()
endfunction()
