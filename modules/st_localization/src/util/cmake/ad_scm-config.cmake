set(_target "ad_scm")

# Include
if(EXISTS ${ADFramework_PATH})
    # Call by ADFrameworkConfig.cmake in release mode
    list(APPEND ${_target}_INCLUDE_DIRS "${ADFramework_PATH}/include/${_target}")
else()
    # Develop mode, try find target
    string(REGEX REPLACE "(.*)/.*" "\\1" _topdir ${CMAKE_CURRENT_LIST_DIR})
    set(_include_dir "${_topdir}/modules/${_target}/include")
    if(EXISTS ${_include_dir})
        list(APPEND ${_target}_INCLUDE_DIRS ${_include_dir})
    endif()
endif()

# Library
list(APPEND ${_target}_LIBRARIES "${_target}")

# Debug
# message(STATUS "${_target}_INCLUDE_DIRS: ${${_target}_INCLUDE_DIRS}")
# message(STATUS "${_target}_LIBRARIES: ${${_target}_LIBRARIES}")
