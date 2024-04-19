# Find AD framework root
if(${CMAKE_CURRENT_LIST_DIR} MATCHES "/usr/local/(.*)")
    string(REGEX REPLACE "(.*)/.*" "\\1" ADFramework_PATH ${CMAKE_CURRENT_LIST_DIR})
    message(STATUS "ADFramework: ${ADFramework_PATH}")
    list(APPEND ADFramework_INCLUDE_DIRS ${ADFramework_PATH}/include)
    list(APPEND ADFramework_LIBRARY_DIRS ${ADFramework_PATH}/lib)
else()
    message(STATUS "ADFramework: develop mode")
endif()

# Search AD component
foreach(component ${ADFramework_FIND_COMPONENTS})
    message("\tUse component: ${component}")
    include(${CMAKE_CURRENT_LIST_DIR}/${component}-config.cmake)
    list(APPEND ADFramework_INCLUDE_DIRS ${${component}_INCLUDE_DIRS})
    list(APPEND ADFramework_LIBRARIES ${${component}_LIBRARIES})
endforeach()


if (WITH_QNX)
    list(REMOVE_ITEM ADFramework_LIBRARIES pthread)
endif()

# Debug
#message(STATUS "ADFramework_INCLUDE_DIRS: ${ADFramework_INCLUDE_DIRS}")
#message(STATUS "ADFramework_LIBRARIES: ${ADFramework_LIBRARIES}")
