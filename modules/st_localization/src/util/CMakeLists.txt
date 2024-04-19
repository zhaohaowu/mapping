cmake_minimum_required(VERSION 3.14.0)

message("start compile hozon sdk")
#using ccache
find_program(CCACHE "ccache")
if(CCACHE)
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE})
endif()

# IF(WITH_TDA)
#   set(ENV{CURRENT_CMAKE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#   set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ti-toolchain-file.cmake)
# ENDIF(WITH_TDA)

project(senseauto-util)

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(PATH_SOURCE_3RDPARTY "${ROOT_DIR}/modules/3rdparty/include")
set(PATH_AD_INCLUDE "${ROOT_DIR}/modules")
message(STATUS "ROOT_DIR: ${ROOT_DIR}/modules,  PATH_SOURCE_3RDPARTY: ${PATH_SOURCE_3RDPARTY}")
# list(APPEND CMAKE_PREFIX_PATH "${ROOT_DIR}/cmake")

# Set rpath for dpkg shlibdeps
# set(CMAKE_INSTALL_RPATH ".${CMAKE_INSTALL_PREFIX}/lib")

# IF(WITH_TDA)
#     set_tda_compile_option()
#     CREATE_ALL_SYMLINK()
# ENDIF()

# include(${CMAKE_CURRENT_LIST_DIR}/cmake/toolchain.cmake)

# add_subdirectory(cmake)

add_subdirectory(modules)

#############
##  CPack  ##
#############

# set(CPACK_GENERATOR "DEB")

# if(WITH_X86)
#     set(CPACK_PACKAGE_NAME ${TARGET_OS}-x86-64-${PROJECT_NAME})
# else()
#     set(CPACK_PACKAGE_NAME ${TARGET_OS}-${PLATFORM_NAME}-${PROJECT_NAME})
# ENDIF()
# set(CPACK_PACKAGE_VERSION ${MODULE_VERSION})
# set(CPACK_PACKAGE_CONTACT "caiyuchen@senseauto.com")

# set(CPACK_SET_DESTDIR ON)
# set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

# Set dependency Auto/Manual 
# set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)
# set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6")

#set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")

# for DEB pre-install process
# set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
#     "${ROOT_DIR}/resource/deb_extra/prerm;${ROOT_DIR}/resource/deb_extra/postinst")

# set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)

# include(CPack)

#############
##  CTest  ##
#############

# include(${CMAKE_CURRENT_LIST_DIR}/cmake/toolchain.cmake)
# enable_testing()
# if(NOT WITH_TDA)
#     add_subdirectory(unittests)
# endif()
