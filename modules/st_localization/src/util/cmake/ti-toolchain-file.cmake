##
# Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
# Song Longjun <songlongjun@senseauto.com>
##

# default we will use linux configuration
IF(WITH_QNX)
    include($ENV{CURRENT_CMAKE_DIR}/ti-qnx-toolchain-file.cmake)
ELSEIF(WITH_X86)
    include($ENV{CURRENT_CMAKE_DIR}/ti-x86-toolchain-file.cmake)
ELSE()
    include($ENV{CURRENT_CMAKE_DIR}/ti-linux-toolchain-file.cmake)
ENDIF()

add_compile_options(-Wno-unused-local-typedefs)
add_compile_options(-Wno-class-memaccess)
add_compile_options(-Wno-pessimizing-move)
add_compile_options(-Wno-unused-but-set-variable)
add_compile_options(-Wno-return-local-addr)
add_compile_options(-Wno-placement-new)


MACRO(set_tda_compile_option)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  set(CMAKE_BUILD_TYPE "Release")
  
  IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
  ELSEIF(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -s")
  ENDIF()

  IF(WITH_QNX)
    link_libraries(${library_name} regex)
  ELSE()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    link_libraries(${library_name} rt)
  ENDIF()
ENDMACRO()

MACRO(remove_linked_libraries libs library)
  IF (WITH_QNX)
    list(REMOVE_ITEM ${libs} ${library})
  ENDIF(WITH_QNX)
ENDMACRO()

macro(SUBDIRLIST curdir result)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
endmacro()

macro(CREATE_SYMLINK curdir osname arch script)
  SUBDIRLIST(${curdir}/${osname}_${arch} subdirs)
  FOREACH(subdir ${subdirs})
    IF(EXISTS ${curdir}/${subdir})
      file(APPEND ${script} "sudo rm -rf ${curdir}/${subdir}\n")
    ENDIF()
    file(APPEND ${script} "sudo ln -s ${curdir}/${osname}_${arch}/${subdir} ${curdir}/${subdir}\n")
  ENDFOREACH()

    # compatible with current toolchain.cmake
    IF (${curdir} STREQUAL ${PATH_SOURCE_3RDPARTY})
    IF (NOT EXISTS ${curdir}/${osname}_${arch}/lib/${osname}_${arch})
      file(APPEND ${script} "sudo ln -s ${curdir}/${osname}_${arch}/lib/ ${curdir}/${osname}_${arch}/lib/${osname}_${arch}\n")
    ENDIF()
  ENDIF()

endmacro()

set(OPENCV_PATH /usr/local/opencv-3.4.12)
set(ADFRAMEWORK_PATH /usr/local/adframework)
set(LOTUFRAMEWORK_PATH /usr/local/lotuframework)
set(PCL_PATH /usr/local/pcl-1.9)

macro(CREATE_ALL_SYMLINK)
  set(TMP_SCRIPT /tmp/symlink.sh)
  file(WRITE ${TMP_SCRIPT} "#! /bin/bash\n")
  CREATE_SYMLINK(${PATH_SOURCE_3RDPARTY} ${TARGET_OS} ${PLATFORM_NAME} ${TMP_SCRIPT})
  CREATE_SYMLINK(${OPENCV_PATH} ${TARGET_OS} ${PLATFORM_NAME} ${TMP_SCRIPT})
  CREATE_SYMLINK(${ADFRAMEWORK_PATH} ${TARGET_OS} ${PLATFORM_NAME} ${TMP_SCRIPT})
  CREATE_SYMLINK(${LOTUFRAMEWORK_PATH} ${TARGET_OS} ${PLATFORM_NAME} ${TMP_SCRIPT})
  CREATE_SYMLINK(${PCL_PATH} ${TARGET_OS} ${PLATFORM_NAME} ${TMP_SCRIPT})
  execute_process(COMMAND chmod +x ${TMP_SCRIPT})
  execute_process(COMMAND /bin/bash ${TMP_SCRIPT})
endmacro()