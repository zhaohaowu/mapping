##
# Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
# Song Longjun <songlongjun@senseauto.com>
##

## Begin of System variable definition
set(CMAKE_SYSTEM_NAME       Linux)
set(TARGET_OS linux)
set(PLATFORM_NAME x86_64)
set(CMAKE_SYSTEM_PROCESSOR  x86_64)
## End of System variable definition

## Begin of Compilation variable definition
add_definitions(-DWITH_TDA)
## End of Compilation variable definition

## Begin of 3rd party path definition
set(PATH_SOURCE_3RDPARTY "/usr/local/senseauto_local/3rdparty/")
set(BOOST_INC_DIR "/usr/local/boost-1.58/${TARGET_OS}_${PLATFORM_NAME}/include")
## End of 3rd party path definition
