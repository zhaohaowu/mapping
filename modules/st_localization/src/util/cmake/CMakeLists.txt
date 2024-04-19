cmake_minimum_required(VERSION 3.0.2)
project(cmake)

#############
## Install ##
#############

file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cmake)
list(REMOVE_ITEM SRCS "${CMAKE_CURRENT_SOURCE_DIR}/toolchain.cmake")
list(REMOVE_ITEM SRCS "${CMAKE_CURRENT_SOURCE_DIR}/ti-toolchain-file.cmake")
list(REMOVE_ITEM SRCS "${CMAKE_CURRENT_SOURCE_DIR}/ti-qnx-toolchain-file.cmake")
list(REMOVE_ITEM SRCS "${CMAKE_CURRENT_SOURCE_DIR}/ti-linux-toolchain-file.cmake")
list(REMOVE_ITEM SRCS "${CMAKE_CURRENT_SOURCE_DIR}/ti-x86-toolchain-file.cmake")
install(FILES ${SRCS} DESTINATION cmake)
