project(ins_fusion)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/cyber
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

add_subdirectory(lib)

if(MIDDLEWARE MATCHES CYBER)
  add_subdirectory(cyber)
endif()