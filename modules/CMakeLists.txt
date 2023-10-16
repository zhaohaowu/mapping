project(mapping_modules)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/util/include
)

add_subdirectory(util)
add_subdirectory(dr)
add_subdirectory(local_mapping)
add_subdirectory(location)
# add_subdirectory(map_fusion)
