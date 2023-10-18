# MAPPING_LIB_PREFIX is defined in compile.py
macro(add_mapping_library mlib)
    add_library(${ARGV})
    set_target_properties(${mlib} PROPERTIES OUTPUT_NAME ${MAPPING_LIB_PREFIX}${mlib})
endmacro()
macro(add_mapping_executable mexe)
    add_executable(${ARGV})
    set_target_properties(${mexe} PROPERTIES OUTPUT_NAME ${MAPPING_LIB_PREFIX}${mexe})
endmacro()