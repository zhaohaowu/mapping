
project(mapping_ut)

file(GLOB_RECURSE SRCS_TEST "*_test.cc" "*.h")

add_executable(${PROJECT_NAME} ${SRCS_TEST})

target_link_libraries(${PROJECT_NAME}
    gtest
    # gtest_main
    # glog
    gflags
)
# add_test(NAME ${PROJECT_NAME}
#     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#     COMMAND ./${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION test)

install(DIRECTORY script DESTINATION test)
