
project(mapping_onboard)

if(MIDDLEWARE MATCHES CYBER)
   add_subdirectory(onboard_cyber)
elseif(MIDDLEWARE MATCHES LITE)
  add_subdirectory(onboard_lite)
endif()

# 打包cyber环境
if(MIDDLEWARE MATCHES CYBER)
    install(
        DIRECTORY onboard_cyber/production/dag onboard_cyber/production/launch
        DESTINATION ${CMAKE_INSTALL_PREFIX}
    )
    install(
        # 注意结尾的斜杠/，这可以使目录重命名到目标路径
        DIRECTORY ${THIRDPARTY_ROOT_PATH}/third_party/x86/cyber/
        DESTINATION ${CMAKE_INSTALL_PREFIX}
        USE_SOURCE_PERMISSIONS
    )
    # 为了能运行cyber，以后看有没有更好的办法
    set(PREPARE_FOR_CYBER_SHELL "
        mkdir -p x86/data/log
        if [ ! -d cyber ]; then
          ln -s x86 cyber
        fi
        mkdir -p x86/lib
        cp -r -d ../depend/third_party/x86/fastrtps/lib/lib* x86/lib
        cp -r -d ../depend/third_party/x86/protobuf/lib/lib* x86/lib
        cp -r -d ../depend/third_party/x86/yaml-cpp/lib/lib* x86/lib
        cp -r -d ../depend/third_party/x86/glog/lib/lib* x86/lib
        cp -r -d ../depend/third_party/x86/gflags/lib/lib* x86/lib
        cp -r -d ../depend/third_party/x86/ceres/lib/lib* x86/lib
        cp -r -d ../depend/third_party/x86/boost/lib/lib* x86/lib
        cp -r -d ../depend/third_party/x86/opencv/lib/lib* x86/lib
    ")
    file(WRITE "${CMAKE_INSTALL_PREFIX}/../prepare_for_cyber.sh" "${PREPARE_FOR_CYBER_SHELL}")
    install(CODE "execute_process(COMMAND bash prepare_for_cyber.sh WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/..)")
endif()