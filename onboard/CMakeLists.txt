
project(mapping_onboard)

if(MIDDLEWARE MATCHES CYBER)
   add_subdirectory(onboard_cyber)
elseif(MIDDLEWARE MATCHES LITE)
  add_subdirectory(onboard_lite)
endif()

# 打包third party环境

if(IND)
  set(PREPARE_FOR_CYBER_SHELL "
      mkdir -p ${TARGET_PLATFORM}/data/log
      if [ ! -d ${TARGET_PLATFORM} ]; then
        ln -s ${TARGET_PLATFORM}
      fi
      mkdir -p ${MAPPING}/release/${TARGET_PLATFORM}/lib
      cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/fast-dds/lib/lib* ${MAPPING}/release/${TARGET_PLATFORM}/lib
      cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/yaml-cpp/lib/lib* ${MAPPING}/release/${TARGET_PLATFORM}/lib
      cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/glog/lib/lib* ${MAPPING}/release/${TARGET_PLATFORM}/lib
      cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/gflags/lib/lib* ${MAPPING}/release/${TARGET_PLATFORM}/lib
      cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/protobuf/lib/lib* ${MAPPING}/release/${TARGET_PLATFORM}/lib
      cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/ceres/lib/lib* ${MAPPING}/release/${TARGET_PLATFORM}/lib
      cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/boost/lib/lib* ${MAPPING}/release/${TARGET_PLATFORM}/lib
      cp -r -d ${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/opencv/lib/lib* ${MAPPING}/release/${TARGET_PLATFORM}/lib
      cp -r -d ${DEPEND}/perception-common-onboard/release/${TARGET_PLATFORM}/lib/lib* ${MAPPING}/release/${TARGET_PLATFORM}/lib
      cp -r -d ${DEPEND}/perception-lib/release/${TARGET_PLATFORM}/lib/lib* ${MAPPING}/release/${TARGET_PLATFORM}/lib
  ")
  file(WRITE "${CMAKE_INSTALL_PREFIX}/../PREPARE_FOR_CYBER_SHELL.sh" "${PREPARE_FOR_CYBER_SHELL}")
  install(CODE "execute_process(COMMAND bash PREPARE_FOR_CYBER_SHELL.sh WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/..)")
endif()

