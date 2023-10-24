/******************************************************************************
 * Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 ******************************************************************************/

#include <stdio.h>
#include <termios.h>

#include <csignal>
#include <iostream>
#include <string>

#include "onboard/onboard_adc/include/mapping_onboard.h"

int32_t main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  hozon::mp::MappingAdc mapping;

  mapping.RegistAlgProcessFunc(
      "ChassisImuCB", std::bind(&hozon::mp::MappingAdc::ChassisImuCallBack,
                                &mapping, std::placeholders::_1));
  mapping.RegistAlgProcessFunc(
      "LaneCB", std::bind(&hozon::mp::MappingAdc::LaneCallBack, &mapping,
                          std::placeholders::_1));
  mapping.RegistAlgProcessFunc(
      "plugin_cb", std::bind(&hozon::mp::MappingAdc::PluginCallback, &mapping,
                             std::placeholders::_1));
  // mapping.RegistAlgProcessFunc(
  //     "MapFusion", std::bind(&hozon::mp::MappingAdc::Process,
  //                       &mapping, std::placeholders::_1));

  std::string work_root = "/opt/app/1";
  std::string socadf_yaml =
      "/runtime_service/mapping/conf/"
      "hz_mapping_config.yaml";
  const char* env_p = std::getenv("DEBUG_MAPPING_WORK_ROOT");
  if (env_p != nullptr) {
    work_root = std::string(env_p);
    std::cout << "work_root: " << work_root << std::endl;
  }
  std::string abs_yaml = work_root + socadf_yaml;
  std::cout << "yamal path: " << abs_yaml << std::endl;
  mapping.Start(abs_yaml);

  while (!mapping.NeedStop()) {
    sleep(1);
  }
  mapping.Stop();
  return 0;
}
