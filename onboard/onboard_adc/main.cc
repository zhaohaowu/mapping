/******************************************************************************
 * Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 ******************************************************************************/

#include <csignal>
#include <iostream>
#include <string>

#include "onboard/onboard_adc/include/mapping_onboard.h"

int32_t main(int argc, char** argv) {
  try {
    google::ParseCommandLineFlags(&argc, &argv, true);
    hozon::mp::MappingAdc mapping;

    mapping.RegistAlgProcessFunc("ChassisImuCB", [ObjectPtr =
                                                      &mapping](auto&& PH1) {
      return ObjectPtr->ChassisImuCallBack(std::forward<decltype(PH1)>(PH1));
    });
    mapping.RegistAlgProcessFunc("LaneCB", [ObjectPtr = &mapping](auto&& PH1) {
      return ObjectPtr->LaneCallBack(std::forward<decltype(PH1)>(PH1));
    });
    mapping.RegistAlgProcessFunc(
        "plugin_cb", [ObjectPtr = &mapping](auto&& PH1) {
          return ObjectPtr->PluginCallback(std::forward<decltype(PH1)>(PH1));
        });
    mapping.RegistAlgProcessFunc("map_service_cycle_cb",
                                 [ObjectPtr = &mapping](auto&& PH1) {
                                   return ObjectPtr->MapServiceCycleCallback(
                                       std::forward<decltype(PH1)>(PH1));
                                 });
    mapping.RegistAlgProcessFunc("map_fusion_cycle_cb",
                                 [ObjectPtr = &mapping](auto&& PH1) {
                                   return ObjectPtr->MapFusionCycleCallback(
                                       std::forward<decltype(PH1)>(PH1));
                                 });

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
  } catch (std::exception const& e) {
    std::cerr << "throw " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "throw (...)" << std::endl;
  }
  return 0;
}
