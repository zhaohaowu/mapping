/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-11-29
 *****************************************************************************/
#include "onboard/onboard_lite/phm_comment_lite/comment_lite.h"

#include <gflags/gflags.h>

#include "adf-lite/include/base.h"
#include "base/utils/log.h"
#include "cfg/config_param.h"
#include "lib/health_manager/health_manager.h"
#include "onboard/onboard_lite/phm_comment_lite/proto/running_mode.pb.h"
#include "yaml-cpp/yaml.h"

namespace hozon {
namespace perception {
namespace common_onboard {

int32_t PhmComponentOnboard::AlgInit() {
  // init glfag
  const char* var = std::getenv("ADFLITE_ROOT_PATH");
  if (var == nullptr) {
    std::cerr << "can not get ADFLITE_ROOT_PATH" << std::endl;
    return -1;
  }
  std::string work_root = std::string(var);
  std::string gflag_file =
      work_root + "/runtime_service/mapping/conf/lite/mapping_config.flags";
  gflags::ReadFromFlagsFile(gflag_file, "", false);
  // phm
  phm_component_ = std::make_unique<PhmComponent>();
  phm_component_->Init();

  RegistAlgProcessFunc("send_running_mode",
                       std::bind(&PhmComponentOnboard::send_running_mode, this,
                                 std::placeholders::_1));
  // phm
  // Receive driving and parking status
  // auto cfgMgr = hozon::netaos::cfg::ConfigParam::Instance();
  HLOG_ERROR << "driving and parking  in";
  hozon::netaos::cfg::ConfigParam::Instance()->MonitorParam<uint8_t>(
      "system/running_mode",
      [this](const std::string&, const std::string&, const uint8_t& runmode) {
        runmode_ = runmode;
        lib::HealthManager::Instance()->NotifySmInfo(
            static_cast<base::RunningMode>(runmode_));
        // HLOG_ERROR << "!!!!!!!!MonitorParam_runmode_ = " << runmode_;
      });  // 平台回调，每次行泊切换都会调用func

  HLOG_INFO << "AlgInit successfully ";

  return 0;
}

int32_t PhmComponentOnboard::send_running_mode(Bundle* input) {
  auto base_msg = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  if (base_msg) {
    auto runmodeptr = std::make_shared<running_mode>();
    if (runmodeptr) {
      runmodeptr->set_mode(runmode_);
    }
    base_msg->proto_msg = runmodeptr;
    hozon::netaos::adf_lite::Bundle bundle;
    bundle.Add("running_mode", base_msg);
    SendOutput(&bundle);
    // HLOG_ERROR << "!!!!!!!!SendOutput_runmode_ = " << runmode_;
  }
  return 0;
}

void PhmComponentOnboard::AlgRelease() {}
}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
