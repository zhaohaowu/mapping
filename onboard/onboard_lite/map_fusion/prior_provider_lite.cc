/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： prior_provider_lite.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.09
 ******************************************************************************/

#include <adf-lite/include/base.h>
#include <base/utils/log.h>
#include <common_onboard/adapter/onboard_lite/onboard_lite.h>
#include <gflags/gflags.h>
#include <proto/localization/node_info.pb.h>

#include <filesystem>

#include "modules/map_fusion/include/map_fusion/prior_provider/prior_provider.h"

DEFINE_string(conf_provider, "../../conf/mapping/map_fusion/map_fusion.yaml",
              "config file path for prior provider");

namespace hozon {
namespace perception {
namespace common_onboard {

class PriorProviderLite : public netaos::adf_lite::Executor {
 public:
  PriorProviderLite() = default;
  ~PriorProviderLite() = default;

  int32_t AlgInit() override {
    hozon::netaos::log::InitLogging("mf_pp", "prior_provider",
                                    hozon::netaos::log::LogLevel::kInfo,
                                    HZ_LOG2CONSOLE, "./", 10, (20));

    hozon::netaos::adf::NodeLogger::GetInstance().CreateLogger(
        "mf_pp", "prior_provider", hozon::netaos::log::LogLevel::kInfo);

    HLOG_WARN << "current dir: " << std::filesystem::current_path();
    REGISTER_MESSAGE_TYPE("loc_plugin", hozon::localization::HafNodeInfo);
    REGISTER_MESSAGE_TYPE("prior_map", hozon::hdmap::Map);

    RegistAlgProcessFunc("on_loc_plugin",
                         std::bind(&PriorProviderLite::OnLocPlugin, this,
                                   std::placeholders::_1));

    pp_ = std::make_shared<hozon::mp::mf::PriorProvider>();
    int ret = pp_->Init(FLAGS_conf_provider);
    if (ret < 0) {
      HLOG_ERROR << "init prior provider failed";
      return -1;
    }

    return 0;
  }

  void AlgRelease() override {}

  int32_t OnLocPlugin(Bundle* input) {
    BaseDataTypePtr loc_plugin_bundle = input->GetOne("loc_plugin");
    if (!loc_plugin_bundle) {
      HLOG_ERROR << "nullptr loc plugin";
      return -1;
    }

    if (!pp_) {
      HLOG_ERROR << "nullptr prior provider";
      return -1;
    }

    std::shared_ptr<hozon::localization::HafNodeInfo> loc_plugin =
        std::static_pointer_cast<hozon::localization::HafNodeInfo>(
            loc_plugin_bundle->proto_msg);
    auto map = pp_->GetPrior();

    BaseDataTypePtr workflow =
        std::make_shared<hozon::netaos::adf_lite::BaseData>();
    workflow->proto_msg = map;
    Bundle bundle;
    bundle.Add("prior_map", workflow);
    SendOutput(&bundle);

    return 0;
  }

 private:
  std::shared_ptr<hozon::mp::mf::PriorProvider> pp_ = nullptr;
};

REGISTER_EXECUTOR_CLASS(PriorProviderLite, PriorProviderLite);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
