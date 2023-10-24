/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenlianchen
 *Date: 2023-08-31
 *****************************************************************************/

#include "onboard/onboard_adc/include/mapping_onboard.h"
#include "ap-release/include/adsfi/include/data_types/debug/pbdebug.h"

DEFINE_string(lm_config, "conf/mapping/local_mapping/local_mapping_conf.yaml",
              "path to local mapping conf yaml");
DEFINE_bool(adc_viz, false, "if use rviz");
DEFINE_string(adc_viz_addr, "tcp://10.6.73.235:9100",
              "RvizAgent's working address");

namespace hozon {
namespace mp {
int32_t MappingAdc::MappingAdc::AlgInit() {
  if (FLAGS_adc_viz) {
    HLOG_INFO << "Start RvizAgent on " << FLAGS_adc_viz_addr;
    int ret = util::RvizAgent::Instance().Init(FLAGS_adc_viz_addr);
    if (ret < 0) {
      HLOG_ERROR << "RvizAgent start failed";
    }
  }
  lmap_ = std::make_unique<lm::LMapApp>(FLAGS_lm_config);
  dr_ = std::make_unique<dr::DRInterface>();
  // topo_ = std::make_unique<mf::TopoAssignment>();
  // if (topo_->Init() < 0) {
  //   return -1;
  // }
  // mpre_ = std::make_unique<mf::MapPrediction>();
  // if (mpre_->Init() < 0) {
  //   return -1;
  // }

  loc_ = std::make_unique<loc::Localization>();
  if (!loc_->Init()) {
    return -1;
  }

  dr_data_ptr_ = std::make_shared<hozon::dead_reckoning::DeadReckoning>();

  return 0;
}
// int32_t MappingAdc::Process(hz_Adsfi::NodeBundle* input) {
//   if (!input) {
//     return -1;
//   }
//   RetrieveLatestData(input);  // 待实现
//   // dr
//   // local_mapping
//   // mapfusion
// }
int32_t MappingAdc::ChassisImuCallBack(hz_Adsfi::NodeBundle* input) {
  std::cout << "----imu callback" << std::endl;
  std::shared_ptr<hz_Adsfi::AlgImuIns> imuinsDataPtr_ =
      std::static_pointer_cast<hz_Adsfi::AlgImuIns>(input->GetOne("imu"));
  if (imuinsDataPtr_ == nullptr) {
    return -1;
  }

  board_.imu_proto.reset();
  board_.imu_proto = std::make_shared<hozon::soc::ImuIns>();
  board_.Adsfi2Proto(imuinsDataPtr_, board_.imu_proto);
  dr_->AddImuData(board_.imu_proto);

  loc_->OnImu(*(board_.imu_proto));
  loc_->OnOriginIns(*(board_.imu_proto));

  // localization output by imu frequency
  auto loc_res = std::make_shared<hozon::localization::Localization>();
  if (loc_->GetCurrentLocalization(loc_res.get())) {
    // get localization here
    std::string seri_loc("");
    if (loc_res->SerializeToString(&seri_loc)) {
      auto debug_loc = std::make_shared<hz_Adsfi::AlgPbDebugFrame>();
      debug_loc->algDebugframe.header.seq = loc_res->header().seq();
      debug_loc->algDebugframe.header.frameId = loc_res->header().frame_id();
      const uint32_t sec =
          static_cast<uint32_t>(loc_res->header().publish_stamp());
      const uint32_t nsec = (loc_res->header().publish_stamp() - sec) * 1e9;
      debug_loc->algDebugframe.header.stamp.sec = sec;
      debug_loc->algDebugframe.header.stamp.nsec = nsec;
      debug_loc->algDebugframe.msg_1 = seri_loc;
      hz_Adsfi::NodeBundle output;
      output.Add("npp_debug_msg_31", debug_loc);
      SendOutput(&output);
    }
  }

  std::shared_ptr<hz_Adsfi::AlgChassisInfo> chassisDataPtr_ =
      std::static_pointer_cast<hz_Adsfi::AlgChassisInfo>(
          input->GetOne("chassis"));
  if (chassisDataPtr_ == nullptr) {
    return -1;
  } else {
    board_.chassis_proto.reset();
    board_.chassis_proto = std::make_shared<hozon::soc::Chassis>();
    board_.Adsfi2Proto(chassisDataPtr_, board_.chassis_proto);
    dr_->AddChassisData(board_.chassis_proto);
  }

  if (dr_->SetLocation(dr_data_ptr_)) {
    lmap_->OnDr(dr_data_ptr_);
    loc_->OnDr(*dr_data_ptr_);
  }
  return 0;
}

int32_t MappingAdc::LaneCallBack(hz_Adsfi::NodeBundle* input) {
  // road_marking
  const auto p_road_marking =
      std::static_pointer_cast<hz_Adsfi::AlgLaneDetectionOutArray>(
          input->GetOne("nnp_cam_lane"));
  if (p_road_marking) {
    std::cout << "-----lane callback" << std::endl;
    board_.adsfi_lane_proto.reset();
    board_.adsfi_lane_proto =
        std::make_shared<hozon::perception::TransportElement>();
    DataBoard::Adsfi2Proto(*p_road_marking, board_.adsfi_lane_proto.get());
    lmap_->OnLaneLine(board_.adsfi_lane_proto);
  } else {
    std::cout << "null lane data" << std::endl;
  }

  return 0;
}

int32_t MappingAdc::PluginCallback(hz_Adsfi::NodeBundle* input) {
  if (!input) {
    return -1;
  }

  const auto p_plugin = std::static_pointer_cast<hz_Adsfi::AlgLocationNodeInfo>(
      input->GetOne("plugin"));
  if (p_plugin) {
    DataBoard::Adsfi2Proto(*p_plugin, board_.plugin_proto.get());
    loc_->OnInspva(*(board_.plugin_proto));
  }

  return 0;
}

void MappingAdc::AlgRelease() { util::RvizAgent::Instance().Term(); }

}  // namespace mp
}  // namespace hozon
