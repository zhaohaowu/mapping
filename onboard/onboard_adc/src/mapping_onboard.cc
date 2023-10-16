/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenlianchen
 *Date: 2023-08-31
 *****************************************************************************/
#include "onboard/onboard_adc/include/mapping_onboard.h"

DEFINE_string(config_yaml, "conf/mapping/local_mapping/local_mapping_conf.yaml",
              "path to local mapping conf yaml");

namespace hozon {
namespace mp {
int32_t MappingAdc::MappingAdc::AlgInit() {
  lmap_ = std::make_unique<lm::LMapApp>(FLAGS_config_yaml);
  dr_ = std::make_unique<dr::DRInterface>();
  // topo_ = std::make_unique<mf::TopoAssignment>();
  // if (topo_->Init() < 0) {
  //   return -1;
  // }
  // mpre_ = std::make_unique<mf::MapPrediction>();
  // if (mpre_->Init() < 0) {
  //   return -1;
  // }

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

void MappingAdc::AlgRelease() {}

}  // namespace mp
}  // namespace hozon
