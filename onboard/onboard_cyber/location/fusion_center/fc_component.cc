/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fc_component.cc
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#include "onboard/onboard_cyber/location/fusion_center/fc_component.h"
#include <gflags/gflags.h>

DEFINE_string(fc_imu_in_topic, "/mapping/location/imu", "imu output topic");
DEFINE_string(fc_ins_in_topic, "/mapping/location/ins_fusion",
              "insfusion output topic");
DEFINE_string(fc_dr_in_topic, "/mapping/location/dr", "dr output topic");
DEFINE_string(fc_pe_in_topic, "/mapping/location/pose_estimate",
              "pose estimate output topic");
DEFINE_string(fc_out_topic, "/mapping/location/fc", "fc output topic");

DEFINE_string(fc_config, "conf/mapping/location/fusion_center/fc_config.yaml",
              "fusion center core config");
DEFINE_string(kf_config, "conf/mapping/location/fusion_center/kalman.yaml",
              "kalman config file");
DEFINE_string(eskf_config, "conf/mapping/location/fusion_center/eskf.yaml",
              "eskf config file");

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

bool FcComponent::Init() {
  fc_ = std::make_shared<FusionCenter>();
  if (!fc_->Init(FLAGS_fc_config, FLAGS_kf_config, FLAGS_eskf_config)) {
    return false;
  }

  imu_reader_ = node_->CreateReader<ImuIns>(
      FLAGS_fc_imu_in_topic,
      [this](const std::shared_ptr<const ImuIns>& msg) {
        OnImu(msg);
      });

  ins_reader_ = node_->CreateReader<HafNodeInfo>(
      FLAGS_fc_ins_in_topic,
      [this](const std::shared_ptr<const HafNodeInfo>& msg) {
        OnInsFusion(msg);
      });

  dr_reader_ = node_->CreateReader<HafNodeInfo>(
      FLAGS_fc_dr_in_topic,
      [this](const std::shared_ptr<const HafNodeInfo>& msg) {
        OnDrFusion(msg);
      });

  pe_reader_ = node_->CreateReader<HafNodeInfo>(
      FLAGS_fc_pe_in_topic,
      [this](const std::shared_ptr<const HafNodeInfo>& msg) {
        OnPoseEstimation(msg);
      });

  fc_writer_ = node_->CreateWriter<Localization>(FLAGS_fc_out_topic);

  return true;
}

void FcComponent::OnImu(const std::shared_ptr<const ImuIns>& msg) {
  if (!msg) {
    return;
  }
  fc_->OnImu(*msg);
}

void FcComponent::OnInsFusion(const std::shared_ptr<const HafNodeInfo>& msg) {
  if (!msg) {
    return;
  }
  fc_->OnIns(*msg);

  auto location = std::make_shared<Localization>();
  if (fc_->GetCurrentOutput(location.get())) {
    fc_writer_->Write(location);
  }
}

void FcComponent::OnDrFusion(const std::shared_ptr<const HafNodeInfo>& msg) {
  if (!msg) {
    return;
  }
  fc_->OnDR(*msg);
}

void FcComponent::OnPoseEstimation(
    const std::shared_ptr<const HafNodeInfo>& msg) {
  if (!msg) {
    return;
  }
  fc_->OnPoseEstimate(*msg);
}

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
