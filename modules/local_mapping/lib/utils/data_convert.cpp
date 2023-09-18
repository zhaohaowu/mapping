/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-05
 *****************************************************************************/
#include "modules/local_mapping/lib/utils/data_convert.h"

namespace hozon {
namespace mp {
namespace lm {

void DataConvert::SetLocation(const adsfi_proto::hz_Adsfi::AlgLocation msg,
                              std::shared_ptr<Location> location) {
  location->timestamp_ =
      msg.header().timestamp().sec() + msg.header().timestamp().nsec() / 1e9;
  location->position_.x() = msg.pose().pose_gcj02().position().x();
  location->position_.y() = msg.pose().pose_gcj02().position().y();
  location->position_.z() = msg.pose().pose_gcj02().position().z();
  location->quaternion_.x() = msg.pose().pose_gcj02().quaternion().x();
  location->quaternion_.y() = msg.pose().pose_gcj02().quaternion().y();
  location->quaternion_.z() = msg.pose().pose_gcj02().quaternion().z();
  location->quaternion_.w() = msg.pose().pose_gcj02().quaternion().w();
  location->euler_angle_.x() = msg.pose().pose_gcj02().euler_angle().x();
  location->euler_angle_.y() = msg.pose().pose_gcj02().euler_angle().y();
  location->euler_angle_.z() = msg.pose().pose_gcj02().euler_angle().z();
  location->linear_vrf_.x() = msg.velocity().twist_vrf().linear_vrf().x();
  location->linear_vrf_.y() = msg.velocity().twist_vrf().linear_vrf().y();
  location->linear_vrf_.z() = msg.velocity().twist_vrf().linear_vrf().z();
  location->angular_vrf_.x() = msg.velocity().twist_vrf().angular_vrf().x();
  location->angular_vrf_.y() = msg.velocity().twist_vrf().angular_vrf().y();
  location->angular_vrf_.z() = msg.velocity().twist_vrf().angular_vrf().z();
  location->heading_ = msg.pose().pose_gcj02().heading();
}

void DataConvert::SetDr(const adsfi_proto::hz_Adsfi::AlgLocation msg,
                        std::shared_ptr<Location> dr_location) {
  dr_location->timestamp_ =
      msg.header().timestamp().sec() + msg.header().timestamp().nsec() / 1e9;
  dr_location->position_.x() = msg.pose().pose_local().position().x();
  dr_location->position_.y() = msg.pose().pose_local().position().y();
  dr_location->position_.z() = msg.pose().pose_local().position().z();
  dr_location->quaternion_.x() = msg.pose().pose_local().quaternion().x();
  dr_location->quaternion_.y() = msg.pose().pose_local().quaternion().y();
  dr_location->quaternion_.z() = msg.pose().pose_local().quaternion().z();
  dr_location->quaternion_.w() = msg.pose().pose_local().quaternion().w();
  dr_location->euler_angle_.x() = msg.pose().pose_local().euler_angle().x();
  dr_location->euler_angle_.y() = msg.pose().pose_local().euler_angle().y();
  dr_location->euler_angle_.z() = msg.pose().pose_local().euler_angle().z();
  dr_location->linear_vrf_.x() = msg.velocity().twist_vrf().linear_vrf().x();
  dr_location->linear_vrf_.y() = msg.velocity().twist_vrf().linear_vrf().y();
  dr_location->linear_vrf_.z() = msg.velocity().twist_vrf().linear_vrf().z();
  dr_location->angular_vrf_.x() = msg.velocity().twist_vrf().angular_vrf().x();
  dr_location->angular_vrf_.y() = msg.velocity().twist_vrf().angular_vrf().y();
  dr_location->angular_vrf_.z() = msg.velocity().twist_vrf().angular_vrf().z();
  dr_location->heading_ = msg.pose().pose_local().heading();
}

void DataConvert::SetLaneLine(
    const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray& msg,
    std::shared_ptr<Lanes> lanes) {
  lanes->timestamp_ =
      msg.header().timestamp().sec() + msg.header().timestamp().nsec() / 1e9;
  for (size_t i = 0; i < msg.lane_detection_front_out().size(); i++) {
    if (msg.lane_detection_front_out()[i].lane_detection_out().size() == 0)
      return;
    int lane_id = msg.lane_detection_front_out()[i]
                      .lane_detection_out()[0]
                      .laneline_seq();
    if (lane_id > 100 || lane_id < -100) continue;
    std::shared_ptr<Lane> lane = std::make_shared<Lane>();
    lane->lane_id_ = msg.lane_detection_front_out()[i]
                         .lane_detection_out()[0]
                         .laneline_seq();
    lane->lane_fit_a_ = msg.lane_detection_front_out()[i]
                            .lane_detection_out()[0]
                            .lane_fit()
                            .coefficients()
                            .a();
    lane->lane_fit_b_ = msg.lane_detection_front_out()[i]
                            .lane_detection_out()[0]
                            .lane_fit()
                            .coefficients()
                            .b();
    lane->lane_fit_c_ = msg.lane_detection_front_out()[i]
                            .lane_detection_out()[0]
                            .lane_fit()
                            .coefficients()
                            .c();
    lane->lane_fit_d_ = msg.lane_detection_front_out()[i]
                            .lane_detection_out()[0]
                            .lane_fit()
                            .coefficients()
                            .d();
    lane->x_start_vrf_ = msg.lane_detection_front_out()[i]
                             .lane_detection_out()[0]
                             .lane_fit()
                             .x_start_vrf();
    lane->x_end_vrf_ = msg.lane_detection_front_out()[i]
                           .lane_detection_out()[0]
                           .lane_fit()
                           .x_end_vrf();
    lanes->front_lanes_.emplace_back(*lane);
  }
  for (size_t i = 0; i < msg.lane_detection_rear_out().size(); i++) {
    if (msg.lane_detection_rear_out()[i].lane_detection_out().size() == 0)
      return;
    int lane_id =
        msg.lane_detection_rear_out()[i].lane_detection_out()[0].laneline_seq();
    if (lane_id > 100 || lane_id < -100) continue;
    std::shared_ptr<Lane> lane = std::make_shared<Lane>();
    lane->lane_id_ =
        msg.lane_detection_rear_out()[i].lane_detection_out()[0].laneline_seq();
    lane->lane_fit_a_ = msg.lane_detection_rear_out()[i]
                            .lane_detection_out()[0]
                            .lane_fit()
                            .coefficients()
                            .a();
    lane->lane_fit_b_ = msg.lane_detection_rear_out()[i]
                            .lane_detection_out()[0]
                            .lane_fit()
                            .coefficients()
                            .b();
    lane->lane_fit_c_ = msg.lane_detection_rear_out()[i]
                            .lane_detection_out()[0]
                            .lane_fit()
                            .coefficients()
                            .c();
    lane->lane_fit_d_ = msg.lane_detection_rear_out()[i]
                            .lane_detection_out()[0]
                            .lane_fit()
                            .coefficients()
                            .d();
    lane->x_start_vrf_ = msg.lane_detection_rear_out()[i]
                             .lane_detection_out()[0]
                             .lane_fit()
                             .x_start_vrf();
    lane->x_end_vrf_ = msg.lane_detection_rear_out()[i]
                           .lane_detection_out()[0]
                           .lane_fit()
                           .x_end_vrf();
    lanes->rear_lanes_.emplace_back(*lane);
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
