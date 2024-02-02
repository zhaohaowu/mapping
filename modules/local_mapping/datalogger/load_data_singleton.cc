/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#include "modules/local_mapping/datalogger/load_data_singleton.h"

namespace hozon {
namespace mp {
namespace lm {

void LocalDataSingleton::Init() {
  dr_data_buffer_.set_capacity(1000);
  obj_data_buffer_.set_capacity(5);
}

bool LocalDataSingleton::GetObjByTimeStamp(double timestamp,
                                           ConstObjDataPtr* msg) {
  return obj_data_buffer_.get_message(timestamp, msg);
}

ConstDrDataPtr LocalDataSingleton::GetDrPoseByTimeStamp(double timestamp) {
  ConstDrDataPtr before = nullptr;
  ConstDrDataPtr after = nullptr;
  dr_data_buffer_.get_messages_around(timestamp, before, after);

  if (before == nullptr && after == nullptr) {
    HLOG_ERROR << "GetDrPoseForTime is null: " << timestamp;
    return nullptr;
  }

  if (before == nullptr) {
    DrDataPtr dr_ptr = std::make_shared<DrData>();

    double delta_t = timestamp - after->timestamp;
    dr_ptr->timestamp = timestamp;
    dr_ptr->pose =
        after->pose + after->quaternion * (after->local_vel * delta_t);
    dr_ptr->quaternion = after->quaternion;
    Eigen::Vector3d delta_ang = after->local_omg * delta_t;
    if (delta_ang.norm() > 1e-12) {
      dr_ptr->quaternion =
          dr_ptr->quaternion * Eigen::Quaterniond(Eigen::AngleAxisd(
                                   delta_ang.norm(), delta_ang.normalized()));
      dr_ptr->quaternion = dr_ptr->quaternion.normalized();
    }

    dr_ptr->local_vel = after->local_vel;
    dr_ptr->local_omg = after->local_omg;
    dr_ptr->local_acc = after->local_acc;
    dr_ptr->gear = after->gear;

    return dr_ptr;
  }

  if (before->timestamp == timestamp && after->timestamp == timestamp) {
    return before;
  }

  // before time == after time is not happend for this if
  if (after->timestamp <= before->timestamp) {
    HLOG_ERROR << "GetDrPoseForTime: after->timestamp <= before->timestamp: "
               << after->timestamp << " < " << before->timestamp;
    return nullptr;
  }

  double ratio =
      (timestamp - before->timestamp) / (after->timestamp - before->timestamp);
  auto dr_pose_state =
      std::make_shared<DrData>(before->Interpolate(ratio, *after, timestamp));
  return dr_pose_state;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
