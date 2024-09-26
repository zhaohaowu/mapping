/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: HOZON
 *******************************************************/
#include "modules/local_mapping/data_mapping/data_mapping.h"

namespace hozon {
namespace mp {

namespace lm {
namespace data_mapping {

bool DataMapping::CvtPbLocation2Location(
    const NetaLoactionPtr& pb_location,
    std::shared_ptr<lm::Location> location_ptr) {
  location_ptr->timestamp = pb_location->header().data_stamp();
  location_ptr->position.x() = pb_location->pose_local().position().x();
  location_ptr->position.y() = pb_location->pose_local().position().y();
  location_ptr->position.z() = pb_location->pose_local().position().z();
  location_ptr->quaternion.w() = pb_location->pose_local().quaternion().w();
  location_ptr->quaternion.x() = pb_location->pose_local().quaternion().x();
  location_ptr->quaternion.y() = pb_location->pose_local().quaternion().y();
  location_ptr->quaternion.z() = pb_location->pose_local().quaternion().z();

  location_ptr->linear_vrf.x() =
      pb_location->pose_local().linear_velocity_vrf().x();
  location_ptr->linear_vrf.y() =
      pb_location->pose_local().linear_velocity_vrf().y();
  location_ptr->linear_vrf.z() =
      pb_location->pose_local().linear_velocity_vrf().z();
  location_ptr->angular_vrf.x() =
      pb_location->pose_local().angular_velocity_vrf().x();
  location_ptr->angular_vrf.y() =
      pb_location->pose_local().angular_velocity_vrf().y();
  location_ptr->angular_vrf.z() =
      pb_location->pose_local().angular_velocity_vrf().z();
  location_ptr->pose = Eigen::Translation3d(location_ptr->position) *
                       Eigen::Affine3d(location_ptr->quaternion);

  // HLOG_DEBUG << "linear_vrf:" << location_ptr->linear_vrf.x();
  // HLOG_DEBUG << "angular_vrf:" << location_ptr->angular_vrf.z();
  // Eigen::Matrix3d R = location_ptr->quaternion.toRotationMatrix();
  // Eigen::Translation3d translation =
  //     Eigen::Translation3d(pb_location->pose_local().position().x(),
  //                          pb_location->pose_local().position().y(), 0.0);

  // location_ptr->pose = translation * R;

  //   Eigen::Vector3d delta_ang = after->angular_velocity * delta_t;
  // if (delta_ang.norm() > 1e-12) {
  //   dr_ptr->quaternion =
  //       dr_ptr->quaternion * Eigen::Quaterniond(Eigen::AngleAxisd(
  //                                delta_ang.norm(), delta_ang.normalized()));
  //   dr_ptr->quaternion = dr_ptr->quaternion.normalized();
  // }
  // dr_ptr->pose = Eigen::Translation3d(dr_ptr->translation) *
  // Eigen::Affine3d(dr_ptr->quaternion);

  return true;
}

}  // namespace data_mapping
}  // namespace lm
}  // namespace mp
}  //  namespace hozon
