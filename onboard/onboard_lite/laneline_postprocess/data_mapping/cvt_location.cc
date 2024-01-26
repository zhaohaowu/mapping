/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: HOZON
 *******************************************************/
#include "onboard/onboard_lite/laneline_postprocess/data_mapping/data_mapping.h"

namespace hozon {
namespace mp {
namespace common_onboard {

bool DataMapping::CvtPbDR2Location(
    const NetaDeadReckoningPtr &pb_dr,
    std::shared_ptr<base::Location> location_ptr) {
  location_ptr->timestamp = pb_dr->header().data_stamp();
  location_ptr->pose = Eigen::Affine3d::Identity();

  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(
          pb_dr->pose().pose_local().euler_angle().z() / 180.0 * M_PI,
          Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.0 / 180.0 * M_PI, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0 / 180.0 * M_PI, Eigen::Vector3d::UnitX());
  Eigen::Translation3d translation =
      Eigen::Translation3d(pb_dr->pose().pose_local().position().x(),
                           pb_dr->pose().pose_local().position().y(), 0.0);

  location_ptr->pose = translation * R;

  location_ptr->linear_velocity[0] =
      pb_dr->velocity().twist_vrf().linear_vrf().x();
  location_ptr->linear_velocity[1] =
      pb_dr->velocity().twist_vrf().linear_vrf().y();
  location_ptr->linear_velocity[2] =
      pb_dr->velocity().twist_vrf().linear_vrf().z();

  location_ptr->angular_velocity[0] =
      pb_dr->velocity().twist_vrf().angular_vrf().x();
  location_ptr->angular_velocity[1] =
      pb_dr->velocity().twist_vrf().angular_vrf().y();
  location_ptr->angular_velocity[2] =
      pb_dr->velocity().twist_vrf().angular_vrf().z();
  std::cout << "location_ptr->datastamp====="
            << std::to_string(location_ptr->timestamp) << std::endl;
  std::cout << "location_ptr->pose==========" << location_ptr->pose.matrix()
            << std::endl;

  return true;
}

bool DataMapping::CvtPbLocation2Location(
    const NetaLoactionPtr &pb_location,
    std::shared_ptr<base::Location> location_ptr) {
  location_ptr->timestamp = pb_location->header().publish_stamp();
  location_ptr->pose = Eigen::Affine3d::Identity();

  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(
          pb_location->pose_local().euler_angles().z() / 180.0 * M_PI,
          Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.0 / 180.0 * M_PI, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0 / 180.0 * M_PI, Eigen::Vector3d::UnitX());
  Eigen::Translation3d translation =
      Eigen::Translation3d(pb_location->pose_local().position().x(),
                           pb_location->pose_local().position().y(), 0.0);

  location_ptr->pose = translation * R;

  // location_ptr->quat = Eigen::Quaterniond(
  //     pb_location->pose_local().quaternion().x(),
  //     pb_location->pose_local().quaternion().y(),
  //     pb_location->pose_local().quaternion().z(),
  //     pb_location->pose_local().quaternion().w());

  location_ptr->linear_velocity[0] =
      pb_location->pose_local().linear_velocity_vrf().x();
  location_ptr->linear_velocity[1] =
      pb_location->pose_local().linear_velocity_vrf().y();
  location_ptr->linear_velocity[2] =
      pb_location->pose_local().linear_velocity_vrf().z();

  location_ptr->angular_velocity[0] =
      pb_location->pose_local().angular_velocity_vrf().x();
  location_ptr->angular_velocity[1] =
      pb_location->pose_local().angular_velocity_vrf().y();
  location_ptr->angular_velocity[2] =
      pb_location->pose_local().angular_velocity_vrf().z();

  return true;
}

}  //  namespace common_onboard
}  // namespace mp
}  //  namespace hozon
