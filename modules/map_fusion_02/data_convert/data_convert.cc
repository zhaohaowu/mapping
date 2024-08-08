#include "modules/map_fusion_02/data_convert/data_convert.h"

namespace hozon {
namespace mp {
namespace mf {

bool DataConvert::LocalMap2ElmentMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& map_msg,
    const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg,
    ElementMap::Ptr elem_map) {
  if (elem_map == nullptr) {
    HLOG_ERROR << "Element map is nullptr";
    return false;
  }

  return true;
}

bool DataConvert::Localization2LocInfo(
    const std::shared_ptr<hozon::localization::Localization>& loc_msg,
    LocInfo::Ptr loc_info) {
  if (loc_info == nullptr) {
    HLOG_ERROR << "Loc info is nullptr";
    return false;
  }

  loc_info->timestamp = loc_msg->header().data_stamp();
  loc_info->position.x() = loc_msg->pose_local().position().x();
  loc_info->position.y() = loc_msg->pose_local().position().y();
  loc_info->position.z() = loc_msg->pose_local().position().z();
  loc_info->quaternion.w() = loc_msg->pose_local().quaternion().w();
  loc_info->quaternion.x() = loc_msg->pose_local().quaternion().x();
  loc_info->quaternion.y() = loc_msg->pose_local().quaternion().y();
  loc_info->quaternion.z() = loc_msg->pose_local().quaternion().z();

  loc_info->linear_vrf.x() = loc_msg->pose_local().linear_velocity().x();
  loc_info->linear_vrf.y() = loc_msg->pose_local().linear_velocity().y();
  loc_info->linear_vrf.z() = loc_msg->pose_local().linear_velocity().z();
  loc_info->angular_vrf.x() = loc_msg->pose_local().angular_velocity().x();
  loc_info->angular_vrf.y() = loc_msg->pose_local().angular_velocity().y();
  loc_info->angular_vrf.z() = loc_msg->pose_local().angular_velocity().z();
  loc_info->pose = Eigen::Translation3d(loc_info->position) *
                   Eigen::Affine3d(loc_info->quaternion);

  return true;
}
}  // namespace mf
}  // namespace mp
}  // namespace hozon