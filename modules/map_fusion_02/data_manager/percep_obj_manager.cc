/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： percep_obj_manager.cc
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/data_manager/percep_obj_manager.h"

#include <Eigen/Core>

#include "modules/map_fusion_02/data_convert/data_convert.h"
#include "modules/map_fusion_02/data_manager/dr_data_manager.h"

namespace hozon {
namespace mp {
namespace mf {

PercepObjManager::PercepObjManager() { CHECK_EQ(this->Init(), true); }

bool PercepObjManager::Init() {
  if (inited_) {
    return true;
  }
  history_objs_.set_capacity(history_objs_size_);
  inverse_history_objs_.set_capacity(inverse_history_objs_size_);
  inited_ = true;
  return true;
}

bool PercepObjManager::PushObjects(
    const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg) {
  if (obj_msg == nullptr) {
    return false;
  }
  if (!obj_msg->header().has_data_stamp()) {
    return false;
  }
  LocInfo::ConstPtr perception_pose =
      DR_MANAGER->GetDrPoseByTimeStamp(obj_msg->header().data_stamp());
  if (perception_pose == nullptr) {
    HLOG_ERROR << "obj_msg perception_pose is nullptr";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& object : obj_msg->perception_obstacle()) {
    auto elem_obj = std::make_shared<Object>();
    DataConvert::cvtPb2obj(object, elem_obj);
    // 车体系位置转到local系保存
    auto local_position = perception_pose->pose * elem_obj->position;
    elem_obj->position = local_position;
    history_objs_.push_back(elem_obj);
    if (object.type() == hozon::perception::PerceptionObstacle::VEHICLE &&
        object.position().x() > 0 && object.velocity().x() <= 0 &&
        (object.theta() < M_PI * 3 / -4 || object.theta() > M_PI * 3 / 4)) {
      inverse_history_objs_.push_back(elem_obj);
    }
  }
  return true;
}

boost::circular_buffer<std::shared_ptr<Object>>
PercepObjManager::GetHistoryObjs() {
  std::lock_guard<std::mutex> lock(mutex_);
  return history_objs_;
}

boost::circular_buffer<std::shared_ptr<Object>>
PercepObjManager::GetInverseHistoryObjs() {
  std::lock_guard<std::mutex> lock(mutex_);
  return inverse_history_objs_;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
