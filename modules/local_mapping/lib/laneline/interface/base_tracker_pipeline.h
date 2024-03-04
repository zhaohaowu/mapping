// Copyright 2020 Hozon Inc. All Rights Reserved.
// Descriptions: lane tracker interface

#pragma once

#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "lib/registerer/registerer.h"
#include "modules/local_mapping/lib/laneline/interface/base_init_options.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"
#include "perception-base/base/measurement/laneline_measurement.h"
#include "perception-base/base/scene/laneline.h"

namespace hozon {
namespace mp {
namespace environment {

namespace perception_base = hozon::perception::base;

class BaseTrackerPipeline {
 public:
  BaseTrackerPipeline() = default;

  virtual ~BaseTrackerPipeline() = default;

  virtual bool Init(const ProcessInitOption& options = ProcessInitOption()) = 0;

  virtual std::string Name() const = 0;

  BaseTrackerPipeline(const BaseTrackerPipeline&) = delete;
  BaseTrackerPipeline& operator=(const BaseTrackerPipeline&) = delete;

 protected:
  void UpdateDrPose() {
    auto& dr_datas = InputDataSingleton::Instance()->dr_data_buffer_;
    novatel2world_pose_ = dr_datas.back()->pose;

    if (dr_datas.buffer_size() > 1)
      last_novatel2world_pose_ = dr_datas.back2()->pose;
    return;
  }

  void TransVehiclePoint2Local(
      std::vector<perception_base::LaneLinePoint>* point_set) {
    Eigen::Vector3d vehicle_pt(0.0, 0.0, 0.0);
    Eigen::Vector3d local_pt(0.0, 0.0, 0.0);

    for (auto& lane_line_point : *point_set) {
      auto& vehicle_point = lane_line_point.vehicle_point;
      vehicle_pt[0] = vehicle_point.x;
      vehicle_pt[1] = vehicle_point.y;
      local_pt = novatel2world_pose_ * vehicle_pt;

      auto& local_point = lane_line_point.local_point;
      local_point.x = local_pt[0];
      local_point.y = local_pt[1];
      local_point.z = 0.0;
    }
  }

  void TransLocalPoint2Vehicle(
      std::vector<perception_base::LaneLinePoint>* point_set) {
    Eigen::Vector3d vehicle_pt(0.0, 0.0, 0.0);
    Eigen::Vector3d local_pt(0.0, 0.0, 0.0);

    for (auto& lane_line_point : *point_set) {
      auto& local_point = lane_line_point.local_point;
      local_pt[0] = local_point.x;
      local_pt[1] = local_point.y;

      vehicle_pt = novatel2world_pose_.inverse() * local_pt;

      auto& vehicle_point = lane_line_point.vehicle_point;
      vehicle_point.x = vehicle_pt[0];
      vehicle_point.y = vehicle_pt[1];
    }
  }

  Eigen::Affine3d novatel2world_pose_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d last_novatel2world_pose_ = Eigen::Affine3d::Identity();
};

PERCEPTION_REGISTER_REGISTERER(BaseTrackerPipeline);
#define PERCEPTION_ENVIRONMENT_REGISTER_BASE_TRACKER_PIPELINE(name) \
  PERCEPTION_REGISTER_CLASS(BaseTrackerPipeline, name)

}  // namespace environment
}  // namespace mp
}  // namespace hozon
