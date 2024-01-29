/*================================================================
*   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
*   file       ：lane_post_process.h
*   author     ：Chenanmeng
*   date       ：2023.02.28
================================================================*/

#pragma once

#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/laneline_postprocess/lib/laneline/curve_fitter/base_curve_fitter.h"
#include "modules/laneline_postprocess/lib/laneline/interface/base_lane_measurement_filter.h"
#include "modules/laneline_postprocess/lib/laneline/interface/base_lane_process.h"
#include "modules/laneline_postprocess/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/pipeline/lane_point_tracker_pipeline.h"
#include "perception-base/base/frame/fusion_frame.h"
#include "perception-base/base/frame/measurement_frame.h"

namespace hozon {
namespace mp {
namespace environment {

namespace perception_base = hozon::perception::base;
namespace perception_lib = hozon::perception::lib;
class LanePostProcess : public BaseLaneProcess {
 public:
  LanePostProcess() = default;
  ~LanePostProcess() = default;

  bool Init(const ProcessInitOption& init_option) override;

  // @brief: run lane post process
  bool Process(
      const ProcessOption& options,
      perception_base::LaneLinesMeasurementConstPtr detect_measurements,
      const perception_base::LaneLinesPtr track_outputs) override;

  virtual bool Process(
      const perception_base::MeasurementFramePtr measurement_ptr,
      perception_base::FusionFramePtr fusion_ptr);

  std::string Name() const override { return "LanePostProcess"; }

  void Reset() override;

 private:
  void TransTrackerLocal2Vehicle(
      std::vector<perception_base::LaneLinePtr>* tracked_outputs);

 private:
  /* lane track */
  std::unique_ptr<LanePointFilterTrackerPipeline> lane_tracker_;
  LanePostProcessParam config_;
  std::vector<perception_base::LaneLinePtr> last_track_lanelines_;
  std::unique_ptr<BaseLaneMeasurementFilter> lane_measurements_filter_;
};

PERCEPTION_ENVIRONMENT_REGISTER_BASE_LANE_PROCESS(LanePostProcess)

}  // namespace environment
}  // namespace mp
}  // namespace hozon
