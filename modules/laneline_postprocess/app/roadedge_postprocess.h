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
#include "modules/laneline_postprocess/lib/laneline/interface/base_roadedge_process.h"
// #include
// "modules/laneline_postprocess/lib/laneline/proto/roadedge_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/pipeline/roadedge_point_tracker_pipeline.h"
#include "perception-base/base/frame/fusion_frame.h"
#include "perception-base/base/frame/measurement_frame.h"

namespace hozon {
namespace mp {
namespace environment {

class RoadEdgePostProcess : public BaseRoadEdgeProcess {
 public:
  RoadEdgePostProcess() = default;
  ~RoadEdgePostProcess() = default;

  bool Init(const ProcessInitOption& init_option) override;

  // @brief: run lane post process
  bool Process(const ProcessOption& options,
               base::RoadEdgesMeasurementConstPtr detect_measurements,
               const base::RoadEdgesPtr track_outputs) override;

  virtual bool Process(const base::MeasurementFramePtr measurement_ptr,
                       base::FusionFramePtr fusion_ptr);

  std::string Name() const override { return "RoadEdgePostProcess"; }

  void Reset() override;

 private:
  /* lane track */
  std::unique_ptr<RoadEdgePointFilterTrackerPipeline> roadedge_tracker_;
  LanePostProcessParam config_;
  std::vector<base::RoadEdgePtr> last_track_roadedges_;
};

PERCEPTION_ENVIRONMENT_REGISTER_BASE_ROADEDGE_PROCESS(RoadEdgePostProcess)

}  // namespace environment
}  // namespace mp
}  // namespace hozon
