// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.h
// @brief: associate history lane_track to current detected lane　object

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "modules/local_mapping/lib/laneline/interface/base_lane_measurement_filter.h"
#include "modules/local_mapping/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/local_mapping/lib/laneline/utils/lane_utils.h"
#include "perception-base/base/measurement/laneline_measurement.h"
namespace hozon {
namespace mp {
namespace environment {

using hozon::perception::base::LaneLineMeasurementPtr;
using hozon::perception::base::LaneLinePoint;
using hozon::perception::base::LaneLinePosition;

class LaneMeasurementFilter : public BaseLaneMeasurementFilter {
 public:
  LaneMeasurementFilter() : BaseLaneMeasurementFilter() {}
  virtual ~LaneMeasurementFilter() {}

  bool Init(const AnomalyFilterInitOptions& options =
                AnomalyFilterInitOptions()) override;
  bool Filter(const AnomalyFilterOptions& options,
              const LaneLinesMeasurementConstPtr& input_measurements,
              const LaneLinesMeasurementPtr& output_measurements) override;

  std::string Name() const override;

 private:
  bool DeleteBehindVehicleLaneLine(
      std::vector<LaneLineMeasurementPtr>* measure_lanelines);

  bool DelFarShortLaneLine(
      std::vector<LaneLineMeasurementPtr>* measure_lanelines);

  bool DelHighlyOverlayShortLaneLine(
      std::vector<LaneLineMeasurementPtr>* measure_lanelines);

  bool DelHighlyOverlayLowConfLaneLine(
      std::vector<LaneLineMeasurementPtr>* measure_lanelines);

  bool DelMiddleOverlayFarLaneline(
      std::vector<LaneLineMeasurementPtr>* measure_lanelines);

  bool DelAnomalyIntervalLaneLine(
      std::vector<LaneLineMeasurementPtr>* measure_lanelines);

  std::vector<LaneLineMeasurementPtr> GetForkOrMergeLaneLine(
      const std::vector<LaneLineMeasurementPtr>& measure_lanelines);

  std::vector<LaneLineMeasurementPtr> GetUnForkOrMergeLaneLine(
      const std::vector<LaneLineMeasurementPtr>& measure_lanelines);

  bool IsSamePosLaneLine(
      const hozon::perception::base::LaneLineMeasurementConstPtr& ori_lanelane,
      const hozon::perception::base::LaneLineMeasurementConstPtr&
          compare_laneline);

 private:
  LaneMeasurementFilterParam lane_meas_filter_param_;
};
PERCEPTION_ENVIRONMENT_REGISTER_BASE_LANE_MEASUREMENT_FILTER(
    LaneMeasurementFilter);

}  // namespace environment
}  // namespace mp
}  // namespace hozon
