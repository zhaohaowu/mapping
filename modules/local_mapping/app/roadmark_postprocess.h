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

#include "perception-base/base/frame/fusion_frame.h"
#include "perception-base/base/frame/measurement_frame.h"

namespace hozon {
namespace mp {
namespace environment {

class RoadMarkPostProcess {
 public:
  RoadMarkPostProcess() = default;
  ~RoadMarkPostProcess() = default;

  bool Process(
      const hozon::perception::base::MeasurementFramePtr measurement_ptr,
      hozon::perception::base::FusionFramePtr fusion_ptr);

 protected:
  bool ProcessArrows(
      const hozon::perception::base::ArrowsMeasurementPtr detect_arrows,
      hozon::perception::base::ArrowsPtr track_arrows);

  bool ProcessNoParkings(
      const hozon::perception::base::NoparkingsMeasurementPtr detect_noparkings,
      hozon::perception::base::NoParkingsPtr track_noparkings);

  bool ProcessSlowDowns(
      const hozon::perception::base::SlowdownsMeasurementPtr detect_slowdowns,
      hozon::perception::base::SlowDownsPtr track_slowdowns);

  bool ProcessWaitZones(
      const hozon::perception::base::WaitZonesMeasurementPtr detect_waitzones,
      hozon::perception::base::WaitZonesPtr track_waitzones);

  bool ProcessStopLines(
      const hozon::perception::base::StopLinesMeasurementPtr detect_stoplines,
      hozon::perception::base::StopLinesPtr track_stoplines);

  bool ProcessZebraCrossings(
      const hozon::perception::base::ZebraCrossingsMeasurementPtr
          detect_ZebraCrossings,
      hozon::perception::base::ZebraCrossingsPtr track_ZebraCrossings);
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
