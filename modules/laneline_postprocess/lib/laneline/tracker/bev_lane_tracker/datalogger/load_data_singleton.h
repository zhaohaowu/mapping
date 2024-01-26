/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/laneline_postprocess/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/datalogger/data_buffer.h"
#include "perception-base/base/location/location.h"
#include "perception-base/base/measurement/laneline_measurement.h"
#include "perception-base/base/measurement/roadedges_measurement.h"
#include "perception-base/base/utils/macros.h"

namespace hozon {
namespace mp {
namespace environment {

using namespace hozon::perception;

class InputDataSingleton {
 public:
  bool Init();
  bool IsStaticState(StaticStrategyParam config);
  bool IsTurnState(StaticStrategyParam config);
  bool GetTurnState();

 public:
  /** @brief sensor data buffer. */
  MessageBuffer<base::LocationConstPtr> dr_data_buffer_;
  MessageBuffer<std::vector<base::LaneLineMeasurementPtr>> lanes_buffer_;
  MessageBuffer<std::vector<base::RoadEdgeMeasurementPtr>> roadedges_buffer_;
  int turn_count_ = 0;
  int turn_count_threshold_ = 4;
  bool turn_state_ = false;

  float static_dist_accu_ = 0.0;
  float static_angular_accu_ = 0.0;

 private:
  std::mutex mutex_;
  bool inited_ = false;
  DECLARE_SINGLETON_PERCEPTION(InputDataSingleton)
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
