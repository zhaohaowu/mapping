/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "depend/perception-base/base/measurement/arrow_measurement.h"
#include "depend/perception-base/base/point/point.h"

namespace hozon {
namespace mp {
namespace lm {

enum class TrackState {
  MATURED,
  NOTMATURED,
};

struct BaseData {
  int tracked_count = 0;

  TrackState state = TrackState::NOTMATURED;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
