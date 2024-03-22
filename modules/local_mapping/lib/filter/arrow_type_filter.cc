// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: lane_point_filter.cc
// @brief: filter 3d points

#include "modules/local_mapping/lib/filter/arrow_type_filter.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <utility>

#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/local_mapping/utils/common.h"

namespace hozon {
namespace mp {
namespace lm {

bool ArrowTypeFilter::Init() {
  history_measure_arrows_.set_capacity(history_measure_size_);
  history_measure_arrows_.clear();

  arrows_type_probability_.clear();
  return true;
}

void ArrowTypeFilter::UpdateWithMeasurement(const ArrowPtr& measurement) {
  history_measure_arrows_.push_back(measurement);
  for (auto& measure_arrow : history_measure_arrows_) {
    if (!(arrows_type_probability_.find(measure_arrow->type) ==
          arrows_type_probability_.end())) {
      arrows_type_probability_[measure_arrow->type] +=
          measure_arrow->confidence;
    } else {
      arrows_type_probability_[measure_arrow->type] = measure_arrow->confidence;
    }
  }

  double type_max_score = DBL_MIN;
  for (const auto& pair : arrows_type_probability_) {
    if (pair.second > type_max_score) {
      type_max_score = pair.second;
      arrow_maxprob_type_ = pair.first;
    }
  }

  auto& track_arrow_ptr = target_ref_->GetTrackedObject();
  track_arrow_ptr->type = arrow_maxprob_type_;
}

void ArrowTypeFilter::UpdateWithoutMeasurement() {
  // 未匹配时， 保持上一帧跟踪结果。
}

void ArrowTypeFilter::Reset() { history_measure_arrows_.clear(); }

}  // namespace lm
}  // namespace mp
}  // namespace hozon
