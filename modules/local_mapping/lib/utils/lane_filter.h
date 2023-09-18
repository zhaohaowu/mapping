/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei
 *Date: 2023-09-011
 *****************************************************************************/
#pragma once
#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "modules/local_mapping/lib/types/common.h"
#include "modules/local_mapping/lib/utils/kalman_filter.h"

namespace hozon {
namespace mp {
namespace lm {

struct PtFilter {
  PtFilter() = default;

  void Init(const LocalMapLane& lane);

  void Predict(std::shared_ptr<std::vector<Eigen::Vector2d>> pts);

  void Update(const std::vector<Eigen::Vector2d>& pts);

  std::vector<KFFilter> filter_;
  bool is_activate_ = false;
};

class LaneFilter {
 public:
  LaneFilter() : filters_(10) {}

  void Init(const std::vector<LocalMapLane>& lanes);

  void Process(std::shared_ptr<std::vector<LocalMapLane>> cur_lane);

 private:
  void FilterLane(LocalMapLane* curtLane, PtFilter* filter);

 private:
  std::vector<PtFilter> filters_;
  std::vector<LocalMapLane> hist_lane_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
