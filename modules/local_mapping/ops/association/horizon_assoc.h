/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: chenlongxi
 *Date: 2023-10-09
 *****************************************************************************/
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/local_mapping/types/types.h"
#include "modules/local_mapping/utils/common.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"

namespace hozon {
namespace mp {
namespace lm {

class HorizonLaneAssoc {
 public:
  static std::unordered_map<int, int> Process(
      const std::vector<LaneLine>& lane_lines_det,
      const std::vector<LaneLine>& lane_lines_lm);
};

using HorizonLaneAssocPtr = std::shared_ptr<HorizonLaneAssoc>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
