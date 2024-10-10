/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_lane_loc.cc
 *   author     ： zhaohaowu
 *   date       ： 2024.07
 ******************************************************************************/

#include <modules/map_fusion/modules/lane_loc/fusion_lane_loc.h>

#include <memory>

#include "base/utils/log.h"
#include "modules/map_fusion/modules/lane_loc/base_lane_loc.h"

namespace hozon {
namespace mp {
namespace mf {
namespace lane_loc {

std::shared_ptr<std::vector<double>> FusionLaneLoc::Run(
    int* fusion_lane_index, std::vector<double>* p_predict,
    double measure_weight,
    const std::shared_ptr<const std::vector<double>>& p_measure_ptr,
    const TurnState& turn_state) {
  std::shared_ptr<std::vector<double>> p_fusion_ptr =
      std::make_shared<std::vector<double>>();
  if (p_measure_ptr->empty()) {
    HLOG_WARN << "why measure_p_lanes_ptr is empty";
    return nullptr;
  }
  static std::vector<double> last_p_fusion = *p_measure_ptr;
  int n = static_cast<int>(last_p_fusion.size());
  if (n == 0) {
    HLOG_WARN << "why last_p_fusion is empty";
    return nullptr;
  }
  if (n == 1) {
    double max_p = FLT_MIN;
    for (int i = 0; i < static_cast<int>(p_measure_ptr->size()); i++) {
      if (p_measure_ptr->at(i) > max_p) {
        max_p = p_measure_ptr->at(i);
        *fusion_lane_index = i + 1;
      }
    }
    *p_fusion_ptr = *p_measure_ptr;
    last_p_fusion = *p_fusion_ptr;
    return p_fusion_ptr;
  }
  // 获取上一步骤的预测结果
  double p_keep = 0;
  double p_left = 0;
  double p_right = 0;
  if (turn_state == STARIGHT) {
    p_keep = 1;
  } else if (turn_state == TURN_LEFT) {
    p_left = 1;
  } else if (turn_state == TURN_RIGHT) {
    p_right = 1;
  }
  p_predict->resize(n);
  p_predict->at(0) = last_p_fusion[0] * p_keep +
                     (last_p_fusion[0] + last_p_fusion[1]) * p_left;
  p_predict->at(n - 1) =
      last_p_fusion[n - 1] * p_keep +
      (last_p_fusion[n - 1] + last_p_fusion[n - 2]) * p_right;
  // HLOG_ERROR << "p_predict->size() " << p_predict->size();
  for (int i = 1; i < n - 1; i++) {
    p_predict->at(i) = last_p_fusion[i - 1] * p_right +
                       last_p_fusion[i] * p_keep +
                       last_p_fusion[i + 1] * p_left;
  }
  // 将预测与观测融合
  if (p_measure_ptr->size() == last_p_fusion.size()) {
    for (int i = 0; i < n; i++) {
      p_fusion_ptr->emplace_back(p_measure_ptr->at(i) * measure_weight +
                                 p_predict->at(i) * (1 - measure_weight));
    }
  } else {
    *p_fusion_ptr = *p_measure_ptr;
  }
  last_p_fusion = *p_fusion_ptr;
  double max_p = FLT_MIN;
  for (int i = 0; i < static_cast<int>(p_fusion_ptr->size()); i++) {
    if (p_fusion_ptr->at(i) > max_p) {
      max_p = p_fusion_ptr->at(i);
      *fusion_lane_index = i + 1;
    }
  }
  return p_fusion_ptr;
}

}  // namespace lane_loc
}  // namespace mf
}  // namespace mp
}  // namespace hozon
