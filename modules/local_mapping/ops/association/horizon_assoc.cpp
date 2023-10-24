/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: chenlongxi
 *Date: 2023-10-09
 *****************************************************************************/
#include "modules/local_mapping/ops/association/horizon_assoc.h"

namespace hozon {
namespace mp {
namespace lm {

std::unordered_map<int, int> HorizonLaneAssoc::Process(
    const std::vector<LaneLine>& lane_lines_det,
    const std::vector<LaneLine>& lane_lines_lm) {
  std::unordered_map<int, int> map_det_lm_;
  if (lane_lines_lm.empty() || lane_lines_det.empty()) {
    return map_det_lm_;
  }
  int num_lm_ = static_cast<int>(lane_lines_lm.size());
  int num_det_ = static_cast<int>(lane_lines_det.size());
  Matxd assoc_scores = Matxd::Zero(num_det_, num_lm_);
  std::unordered_set<int> lm_used_index;
  // 计算匹配分
  for (int i = 0; i < num_det_; ++i) {
    // HLOG_ERROR << "lanes_det.lanes_[i].points_.size(): "
    //            << lanes_det.lanes_[i].points_.size();
    double min_average_y = DBL_MAX;
    int lm_index = 0;
    for (int j = 0; j < num_lm_; ++j) {
      // 计算每条地图车道线与当前感知车道线的y方向距离分数
      double sum_y = 0;
      int count = 0;
      for (int m = static_cast<int>(lane_lines_lm[j].points_.size()) - 1;
           m >= 0; --m) {
        // HLOG_ERROR << "lanes_lm[j].points_[m].x(): "
        //            << lane_lines_lm[j].points_[m].x();
        // HLOG_ERROR << "lanes_det.lanes_[i].start_point_x_: "
        //            << lane_lines_det[i].start_point_x_;
        // HLOG_ERROR << "index j m i: " << j << ", " << m << ", " << i;
        if (lane_lines_lm[j].points_[m].x() <
            lane_lines_det[i].start_point_x_) {
          break;
        }
        if (lane_lines_lm[j].points_[m].x() > 50) {
          continue;
        }
        double temp_x = lane_lines_lm[j].points_[m].x();
        double temp_y = lane_lines_det[i].c0_ + lane_lines_det[i].c1_ * temp_x +
                        lane_lines_det[i].c2_ * temp_x * temp_x +
                        lane_lines_det[i].c3_ * temp_x * temp_x * temp_x;
        // HLOG_ERROR << "c3: " << lanes_det.lanes_[i].c3_
        //            << "c2: " << lanes_det.lanes_[i].c2_
        //            << "c1: " << lanes_det.lanes_[i].c1_
        //            << "c0: " << lanes_det.lanes_[i].c0_;
        // HLOG_ERROR << "temp_y: " << temp_y;
        // HLOG_ERROR << "lanes_lm[j].points_[m].y(): "
        //            << lanes_lm[j].points_[m].y();
        // if (abs(lanes_lm[j].points_[m].y() - temp_y) > 1) continue;
        sum_y += abs(lane_lines_lm[j].points_[m].y() - temp_y);
        count++;
      }
      if (count == 0) {
        continue;
      }
      double average_y = sum_y / count;
      assoc_scores(i, j) = average_y;
      if (average_y < 1 && average_y < min_average_y &&
          lm_used_index.find(j) == lm_used_index.end()) {
        min_average_y = average_y;
        lm_index = j;
      }
    }
    if (min_average_y < 1) {
      map_det_lm_[i] = lm_index;
      lm_used_index.insert(lm_index);
    }
  }
  // std::cout << std::setprecision(6) << assoc_scores << std::endl;
  return map_det_lm_;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
