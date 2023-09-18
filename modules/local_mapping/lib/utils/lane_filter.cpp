/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei
 *Date: 2023-09-011
 *****************************************************************************/
#include "modules/local_mapping/lib/utils/lane_filter.h"

namespace hozon {
namespace mp {
namespace lm {

void LaneToPts(const LaneCubicSpline& cubic_curve,
               std::shared_ptr<std::vector<Eigen::Vector2d>> pts,
               float gap = 1.0) {
  double a = cubic_curve.c0_;
  double b = cubic_curve.c1_;
  double c = cubic_curve.c2_;
  double d = cubic_curve.c3_;
  double start_x = cubic_curve.start_point_x_;
  double end_x = cubic_curve.end_point_x_;

  float fac = 1.0;
  for (size_t i = 0;; ++i) {
    float curr_x = start_x + static_cast<float>(i) * gap * fac;
    if ((fac > 0 && curr_x >= end_x) || (fac < 0 && curr_x <= end_x)) {
      break;
    }
    float curr_y =
        a * curr_x * curr_x * curr_x + b * curr_x * curr_x + c * curr_x + d;
    Eigen::Vector2d pt(curr_x, curr_y);
    pts->push_back(pt);
  }
}

void PtFilter::Init(const LocalMapLane& lane) {
  std::shared_ptr<std::vector<Eigen::Vector2d>> pts =
      std::make_shared<std::vector<Eigen::Vector2d>>();
  for (auto& lane_param : lane.lane_param_) {
    LaneToPts(lane_param, pts);
  }
  filter_.clear();
  for (auto& pt : *pts) {
    filter_.emplace_back(KFFilter(pt));
  }
  is_activate_ = true;
}

void PtFilter::Predict(std::shared_ptr<std::vector<Eigen::Vector2d>> pts) {
  for (auto& pt_filter : filter_) {
    Eigen::Vector2d pt = pt_filter.Predict();
    pts->push_back(pt);
  }
}

void PtFilter::Update(const std::vector<Eigen::Vector2d>& pts) {
  for (size_t i = 0; i < filter_.size(); i++) {
    filter_[i].Update(pts[i]);
  }
}

void LaneFilter::Init(const std::vector<LocalMapLane>& lanes) {
  for (size_t i = 0; i < lanes.size(); i++) {
    filters_[i].Init(lanes[i]);
  }
  hist_lane_ = lanes;
}

void LaneFilter::FilterLane(LocalMapLane* curtLane, PtFilter* filter) {
  std::shared_ptr<std::vector<Eigen::Vector2d>> predict_pts =
      std::make_shared<std::vector<Eigen::Vector2d>>();
  filter->Predict(predict_pts);

  std::shared_ptr<std::vector<Eigen::Vector2d>> cur_pts =
      std::make_shared<std::vector<Eigen::Vector2d>>();
  for (auto& lane_param : curtLane->lane_param_) {
    LaneToPts(lane_param, cur_pts);
  }
  filter->Update(*cur_pts);
}

void LaneFilter::Process(std::shared_ptr<std::vector<LocalMapLane>> cur_lane) {
  return;
  for (size_t i = 0; i < hist_lane_.size(); i++) {
    bool is_matched = false;
    for (size_t j = 0; j < cur_lane->size(); j++) {
      if (hist_lane_[i].track_id_ == (*cur_lane)[j].track_id_) {
        FilterLane(&(*cur_lane)[j], &(filters_[i]));
        is_matched = true;
        break;
      }
    }
    if (!is_matched) {
      filters_[i].is_activate_ = false;
    }
  }

  for (size_t i = 0; i < cur_lane->size(); i++) {
    bool is_matched = false;
    for (size_t j = 0; j < hist_lane_.size(); j++) {
      if ((*cur_lane)[i].track_id_ == hist_lane_[j].track_id_) {
        is_matched = true;
        break;
      }
    }
    if (!is_matched) {
      for (size_t k = 0; k < hist_lane_.size(); k++) {
        if (!filters_[k].is_activate_) {
          filters_[k].Init((*cur_lane)[i]);
          break;
        }
      }
    }
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
