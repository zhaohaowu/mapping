/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei
 *Date: 2023-09-011
 *****************************************************************************/
#include "modules/local_mapping/lib/utils/lane_filter.h"

namespace hozon {
namespace mp {
namespace lm {

void PtFilter::Init(const LaneCubicSpline& lane) {
  std::shared_ptr<std::vector<Eigen::Vector2d>> pts =
      std::make_shared<std::vector<Eigen::Vector2d>>();
  CommonUtil::SampleEKFPoints(lane, pts);
  filter_.clear();
  for (auto& pt : *pts) {
    filter_.emplace_back(KFFilter(pt));
  }
}

void PtFilter::Predict(double theta, const Eigen::Vector2d& T,
                       std::shared_ptr<std::vector<Eigen::Vector2d>> pts) {
  for (auto& pt_filter : filter_) {
    Eigen::Vector2d pt = pt_filter.Predict(theta, T);

    pts->push_back(pt);
  }
}

void PtFilter::Update(const std::vector<Eigen::Vector2d>& measure_pts,
                      std::shared_ptr<std::vector<Eigen::Vector3d>> pts) {
  for (size_t i = 0; i < filter_.size(); i++) {
    pts->emplace_back(filter_[i].Update(measure_pts[i]));
  }
}

LaneCubicSpline MapLaneTolane(std::shared_ptr<const Lane> cur_lane) {
  LaneCubicSpline cur_lane_func;
  cur_lane_func.c0_ = cur_lane->lane_fit_a_;
  cur_lane_func.c1_ = cur_lane->lane_fit_b_;
  cur_lane_func.c2_ = cur_lane->lane_fit_c_;
  cur_lane_func.c3_ = cur_lane->lane_fit_d_;
  cur_lane_func.start_point_x_ = cur_lane->x_start_vrf_;
  cur_lane_func.end_point_x_ = cur_lane->x_end_vrf_;
  return cur_lane_func;
}

void LaneFilter::Init(std::shared_ptr<const Lane> cur_lane) {
  LaneCubicSpline cur_lane_func = MapLaneTolane(cur_lane);
  kf_.Init(cur_lane_func);
  std::shared_ptr<std::vector<Eigen::Vector2d>> pts =
      std::make_shared<std::vector<Eigen::Vector2d>>();
  CommonUtil::SampleEKFPoints(cur_lane_func, pts);
  last_lane_pts_ = *pts;
}

void LaneFilter::GetCurLanePts(
    const std::vector<Eigen::Vector2d>& predicted_pts,
    const LaneCubicSpline& cubic_curve,
    std::shared_ptr<std::vector<Eigen::Vector2d>> pts) {
  double a = cubic_curve.c0_;
  double b = cubic_curve.c1_;
  double c = cubic_curve.c2_;
  double d = cubic_curve.c3_;

  for (auto& pre_pt : predicted_pts) {
    double curr_x = pre_pt.x();
    float curr_y =
        a * curr_x * curr_x * curr_x + b * curr_x * curr_x + c * curr_x + d;
    pts->emplace_back(Eigen::Vector2d(curr_x, curr_y));
  }
}
bool LaneFilter::LaneProcess(
    std::shared_ptr<const Lane> cur_lane,
    std::shared_ptr<LaneCubicSpline> filtered_lane_func) {
  Eigen::Vector3d delta_pose = GetRelativePose(last_lane_pose_, cur_lane_pose_);
  Eigen::Vector2d T(delta_pose.x(), delta_pose.y());

  std::shared_ptr<std::vector<Eigen::Vector2d>> predicted_pts =
      std::make_shared<std::vector<Eigen::Vector2d>>();
  kf_.Predict(delta_pose.z(), T, predicted_pts);
  std::shared_ptr<std::vector<Eigen::Vector2d>> measure_pts =
      std::make_shared<std::vector<Eigen::Vector2d>>();

  LaneCubicSpline cur_lane_func = MapLaneTolane(cur_lane);
  // CommonUtil::SampleEKFPoints(cur_lane_func, measure_pts);
  GetCurLanePts(*predicted_pts, cur_lane_func, measure_pts);
  std::shared_ptr<std::vector<Eigen::Vector3d>> filtered_pts =
      std::make_shared<std::vector<Eigen::Vector3d>>();
  kf_.Update(*measure_pts, filtered_pts);
  CommonUtil::FitEKFLane(*filtered_pts, filtered_lane_func);
  last_lane_func_ = *filtered_lane_func;
  last_lane_pts_ = *measure_pts;
  return true;
}

Eigen::Vector3d LaneFilter::GetRelativePose(const Eigen::Vector3d& pose0,
                                            const Eigen::Vector3d& pose1) {
  double delta_theta = pose1.z() - pose0.z();

  Eigen::Matrix2d rot_mat;
  rot_mat << std::cos(pose0.z()), -std::sin(pose0.z()), std::sin(pose0.z()),
      std::cos(pose0.z());
  Eigen::Vector2d delta_t =
      Eigen::Vector2d(pose1.x() - pose0.x(), pose1.y() - pose0.y());
  Eigen::Vector2d delta_trans = rot_mat * delta_t;

  return Eigen::Vector3d(delta_trans.x(), delta_trans.y(), delta_theta);
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
