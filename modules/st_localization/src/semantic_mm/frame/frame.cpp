/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/frame/frame.hpp"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "localization/common/log.hpp"
#include "semantic_mm/common/camera_model.hpp"
#include "semantic_mm/common/configure.hpp"
#include "semantic_mm/common/math_tools.hpp"

namespace senseAD {
namespace localization {
namespace smm {

Frame::Frame(uint64_t timestamp, const std::string& camera_name,
             const NavState& nav_state, const OdomState& odom_state,
             const PerceptData::Ptr& percept_data)
    : timestamp_(timestamp),
      camera_name_(camera_name),
      nav_state_(nav_state),
      odom_state_(odom_state),
      perception_data_(percept_data) {}

void Frame::SetTimestamp(uint64_t timestamp) { timestamp_ = timestamp; }

uint64_t Frame::GetTimestamp() const { return timestamp_; }

void Frame::SetCameraName(const std::string& camera_name) {
  camera_name_ = camera_name;
}

std::string Frame::GetCameraName() const { return camera_name_; }

void Frame::SetNavState(const NavState& nav_state) { nav_state_ = nav_state; }

NavState Frame::GetNavState() const { return nav_state_; }

void Frame::SetOdomState(const OdomState& odom_state) {
  odom_state_ = odom_state;
}

OdomState Frame::GetOdomState() const { return odom_state_; }

void Frame::SetPerceptionData(const PerceptData::Ptr& percept_data) {
  perception_data_ = percept_data;
}

PerceptData::Ptr Frame::GetPerceptionData() const { return perception_data_; }

void Frame::SetVanishingPoint(const Eigen::Vector2d& p) {
  vanishing_point_ = p;
}

Eigen::Vector2d Frame::GetVanishingPoint() const { return vanishing_point_; }

void Frame::SetHomoPitch(double pitch) {
  h_pitch_ = pitch;

  // update homography as well
  // NOTE: actually roll of camera
  Eigen::Matrix3d R_pitch;
  R_pitch << 1., 0., 0., 0., cos(h_pitch_), -sin(h_pitch_), 0., sin(h_pitch_),
      cos(h_pitch_);
  H_adapted_ = H_base_ * R_pitch * K_inv_;
}

double Frame::GetHomoPitch() const { return h_pitch_; }

Eigen::Matrix3d Frame::GetAdaptedHomography() const { return H_adapted_; }

void Frame::SetOptimHomoChi2Errors(std::pair<double, double> chi2_errors) {
  chi2_errors_ = chi2_errors;
}
std::pair<double, double> Frame::GetOptimHomoChi2Errors() const {
  return chi2_errors_;
}

void Frame::SetMatchIndex(const MatchIndex::Ptr& match) {
  match_indices_ = match;
}
MatchIndex::Ptr Frame::GetMatchIndex() const { return match_indices_; }

adLocStatus_t Frame::PreProcess() {
  auto status = PreProcessLaneLine();
  if (status != LOC_SUCCESS) {
    LC_LERROR(FRAME) << "preprocess for laneline failed.";
    return status;
  }
  // TODO(xx): preprocess for other semantic elements ...

  return LOC_SUCCESS;
}

adLocStatus_t Frame::CheckPerceptionData(SMMEvalData* eval_data) const {
  if (!eval_data) {
    LC_LERROR(SMM) << "evaluation data object is nullptr.";
    return LOC_NULL_PTR;
  }

  int valid_line_count = 0;
  int laneline_num = 0;
  int curb_num = 0;
  Eigen::Matrix3d H;
  auto status = Configure::GetInstance()->GetCameraHomography(camera_name_, &H);
  if (status != LOC_SUCCESS) {
    LC_LERROR(FRAME) << "get camera H failed, name: " << camera_name_;
    return status;
  }
  const auto& percept_lines = perception_data_->lane_lines;
  // check each percepted lane line
  for (const auto& line : percept_lines) {
    const auto& img_points = line.second.img_pts;
    if (img_points.empty()) continue;
    // project points from image to bird view
    // and check data quality
    double line_min_x = std::numeric_limits<double>::max();
    double line_max_x = std::numeric_limits<double>::min();
    int point_count = 0;
    for (const auto& point : img_points) {
      Point2D_t bv_point = H * point;
      // ignore point out of range
      if (bv_point.x < 0.0 || bv_point.x > 50.0 || bv_point.y > 15.0 ||
          bv_point.y < -15.0)
        continue;

      ++point_count;
      if (bv_point.x > line_max_x) line_max_x = bv_point.x;
      if (bv_point.x < line_min_x) line_min_x = bv_point.x;
    }
    if (line_max_x - line_min_x > 10.0 && point_count > 5) {
      ++valid_line_count;
      if (line.second.line_type == LineType::LaneMarking) {
        ++laneline_num;
      } else if (line.second.line_type == LineType::Curb) {
        ++curb_num;
      }
    }
  }

  if (valid_line_count > 0) {
    eval_data->is_perception_valid = true;
  }
  eval_data->roadline_statistics[camera_name_] =
      std::pair<int, int>(laneline_num, curb_num);

  return LOC_SUCCESS;
}

adLocStatus_t Frame::PreProcessLaneLine() {
  // get param
  auto smm_param = Configure::GetInstance()->GetLocalizationSMMParam();
  bool enable_calib_homography = smm_param.enable_calib_homography;
  bool enable_temporal_fusion = smm_param.enable_temporal_fusion;
  if (Configure::GetInstance()->GetCameraHomography(
          camera_name_, &H_adapted_) != LOC_SUCCESS) {
    LC_LERROR(FRAME) << "get camera H failed, name: " << camera_name_;
    return LOC_INVALID;
  }

  // calib homography by vanishing point
  if (enable_calib_homography) {
    if (Configure::GetInstance()->GetCameraHomographyBaseComponent(
            camera_name_, &H_base_, &K_inv_) != LOC_SUCCESS) {
      LC_LERROR(FRAME) << "get camera H bases failed, name: " << camera_name_;
      return LOC_INVALID;
    }
    if (CalibHomographyPitchByVanishingPoint() != LOC_SUCCESS) {
      LC_LDEBUG(FRAME) << "initial calib homography failed!";
    }
  }

  // project and sample BV points
  double max_long_range = 40.0;
  if (!enable_calib_homography && enable_temporal_fusion) max_long_range = 30.0;
  double sample_dist = enable_calib_homography ? 2.0 : 1.0;
  double homo_proj_cov_coeff = 0.0004;
  auto pre_status = ProjectLaneLineBVPoints(
      H_adapted_, nav_state_, max_long_range, sample_dist, homo_proj_cov_coeff,
      &perception_data_->lane_lines);
  if (pre_status != LOC_SUCCESS) {
    LC_LERROR(FRAME) << "initial preprocess laneline failed!";
    return pre_status;
  }

  return LOC_SUCCESS;
}

adLocStatus_t Frame::ProjectLaneLineBVPoints(
    const Eigen::Matrix3d& H, const NavState& nav_state,
    double max_longitudinal_range, double sample_dist,
    double homo_proj_cov_coeff,
    std::unordered_map<id_t, PerceptLaneLine>* percept_lls) {
  if (percept_lls == nullptr) return LOC_NULL_PTR;

  // preprocess for image points and BV points
  // TODO(xx): consider image point preprocess
  for (auto& item : (*percept_lls)) {
    item.second.processed_bv_points.clear();
    item.second.processed_img_points.clear();
    // convert to BV view
    const auto& img_pts = item.second.img_pts;
    if (img_pts.empty()) continue;
    std::vector<Point2D_t> bv_pts;
    bv_pts.reserve(img_pts.size());
    for (const auto& pt : img_pts) bv_pts.emplace_back(H * pt);

    // sample BV points by distance
    std::vector<Point3D_t> sampled_bv_pts;
    std::vector<Point2D_t> sampled_img_pts;
    int sample_num =
        static_cast<int>(max_longitudinal_range / sample_dist + 0.5);
    sampled_bv_pts.reserve(sample_num);
    sampled_img_pts.reserve(sample_num);
    for (size_t i = 0; i < bv_pts.size(); ++i) {
      Point3D_t pt_3d(bv_pts[i].x, bv_pts[i].y, 0);
      // longitude distance check
      if (pt_3d.x > max_longitudinal_range || pt_3d.x < 0.0) continue;
      if (sampled_bv_pts.empty()) {  // sample first point
        sampled_bv_pts.emplace_back(pt_3d);
        sampled_img_pts.emplace_back(img_pts[i]);
        continue;
      }
      // sample point given distance interval
      if ((pt_3d - sampled_bv_pts.back()).Norm() < sample_dist) continue;
      sampled_bv_pts.emplace_back(pt_3d);
      sampled_img_pts.emplace_back(img_pts[i]);
    }
    // TODO(xxx): processed_bv_points could be empty!!!
    if (sampled_bv_pts.size() >= static_cast<size_t>(3)) {
      // check whether BV points sort from near to far
      if (sampled_bv_pts.front().x > sampled_bv_pts.back().x) {
        std::reverse(sampled_bv_pts.begin(), sampled_bv_pts.end());
        std::reverse(sampled_img_pts.begin(), sampled_img_pts.end());
      }
      item.second.processed_bv_points = std::move(sampled_bv_pts);
      item.second.processed_img_points = std::move(sampled_img_pts);
    }
  }

  // calculate the uncertainty covariance for each BV point
  // propagate position covariance of vehicle to projected point
  auto R_vw = nav_state.pose.so3().inverse().matrix();
  Eigen::Matrix3d trans_cov =
      R_vw * nav_state.pose_cov.topLeftCorner(3, 3) * R_vw.transpose();
  Eigen::Matrix2d trans_pt_cov_2d = trans_cov.topLeftCorner(2, 2);
  if (trans_pt_cov_2d(0, 0) > 1e4 || trans_pt_cov_2d(1, 1) > 1e4 ||
      trans_pt_cov_2d(0, 0) <= 0.0 || trans_pt_cov_2d(1, 1) <= 0.0) {
    // pose covariance is invalid
    trans_pt_cov_2d = Eigen::Matrix2d::Identity();
    trans_pt_cov_2d *= 1e4;
  }
  double heading_cov = nav_state.pose_cov(5, 5);
  if (heading_cov <= 0.0 || heading_cov > 0.1) {
    // heading covariance is invalid
    heading_cov = 0.1;
  }
  for (auto& item : (*percept_lls)) {
    auto& bv_points = item.second.processed_bv_points;
    if (bv_points.empty()) continue;
    item.second.processed_bv_point_covs.clear();
    item.second.processed_bv_point_covs.reserve(bv_points.size());
    for (size_t i = 0; i < bv_points.size(); ++i) {
      Point3D_t bv_pt = bv_points[i];
      // propagate heading covariance of vehicle to projected point
      Eigen::Matrix2d rotate_pt_cov_2d;
      rotate_pt_cov_2d(0, 0) = heading_cov * bv_pt.y * bv_pt.y;
      rotate_pt_cov_2d(0, 1) = -heading_cov * bv_pt.x * bv_pt.y;
      rotate_pt_cov_2d(1, 0) = rotate_pt_cov_2d(0, 1);
      rotate_pt_cov_2d(1, 1) = heading_cov * bv_pt.x * bv_pt.x;

      // homography projection related covariance, give a default value
      double D2 = bv_pt.SquaredNorm2D();
      Eigen::Matrix2d homo_project_pt_cov = Eigen::Matrix2d::Zero();
      homo_project_pt_cov(0, 0) = homo_proj_cov_coeff * D2;
      homo_project_pt_cov(1, 1) = homo_proj_cov_coeff * D2;

      Eigen::Matrix2d bv_pt_cov =
          trans_pt_cov_2d + rotate_pt_cov_2d + homo_project_pt_cov;
      item.second.processed_bv_point_covs.emplace_back(bv_pt_cov);
    }
  }

  // // make the linked relation for laneline and roadside
  // std::vector<id_t> laneline_ids, roadside_ids;
  // for (const auto& item : (*percept_lls)) {
  //     if (item.second.line_type == LineType::LaneMarking) {
  //         laneline_ids.emplace_back(item.first);
  //     } else if (item.second.line_type == LineType::Curb) {
  //         roadside_ids.emplace_back(item.first);
  //     }
  // }
  // for (const auto& rs_id : roadside_ids) {
  //     const auto& rs_points = percept_lls->at(rs_id).processed_bv_points;
  //     id_t min_ll_id;
  //     double min_distance = std::numeric_limits<double>::max();
  //     for (const auto& ll_id : laneline_ids) {
  //         const auto& ll_points =
  //         percept_lls->at(ll_id).processed_bv_points;
  //         int valid_num = 0;
  //         double dist = Line2LineDistance2D(rs_points, ll_points,
  //         &valid_num);
  //         if (valid_num > 3 && dist < min_distance) {
  //             min_distance = dist;
  //             min_ll_id = ll_id;
  //         }
  //     }
  //     if (min_distance < 0.5) {
  //         percept_lls->at(rs_id).linked_id = min_ll_id;
  //     }
  // }

  return LOC_SUCCESS;
}

adLocStatus_t Frame::EstimateLaneLineVanishingPoint() {
  CameraModel::Ptr cam_model = nullptr;
  auto status =
      Configure::GetInstance()->GetCameraModel(camera_name_, &cam_model);
  if (status != LOC_SUCCESS) return status;
  int camera_width = cam_model->GetCameraWidth();
  int camera_height = cam_model->GetCameraHeight();
  double sample_ratio = 0.6;
  if (camera_name_.find("30") != std::string::npos) {
    sample_ratio = 0.4;
  }

  std::unordered_map<id_t, std::vector<Point2D_t>> laneline_img_pts;
  for (const auto& item : perception_data_->lane_lines) {
    if (item.second.img_pts.empty()) continue;
    std::vector<Point2D_t> filtered_img_pts;
    filtered_img_pts.reserve(item.second.img_pts.size());
    int sample_num =
        static_cast<int>(item.second.img_pts.size() * sample_ratio + 0.5);
    for (const auto& pt : item.second.img_pts) {
      if (pt.y > camera_height - 10 || pt.x < 10 || pt.x > camera_width - 10)
        continue;
      filtered_img_pts.emplace_back(pt);
      if (filtered_img_pts.size() > sample_num) break;
    }
    if (filtered_img_pts.size() > 5) {
      laneline_img_pts.insert(
          std::make_pair(item.first, std::move(filtered_img_pts)));
    }
  }

  std::unordered_map<id_t, Eigen::Vector2d> line_coefficients;
  for (const auto& item : laneline_img_pts) {
    Eigen::VectorXd coefficient;
    if (PolyLineFit(item.second, 1, &coefficient)) {
      // check straight line fitting is valid
      double diff_sum = 0.0;
      for (const auto& pt : item.second) {
        double new_pt_y = coefficient(0) * pt.x + coefficient(1);
        double origin_pt_y = pt.y;
        diff_sum += std::fabs(new_pt_y - origin_pt_y);
      }
      double mean_diff = diff_sum / item.second.size();
      if (mean_diff < 1.2) {
        line_coefficients.insert(std::make_pair(
            item.first, Eigen::Vector2d{coefficient(0), coefficient(1)}));
      }
    }
  }
  if (line_coefficients.size() < 2) {
    LC_LDEBUG(FRAME) << "too few valid lines to find vanishing point!";
    return LOC_INVALID;
  }

  Eigen::MatrixXd A(line_coefficients.size(), 2);
  Eigen::VectorXd b(line_coefficients.size());
  int index = 0;
  for (const auto& item : line_coefficients) {
    A(index, 0) = item.second(0);
    A(index, 1) = -1.0;
    b(index) = -item.second(1);
    ++index;
  }
  Eigen::VectorXd vanishing_point = A.householderQr().solve(b);

  // check vanishing point is valid
  double max_dis = 0.0;
  double sum_dis = 0.0;
  for (const auto& item : line_coefficients) {
    Eigen::Vector2d param = item.second;
    Point2D_t pt1(vanishing_point(0) + 5,
                  item.second(0) * (vanishing_point(0) + 5) + item.second(1));
    Point2D_t pt2(vanishing_point(0) - 5,
                  item.second(0) * (vanishing_point(0) - 5) + item.second(1));
    double dis = Point2LineDist2D(
        Point2D_t(vanishing_point(0), vanishing_point(1)), pt1, pt2);
    sum_dis += dis;
    if (dis > max_dis) max_dis = dis;
  }
  double mean_dis = sum_dis / line_coefficients.size();
  if (mean_dis > 10) {
    LC_LDEBUG(FRAME) << "invalid vanishing point, max/mean dist(pixel) "
                     << max_dis << " " << mean_dis;
    return LOC_INVALID;
  }

  vanishing_point_ = vanishing_point;
  return LOC_SUCCESS;
}

adLocStatus_t Frame::CalibHomographyPitchByVanishingPoint() {
  // get intrinsic/extrinsic
  adLocStatus_t status;
  SE3d Tgc;
  status =
      Configure::GetInstance()->GetCameraToGroundExtrinsic(camera_name_, &Tgc);
  if (status != LOC_SUCCESS) return status;

  // estimate laneline vanishing point
  status = EstimateLaneLineVanishingPoint();
  if (status != LOC_SUCCESS) return status;

  // get initial camera to ground roll
  Eigen::Vector3d ypr_rdf_flu(0, -M_PI_2, M_PI_2);
  // SO3d Rrdf_flu(ypr_rdf_flu); old Sophus
  SO3d Rrdf_flu = SO3d::exp(ypr_rdf_flu);
  SO3d Rgc_rdf = Rrdf_flu * Tgc.so3();
  // Eigen::Vector3d init_ypr = Rgc_rdf.toYPR(); old Sophus
  Eigen::Vector3d init_ypr =
      Rgc_rdf.unit_quaternion().toRotationMatrix().eulerAngles(2, 1, 0);
  double init_roll = init_ypr(2);

  // NOTE: actually roll of camera!
  Eigen::Vector3d laneline_direction =
      K_inv_ * Eigen::Vector3d(vanishing_point_(0), vanishing_point_(1), 1);
  laneline_direction.normalize();
  double curr_roll = std::asin(laneline_direction(1));

  double diff_roll = NormalizeAngleDiff(curr_roll - init_roll);
  if (std::fabs(diff_roll) > 0.6 * M_PI / 180.0) {
    LC_LDEBUG(FRAME) << "large homo pitch from vanishing point";
    diff_roll = diff_roll / std::fabs(diff_roll) * 0.6 * M_PI / 180.0;
  }
  SetHomoPitch(diff_roll);
  LC_LDEBUG(FRAME) << camera_name_ << " intial homo pitch(deg) "
                   << diff_roll * 180 / M_PI;
  return LOC_SUCCESS;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
