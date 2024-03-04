// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: lane_point_filter.cc
// @brief: filter 3d points

#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/data_fusion/roadedge_point_filter.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <utility>

#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/base/curve_fitter.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"
// #include "perception-common/common/performance/perf_util.h"
namespace hozon {
namespace mp {
namespace environment {

bool RoadEdgePointFilter::Init(const LanePointFilterInitOptions& init_options) {
  // 点更新管理器初始化
  point_manager_ = std::make_shared<AdaptorPointManager>();

  // 卡尔曼参数矩阵设置
  P_.setIdentity();
  P_ = P_ * init_p_;

  A_.setZero();
  H_.setZero();
  HZ_.setZero();
  Eigen::VectorXd r_noise_list(20);
  Eigen::VectorXd q_noise_list(40);
  for (int i = 0; i < 20; ++i) {
    r_noise_list(i) = 0.1 + 0.1 * (i / 2);
  }
  for (int i = 0; i < 40; ++i) {
    q_noise_list(i) = 0.01 + 0.01 * (i / 2);
  }
  Eigen::MatrixXd diagonalMatrix = r_noise_list.asDiagonal();
  R_.setIdentity();
  R_ = R_ * diagonalMatrix;
  diagonalMatrix = q_noise_list.asDiagonal();
  Q_.setIdentity();
  Q_ = Q_ * diagonalMatrix;
  // std::cout << "R_ data:" << R_.matrix() << std::endl;

  // 给X矩阵赋初始值（跟踪点）
  auto& track_point_set =
      lane_target_ref_->GetConstTrackedLaneLine()->point_set;
  ConvertPointSet2Eigen(track_point_set, &X_);

  // 构建A矩阵
  CreateAMatrix();

  // lane_curve_fitter_param
  lane_point_filter_param_ = init_options.lane_point_filter_param;
  pyfilt_.reset(BaseCurveFitterRegisterer::GetInstanceByName(
      lane_point_filter_param_.lane_curve_fitter_param().curve_function()));
  BaseCurveFitterInitOptions curve_fitter_init_options;
  curve_fitter_init_options.avg_dist_thresh =
      lane_point_filter_param_.lane_curve_fitter_param().avg_dist_thresh();
  curve_fitter_init_options.max_dist_thresh =
      lane_point_filter_param_.lane_curve_fitter_param().max_dist_thresh();
  curve_fitter_init_options.ransac_select_ratio =
      lane_point_filter_param_.lane_curve_fitter_param().ransac_select_ratio();
  curve_fitter_init_options.ransac_max_select_pt_num =
      lane_point_filter_param_.lane_curve_fitter_param()
          .ransac_max_select_pt_num();
  curve_fitter_init_options.ransac_epoch_num =
      lane_point_filter_param_.lane_curve_fitter_param().ransac_epoch_num();
  curve_fitter_init_options.do_normalization =
      lane_point_filter_param_.lane_curve_fitter_param().do_normalization();
  curve_fitter_init_options.do_extend_point =
      lane_point_filter_param_.lane_curve_fitter_param().do_extend_point();
  // For segment curve fitting
  curve_fitter_init_options.fix_param_xmin =
      lane_point_filter_param_.lane_curve_fitter_param().fix_param_xmin();
  curve_fitter_init_options.linear_select_ratio =
      lane_point_filter_param_.lane_curve_fitter_param().linear_select_ratio();
  curve_fitter_init_options.linear_xmin =
      lane_point_filter_param_.lane_curve_fitter_param().linear_xmin();
  curve_fitter_init_options.linear_xmax =
      lane_point_filter_param_.lane_curve_fitter_param().linear_xmax();
  curve_fitter_init_options.linear_len =
      lane_point_filter_param_.lane_curve_fitter_param().linear_len();
  curve_fitter_init_options.quadratic_len =
      lane_point_filter_param_.lane_curve_fitter_param().quadratic_len();
  pyfilt_->Init(curve_fitter_init_options);

  // 设置LaneLinePolynomial参数
  polynomial_ = std::make_shared<LaneLinePolynomial>();
  polynomial_->order = lane_point_filter_param_.curve_fitting_order();

  return true;
}

void RoadEdgePointFilter::CreateAMatrix() {
  const double cur_point_w1 = 1.0;
  const double cur_point_w2 = 0.55;
  const double cur_point_w3 = 0.5;
  const double adjcent_point_w = 0.2;
  const double adjcent2_point_w = 0.05;
  const double opposite_coord_w = 0.0;

  // const double cur_point_w2 = 0.75;
  // const double cur_point_w3 = 0.7;
  // const double adjcent_point_w = 0.1;
  // const double adjcent2_point_w = 0.05;
  // const double opposite_coord_w = 0.0;

  Eigen::VectorXd matrix_1(6);
  matrix_1 << cur_point_w1, opposite_coord_w, opposite_coord_w,
      opposite_coord_w, opposite_coord_w, opposite_coord_w;
  Eigen::VectorXd matrix_2(8);
  matrix_2 << adjcent_point_w, opposite_coord_w, cur_point_w2, opposite_coord_w,
      adjcent_point_w, opposite_coord_w, adjcent2_point_w, opposite_coord_w;
  Eigen::VectorXd matrix_3(10);
  matrix_3 << adjcent2_point_w, opposite_coord_w, adjcent_point_w,
      opposite_coord_w, cur_point_w3, opposite_coord_w, adjcent_point_w,
      opposite_coord_w, adjcent2_point_w, opposite_coord_w;
  Eigen::VectorXd matrix_4(8);
  matrix_4 << adjcent2_point_w, opposite_coord_w, adjcent_point_w,
      opposite_coord_w, cur_point_w2, opposite_coord_w, adjcent_point_w,
      opposite_coord_w;
  Eigen::VectorXd matrix_5(5);
  matrix_5 << opposite_coord_w, opposite_coord_w, opposite_coord_w,
      opposite_coord_w, cur_point_w1;
  Eigen::VectorXd matrix_6(9);
  matrix_6 << adjcent2_point_w, opposite_coord_w, adjcent_point_w,
      opposite_coord_w, cur_point_w3, opposite_coord_w, adjcent_point_w,
      opposite_coord_w, adjcent2_point_w;
  Eigen::VectorXd matrix_7(7);
  matrix_7 << adjcent2_point_w, opposite_coord_w, adjcent_point_w,
      opposite_coord_w, cur_point_w2, opposite_coord_w, adjcent_point_w;
  for (int i = 0; i < pt_size_; i++) {
    if (i == 0) {
      A_.block(i, i, 1, 6) = matrix_1.transpose();
      A_.block(i + 1, i + 1, 1, 6) = matrix_1.transpose();
    } else if (i == 1) {
      A_.block(i * 2, i * 2 - 2, 1, 8) = matrix_2.transpose().replicate(1, 1);
      A_.block(i * 2 + 1, i * 2 - 2 + 1, 1, 8) =
          matrix_2.transpose().replicate(1, 1);
    } else if (i == pt_size_ - 3) {
      A_.block(i * 2, i * 2 - 4, 1, 9) = matrix_6.transpose().replicate(1, 1);
      A_.block(i * 2 + 1, i * 2 - 4 + 1, 1, 9) =
          matrix_6.transpose().replicate(1, 1);
    } else if (i == pt_size_ - 2) {
      A_.block(i * 2, i * 2 - 4, 1, 7) = matrix_7.transpose().replicate(1, 1);
      A_.block(i * 2 + 1, i * 2 - 4 + 1, 1, 7) =
          matrix_7.transpose().replicate(1, 1);
    } else if (i == pt_size_ - 1) {
      A_.block(i * 2, i * 2 - 4, 1, 5) = matrix_5.transpose().replicate(1, 1);
      A_.block(i * 2 + 1, i * 2 - 4 + 1, 1, 5) =
          matrix_5.transpose().replicate(1, 1);
    } else {
      A_.block(i * 2, i * 2 - 4, 1, 10) = matrix_3.transpose().replicate(1, 1);
      A_.block(i * 2 + 1, i * 2 - 4 + 1, 1, 10) =
          matrix_3.transpose().replicate(1, 1);
    }
  }
}

void RoadEdgePointFilter::RevisePredictFit() {
  Eigen::Matrix<double, 40, 1> XT = A_ * X_;
  B_.setIdentity();
  // 跟踪点曲线拟合
  CurveFitter curve_fitter(2);
  std::vector<Eigen::Vector2d> points;
  points.resize(4);
  const auto& current_pose =
      InputDataSingleton::Instance()->dr_data_buffer_.back()->pose;
  // 跟踪点转到local系保存
  std::vector<Eigen::Vector2d> vec_vehicle_pt;
  for (int i = 0; i < pt_size_; ++i) {
    Eigen::Vector3d local_point(X_[i * 2], X_[i * 2 + 1], 0.0);
    auto vehicle_pt = current_pose.inverse() * local_point;
    vec_vehicle_pt.emplace_back(
        Eigen::Vector2d(vehicle_pt.x(), vehicle_pt.y()));
  }
  for (int i = 1; i < pt_size_ - 2; ++i) {
    // 取四个跟踪点求二次曲线方程
    points.clear();
    int start = (i == pt_size_ - 2) ? i - 2 : i - 1;
    int end = (i == pt_size_ - 2) ? i + 1 : i + 2;
    for (int j = start; j <= end; ++j) {
      points.emplace_back(vec_vehicle_pt[j]);
    }
    if (!curve_fitter.PolyFitProcess(points)) {
      continue;
    }
    // 根据XT横坐标计算二次曲线上y
    float wp_x = XT[i * 2], wp_y = XT[i * 2 + 1];
    Eigen::Vector3d weight_local_pt(wp_x, wp_y, 0.0);
    auto vehicle_pt = current_pose.inverse() * weight_local_pt;
    float vehicle_y = curve_fitter.evalueValue(vehicle_pt.x());
    auto local_pt =
        current_pose * Eigen::Vector3d(vehicle_pt.x(), vehicle_y, 0.0);
    float wp_y_t = local_pt.y();
    XT[i * 2 + 1] = wp_y_t;
    float ratio = wp_y_t / wp_y;
    B_(i * 2 + 1, i * 2 + 1) = ratio;
  }
  return;
}

void RoadEdgePointFilter::CalculateNormalV2() {
  for (int i = 1; i < pt_size_ - 1; ++i) {
    // 取三个跟踪点求圆心和半径
    float x1 = X_[i * 2 - 2], y1 = X_[i * 2 - 1];
    float x2 = X_[i * 2], y2 = X_[i * 2 + 1];
    float x3 = X_[i * 2 + 2], y3 = X_[i * 2 + 3];
    // 点离得很近不做计算
    if (fabs(x1 - x2) < 1.0 || fabs(x2 - x3) < 1.0 || fabs(y2 - y1) < 0.01 ||
        fabs(y3 - y2) < 0.01) {
      continue;
    }
    float ma = (y2 - y1) / (x2 - x1);
    float mb = (y3 - y2) / (x3 - x2);
    // 三点近似直线不做计算
    if (fabs(mb - ma) < 1e-4) {
      continue;
    }
    // 求圆心坐标
    float centerX = (ma * mb * (y1 - y3) + mb * (x1 + x2) - ma * (x2 + x3)) /
                    (2 * (mb - ma));
    float centerY = ((y1 + y2) / 2) - (centerX - (x1 + x2) / 2) / ma;
    // 求三个点每个点的法向量
    for (int j = i - 1; j <= i + 1; j++) {
      float normal_x = centerX - X_[j * 2];
      float normal_y = centerY - X_[j * 2 + 1];
      float mag = sqrt(normal_y * normal_y + normal_x * normal_x);
      normal_x = normal_x / mag;
      normal_y = normal_y / mag;
      X_NORMAL_[2 * j + 0] = normal_x;
      X_NORMAL_[2 * j + 1] = normal_y;
    }
  }
  return;
}

bool RoadEdgePointFilter::IsAbnormalPose(
    const Eigen::Affine3d& novatel2world_pose) {
  const auto& current_pose =
      InputDataSingleton::Instance()->dr_data_buffer_.back()->pose;
  const auto& last_pose =
      InputDataSingleton::Instance()->dr_data_buffer_.back2()->pose;
  Eigen::Affine3d delta_pose = last_pose.inverse() * novatel2world_pose;
  // yaw(z) picth(y) roll(x)
  Eigen::Vector3d euler_angles = delta_pose.linear().eulerAngles(2, 1, 0);
  float yaw =
      std::min(std::min(fabs(euler_angles[0]), fabs(euler_angles[0] - M_PI)),
               fabs(euler_angles[0] + M_PI));
  float pitch =
      std::min(std::min(fabs(euler_angles[1]), fabs(euler_angles[1] - M_PI)),
               fabs(euler_angles[1] + M_PI));
  float roll =
      std::min(std::min(fabs(euler_angles[2]), fabs(euler_angles[2] - M_PI)),
               fabs(euler_angles[2] + M_PI));

  bool is_angle_big_change =
      (yaw * 180 / M_PI >
       lane_point_filter_param_.max_delta_change_for_pose_angle()) ||
      (pitch * 180 / M_PI >
       lane_point_filter_param_.max_delta_change_for_pose_angle()) ||
      (roll * 180 / M_PI >
       lane_point_filter_param_.max_delta_change_for_pose_angle());

  float lateral_postion_error = delta_pose.translation()[1];
  bool is_position_big_change =
      fabs(lateral_postion_error) >
      lane_point_filter_param_.max_delta_change_for_pose_position();

  bool is_pose_error = is_angle_big_change || is_position_big_change;
  if (is_pose_error) {
    HLOG_ERROR << " [RoadEdgePointFilter]: "
               << " IsAbnormalPose: "
               << " is_angle_big_change, " << is_angle_big_change
               << " is_position_big_change, " << is_position_big_change
               << " Current pose error, yaw:" << yaw * 180 / M_PI
               << ", pitch: " << pitch * 180 / M_PI
               << ", roll: " << roll * 180 / M_PI
               << ", lateral_position_error: " << lateral_postion_error;
  }
  return is_pose_error;
}

void RoadEdgePointFilter::UpdateWithMeasurement(
    const LaneFilterOptions& filter_options,
    const perception_base::RoadEdgeMeasurementPtr& measurement) {
  point_manager_->AddObservePoints(measurement->point_set);
  // PERF_FUNCTION();
  // PERF_BLOCK_START();
  // 如果前后帧姿态异常，则不做任何处理。
  if (IsAbnormalPose(filter_options.novatel2world_pose)) {
    return;
  }
  PredictStage();
  // PERF_BLOCK_END("PredictStage Used Time");
  UpdateStage(measurement);
  // PERF_BLOCK_END("UpdateStage Used Time");
  UpdateResult();
  // PERF_BLOCK_END("UpdateResult Used Time");
  return;
}

void RoadEdgePointFilter::UpdateWithoutMeasurement(
    const LaneFilterOptions& filter_options) {
  // 没有检测时， 保持上一帧结果进行输出。
  // PredictStage();
  // LANE_PREDICT_POINT_DEBUG_INFO(X_);
  UpdateResult();
  return;
}

bool RoadEdgePointFilter::ConvertPointSet2Eigen(
    const std::vector<perception_base::LaneLinePoint>& point_set,
    Eigen::Matrix<double, 40, 1>* eigen_vector) {
  for (size_t p_idx = 0; p_idx < point_set.size(); ++p_idx) {
    auto& point = point_set[p_idx];
    auto local_point = point.local_point;
    (*eigen_vector)(p_idx * 2, 0) = local_point.x;
    (*eigen_vector)(p_idx * 2 + 1, 0) = local_point.y;
  }
  return true;
}

bool RoadEdgePointFilter::ConvertEigen2PointSet(
    const Eigen::Matrix<double, 40, 1>& eigen_vector,
    std::vector<perception_base::LaneLinePoint>* point_set) {
  assert(eigen_vector.rows() / 2 == point_set->size());

  auto& current_pose =
      InputDataSingleton::Instance()->dr_data_buffer_.back()->pose;

  Eigen::Vector3d local_pt(0.0, 0.0, 0.0);
  Eigen::Vector3d vehicle_pt(0.0, 0.0, 0.0);
  for (size_t idx = 0; idx < point_set->size(); ++idx) {
    auto& point = point_set->at(idx);
    point.local_point.x = eigen_vector[2 * idx];
    point.local_point.y = eigen_vector[2 * idx + 1];
    point.local_point.z = 0.0;

    local_pt[0] = point.local_point.x;
    local_pt[1] = point.local_point.y;
    local_pt[2] = point.local_point.z;

    vehicle_pt = current_pose.inverse() * local_pt;
    point.vehicle_point.x = vehicle_pt.x();
    point.vehicle_point.y = vehicle_pt.y();
    point.vehicle_point.z = vehicle_pt.z();
  }
  return true;
}

void RoadEdgePointFilter::CalculateNormal() {
  X_NORMAL_.setZero();
  for (size_t idx = 1; idx < pt_size_ - 1; ++idx) {
    auto bottom_idx = idx - 1;
    auto top_idx = idx + 1;
    auto bottom_x = X_[2 * bottom_idx + 0];
    auto bottom_y = X_[2 * bottom_idx + 1];
    auto top_x = X_[2 * top_idx + 0];
    auto top_y = X_[2 * top_idx + 1];
    float normal_x = bottom_y - top_y;
    float normal_y = top_x - bottom_x;
    float mag = sqrt(normal_y * normal_y + normal_x * normal_x);
    normal_x = normal_x / mag;
    normal_y = normal_y / mag;
    X_NORMAL_[2 * idx + 0] = normal_x;
    X_NORMAL_[2 * idx + 1] = normal_y;
  }

  size_t first_index = 0;
  X_NORMAL_[2 * first_index + 0] = X_NORMAL_[2 * (first_index + 1) + 0];
  X_NORMAL_[2 * first_index + 1] = X_NORMAL_[2 * (first_index + 1) + 1];

  size_t last_index = pt_size_ - 1;
  X_NORMAL_[2 * last_index + 0] = X_NORMAL_[2 * (last_index - 1) + 0];
  X_NORMAL_[2 * last_index + 1] = X_NORMAL_[2 * (last_index - 1) + 1];

  return;
}

void RoadEdgePointFilter::KalmanUpdate(
    const std::vector<perception_base::LaneLinePoint>& measurement_points) {
  CalculateNormal();
  CalculateNormalV2();

  H_.setZero();
  HZ_.setZero();

  int match_idxs = 0;
  // 观测点和跟踪点进行匹配操作
  for (size_t m_p_idx = 0; m_p_idx < measurement_points.size(); ++m_p_idx) {
    for (size_t t_p_idx = 0; t_p_idx < pt_size_ - 1; ++t_p_idx) {
      Eigen::Vector2f t_point(X_[2 * t_p_idx + 0], X_[2 * t_p_idx + 1]);
      Eigen::Vector2f t_top_point(X_[2 * (t_p_idx + 1) + 0],
                                  X_[2 * (t_p_idx + 1) + 1]);
      Eigen::Vector2f BA = t_top_point - t_point;
      Eigen::Vector2f m_point(measurement_points[m_p_idx].local_point.x,
                              measurement_points[m_p_idx].local_point.y);
      Eigen::Vector2f BP = m_point - t_point;
      Eigen::Vector2f AP = m_point - t_top_point;

      if (BP.norm() < 5 && BP.transpose().dot(BA) >= -0.0001 &&
          AP.transpose().dot(BA) <= 0.0001) {
        H_(m_p_idx, t_p_idx * 2) -= X_NORMAL_[t_p_idx * 2];
        H_(m_p_idx, t_p_idx * 2 + 1) -= X_NORMAL_[t_p_idx * 2 + 1];
        HLOG_DEBUG << "normal value:" << X_NORMAL_[t_p_idx * 2] << ","
                   << X_NORMAL_[t_p_idx * 2 + 1];

        HZ_[m_p_idx] =
            (m_point[0] - X_[t_p_idx * 2 + 0]) * X_NORMAL_[t_p_idx * 2 + 0] +
            (m_point[1] - X_[t_p_idx * 2 + 1]) * X_NORMAL_[t_p_idx * 2 + 1];
        match_idxs++;
        break;
      }
    }
  }
  // 卡尔曼更新
  auto k = P_ * H_.transpose() * ((H_ * P_ * H_.transpose() + R_).inverse());

  auto& x_gain = k * (-HZ_);
  // for (int i = 0; i < x_gain.size() / 2; ++i) {
  //     HLOG_DEBUG << "X_GAIN:" << i << " == "  << x_gain[i * 2] << "," <<
  //     x_gain[i * 2 + 1];
  // }
  X_ = X_ + k * (-HZ_);
  P_ = P_ - k * H_ * P_;
}

void RoadEdgePointFilter::UpdateStage(
    const perception_base::RoadEdgeMeasurementPtr& measurement_line) {
  auto& measurement_points = measurement_line->point_set;

  // PERF_FUNCTION();
  // PERF_BLOCK_START();
  // 删除车身后的点， 并同步更新到X_和P_。

  // point_manager_->Process(&X_, &P_);

  point_manager_->Process(&X_, &P_);

  // PERF_BLOCK_END("UpdatePoints Used Time");

  KalmanUpdate(measurement_points);
  // PERF_BLOCK_END("KalmanUpdate Used Time");
}

void RoadEdgePointFilter::PredictStage() {
  RevisePredictFit();
  A_update_ = B_ * A_;
  X_ = A_update_ * X_;
  P_ = A_update_ * P_ * A_update_.transpose() + Q_;
}

void RoadEdgePointFilter::UpdateResult() {
  auto& tracked_pt = lane_target_ref_->GetTrackedLaneLine()->point_set;
  // 将X_点存入跟踪线数据结构中进行发送
  ConvertEigen2PointSet(X_, &tracked_pt);
  // for (auto& point : tracked_pt) {
  //   HLOG_DEBUG << "local x: " << point.local_point.x << " , y: " <<
  //   point.local_point.y;
  // }

  // 拿车身坐标系下的点进行三次方程拟合
  auto& track_polynomial =
      lane_target_ref_->GetTrackedLaneLine()->vehicle_curve;
  std::vector<perception_base::Point3DF> lane_vehicle_pts;
  lane_vehicle_pts.clear();
  for (const auto& lane_point : tracked_pt) {
    lane_vehicle_pts.push_back(lane_point.vehicle_point);
  }

  if (!pyfilt_->CurveFitting(lane_vehicle_pts, polynomial_)) {
    HLOG_ERROR << "LaneLinePolynomial fitting failed.";
    return;
  }

  track_polynomial.min = polynomial_->min;
  track_polynomial.max = polynomial_->max;
  track_polynomial.coeffs.resize(4);
  track_polynomial.coeffs[0] = polynomial_->params[0];
  track_polynomial.coeffs[1] = polynomial_->params[1];
  track_polynomial.coeffs[2] = polynomial_->params[2];
  track_polynomial.coeffs[3] = polynomial_->params[3];
  // 三次方程拟合，给到下游参考使用
}

void RoadEdgePointFilter::Reset() { return; }

}  // namespace environment
}  // namespace mp
}  // namespace hozon
