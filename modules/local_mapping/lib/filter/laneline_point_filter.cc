// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: lane_point_filter.cc
// @brief: kalman filter points

#include "modules/local_mapping/lib/filter/laneline_point_filter.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <utility>

#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace lm {

bool LaneLinePointFilter::Init(const FilterInitOption& init_options) {
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
  auto& track_point_set = target_ref_->GetConstTrackedObject()->world_points;
  ConvertPointSet2Eigen(track_point_set, &X_);

  // 构建A矩阵
  CreateAMatrix();

  return true;
}

void LaneLinePointFilter::CreateAMatrix() {
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

void LaneLinePointFilter::RevisePredictFit() {
  Eigen::Matrix<double, 40, 1> XT = A_ * X_;
  B_.setIdentity();
  // 跟踪点曲线拟合
  CurveFitter curve_fitter(2);
  std::vector<Eigen::Vector2d> points;
  points.resize(4);
  const auto& current_pose = PoseManager::Instance()->GetCurrentPose();
  // 跟踪点转到local系保存
  std::vector<Eigen::Vector2d> vec_vehicle_pt;
  for (int i = 0; i < pt_size_; ++i) {
    Eigen::Vector3d local_point(X_[i * 2], X_[i * 2 + 1], 0.0);
    auto vehicle_pt = current_pose.inverse() * local_point;
    vec_vehicle_pt.emplace_back(vehicle_pt.x(), vehicle_pt.y());
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
    double vehicle_y = curve_fitter.evalueValue(vehicle_pt.x());
    // 异常情况兜底
    if (std::abs(vehicle_y - vehicle_pt.y()) > 1.0) {
      vehicle_y = vehicle_pt.y();
    }
    auto local_pt =
        current_pose * Eigen::Vector3d(vehicle_pt.x(), vehicle_y, 0.0);
    auto wp_x_t = static_cast<float>(local_pt.x());
    auto wp_y_t = static_cast<float>(local_pt.y());
    XT[i * 2] = wp_x_t;
    XT[i * 2 + 1] = wp_y_t;
    float ratio_x = std::abs(wp_x) < 1e-3 ? 1.0 : wp_x_t / wp_x;
    B_(i * 2, i * 2) = ratio_x;
    float ratio_y = std::abs(wp_y) < 1e-3 ? 1.0 : wp_y_t / wp_y;
    B_(i * 2 + 1, i * 2 + 1) = ratio_y;
  }
  return;
}

void LaneLinePointFilter::CalculateNormalV2() {
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
      normal_x = mag < 1e-4 ? 0.0 : normal_x / mag;
      normal_y = mag < 1e-4 ? 0.0 : normal_y / mag;
      X_NORMAL_[2 * j + 0] = normal_x;
      X_NORMAL_[2 * j + 1] = normal_y;
    }
  }
  return;
}

bool LaneLinePointFilter::IsAbnormalPose(
    const Eigen::Affine3d& novatel2world_pose) {
  Eigen::Affine3d delta_pose = PoseManager::Instance()->GetDeltaPose();
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

  // todo 配置化参数
  // bool is_angle_big_change =
  //     (yaw * 180 / M_PI >
  //      lane_point_filter_param_.max_delta_change_for_pose_angle()) ||
  //     (pitch * 180 / M_PI >
  //      lane_point_filter_param_.max_delta_change_for_pose_angle()) ||
  //     (roll * 180 / M_PI >
  //      lane_point_filter_param_.max_delta_change_for_pose_angle());
  bool is_angle_big_change = false;
  float lateral_postion_error = delta_pose.translation()[1];
  // bool is_position_big_change =
  //     fabs(lateral_postion_error) >
  //     lane_point_filter_param_.max_delta_change_for_pose_position();
  bool is_position_big_change = false;

  bool is_pose_error = is_angle_big_change || is_position_big_change;
  if (is_pose_error) {
    HLOG_ERROR << " [LaneLinePointFilter]: "
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

void LaneLinePointFilter::UpdateWithMeasurement(
    const FilterOption& filter_options, const LaneLinePtr& measurement) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();
  // 如果前后帧姿态异常，则不做任何处理。
  if (IsAbnormalPose(PoseManager::Instance()->GetCurrentPose())) {
    return;
  }
  PredictStage();
  // PERF_BLOCK_END("PredictStage Used Time");
  UpdateStage(measurement);
  // PERF_BLOCK_END("UpdateStage Used Time");
  UpdateResult(true);
  // PERF_BLOCK_END("UpdateResult Used Time");
  return;
}

void LaneLinePointFilter::UpdateWithoutMeasurement(
    const FilterOption& filter_options) {
  // 没有检测时， 保持上一帧结果进行输出。
  // PredictStage();
  // LANE_PREDICT_POINT_DEBUG_INFO(X_);
  UpdateResult(false);
  return;
}

bool LaneLinePointFilter::ConvertPointSet2Eigen(
    const std::vector<Eigen::Vector3d>& point_set,
    Eigen::Matrix<double, 40, 1>* eigen_vector) {
  for (int64_t p_idx = 0; p_idx < point_set.size(); ++p_idx) {
    const auto& point = point_set[p_idx];
    (*eigen_vector)(p_idx * 2, 0) = point.x();
    (*eigen_vector)(p_idx * 2 + 1, 0) = point.y();
  }
  return true;
}

bool LaneLinePointFilter::ConvertEigen2PointSet(
    const Eigen::Matrix<double, 40, 1>& eigen_vector,
    const LaneLinePtr& track_lane) {
  track_lane->vehicle_points.resize(20);
  track_lane->world_points.resize(20);
  assert(eigen_vector.rows() / 2 == track_lane->vehicle_points.size());

  const auto& current_pose = PoseManager::Instance()->GetCurrentPose();

  for (int64_t idx = 0; idx < track_lane->world_points.size(); ++idx) {
    auto& loca_point = track_lane->world_points[idx];
    auto& vehicle_point = track_lane->vehicle_points[idx];
    loca_point[0] = eigen_vector[2 * idx];
    loca_point[1] = eigen_vector[2 * idx + 1];
    loca_point[2] = 0.0;
    vehicle_point = current_pose.inverse() * loca_point;
  }
  return true;
}

void LaneLinePointFilter::CalculateNormal() {
  X_NORMAL_.setZero();
  for (int64_t idx = 1; idx < pt_size_ - 1; ++idx) {
    auto bottom_idx = idx - 1;
    auto top_idx = idx + 1;
    auto bottom_x = X_[2 * bottom_idx + 0];
    auto bottom_y = X_[2 * bottom_idx + 1];
    auto top_x = X_[2 * top_idx + 0];
    auto top_y = X_[2 * top_idx + 1];
    float normal_x = bottom_y - top_y;
    float normal_y = top_x - bottom_x;
    float mag = sqrt(normal_y * normal_y + normal_x * normal_x);
    normal_x = mag < 1e-4 ? 0.0 : normal_x / mag;
    normal_y = mag < 1e-4 ? 0.0 : normal_y / mag;
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

void LaneLinePointFilter::KalmanUpdate(
    const std::vector<Eigen::Vector3d>& measurement_points) {
  CalculateNormal();
  CalculateNormalV2();

  H_.setZero();
  HZ_.setZero();

  // 观测点和跟踪点进行匹配操作
  for (int64_t m_p_idx = 0; m_p_idx < measurement_points.size(); ++m_p_idx) {
    for (int64_t t_p_idx = 0; t_p_idx < pt_size_ - 1; ++t_p_idx) {
      Eigen::Vector2f t_point(X_[2 * t_p_idx + 0], X_[2 * t_p_idx + 1]);
      Eigen::Vector2f t_top_point(X_[2 * (t_p_idx + 1) + 0],
                                  X_[2 * (t_p_idx + 1) + 1]);
      Eigen::Vector2f BA = t_top_point - t_point;
      Eigen::Vector2f m_point(measurement_points[m_p_idx].x(),
                              measurement_points[m_p_idx].y());
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
        break;
      }
    }
  }
  // 卡尔曼更新
  auto k = P_ * H_.transpose() * ((H_ * P_ * H_.transpose() + R_).inverse());

  auto& x_gain = k * (-HZ_);
  // for (int i = 0; i < x_gain.size() / 2; ++i) {
  //     HLOG_INFO << "X_GAIN:" << i << " == "  << x_gain[i * 2] << "," <<
  //     x_gain[i * 2 + 1];
  // }
  X_ = X_ + k * (-HZ_);
  P_ = P_ - k * H_ * P_;
}

void LaneLinePointFilter::UpdateStage(const LaneLinePtr& measurement_line) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();
  // 跟踪点的自适应更新维护。
  point_manager_->Process(measurement_line, &X_, &P_);
  // PERF_BLOCK_END("UpdatePoints Used Time");
  const auto& measurement_points = measurement_line->world_points;
  KalmanUpdate(measurement_points);
  // PERF_BLOCK_END("KalmanUpdate Used Time");
}

void LaneLinePointFilter::PredictStage() {
  RevisePredictFit();
  A_update_ = B_ * A_;
  X_ = A_update_ * X_;
  P_ = A_ * P_ * A_.transpose() + Q_;
}

void LaneLinePointFilter::calculate_stability_error(
    const std::vector<Eigen::Vector3d>& fit_points) {
  LaneLineStabilityData stability_data{};
  // 1.从0到10m每间隔1m取一个点取平均
  double offset = 0.0;
  for (int i = 0; i < 10; ++i) {
    offset += curve_fit_.evalueValue(i);
  }
  offset = offset / 10;
  stability_data.offset = offset;
  // 2.计算在每个点的一阶导累加平均
  double heading = 0.0;
  for (const auto& pt : fit_points) {
    heading += curve_fit_.evalueHeading(pt.x());
  }
  heading = heading / (fit_points.size() + 1);
  stability_data.heading = heading;
  if (std::isnan(offset) || std::isnan(heading)) {
    stability_data.status =
        CalculateStatus::LaneStabilityError_CalculateStatus_FAILED_ERROR_VALUE;
  } else {
    stability_data.status =
        CalculateStatus::LaneStabilityError_CalculateStatus_SUCCESS;
  }
  // 3.添加进历史数据
  latest_stability_data_.push_back(stability_data);
  auto& stability_error = target_ref_->GetTrackedObject()->stability_error;
  // 数据不足3帧
  if (latest_stability_data_.size() < 3) {
    stability_error.set_calculate_status(
        CalculateStatus::
            LaneStabilityError_CalculateStatus_FAILED_INSUFFICIENT_DATA);
    return;
  }
  // 4.遍历确认计算状态
  bool error_flag = false;
  double mean_offset = 0.0;
  double mean_heading = 0.0;
  for (const auto& data : latest_stability_data_) {
    // 有次计算存在错误设置状态后直接返回
    if (data.status !=
        CalculateStatus::LaneStabilityError_CalculateStatus_SUCCESS) {
      stability_error.set_calculate_status(data.status);
      error_flag = true;
      break;
    }
    mean_offset += data.offset / 3.0;
    mean_heading += data.heading / 3.0;
  }
  if (error_flag) {
    return;
  }
  // 5.计算MAE和VAE
  double offset_mae = 0.0;
  double offset_var = 0.0;
  double heading_mae = 0.0;
  double heading_var = 0.0;
  for (const auto& data : latest_stability_data_) {
    double offset_error = data.offset - mean_offset;
    offset_mae += std::abs(offset_error) / 3.0;
    offset_var += offset_error * offset_error / 3.0;
    double heading_error = data.heading - mean_heading;
    heading_mae += std::abs(heading_error) / 3.0;
    heading_var += heading_error * heading_error / 3.0;
  }
  HLOG_DEBUG << "local_map track_id: " << target_ref_->Id()
             << ", stability_error offset_mae: " << offset_mae
             << ", offset_var: " << offset_var
             << ", heading_mae: " << heading_mae
             << ", heading_var: " << heading_var;
  stability_error.set_calculate_status(
      CalculateStatus::LaneStabilityError_CalculateStatus_SUCCESS);
  stability_error.set_offset_mae(offset_mae);
  stability_error.set_offset_var(offset_var);
  stability_error.set_heading_mae(heading_mae);
  stability_error.set_heading_var(heading_var);
}

void LaneLinePointFilter::MergeMapTrackLanePoints() {
  if (target_vehicle_pts_.empty()) {
    return;
  }
  const auto& tracked_lane = target_ref_->GetTrackedObject();
  const auto& tracked_points = tracked_lane->vehicle_points;
  // 先删掉超过最新的后处理跟踪点
  target_vehicle_pts_.erase(
      std::remove_if(target_vehicle_pts_.begin(), target_vehicle_pts_.end(),
                     [&](const auto& pt) {
                       return pt.x() >= tracked_points[0].x() - 1.0;
                     }),
      target_vehicle_pts_.end());
  // 插入最新的后处理跟踪点
  target_vehicle_pts_.insert(target_vehicle_pts_.end(), tracked_points.begin(),
                             tracked_points.end());
  int erase_pos = 0;
  for (int i = 0; i < static_cast<int>(target_vehicle_pts_.size()); ++i) {
    if (std::abs(target_vehicle_pts_[i].y()) > 80.0 ||
        target_vehicle_pts_[i].x() < -80.0) {
      erase_pos = i > erase_pos ? i : erase_pos;
    }
  }
  // 删除掉80m之后的点
  target_vehicle_pts_.erase(target_vehicle_pts_.begin(),
                            target_vehicle_pts_.begin() + erase_pos);
  tracked_lane->vehicle_points = target_vehicle_pts_;
}

void LaneLinePointFilter::UpdateResult(bool match_flag) {
  auto tracked_lane = target_ref_->GetTrackedObject();
  const auto& pts = tracked_lane->vehicle_points;
  if (match_flag) {
    // 保存上一帧的点
    target_vehicle_pts_ = pts;
    // 将X_点存入跟踪线数据结构中进行发送
    ConvertEigen2PointSet(X_, tracked_lane);
    // 点处理完后和保存的车道线元素融合，直接append到地图车道线
    MergeMapTrackLanePoints();
  }

  // 拿车身坐标系下的点进行三次方程拟合
  auto& track_polynomial = tracked_lane->vehicle_curve;
  std::vector<Eigen::Vector3d> fit_points;
  if (pts.size() > 20) {
    fit_points.assign(pts.end() - 20, pts.end());
  } else {
    fit_points.assign(pts.begin(), pts.end());
  }
  if (!curve_fit_.PolyFitProcess(fit_points)) {
    LaneLineStabilityData stability_data{};
    stability_data.status =
        CalculateStatus::LaneStabilityError_CalculateStatus_FAILED_FIT;
    latest_stability_data_.push_back(stability_data);
    if (target_ref_->IsTracked()) {
      calculate_stability_error(fit_points);
    }
    HLOG_ERROR << "curve_fit error !!!";
    return;
  }
  if (target_ref_->IsTracked()) {
    calculate_stability_error(fit_points);
  }

  track_polynomial.min = curve_fit_.x_min;
  track_polynomial.max = curve_fit_.x_max;
  track_polynomial.coeffs.resize(4);
  track_polynomial.coeffs[0] = curve_fit_.params[0];
  track_polynomial.coeffs[1] = curve_fit_.params[1];
  track_polynomial.coeffs[2] = curve_fit_.params[2];
  track_polynomial.coeffs[3] = curve_fit_.params[3];
}

void LaneLinePointFilter::Reset() { return; }

}  // namespace lm
}  // namespace mp
}  // namespace hozon
