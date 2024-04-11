// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: lane_point_filter.cc
// @brief: filter 3d points

#include "modules/local_mapping/lib/filter/point_manager.h"

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

void AdaptorPointManager::UturnsStateCorrection(
    Eigen::Matrix<double, 40, 1>* XPtr, Eigen::Matrix<double, 40, 40>* PPtr) {
  X_ = *XPtr;
  P_ = *PPtr;
  pt_size_ = X_.size() / 2;
  pose_ = PoseManager::Instance()->GetCurrentPose();
  Eigen::Vector3d farthest_track_pt(X_[(pt_size_ - 1) * 2],
                                    X_[(pt_size_ - 1) * 2 + 1], 0.0);
  fastest_track_pt_ = pose_.inverse() * farthest_track_pt;
  Eigen::Vector3d near_track_pt(X_[0], X_[1], 0.0);
  near_track_pt_ = pose_.inverse() * near_track_pt;
  // 判定是否处于掉头的状态下（若是，则将车道线点翻转为此时车系下从小到大的顺序。）
  if (near_track_pt_.x() > fastest_track_pt_.x()) {
    for (int i = 0; i < pt_size_ / 2; ++i) {
      int64_t j = pt_size_ - i - 1;
      std::swap(X_[i * 2], X_[j * 2]);
      std::swap(X_[i * 2 + 1], X_[j * 2 + 1]);
    }
    std::swap(fastest_track_pt_, near_track_pt_);
    P_.setIdentity();  // 重置协方差矩阵
  }
  return;
}

void AdaptorPointManager::GetVertialIntervalOfDPs() {
  auto& measurement_points = (latest_measurement_lines_.back());
  near_measure_pt_ = measurement_points.front().vehicle_point;
  far_measure_pt_ = measurement_points.back().vehicle_point;
  // 选取阈值, 取检测点的平均点间隔
  threshold_ = 0;
  for (int i = 1; i < measurement_points.size(); ++i) {
    threshold_ += measurement_points[i].vehicle_point.x() -
                  measurement_points[i - 1].vehicle_point.x();
  }
  threshold_ = threshold_ / (measurement_points.size() - 1);
  return;
}

void AdaptorPointManager::CopyMatrix(Eigen::Matrix<double, 40, 1>* XPtr,
                                     Eigen::Matrix<double, 40, 40>* PPtr) {
  *XPtr = X_;
  *PPtr = P_;
}

void AdaptorPointManager::UpdatePointsNear(
    const std::vector<InnerPoint>& measurement_points) {
  // 跟踪最近点距离本车超过1m并且连续两帧检测小于跟踪
  // 检测点逐个替换跟踪点，直到检测点和跟踪点距离相近
  if (near_track_pt_.x() > 1.0 &&
      near_track_pt_.x() > near_measure_pt_.x() + 0.5) {
    // 找到检测需要添加的起始索引det_ind
    int det_ind = -1;
    for (int i = 0; i < measurement_points.size(); ++i) {
      if (measurement_points[i].vehicle_point.x() > -threshold_) {
        det_ind = i;
        break;
      }
    }
    if (det_ind == -1) {
      return;
    }
    // 跟踪点的替换从0开始
    for (int i = 0, j = det_ind; i < pt_size_ && j < measurement_points.size();
         ++i, ++j) {
      Eigen::Vector3d track_pt(X_[i * 2], X_[i * 2 + 1], 0.0);
      track_pt = pose_.inverse() * track_pt;
      const auto& measure_pt = measurement_points[j];
      // 近处跟踪点远大于检测点, 找到检测和跟踪基本差不多距离的点索引i
      // 把跟踪的点进行替换
      if (measure_pt.vehicle_point.x() < track_pt.x() - threshold_) {
        X_[2 * i] = measure_pt.local_point.x();
        X_[2 * i + 1] = measure_pt.local_point.y();
        P_.block<2, 2>(2 * i, 2 * i) = Eigen::Matrix<double, 2, 2>::Identity();
      } else {
        HLOG_DEBUG << "UpdatePointsNear: " << i << ", measure_pt"
                   << measure_pt.vehicle_point.x()
                   << ", track_pt: " << track_pt.x()
                   << ", threshold: " << threshold_;
        break;
      }
    }
  }
  return;
}

void AdaptorPointManager::UpdatePointsFar(
    const std::vector<InnerPoint>& measurement_points) {
  int64_t append_size = 0;
  std::vector<int> append_index;
  Eigen::VectorXd inter_points;
  double add_mea_threshold = fastest_track_pt_.x() + threshold_;
  // 在跟踪远端均匀插入点，每次判断需要插入的点和前面点的间隔超过阈值才插入
  for (int i = 0; i < measurement_points.size(); ++i) {
    if (measurement_points[i].vehicle_point.x() > add_mea_threshold) {
      append_size++;
      append_index.push_back(i);
      add_mea_threshold = measurement_points[i].vehicle_point.x() + threshold_;
    }
  }
  inter_points.resize(append_size * 2);
  for (int i = 0; i < append_size; ++i) {
    inter_points[i * 2] = measurement_points[append_index[i]].local_point.x();
    inter_points[i * 2 + 1] =
        measurement_points[append_index[i]].local_point.y();
  }
  int64_t rest_size = pt_size_ - append_size;
  HLOG_INFO << "append_size: " << append_size << ", rest_size: " << rest_size;
  // 在最远处插值跟踪点, 更新集合里面的 X_ 和 P_
  if (append_size > 0) {
    X_SWAP_.setZero();
    P_SWAP_.setZero();
    X_SWAP_.block(0, 0, rest_size * 2, 1) =
        X_.block(append_size * 2, 0, rest_size * 2, 1);
    X_SWAP_.block(rest_size * 2, 0, append_size * 2, 1) = inter_points;
    X_ = X_SWAP_;

    P_SWAP_.block(0, 0, rest_size * 2, rest_size * 2) = P_.block(
        append_size * 2, append_size * 2, rest_size * 2, rest_size * 2);
    P_SWAP_.block(rest_size * 2, rest_size * 2, append_size * 2,
                  append_size * 2) =
        Eigen::MatrixXd::Identity(append_size * 2, append_size * 2) * init_p_;
    P_ = P_SWAP_;
  }
  return;
}

void AdaptorPointManager::DelPointsFar(
    const std::vector<InnerPoint>& measurement_points) {
  // 跟踪比检测远的情况
  int count_track_over_dect = 0;
  // 检测远端的平均距离
  double mean_far_dect_x = 0.0;
  for (int i = latest_measurement_lines_.size() - 1, j = 1; i >= 0; i--, j++) {
    // HLOG_INFO << "###### latest_measurement_lines_ size():"
    //           << (latest_measurement_lines_[i]);
    const auto& dect_pt = (latest_measurement_lines_[i]).back().vehicle_point;
    if (fastest_track_pt_.x() > dect_pt.x() + threshold_ * j) {
      count_track_over_dect++;
      mean_far_dect_x += dect_pt.x();
    } else {
      break;
    }
  }
  // HLOG_INFO << "count_track_over_dect: " << count_track_over_dect
  //           << ", y: " << fastest_track_pt_.y();
  if (count_track_over_dect >= 3) {
    // 近处插值点，目的是保证pt_size_
    mean_far_dect_x = mean_far_dect_x / count_track_over_dect;
    int64_t del_pt_size = 0;
    int64_t del_pt_index = -1;
    for (int i = 0; i < pt_size_; ++i) {
      Eigen::Vector3d track_pt(X_[i * 2], X_[i * 2 + 1], 0.0);
      track_pt = pose_.inverse() * track_pt;
      if (track_pt.x() > mean_far_dect_x + 0.5) {
        del_pt_index = i;
        break;
      }
    }
    del_pt_size = pt_size_ - del_pt_index;
    int64_t count_inter_points = del_pt_size;
    // 近处开始插值点
    X_SWAP_.setZero();
    P_SWAP_.setZero();
    int64_t index = 0;
    if (pt_size_ > 2 * del_pt_size) {
      // 需要删除的点不超过一半
      for (int i = 0; i < pt_size_; ++i) {
        if (count_inter_points > 0) {
          index = i / 2;
          if (i % 2 == 1) {
            // 均值插入
            X_SWAP_[2 * i + 0] = (X_[2 * index] + X_[2 * index + 2]) / 2;
            X_SWAP_[2 * i + 1] = (X_[2 * index + 1] + X_[2 * index + 3]) / 2;
            count_inter_points--;
          } else {
            X_SWAP_[2 * i + 0] = X_[index * 2];
            X_SWAP_[2 * i + 1] = X_[index * 2 + 1];
          }
        } else {
          // 不再插值点
          index = i - del_pt_size;
          X_SWAP_[2 * i + 0] = X_[index * 2];
          X_SWAP_[2 * i + 1] = X_[index * 2 + 1];
        }
        // P_矩阵的插值方式不区分
        P_SWAP_.setIdentity();
        P_SWAP_ = P_SWAP_ * init_p_;
      }
    } else {
      // 超过一半的点都需要删除,极端情况,直接用检测替换跟踪
      P_SWAP_.setIdentity();
      P_SWAP_ = P_SWAP_ * init_p_;
      X_SWAP_.setZero();
      for (int i = 0; i < pt_size_; ++i) {
        const auto& measure_pt = measurement_points[i];
        X_SWAP_[2 * i] = measure_pt.local_point.x();
        X_SWAP_[2 * i + 1] = measure_pt.local_point.y();
      }
    }
    X_ = X_SWAP_;
    P_ = P_SWAP_;
  }
  return;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
