/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei/zhaohaowu
 *Date: 2023-09-01
 *****************************************************************************/
#include "modules/local_mapping/utils/map_manager.h"

namespace hozon {
namespace mp {
namespace lm {

void MapManager::CutLocalMap(LocalMap* local_map, const double& length_x,
                             const double& length_y) {
  for (size_t i = 0; i < local_map->lane_lines_.size(); ++i) {
    for (size_t j = 0; j < local_map->lane_lines_[i].points_.size(); ++j) {
      if (local_map->lane_lines_[i].points_[j].x() < -length_x ||
          local_map->lane_lines_[i].points_[j].x() > length_x ||
          local_map->lane_lines_[i].points_[j].y() < -length_y ||
          local_map->lane_lines_[i].points_[j].y() > length_y) {
        local_map->lane_lines_[i].points_.erase(
            local_map->lane_lines_[i].points_.begin() + static_cast<int>(j));
        --j;
      }
    }
    for (size_t j = 0; j < local_map->lane_lines_[i].fit_points_.size(); ++j) {
      if (local_map->lane_lines_[i].fit_points_[j].x() < -length_x ||
          local_map->lane_lines_[i].fit_points_[j].x() > length_x ||
          local_map->lane_lines_[i].fit_points_[j].y() < -length_y ||
          local_map->lane_lines_[i].fit_points_[j].y() > length_y) {
        local_map->lane_lines_[i].fit_points_.erase(
            local_map->lane_lines_[i].fit_points_.begin() +
            static_cast<int>(j));
        --j;
      }
    }
    for (size_t j = 0; j < local_map->lane_lines_[i].control_points_.size();
         ++j) {
      if (local_map->lane_lines_[i].control_points_[j].x() < -length_x ||
          local_map->lane_lines_[i].control_points_[j].x() > length_x ||
          local_map->lane_lines_[i].control_points_[j].y() < -length_y ||
          local_map->lane_lines_[i].control_points_[j].y() > length_y) {
        local_map->lane_lines_[i].control_points_.erase(
            local_map->lane_lines_[i].control_points_.begin() +
            static_cast<int>(j));
        --j;
      }
    }
    if (local_map->lane_lines_[i].points_.empty() ||
        ((local_map->lane_lines_[i].edge_laneline_count_ < -5) &&
         (!local_map->lane_lines_[i].ismature_))) {
      local_map->lane_lines_.erase(local_map->lane_lines_.begin() +
                                   static_cast<int>(i));
      --i;
    }
  }
}

void MapManager::UpdateLaneLine(LocalMap* local_map,
                                const Sophus::SE3d& T_C_L) {
  Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
  T << T_C_L.matrix()(0, 0), T_C_L.matrix()(0, 1), T_C_L.matrix()(0, 3),
      T_C_L.matrix()(1, 0), T_C_L.matrix()(1, 1), T_C_L.matrix()(1, 3), 0, 0, 1;
  for (auto& lane_line : local_map->lane_lines_) {
    for (auto& point : lane_line.points_) {
      point = T_C_L * point;
      // point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      // point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
    for (auto& point : lane_line.fit_points_) {
      point = T_C_L * point;
      // point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      // point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
    for (auto& point : lane_line.control_points_) {
      point = T_C_L * point;
      // point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      // point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
  }
}

void MapManager::AddNewLaneLine(LocalMap* local_map,
                                const LaneLine& cur_lane_line,
                                const bool& use_perception_match) {
  LaneLine lane_line;
  int tmp_id = 0;
  for (const auto& lane_line : local_map->lane_lines_) {
    if (lane_line.track_id_ > tmp_id) {
      tmp_id = lane_line.track_id_;
    }
  }
  tmp_id++;
  if (use_perception_match) {
    lane_line.track_id_ = cur_lane_line.track_id_;
  } else {
    lane_line.track_id_ = tmp_id;
  }
  lane_line.lanepos_ = cur_lane_line.lanepos_;
  lane_line.lanetype_ = cur_lane_line.lanetype_;
  lane_line.points_ = cur_lane_line.points_;
  lane_line.c2_ = cur_lane_line.c2_;
  lane_line.need_fit_ = true;
  int tmp_lanepose = static_cast<int>(lane_line.lanepos_);
  lane_line.edge_laneline_count_ = 0;
  lane_line.ismature_ = (static_cast<int>(cur_lane_line.lanepos_) >= -2) &&
                        (static_cast<int>(cur_lane_line.lanepos_) <= 2);
  local_map->lane_lines_.emplace_back(lane_line);
}

void MapManager::MergeOldLaneLine(LocalMap* local_map,
                                  const LaneLine& cur_lane_line,
                                  const LaneLine& map_lane_line) {
  if (map_lane_line.points_.empty()) {
    return;
  }
  std::vector<Eigen::Vector3d> new_pts;
  for (auto p_map : map_lane_line.points_) {
    if (p_map.x() < cur_lane_line.start_point_x_) {
      new_pts.emplace_back(p_map);
    }
  }
  double tmp_x = 0.0;
  if (!new_pts.empty()) {
    tmp_x = new_pts[new_pts.size() - 1].x() + 1;
  } else {
    tmp_x = cur_lane_line.start_point_x_;
  }
  double x = tmp_x;
  while (x <= cur_lane_line.end_point_x_) {
    double y = cur_lane_line.c3_ * pow(x, 3) + cur_lane_line.c2_ * pow(x, 2) +
               cur_lane_line.c1_ * x + cur_lane_line.c0_;
    Eigen::Vector3d tmp(x, y, 0);
    new_pts.emplace_back(tmp);
    x += 1;
  }
  // HLOG_ERROR << "map_lane.track_id_: " << map_lane.track_id_;
  for (auto& lane_line : local_map->lane_lines_) {
    if (lane_line.track_id_ == map_lane_line.track_id_) {
      lane_line.points_ = new_pts;
      lane_line.lanepos_ = cur_lane_line.lanepos_;
      lane_line.lanetype_ = cur_lane_line.lanetype_;
      lane_line.c2_ = cur_lane_line.c2_;
      if (!lane_line.ismature_) {
        lane_line.edge_laneline_count_++;
        // if (lane_line.edge_laneline_count_ >= 5) {
        //   lane_line.ismature_ = true;
        // }
      }
      return;
    }
  }
}

void MapManager::UpdateLaneByPerception(LocalMap* local_map,
                                        const Perception& cur_lane_lines) {
  if (cur_lane_lines.lane_lines_.size() < 2) {
    local_map->lanes_.clear();
    return;
  }
  int left_lanepos = 0;
  int right_lanepos = 0;
  for (const auto& cur_lane_line : cur_lane_lines.lane_lines_) {
    left_lanepos =
        std::min(left_lanepos, static_cast<int>(cur_lane_line.lanepos_));
    right_lanepos =
        std::max(right_lanepos, static_cast<int>(cur_lane_line.lanepos_));
  }
  if (left_lanepos == 0 || right_lanepos == 0) {
    return;
  }
  std::vector<Lane> lanes;
  UpdateLeftAndRightLine(left_lanepos, right_lanepos,
                         cur_lane_lines.lane_lines_, &lanes);
  // add width, center_points, left_lane_id and right_lane_id
  for (auto& lane : lanes) {
    double width = 0;
    int num = 0;
    for (int x = 0; x < 5; x++) {
      double left_y = lane.left_line_.c0_ + lane.left_line_.c1_ * x +
                      lane.left_line_.c2_ * x * x +
                      lane.left_line_.c3_ * x * x * x;
      double right_y = lane.right_line_.c0_ + lane.right_line_.c1_ * x +
                       lane.right_line_.c2_ * x * x +
                       lane.right_line_.c3_ * x * x * x;
      width += (left_y - right_y);
      num++;
    }
    width = width / num;
    lane.width_ = width;

    std::vector<Eigen::Vector3d> center_points;
    double x_max =
        std::min(lane.left_line_.end_point_x_, lane.right_line_.end_point_x_);
    double x = 0;
    while (x < x_max) {
      double left_y = lane.left_line_.c0_ + lane.left_line_.c1_ * x +
                      lane.left_line_.c2_ * x * x +
                      lane.left_line_.c3_ * x * x * x;
      double right_y = lane.right_line_.c0_ + lane.right_line_.c1_ * x +
                       lane.right_line_.c2_ * x * x +
                       lane.right_line_.c3_ * x * x * x;
      double y = (left_y + right_y) / 2;
      Eigen::Vector3d center_point = {x, y, 0};
      center_points.emplace_back(center_point);
      x++;
    }
    lane.center_line_.points_ = center_points;
    if (static_cast<int>(lane.left_line_.lanepos_) == left_lanepos &&
        static_cast<int>(lane.right_line_.lanepos_) == right_lanepos) {
      lane.left_lane_id_ = 1000;
      lane.right_lane_id_ = 1000;
      break;
    }
    if (static_cast<int>(lane.left_line_.lanepos_) == left_lanepos) {
      lane.left_lane_id_ = 1000;
      lane.right_lane_id_ = lane.lane_id_ + 1;
      break;
    }
    if (static_cast<int>(lane.right_line_.lanepos_) == right_lanepos) {
      lane.left_lane_id_ = lane.lane_id_ - 1;
      lane.right_lane_id_ = 1000;
      break;
    }
    lane.left_lane_id_ = lane.lane_id_ - 1;
    lane.right_lane_id_ = lane.lane_id_ + 1;
  }
  if (lanes.empty()) {
    return;
  }
  local_map->lanes_ = lanes;
}

void MapManager::UpdateLeftAndRightLine(int left_lanepos, int right_lanepos,
                                        const std::vector<LaneLine>& lane_lines,
                                        std::vector<Lane>* lanes) {
  for (int i = left_lanepos; i < -1; i++) {
    Lane lane;
    lane.lane_id_ = i + 1;
    for (const auto& cur_lane_line : lane_lines) {
      if (static_cast<int>(cur_lane_line.lanepos_) == i) {
        lane.left_line_ = cur_lane_line;
      }
      if (static_cast<int>(cur_lane_line.lanepos_) == i + 1) {
        lane.right_line_ = cur_lane_line;
      }
    }
    lanes->push_back(lane);
  }

  Lane cur_lane;
  cur_lane.lane_id_ = 0;
  for (const auto& cur_lane_line : lane_lines) {
    if (static_cast<int>(cur_lane_line.lanepos_) == -1) {
      cur_lane.left_line_ = cur_lane_line;
    }
    if (static_cast<int>(cur_lane_line.lanepos_) == 1) {
      cur_lane.right_line_ = cur_lane_line;
    }
  }
  lanes->push_back(cur_lane);

  for (int i = 1; i < right_lanepos; i++) {
    Lane lane;
    lane.lane_id_ = i;
    for (const auto& cur_lane_line : lane_lines) {
      if (static_cast<int>(cur_lane_line.lanepos_) == i) {
        lane.left_line_ = cur_lane_line;
      }
      if (static_cast<int>(cur_lane_line.lanepos_) == i + 1) {
        lane.right_line_ = cur_lane_line;
      }
    }
    lanes->push_back(lane);
  }
}

void MapManager::UpdateLaneByLocalmap(LocalMap* local_map) {
  if (local_map->lane_lines_.size() < 2) {
    local_map->lanes_.clear();
    return;
  }
  int left_lanepos = 0;
  int right_lanepos = 0;
  double left_y = 0;
  double right_y = 0;
  for (const auto& lane_line : local_map->lane_lines_) {
    if (lane_line.fit_points_.empty() ||
        lane_line.lanepos_ == LanePositionType::OTHER) {
      continue;
    }
    left_lanepos = std::min(left_lanepos, static_cast<int>(lane_line.lanepos_));
    right_lanepos =
        std::max(right_lanepos, static_cast<int>(lane_line.lanepos_));
    left_y =
        std::max(left_y, static_cast<double>(lane_line.fit_points_.back().y()));
    right_y = std::min(right_y,
                       static_cast<double>(lane_line.fit_points_.back().y()));
  }
  if (left_lanepos == 0 || right_lanepos == 0) {
    return;
  }
  for (auto& lane_line : local_map->lane_lines_) {
    if (lane_line.fit_points_.empty() ||
        lane_line.lanepos_ != LanePositionType::OTHER) {
      continue;
    }
    if (lane_line.fit_points_.back().y() > left_y) {
      lane_line.lanepos_ = static_cast<LanePositionType>(left_lanepos - 1);
      left_lanepos--;
      left_y = lane_line.fit_points_.back().y();
    }
    if (lane_line.fit_points_.back().y() < right_y) {
      lane_line.lanepos_ = static_cast<LanePositionType>(right_lanepos + 1);
      right_lanepos++;
      right_y = lane_line.fit_points_.back().y();
    }
  }
  // HLOG_ERROR << "left_lanepos " << left_lanepos << " right_lanepos "
  //            << right_lanepos;
  // HLOG_ERROR << "left_y " << left_y << " right_y " << right_y;
  std::vector<Lane> map_lanes;
  UpdateLeftAndRightLine(left_lanepos, right_lanepos, local_map->lane_lines_,
                         &map_lanes);
  // add width, center_points, left_lane_id and right_lane_id
  for (auto& lane : map_lanes) {
    double width = 0;
    int num = 0;
    for (int x = 0; x < 5; x++) {
      double left_y = lane.left_line_.c0_ + lane.left_line_.c1_ * x +
                      lane.left_line_.c2_ * x * x +
                      lane.left_line_.c3_ * x * x * x;
      double right_y = lane.right_line_.c0_ + lane.right_line_.c1_ * x +
                       lane.right_line_.c2_ * x * x +
                       lane.right_line_.c3_ * x * x * x;
      width += (left_y - right_y);
      num++;
    }
    width = width / num;
    lane.width_ = width;

    std::vector<Eigen::Vector3d> center_points;
    double x_max =
        std::min(lane.left_line_.end_point_x_, lane.right_line_.end_point_x_);
    double x = 0;
    while (x < x_max) {
      double left_y = lane.left_line_.c0_ + lane.left_line_.c1_ * x +
                      lane.left_line_.c2_ * x * x +
                      lane.left_line_.c3_ * x * x * x;
      double right_y = lane.right_line_.c0_ + lane.right_line_.c1_ * x +
                       lane.right_line_.c2_ * x * x +
                       lane.right_line_.c3_ * x * x * x;
      double y = (left_y + right_y) / 2;
      Eigen::Vector3d center_point = {x, y, 0};
      center_points.emplace_back(center_point);
      x++;
    }
    lane.center_line_.points_ = center_points;
    if (static_cast<int>(lane.left_line_.lanepos_) == left_lanepos &&
        static_cast<int>(lane.right_line_.lanepos_) == right_lanepos) {
      lane.left_lane_id_ = 1000;
      lane.right_lane_id_ = 1000;
      break;
    }
    if (static_cast<int>(lane.left_line_.lanepos_) == left_lanepos) {
      lane.left_lane_id_ = 1000;
      lane.right_lane_id_ = lane.lane_id_ + 1;
      break;
    }
    if (static_cast<int>(lane.right_line_.lanepos_) == right_lanepos) {
      lane.left_lane_id_ = lane.lane_id_ - 1;
      lane.right_lane_id_ = 1000;
      break;
    }
    lane.left_lane_id_ = lane.lane_id_ - 1;
    lane.right_lane_id_ = lane.lane_id_ + 1;
  }
  if (map_lanes.empty()) {
    return;
  }
  local_map->map_lanes_ = map_lanes;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
