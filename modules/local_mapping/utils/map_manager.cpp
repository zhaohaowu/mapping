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
  for (auto& lane_line : local_map->lane_lines_) {
    lane_line.points_.erase(
        std::remove_if(lane_line.points_.begin(), lane_line.points_.end(),
                       [&](Eigen::Vector3d& point) {
                         return (point.x() < -length_x ||
                                 point.x() > length_x ||
                                 point.y() < -length_y || point.y() > length_y);
                       }),
        lane_line.points_.end());
    lane_line.fit_points_.erase(
        std::remove_if(
            lane_line.fit_points_.begin(), lane_line.fit_points_.end(),
            [&](Eigen::Vector3d& point) {
              return (point.x() < -length_x || point.x() > length_x ||
                      point.y() < -length_y || point.y() > length_y);
            }),
        lane_line.fit_points_.end());
    lane_line.control_points_.erase(
        std::remove_if(
            lane_line.control_points_.begin(), lane_line.control_points_.end(),
            [&](Eigen::Vector3d& point) {
              return (point.x() < -length_x || point.x() > length_x ||
                      point.y() < -length_y || point.y() > length_y);
            }),
        lane_line.control_points_.end());
  }
  local_map->lane_lines_.erase(
      std::remove_if(
          local_map->lane_lines_.begin(), local_map->lane_lines_.end(),
          [&](const LaneLine& lane_line) { return lane_line.points_.empty(); }),
      local_map->lane_lines_.end());
  for (auto& edge_line : local_map->edge_lines_) {
    edge_line.points_.erase(
        std::remove_if(edge_line.points_.begin(), edge_line.points_.end(),
                       [&](const Eigen::Vector3d& point) {
                         return (point.x() < -length_x ||
                                 point.x() > length_x ||
                                 point.y() < -length_y || point.y() > length_y);
                       }),
        edge_line.points_.end());
    edge_line.fit_points_.erase(
        std::remove_if(
            edge_line.fit_points_.begin(), edge_line.fit_points_.end(),
            [&](const Eigen::Vector3d& point) {
              return (point.x() < -length_x || point.x() > length_x ||
                      point.y() < -length_y || point.y() > length_y);
            }),
        edge_line.fit_points_.end());
    edge_line.control_points_.erase(
        std::remove_if(
            edge_line.control_points_.begin(), edge_line.control_points_.end(),
            [&](const Eigen::Vector3d& point) {
              return (point.x() < -length_x || point.x() > length_x ||
                      point.y() < -length_y || point.y() > length_y);
            }),
        edge_line.control_points_.end());
  }
  local_map->edge_lines_.erase(
      std::remove_if(
          local_map->edge_lines_.begin(), local_map->edge_lines_.end(),
          [&](const LaneLine& edge_line) { return edge_line.points_.empty(); }),
      local_map->edge_lines_.end());
  local_map->stop_lines_.erase(
      std::remove_if(
          local_map->stop_lines_.begin(), local_map->stop_lines_.end(),
          [&](const StopLine& stop_line) {
            auto point = (stop_line.left_point_ + stop_line.right_point_) / 2;
            return point.x() < -length_x || point.x() > length_x ||
                   point.y() < -length_y || point.y() > length_y;
          }),
      local_map->stop_lines_.end());
  local_map->arrows_.erase(
      std::remove_if(local_map->arrows_.begin(), local_map->arrows_.end(),
                     [&](const Arrow& arrow) {
                       Eigen::Vector3d point((arrow.min_x + arrow.max_x) / 2,
                                             (arrow.min_y + arrow.max_y) / 2,
                                             0);
                       return point.x() < -length_x || point.x() > length_x ||
                              point.y() < -length_y || point.y() > length_y;
                     }),
      local_map->arrows_.end());
  local_map->zebra_crossings_.erase(
      std::remove_if(local_map->zebra_crossings_.begin(),
                     local_map->zebra_crossings_.end(),
                     [&](const ZebraCrossing& zebra_crossing) {
                       Eigen::Vector3d point(
                           (zebra_crossing.min_x + zebra_crossing.max_x) / 2,
                           (zebra_crossing.min_y + zebra_crossing.max_y) / 2,
                           0);
                       return point.x() < -length_x || point.x() > length_x ||
                              point.y() < -length_y || point.y() > length_y;
                     }),
      local_map->zebra_crossings_.end());
}

void MapManager::UpdatePerception(LocalMap* local_map,
                                  const Sophus::SE3d& T_C_L) {
  Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
  T << T_C_L.matrix()(0, 0), T_C_L.matrix()(0, 1), T_C_L.matrix()(0, 3),
      T_C_L.matrix()(1, 0), T_C_L.matrix()(1, 1), T_C_L.matrix()(1, 3), 0, 0, 1;
  for (auto& lane_line : local_map->lane_lines_) {
    for (auto& point : lane_line.points_) {
      point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
    for (auto& point : lane_line.fit_points_) {
      point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
    for (auto& point : lane_line.control_points_) {
      point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
  }
  for (auto& edge_line : local_map->edge_lines_) {
    for (auto& point : edge_line.points_) {
      point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
    for (auto& point : edge_line.fit_points_) {
      point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
    for (auto& point : edge_line.control_points_) {
      point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
  }
  for (auto& stop_line : local_map->stop_lines_) {
    stop_line.left_point_.x() = T(0, 0) * stop_line.left_point_.x() +
                                T(0, 1) * stop_line.left_point_.y() + T(0, 2);
    stop_line.right_point_.x() = T(0, 0) * stop_line.right_point_.x() +
                                 T(0, 1) * stop_line.right_point_.y() + T(0, 2);
  }
  for (auto& arrow : local_map->arrows_) {
    for (auto& point : arrow.points_.points_) {
      point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
    arrow.min_x = T(0, 0) * arrow.min_x + T(0, 1) * arrow.min_y + T(0, 2);
    arrow.min_y = T(1, 0) * arrow.min_x + T(1, 1) * arrow.min_y + T(1, 2);
    arrow.max_x = T(0, 0) * arrow.max_x + T(0, 1) * arrow.max_y + T(0, 2);
    arrow.max_y = T(1, 0) * arrow.max_x + T(1, 1) * arrow.max_y + T(1, 2);
  }
  for (auto& zebra_crossing : local_map->zebra_crossings_) {
    for (auto& point : zebra_crossing.points_.points_) {
      point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
    zebra_crossing.min_x = T(0, 0) * zebra_crossing.min_x +
                           T(0, 1) * zebra_crossing.min_y + T(0, 2);
    zebra_crossing.min_y = T(1, 0) * zebra_crossing.min_x +
                           T(1, 1) * zebra_crossing.min_y + T(1, 2);
    zebra_crossing.max_x = T(0, 0) * zebra_crossing.max_x +
                           T(0, 1) * zebra_crossing.max_y + T(0, 2);
    zebra_crossing.max_y = T(1, 0) * zebra_crossing.max_x +
                           T(1, 1) * zebra_crossing.max_y + T(1, 2);
  }
}

void MapManager::AddNewLaneLine(LocalMap* local_map,
                                const LaneLine& per_lane_line) {
  LaneLine lane_line;
  int tmp_id = 0;
  for (const auto& lane_line : local_map->lane_lines_) {
    if (lane_line.track_id_ > tmp_id) {
      tmp_id = lane_line.track_id_;
    }
  }
  tmp_id++;
  lane_line.track_id_ = tmp_id;
  lane_line.lanepos_ = per_lane_line.lanepos_;
  lane_line.lanetype_ = per_lane_line.lanetype_;
  lane_line.color_ = per_lane_line.color_;
  lane_line.points_ = per_lane_line.points_;
  lane_line.c2_ = per_lane_line.c2_;
  lane_line.count_ = 0;
  lane_line.ismature_ = static_cast<int>(lane_line.lanepos_) == -1 ||
                        static_cast<int>(lane_line.lanepos_) == 1;
  local_map->lane_lines_.emplace_back(lane_line);
}

void MapManager::AddNewEdgeLine(LocalMap* local_map,
                                const LaneLine& per_edge_line) {
  LaneLine edge_line;
  int tmp_id = 0;
  for (const auto& edge_line : local_map->edge_lines_) {
    if (edge_line.track_id_ > tmp_id) {
      tmp_id = edge_line.track_id_;
    }
  }
  tmp_id++;
  edge_line.track_id_ = tmp_id;
  edge_line.lanepos_ = per_edge_line.lanepos_;
  edge_line.lanetype_ = per_edge_line.lanetype_;
  edge_line.points_ = per_edge_line.points_;
  edge_line.count_ = 0;
  edge_line.ismature_ = false;
  local_map->edge_lines_.emplace_back(edge_line);
}

void MapManager::AddNewStopLine(LocalMap* local_map,
                                const StopLine& per_stop_line) {
  StopLine stop_line;
  int tmp_id = 0;
  for (const auto& stop_line : local_map->stop_lines_) {
    if (stop_line.track_id_ > tmp_id) {
      tmp_id = stop_line.track_id_;
    }
  }
  tmp_id++;
  stop_line.track_id_ = tmp_id;
  stop_line.left_point_ = per_stop_line.left_point_;
  stop_line.right_point_ = per_stop_line.right_point_;
  stop_line.ismature_ = false;
  local_map->stop_lines_.emplace_back(stop_line);
}

void MapManager::AddNewArrow(LocalMap* local_map, const Arrow& per_arrow) {
  Arrow arrow;
  int tmp_id = 0;
  for (const auto& arrow : local_map->arrows_) {
    if (arrow.track_id_ > tmp_id) {
      tmp_id = arrow.track_id_;
    }
  }
  tmp_id++;
  arrow.track_id_ = tmp_id;
  arrow.confidence_ = per_arrow.confidence_;
  arrow.heading_ = per_arrow.heading_;
  arrow.points_ = per_arrow.points_;
  arrow.type_ = per_arrow.type_;
  arrow.min_x = per_arrow.min_x;
  arrow.min_y = per_arrow.min_y;
  arrow.max_x = per_arrow.max_x;
  arrow.max_y = per_arrow.max_y;
  arrow.ismature_ = false;
  local_map->arrows_.emplace_back(arrow);
}

void MapManager::AddNewZebraCrossing(LocalMap* local_map,
                                     const ZebraCrossing& per_zebra_crossing) {
  ZebraCrossing zebra_crossing;
  int tmp_id = 0;
  for (const auto& zebra_crossing : local_map->zebra_crossings_) {
    if (zebra_crossing.track_id_ > tmp_id) {
      tmp_id = zebra_crossing.track_id_;
    }
  }
  tmp_id++;
  zebra_crossing.track_id_ = tmp_id;
  zebra_crossing.confidence_ = per_zebra_crossing.confidence_;
  zebra_crossing.heading_ = per_zebra_crossing.heading_;
  zebra_crossing.points_ = per_zebra_crossing.points_;
  zebra_crossing.min_x = per_zebra_crossing.min_x;
  zebra_crossing.min_y = per_zebra_crossing.min_y;
  zebra_crossing.max_x = per_zebra_crossing.max_x;
  zebra_crossing.max_y = per_zebra_crossing.max_y;
  zebra_crossing.ismature_ = false;
  local_map->zebra_crossings_.emplace_back(zebra_crossing);
}

void MapManager::MergeOldLaneLine(LocalMap* local_map,
                                  const LaneLine& per_lane_line,
                                  const LaneLine& map_lane_line) {
  if (map_lane_line.points_.empty()) {
    return;
  }
  std::vector<Eigen::Vector3d> new_pts;
  for (auto point : map_lane_line.points_) {
    if (point.x() < per_lane_line.start_point_x_) {
      new_pts.emplace_back(point);
    }
  }
  double tmp_x = 0.0;
  if (!new_pts.empty()) {
    tmp_x = new_pts[new_pts.size() - 1].x() + 1;
  } else {
    tmp_x = per_lane_line.start_point_x_;
  }
  double x = tmp_x;
  while (x <= per_lane_line.end_point_x_) {
    double y = per_lane_line.c3_ * pow(x, 3) + per_lane_line.c2_ * pow(x, 2) +
               per_lane_line.c1_ * x + per_lane_line.c0_;
    new_pts.emplace_back(x, y, 0);
    x += 1;
  }
  // HLOG_ERROR << "map_lane.track_id_: " << map_lane.track_id_;
  for (auto& lane_line : local_map->lane_lines_) {
    if (lane_line.track_id_ == map_lane_line.track_id_) {
      lane_line.points_ = new_pts;
      lane_line.lanepos_ = per_lane_line.lanepos_;
      lane_line.lanetype_ = per_lane_line.lanetype_;
      lane_line.color_ = per_lane_line.color_;
      if (!lane_line.ismature_) {
        lane_line.count_++;
        if (lane_line.count_ >= 10) {
          lane_line.ismature_ = true;
        }
      }
      return;
    }
  }
}

void MapManager::MergeOldEdgeLine(LocalMap* local_map,
                                  const LaneLine& per_edge_line,
                                  const LaneLine& map_edge_line) {
  if (map_edge_line.points_.empty()) {
    return;
  }
  std::vector<Eigen::Vector3d> new_pts;
  for (auto point : map_edge_line.points_) {
    if (point.x() < per_edge_line.start_point_x_) {
      new_pts.emplace_back(point);
    }
  }
  double tmp_x = 0.0;
  if (!new_pts.empty()) {
    tmp_x = new_pts[new_pts.size() - 1].x() + 1;
  } else {
    tmp_x = per_edge_line.start_point_x_;
  }
  double x = tmp_x;
  while (x <= per_edge_line.end_point_x_) {
    double y = per_edge_line.c3_ * pow(x, 3) + per_edge_line.c2_ * pow(x, 2) +
               per_edge_line.c1_ * x + per_edge_line.c0_;
    new_pts.emplace_back(x, y, 0);
    x += 1;
  }
  // HLOG_ERROR << "map_lane.track_id_: " << map_lane.track_id_;
  for (auto& edge_line : local_map->edge_lines_) {
    if (edge_line.track_id_ == map_edge_line.track_id_) {
      edge_line.points_ = new_pts;
      edge_line.lanepos_ = per_edge_line.lanepos_;
      edge_line.lanetype_ = per_edge_line.lanetype_;
      if (!edge_line.ismature_) {
        edge_line.count_++;
        if (edge_line.count_ >= 10) {
          edge_line.ismature_ = true;
        }
      }
      return;
    }
  }
}

void MapManager::MergeOldStopLine(LocalMap* local_map,
                                  const StopLine& per_stop_line,
                                  const StopLine& map_stop_line) {
  for (auto& stop_line : local_map->stop_lines_) {
    if (stop_line.track_id_ == map_stop_line.track_id_) {
      if (!stop_line.ismature_) {
        stop_line.count_++;
        if (stop_line.count_ >= 2) {
          stop_line.ismature_ = true;
        }
      }
      stop_line.left_point_ = per_stop_line.left_point_;
      stop_line.right_point_ = per_stop_line.right_point_;
      stop_line.track_id_ = per_stop_line.track_id_;
      return;
    }
  }
}

void MapManager::MergeOldArrow(LocalMap* local_map, const Arrow& per_arrow,
                               const Arrow& map_arrow) {
  for (auto& arrow : local_map->arrows_) {
    if (arrow.track_id_ == map_arrow.track_id_) {
      if (!arrow.ismature_) {
        arrow.count_++;
        if (arrow.count_ >= 10) {
          arrow.ismature_ = true;
        }
      }
      arrow.confidence_ = per_arrow.confidence_;
      arrow.heading_ = per_arrow.heading_;
      arrow.points_ = per_arrow.points_;
      arrow.type_ = per_arrow.type_;
      arrow.min_x = per_arrow.min_x;
      arrow.min_y = per_arrow.min_y;
      arrow.max_x = per_arrow.max_x;
      arrow.max_y = per_arrow.max_y;
      return;
    }
  }
}

void MapManager::MergeOldZebraCrossing(
    LocalMap* local_map, const ZebraCrossing& per_zebra_crossing,
    const ZebraCrossing& map_zebra_crossing) {
  for (auto& zebra_crossing : local_map->zebra_crossings_) {
    if (zebra_crossing.track_id_ == map_zebra_crossing.track_id_) {
      if (!zebra_crossing.ismature_) {
        zebra_crossing.count_++;
        if (zebra_crossing.count_ >= 10) {
          zebra_crossing.ismature_ = true;
        }
      }
      zebra_crossing.confidence_ = per_zebra_crossing.confidence_;
      zebra_crossing.heading_ = per_zebra_crossing.heading_;
      zebra_crossing.points_ = per_zebra_crossing.points_;
      zebra_crossing.min_x = per_zebra_crossing.min_x;
      zebra_crossing.min_y = per_zebra_crossing.min_y;
      zebra_crossing.max_x = per_zebra_crossing.max_x;
      zebra_crossing.max_y = per_zebra_crossing.max_y;
      return;
    }
  }
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
  local_map->map_lanes_.clear();
  if (local_map->lane_lines_.size() < 2) {
    return;
  }
  int left_lanepos = 0;
  int right_lanepos = 0;
  for (const auto& lane_line : local_map->lane_lines_) {
    if (!lane_line.ismature_) {
      continue;
    }
    left_lanepos = std::min(left_lanepos, static_cast<int>(lane_line.lanepos_));
    right_lanepos =
        std::max(right_lanepos, static_cast<int>(lane_line.lanepos_));
  }
  if (left_lanepos == 0 || right_lanepos == 0) {
    return;
  }
  std::vector<Lane> map_lanes;
  UpdateLeftAndRightLine(left_lanepos, right_lanepos, local_map->lane_lines_,
                         &map_lanes);
  // add width, center_points, left_lane_id and right_lane_id
  for (auto& lane : map_lanes) {
    if (static_cast<int>(lane.left_line_.lanepos_) == left_lanepos &&
        static_cast<int>(lane.right_line_.lanepos_) == right_lanepos) {
      lane.left_lane_id_ = 1000;
      lane.right_lane_id_ = 1000;
      break;
    }
    if (static_cast<int>(lane.left_line_.lanepos_) == left_lanepos) {
      lane.left_lane_id_ = 1000;
      lane.right_lane_id_ = lane.lane_id_ + 1;
      continue;
    }
    if (static_cast<int>(lane.right_line_.lanepos_) == right_lanepos) {
      lane.left_lane_id_ = lane.lane_id_ - 1;
      lane.right_lane_id_ = 1000;
      continue;
    }
    lane.left_lane_id_ = lane.lane_id_ - 1;
    lane.right_lane_id_ = lane.lane_id_ + 1;
  }
  if (map_lanes.empty()) {
    return;
  }
  local_map->map_lanes_ = map_lanes;
}

void MapManager::UpdateLanepos(LocalMap* local_map) {
  std::map<double, LaneLine> map_left;
  std::map<double, LaneLine, std::greater<>> map_right;
  std::vector<LaneLine> immature_lane_lines;
  for (auto& lane_line : local_map->lane_lines_) {
    std::vector<Eigen::Vector3d> points;
    for (const auto& point : lane_line.points_) {
      if (point.x() > -20 && point.x() < 20) {
        points.emplace_back(point);
      }
    }
    if (points.empty()) {
      continue;
    }
    std::vector<double> c(4);
    CommonUtil::FitLaneLine(points, &c);
    lane_line.c0_ = c[0];
    lane_line.c1_ = c[1];
    lane_line.c2_ = c[2];
    lane_line.c3_ = c[3];
    if (!lane_line.ismature_) {
      immature_lane_lines.emplace_back(lane_line);
      continue;
    }
    if (lane_line.c0_ > 0) {
      map_left[lane_line.c0_] = lane_line;
    } else if (lane_line.c0_ < 0) {
      map_right[lane_line.c0_] = lane_line;
    }
  }
  local_map->lane_lines_.clear();
  local_map->lane_lines_ = immature_lane_lines;
  int left_index = -1;
  for (auto& left : map_left) {
    left.second.lanepos_ = static_cast<LanePositionType>(left_index);
    local_map->lane_lines_.emplace_back(left.second);
    left_index--;
  }
  int right_index = 1;
  for (auto& right : map_right) {
    right.second.lanepos_ = static_cast<LanePositionType>(right_index);
    // if (right.second.lanepos_ == 1 || right.second.lanepos_ == 2)
    local_map->lane_lines_.emplace_back(right.second);
    right_index++;
  }
  map_left.clear();
  map_right.clear();
  std::vector<LaneLine> tmp_edge_lines;
  for (auto& edge_line : local_map->edge_lines_) {
    if (!edge_line.ismature_) {
      tmp_edge_lines.emplace_back(edge_line);
      continue;
    }
    std::vector<Eigen::Vector3d> points;
    for (const auto& point : edge_line.points_) {
      if (point.x() > -20 && point.x() < 20) {
        points.emplace_back(point);
      }
    }
    if (points.empty()) {
      continue;
    }
    std::vector<double> c(4);
    CommonUtil::FitLaneLine(points, &c);
    edge_line.c0_ = c[0];
    edge_line.c1_ = c[1];
    edge_line.c2_ = c[2];
    edge_line.c3_ = c[3];
    if (edge_line.c0_ > 0) {
      map_left[edge_line.c0_] = edge_line;
    } else if (edge_line.c0_ < 0) {
      map_right[edge_line.c0_] = edge_line;
    }
  }
  local_map->edge_lines_.clear();
  local_map->edge_lines_ = tmp_edge_lines;
  if (!map_left.empty()) {
    map_left.begin()->second.lanepos_ = static_cast<LanePositionType>(-1);
    local_map->edge_lines_.emplace_back(map_left.begin()->second);
  }
  if (!map_right.empty()) {
    map_right.begin()->second.lanepos_ = static_cast<LanePositionType>(1);
    local_map->edge_lines_.emplace_back(map_right.begin()->second);
  }
}

void MapManager::MergeMapLeftRight(LocalMap* local_map,
                                   LaneLine* cur_lane_line) {
  // HLOG_ERROR << "cur_lane_line->points_: " << cur_lane_line->points_.size();
  // for (auto cur_lane_line_point : cur_lane_line->points_) {
  //   HLOG_ERROR << "x: " << cur_lane_line_point.x()
  //              << "  y: " << cur_lane_line_point.y();
  // }
  int tmp_track_id = 0;
  for (auto& local_map_line : local_map->lane_lines_) {
    if ((local_map_line.track_id_ == cur_lane_line->track_id_) ||
        !local_map_line.ismature_ || local_map_line.points_.empty() ||
        cur_lane_line->points_.empty()) {
      continue;
    }
    double sum_y = 0;
    int count = 0;
    for (auto cur_lane_line_point : cur_lane_line->points_) {
      if (cur_lane_line_point.x() > (cur_lane_line->end_point_x_ * 0.7 +
                                     cur_lane_line->start_point_x_ * 0.3) ||
          ((fabs(cur_lane_line->c2_) > 0.001) &&
           (cur_lane_line_point.x() > (cur_lane_line->end_point_x_ * 0.5 +
                                       cur_lane_line->start_point_x_ * 0.5)))) {
        continue;
      }
      double temp_y =
          CommonUtil::CalCubicCurveY(local_map_line, cur_lane_line_point.x());
      sum_y += fabs(cur_lane_line_point.y() - temp_y);
      count++;
    }

    // HLOG_ERROR << "新增车道线与地图其他车道线count: " << count;
    // HLOG_ERROR << "新增车道线与地图其他车道线sum_y: " << sum_y;
    // HLOG_ERROR << "新增车道线与地图其他车道线y距离均值: " << (sum_y / count)
    //            << " 计数器数量: " << count << " c2值: " <<
    //            cur_lane_line->c2_;
    // HLOG_ERROR << "当前车道线id: " << cur_lane_line->track_id_;
    // HLOG_ERROR << "当前地图车道线id: " << local_map_line.track_id_;
    // HLOG_ERROR << "当前地图车道线数量: " << local_map->lane_lines_.size();
    // HLOG_ERROR << "当前车道线lanepose: " << cur_lane_line->lanepos_;
    // HLOG_ERROR << "当前车道线count_: " << cur_lane_line->count_;
    if (count == 0 || (sum_y / count) > 0.5) {
      // HLOG_ERROR << "跳过这条地图车道线";
      continue;
    }
    MergePointsLeftRight(cur_lane_line, &local_map_line);
    tmp_track_id = cur_lane_line->track_id_;
    break;
  }
  for (int i = 0; i < static_cast<int>(local_map->lane_lines_.size()); i++) {
    if (local_map->lane_lines_[i].track_id_ != tmp_track_id) {
      continue;
    }
    // HLOG_ERROR << "删除地图车道线id:" << local_map->lane_lines_[i].track_id_;
    local_map->lane_lines_.erase(local_map->lane_lines_.begin() + i);
    i--;
  }
}

void MapManager::MergeMapFrontBack(LocalMap* local_map,
                                   LaneLine* cur_lane_line) {
  int tmp_track_id = 0;
  for (auto& local_map_line : local_map->lane_lines_) {
    bool need_skip = false;
    if ((local_map_line.track_id_ == cur_lane_line->track_id_) ||
        !local_map_line.ismature_ || local_map_line.points_.empty() ||
        cur_lane_line->points_.empty() ||
        (cur_lane_line->points_.front().x() >
         local_map_line.points_.back().x() + 50) ||
        (local_map_line.points_.front().x() >
         cur_lane_line->points_.back().x() + 50) ||
        ((cur_lane_line->points_.front().x() <
          local_map_line.points_.back().x()) &&
         (cur_lane_line->points_.back().x() >
          local_map_line.points_.front().x())) ||
        (fabs(local_map_line.c2_) > 0.001) ||
        (cur_lane_line->lanetype_ != local_map_line.lanetype_)) {
      continue;
    }
    for (auto& stop_line : local_map->stop_lines_) {
      if (fabs(local_map_line.end_point_x_ -
               (stop_line.left_point_.x() + stop_line.right_point_.x()) / 2) <
          5.0) {
        need_skip = true;
        break;
      }
    }
    if (need_skip) {
      continue;
    }
    int n = static_cast<int>(local_map_line.points_.size());
    if (n <= 1) {
      continue;
    }
    Eigen::MatrixXd A(n, 2);
    Eigen::VectorXd b(n);
    for (int j = 0; j < n; j++) {
      A(j, 0) = local_map_line.points_[j].x();
      A(j, 1) = 1.0;
      b(j) = local_map_line.points_[j].y();
    }
    Eigen::Vector2d x = (A.transpose() * A).inverse() * A.transpose() * b;
    double sum_distance = 0;
    int count = 0;
    for (auto cur_line_point : cur_lane_line->points_) {
      if (cur_line_point.x() > 50) {
        continue;
      }
      sum_distance +=
          std::fabs(x[0] * cur_line_point.x() - cur_line_point.y() + x[1]) /
          std::sqrt(x[0] * x[0] + 1);
      count++;
    }
    if (count == 0 || sum_distance / count > 1) {
      continue;
    }
    MergePointsFrontBack(cur_lane_line, &local_map_line);
    tmp_track_id = cur_lane_line->track_id_;
    break;
  }
  for (int i = 0; i < static_cast<int>(local_map->lane_lines_.size()); i++) {
    if (local_map->lane_lines_[i].track_id_ != tmp_track_id) {
      continue;
    }
    local_map->lane_lines_.erase(local_map->lane_lines_.begin() + i);
    i--;
  }
}

void MapManager::MergePointsLeftRight(LaneLine* cur_lane_line,
                                      LaneLine* local_map_line) {
  std::vector<Eigen::Vector3d> new_points;
  if (cur_lane_line->points_[0].x() < local_map_line->points_[0].x() &&
      cur_lane_line->points_.back().x() < local_map_line->points_.back().x()) {
    for (auto point : cur_lane_line->points_) {
      if (point.x() > local_map_line->points_[0].x() - 0.9) {
        break;
      }
      new_points.emplace_back(point);
    }
    std::copy(local_map_line->points_.begin(), local_map_line->points_.end(),
              std::back_inserter(new_points));
  } else if (cur_lane_line->points_[0].x() > local_map_line->points_[0].x() &&
             cur_lane_line->points_.back().x() >
                 local_map_line->points_.back().x()) {
    for (auto point : local_map_line->points_) {
      if (point.x() > cur_lane_line->points_[0].x() - 0.9) {
        break;
      }
      new_points.emplace_back(point);
    }
    std::copy(cur_lane_line->points_.begin(), cur_lane_line->points_.end(),
              std::back_inserter(new_points));
  } else if (cur_lane_line->points_[0].x() < local_map_line->points_[0].x() &&
             cur_lane_line->points_.back().x() >
                 local_map_line->points_.back().x()) {
    std::copy(cur_lane_line->points_.begin(), cur_lane_line->points_.end(),
              std::back_inserter(new_points));
  } else if (cur_lane_line->points_[0].x() > local_map_line->points_[0].x() &&
             cur_lane_line->points_.back().x() <
                 local_map_line->points_.back().x()) {
    std::copy(local_map_line->points_.begin(), local_map_line->points_.end(),
              std::back_inserter(new_points));
  }

  if (cur_lane_line->track_id_ < local_map_line->track_id_) {
    local_map_line->track_id_ = cur_lane_line->track_id_;
    local_map_line->lanepos_ = cur_lane_line->lanepos_;
    local_map_line->lanetype_ = cur_lane_line->lanetype_;
    local_map_line->points_ = new_points;
  } else {
    local_map_line->points_ = new_points;
  }
  if (local_map_line->count_ < cur_lane_line->count_) {
    local_map_line->count_ = cur_lane_line->count_;
  }
}

void MapManager::MergePointsFrontBack(LaneLine* cur_lane_line,
                                      LaneLine* local_map_line) {
  std::vector<Eigen::Vector3d> new_points;
  if (local_map_line->points_.front().x() > cur_lane_line->points_.back().x()) {
    if (fabs(local_map_line->points_.front().y() -
             cur_lane_line->points_.back().y()) > 1) {
      return;
    }
    for (const auto& point : cur_lane_line->points_) {
      new_points.emplace_back(point);
    }
    double x_n = local_map_line->points_.front().x();
    double y_n = local_map_line->points_.front().y();
    double x_0 = cur_lane_line->points_.back().x();
    double y_0 = cur_lane_line->points_.back().y();
    double x = x_0;
    while (x < x_n) {
      double y = (y_n - y_0) / (x_n - x_0) * (x - x_0) + y_0;
      Eigen::Vector3d point = {x, y, 0.0};
      new_points.emplace_back(point);
      x++;
    }
    for (const auto& point : local_map_line->points_) {
      new_points.emplace_back(point);
    }
  } else {
    if (fabs(cur_lane_line->points_.front().y() -
             local_map_line->points_.back().y()) > 1) {
      return;
    }
    for (const auto& point : local_map_line->points_) {
      new_points.emplace_back(point);
    }
    double x_n = cur_lane_line->points_.front().x();
    double y_n = cur_lane_line->points_.front().y();
    double x_0 = local_map_line->points_.back().x();
    double y_0 = local_map_line->points_.back().y();
    double x = x_0;
    while (x < x_n) {
      double y = (y_n - y_0) / (x_n - x_0) * (x - x_0) + y_0;
      Eigen::Vector3d point = {x, y, 0.0};
      new_points.emplace_back(point);
      x++;
    }
    for (const auto& point : cur_lane_line->points_) {
      new_points.emplace_back(point);
    }
  }
  if (cur_lane_line->track_id_ < local_map_line->track_id_) {
    local_map_line->track_id_ = cur_lane_line->track_id_;
    local_map_line->lanepos_ = cur_lane_line->lanepos_;
    local_map_line->lanetype_ = cur_lane_line->lanetype_;
    local_map_line->points_ = new_points;
  } else {
    local_map_line->points_ = new_points;
  }
  if (local_map_line->count_ < cur_lane_line->count_) {
    local_map_line->count_ = cur_lane_line->count_;
  }
}

void MapManager::MapMerge(LocalMap* local_map) {
  std::map<int, LaneLine> map;
  std::vector<LaneLine> immature_lane_lines;
  for (const auto& lane_line : local_map->lane_lines_) {
    if (!lane_line.ismature_) {
      immature_lane_lines.emplace_back(lane_line);
      continue;
    }
    map[static_cast<int>(lane_line.lanepos_)] = lane_line;
  }
  local_map->lane_lines_.clear();
  local_map->lane_lines_ = immature_lane_lines;
  for (const auto& lane_line : map) {
    local_map->lane_lines_.emplace_back(lane_line.second);
  }
  for (int i = 0; i < static_cast<int>(local_map->lane_lines_.size() - 1);
       i++) {
    if (!local_map->lane_lines_[i].ismature_) {
      continue;
    }
    LaneLine cur_lane_line = local_map->lane_lines_[i];
    LaneLine next_lane_line = local_map->lane_lines_[i + 1];
    if (cur_lane_line.points_.empty() || next_lane_line.points_.empty()) {
      continue;
    }
    double cur_front = cur_lane_line.points_.front().x();
    double cur_back = cur_lane_line.points_.back().x();
    double next_front = next_lane_line.points_.front().x();
    double next_back = next_lane_line.points_.back().x();
    // |
    // ||
    //  |
    bool condition1 =
        cur_front > next_front && cur_front < next_back && cur_back > next_back;
    // |
    //
    //  |
    bool condition2 = cur_front > next_back;
    //  |
    // ||
    // |
    bool condition3 =
        cur_front < next_front && cur_back > next_front && cur_back < next_back;
    //  |
    //
    // |
    bool condition4 = cur_back < next_front;
    //  |
    // ||
    //  |
    bool condition5 = cur_front > next_front && cur_back < next_back;
    // |
    // ||
    // |
    bool condition6 = cur_front < next_front && cur_back > next_front;
    if (condition1) {
      double sum = 0.0;
      int n = 0;
      for (const auto& cur_point : cur_lane_line.points_) {
        if (cur_point.x() > next_back) {
          break;
        }
        double next_y = next_lane_line.c0_ +
                        next_lane_line.c1_ * cur_point.x() +
                        next_lane_line.c2_ * pow(cur_point.x(), 2) +
                        next_lane_line.c3_ * pow(cur_point.x(), 3);
        sum += fabs(cur_point.y() - next_y);
        n++;
      }
      double ave_dis = sum / n;
      if (ave_dis < 0.5 && n >= 3) {
        HLOG_INFO << "condition1 ave_dis " << ave_dis << " n " << n;
        local_map->lane_lines_[i].need_delete_ = true;
        for (const auto& cur_point : cur_lane_line.points_) {
          if (cur_point.x() > next_back) {
            local_map->lane_lines_[i + 1].points_.emplace_back(cur_point);
          }
        }
      }
    } else if (condition2) {
      double ave_dis = fabs(cur_lane_line.points_.front().y() -
                            next_lane_line.points_.back().y());
      if (ave_dis < 0.3) {
        HLOG_INFO << "condition2 ave_dis " << ave_dis;
        local_map->lane_lines_[i].need_delete_ = true;
        double x0 = next_lane_line.points_.back().x();
        double y0 = next_lane_line.points_.back().y();
        double x1 = cur_lane_line.points_.front().x();
        double y1 = cur_lane_line.points_.front().y();
        double k = (y1 - y0) / (x1 - x0);
        double x = x0;
        while (x < x1 - 1) {
          x++;
          double y = k * (x - x0) + y0;
          local_map->lane_lines_[i + 1].points_.emplace_back(x, y, 0);
        }
        for (const auto& cur_point : cur_lane_line.points_) {
          local_map->lane_lines_[i + 1].points_.emplace_back(cur_point);
        }
      }
    } else if (condition3) {
      double sum = 0.0;
      int n = 0;
      for (const auto& next_point : next_lane_line.points_) {
        if (next_point.x() > cur_back) {
          break;
        }
        double cur_y = cur_lane_line.c0_ + cur_lane_line.c1_ * next_point.x() +
                       cur_lane_line.c2_ * pow(next_point.x(), 2) +
                       cur_lane_line.c3_ * pow(next_point.x(), 3);
        sum += fabs(cur_y - next_point.y());
        n++;
      }
      double ave_dis = sum / n;
      if (ave_dis < 0.5 && n >= 3) {
        HLOG_INFO << "condition3 ave_dis " << ave_dis << " n " << n;
        local_map->lane_lines_[i].need_delete_ = true;
        for (const auto& next_point : next_lane_line.points_) {
          if (next_point.x() > cur_back) {
            cur_lane_line.points_.emplace_back(next_point);
          }
        }
        local_map->lane_lines_[i + 1].points_ = cur_lane_line.points_;
      }
    } else if (condition4) {
      double ave_dis = fabs(cur_lane_line.points_.back().y() -
                            next_lane_line.points_.front().y());
      if (ave_dis < 0.3) {
        HLOG_INFO << "condition4 ave_dis " << ave_dis;
        local_map->lane_lines_[i].need_delete_ = true;
        double x0 = cur_lane_line.points_.back().x();
        double y0 = cur_lane_line.points_.back().y();
        double x1 = next_lane_line.points_.front().x();
        double y1 = next_lane_line.points_.front().y();
        double k = (y1 - y0) / (x1 - x0);
        double x = x0;
        while (x < x1 - 1) {
          x++;
          double y = k * (x - x0) + y0;
          cur_lane_line.points_.emplace_back(x, y, 0);
        }
        for (const auto& next_point : next_lane_line.points_) {
          cur_lane_line.points_.emplace_back(next_point);
        }
        local_map->lane_lines_[i + 1].points_ = cur_lane_line.points_;
      }
    } else if (condition5) {
      double sum = 0.0;
      int n = 0;
      for (const auto& cur_point : cur_lane_line.points_) {
        double next_y = next_lane_line.c0_ +
                        next_lane_line.c1_ * cur_point.x() +
                        next_lane_line.c2_ * pow(cur_point.x(), 2) +
                        next_lane_line.c3_ * pow(cur_point.x(), 3);
        sum += fabs(cur_point.y() - next_y);
        n++;
      }
      double ave_dis = sum / n;
      if (ave_dis < 0.5 && n >= 3) {
        HLOG_INFO << "condition5 ave_dis " << ave_dis << " n " << n;
        local_map->lane_lines_[i].need_delete_ = true;
      }
    } else if (condition6) {
      double sum = 0.0;
      int n = 0;
      for (const auto& next_point : next_lane_line.points_) {
        double cur_y = cur_lane_line.c0_ + cur_lane_line.c1_ * next_point.x() +
                       cur_lane_line.c2_ * pow(next_point.x(), 2) +
                       cur_lane_line.c3_ * pow(next_point.x(), 3);
        sum += fabs(cur_y - next_point.y());
        n++;
      }
      double ave_dis = sum / n;
      if (ave_dis < 0.5 && n >= 3) {
        HLOG_INFO << "condition6 ave_dis " << ave_dis << " n " << n;
        local_map->lane_lines_[i].need_delete_ = true;
        local_map->lane_lines_[i + 1].points_ = cur_lane_line.points_;
      }
    }
  }
  local_map->lane_lines_.erase(
      std::remove_if(
          local_map->lane_lines_.begin(), local_map->lane_lines_.end(),
          [&](const LaneLine& lane_line) { return lane_line.need_delete_; }),
      local_map->lane_lines_.end());
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
