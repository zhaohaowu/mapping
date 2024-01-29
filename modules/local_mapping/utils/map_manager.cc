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
  local_map->history_per_stop_lines_.erase(
      std::remove_if(
          local_map->history_per_stop_lines_.begin(),
          local_map->history_per_stop_lines_.end(),
          [&](const boost::circular_buffer<StopLine>& stop_line_buffer) {
            StopLine stop_line_tmp;
            for (const auto& stop_line : local_map->stop_lines_) {
              if (stop_line.track_id_ == stop_line_buffer.back().track_id_) {
                stop_line_tmp = stop_line;
                break;
              }
            }
            return stop_line_tmp.mid_point_.x() < -length_x ||
                   stop_line_tmp.mid_point_.x() > length_x ||
                   stop_line_tmp.mid_point_.y() < -length_y ||
                   stop_line_tmp.mid_point_.y() > length_y;
          }),
      local_map->history_per_stop_lines_.end());
  local_map->stop_lines_.erase(
      std::remove_if(local_map->stop_lines_.begin(),
                     local_map->stop_lines_.end(),
                     [&](const StopLine& stop_line) {
                       return stop_line.mid_point_.x() < -length_x ||
                              stop_line.mid_point_.x() > length_x ||
                              stop_line.mid_point_.y() < -length_y ||
                              stop_line.mid_point_.y() > length_y;
                     }),
      local_map->stop_lines_.end());
  local_map->arrows_.erase(
      std::remove_if(local_map->arrows_.begin(), local_map->arrows_.end(),
                     [&](const Arrow& arrow) {
                       return arrow.mid_point_.x() < -length_x ||
                              arrow.mid_point_.x() > length_x ||
                              arrow.mid_point_.y() < -length_y ||
                              arrow.mid_point_.y() > length_y;
                     }),
      local_map->arrows_.end());
  local_map->zebra_crossings_.erase(
      std::remove_if(local_map->zebra_crossings_.begin(),
                     local_map->zebra_crossings_.end(),
                     [&](const ZebraCrossing& zebra_crossing) {
                       return zebra_crossing.mid_point_.x() < -length_x ||
                              zebra_crossing.mid_point_.x() > length_x ||
                              zebra_crossing.mid_point_.y() < -length_y ||
                              zebra_crossing.mid_point_.y() > length_y;
                     }),
      local_map->zebra_crossings_.end());
}

void MapManager::UpdateLocalMap(LocalMap* local_map,
                                const Sophus::SE3d& T_C_L) {
  for (auto& lane_line : local_map->lane_lines_) {
    for (auto& point : lane_line.points_) {
      point = T_C_L * point;
    }
    for (auto& point : lane_line.fit_points_) {
      point = T_C_L * point;
    }
    for (auto& point : lane_line.control_points_) {
      point = T_C_L * point;
    }
  }
  for (auto& edge_line : local_map->edge_lines_) {
    for (auto& point : edge_line.points_) {
      point = T_C_L * point;
    }
    for (auto& point : edge_line.fit_points_) {
      point = T_C_L * point;
    }
    for (auto& point : edge_line.control_points_) {
      point = T_C_L * point;
    }
  }
  for (auto& stop_line : local_map->stop_lines_) {
    Eigen::Matrix3d R_C_L = T_C_L.so3().matrix();
    Eigen::Matrix3d R_L_S;
    R_L_S << cos(stop_line.heading_), -sin(stop_line.heading_), 0,
        sin(stop_line.heading_), cos(stop_line.heading_), 0, 0, 0, 1;
    Eigen::Matrix3d R_C_S = R_C_L * R_L_S;
    stop_line.heading_ = atan2(R_C_S(1, 0), R_C_S(0, 0));
    HLOG_INFO << "before stop_line.mid_point_.x() " << stop_line.mid_point_.x();
    stop_line.mid_point_ = T_C_L * stop_line.mid_point_;
    HLOG_INFO << "after stop_line.mid_point_.x() " << stop_line.mid_point_.x();
  }
  for (auto& arrow : local_map->arrows_) {
    Eigen::Matrix3d R_C_L = T_C_L.so3().matrix();
    Eigen::Matrix3d R_L_S;
    R_L_S << cos(arrow.heading_), -sin(arrow.heading_), 0, sin(arrow.heading_),
        cos(arrow.heading_), 0, 0, 0, 1;
    Eigen::Matrix3d R_C_S = R_C_L * R_L_S;
    arrow.heading_ = atan2(R_C_S(1, 0), R_C_S(0, 0));
    arrow.mid_point_ = T_C_L * arrow.mid_point_;
  }
  for (auto& zebra_crossing : local_map->zebra_crossings_) {
    Eigen::Matrix3d R_C_L = T_C_L.so3().matrix();
    Eigen::Matrix3d R_L_S;
    R_L_S << cos(zebra_crossing.heading_), -sin(zebra_crossing.heading_), 0,
        sin(zebra_crossing.heading_), cos(zebra_crossing.heading_), 0, 0, 0, 1;
    Eigen::Matrix3d R_C_S = R_C_L * R_L_S;
    zebra_crossing.heading_ = atan2(R_C_S(1, 0), R_C_S(0, 0));
    zebra_crossing.mid_point_ = T_C_L * zebra_crossing.mid_point_;
  }
  // for (auto& history_map_stop_line : local_map->history_map_stop_lines_) {
  //   for (auto& one_history_map_stop_line : history_map_stop_line) {
  //     one_history_map_stop_line.mid_point_.x() =
  //         one_history_map_stop_line.mid_point_.x() =
  //             T(0, 0) * one_history_map_stop_line.mid_point_.x() +
  //             T(0, 1) * one_history_map_stop_line.mid_point_.y() + T(0, 2);
  //   }
  // }
}

void MapManager::AddNewLaneLine(LocalMap* local_map,
                                const LaneLine& per_lane_line) {
  static int global_track_id = 0;
  if (global_track_id >= INT_MAX) {
    global_track_id = 0;
  }
  LaneLine lane_line;
  lane_line.track_id_ = global_track_id++;
  lane_line.lanepos_ = per_lane_line.lanepos_;
  lane_line.lanetype_ = per_lane_line.lanetype_;
  lane_line.color_ = per_lane_line.color_;
  lane_line.points_ = per_lane_line.points_;
  lane_line.tracked_count_ = 0;
  lane_line.ismature_ = false;
  local_map->lane_lines_.emplace_back(lane_line);
}

void MapManager::AddNewEdgeLine(LocalMap* local_map,
                                const LaneLine& per_edge_line) {
  static int global_track_id = 0;
  if (global_track_id >= INT_MAX) {
    global_track_id = 0;
  }
  LaneLine edge_line;
  edge_line.track_id_ = global_track_id++;
  edge_line.lanepos_ = per_edge_line.lanepos_;
  edge_line.edgetype_ = per_edge_line.edgetype_;
  edge_line.points_ = per_edge_line.points_;
  edge_line.tracked_count_ = 0;
  edge_line.ismature_ = false;
  local_map->edge_lines_.emplace_back(edge_line);
}

void MapManager::AddNewStopLine(LocalMap* local_map,
                                const StopLine& per_stop_line) {
  StopLine stop_line;
  StopLine temp_per_stop_line = per_stop_line;
  boost::circular_buffer<StopLine> history_per_stop_line(10);
  static int global_track_id = 0;
  if (global_track_id >= INT_MAX) {
    global_track_id = 0;
  }
  global_track_id++;
  stop_line.track_id_ = global_track_id;
  stop_line.mid_point_ = per_stop_line.mid_point_;
  stop_line.heading_ = per_stop_line.heading_;
  stop_line.length_ = per_stop_line.length_;
  stop_line.ismature_ = false;
  stop_line.isstable_ = false;
  local_map->stop_lines_.emplace_back(stop_line);
  temp_per_stop_line.track_id_ = global_track_id;
  history_per_stop_line.push_back(temp_per_stop_line);
  local_map->history_per_stop_lines_.emplace_back(history_per_stop_line);
}

void MapManager::AddNewArrow(LocalMap* local_map, const Arrow& per_arrow) {
  static int global_track_id = 0;
  if (global_track_id >= INT_MAX) {
    global_track_id = 0;
  }
  Arrow arrow;
  arrow.track_id_ = global_track_id++;
  arrow.confidence_ = per_arrow.confidence_;
  arrow.heading_ = per_arrow.heading_;
  arrow.type_ = per_arrow.type_;
  arrow.mid_point_ = per_arrow.mid_point_;
  arrow.length_ = per_arrow.length_;
  arrow.width_ = per_arrow.width_;
  arrow.ismature_ = false;
  local_map->arrows_.emplace_back(arrow);
}

void MapManager::AddNewZebraCrossing(LocalMap* local_map,
                                     const ZebraCrossing& per_zebra_crossing) {
  static int global_track_id = 0;
  if (global_track_id >= INT_MAX) {
    global_track_id = 0;
  }
  ZebraCrossing zebra_crossing;
  zebra_crossing.track_id_ = global_track_id++;
  zebra_crossing.confidence_ = per_zebra_crossing.confidence_;
  zebra_crossing.heading_ = per_zebra_crossing.heading_;
  zebra_crossing.mid_point_ = per_zebra_crossing.mid_point_;
  zebra_crossing.length_ = per_zebra_crossing.length_;
  zebra_crossing.width_ = per_zebra_crossing.width_;
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
    double y = CommonUtil::CalCubicCurveY(per_lane_line, x);
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
        lane_line.tracked_count_++;
        if (lane_line.tracked_count_ >= 10) {
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
    double y = CommonUtil::CalCubicCurveY(per_edge_line, x);
    new_pts.emplace_back(x, y, 0);
    x += 1;
  }
  // HLOG_ERROR << "map_lane.track_id_: " << map_lane.track_id_;
  for (auto& edge_line : local_map->edge_lines_) {
    if (edge_line.track_id_ == map_edge_line.track_id_) {
      edge_line.points_ = new_pts;
      edge_line.lanepos_ = per_edge_line.lanepos_;
      edge_line.edgetype_ = per_edge_line.edgetype_;
      if (!edge_line.ismature_) {
        edge_line.tracked_count_++;
        if (edge_line.tracked_count_ >= 10) {
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
        stop_line.tracked_count_++;
        if (stop_line.tracked_count_ >= 10) {
          stop_line.ismature_ = true;
        }
      }
      if (!stop_line.isstable_) {
        for (const auto& history_per_stop_line :
             local_map->history_per_stop_lines_) {
          if (history_per_stop_line.back().track_id_ ==
                  map_stop_line.track_id_ &&
              history_per_stop_line.size() == 10) {
            double max_x = DBL_MIN;
            double min_x = DBL_MAX;
            double max_y = DBL_MIN;
            double min_y = DBL_MAX;
            double max_heading = DBL_MIN;
            double min_heading = DBL_MAX;
            double max_length = DBL_MIN;
            double min_length = DBL_MAX;
            for (const auto& temp_stop_line : history_per_stop_line) {
              max_x = fmax(max_x, temp_stop_line.mid_point_.x());
              min_x = fmin(min_x, temp_stop_line.mid_point_.x());
              max_y = fmax(max_y, temp_stop_line.mid_point_.y());
              min_y = fmin(min_y, temp_stop_line.mid_point_.y());
              max_heading = fmax(max_heading, temp_stop_line.heading_);
              min_heading = fmin(min_heading, temp_stop_line.heading_);
              max_length = fmax(max_length, temp_stop_line.length_);
              min_length = fmin(min_length, temp_stop_line.length_);
            }
            if (max_x - min_x < 0.3 && max_y - min_y < 0.1 &&
                max_heading - min_heading < 2 &&
                max_length - min_length < 0.3) {
              stop_line.heading_ = (max_heading + min_heading) / 2;
              stop_line.mid_point_ =
                  Eigen::Vector3d{(max_x + min_x) / 2, (max_y + min_y) / 2, 0};
              stop_line.length_ = (max_length + min_length) / 2;
              stop_line.isstable_ = true;
            }
          }
        }
        for (auto& history_per_stop_line : local_map->history_per_stop_lines_) {
          if (history_per_stop_line.back().track_id_ ==
              map_stop_line.track_id_) {
            StopLine temp_per_stop_line = per_stop_line;
            temp_per_stop_line.track_id_ = map_stop_line.track_id_;
            history_per_stop_line.push_back(temp_per_stop_line);
            break;
          }
        }
      }
      if (!stop_line.isstable_) {
        stop_line.confidence_ = per_stop_line.confidence_;
        stop_line.mid_point_ =
            0.5 * per_stop_line.mid_point_ + 0.5 * map_stop_line.mid_point_;
        stop_line.length_ =
            0.5 * per_stop_line.length_ + 0.5 * map_stop_line.length_;
        LaneLine left_lane_line;
        LaneLine right_lane_line;
        for (const auto& laneline : local_map->lane_lines_) {
          if (laneline.is_after_stop_line_) {
            continue;
          }
          if (laneline.lanepos_ == -1) {
            left_lane_line = laneline;
          }
          if (laneline.lanepos_ == 1) {
            right_lane_line = laneline;
          }
        }
        int left_size = static_cast<int>(left_lane_line.fit_points_.size());
        int right_size = static_cast<int>(right_lane_line.fit_points_.size());
        if (left_size < 2 || right_size < 2) {
          stop_line.heading_ = per_stop_line.heading_;
        } else {
          std::vector<Eigen::Vector3d> left_points;
          std::vector<Eigen::Vector3d> right_points;
          left_points.emplace_back(
              left_lane_line.fit_points_[left_size - 2].x(),
              left_lane_line.fit_points_[left_size - 2].y(), 0);
          left_points.emplace_back(
              left_lane_line.fit_points_[left_size - 1].x(),
              left_lane_line.fit_points_[left_size - 1].y(), 0);
          right_points.emplace_back(
              right_lane_line.fit_points_[right_size - 2].x(),
              right_lane_line.fit_points_[right_size - 2].y(), 0);
          right_points.emplace_back(
              right_lane_line.fit_points_[right_size - 1].x(),
              right_lane_line.fit_points_[right_size - 1].y(), 0);
          stop_line.heading_ =
              CommonUtil::CalMainLaneHeading(left_points, right_points) + 1.57;
        }
      }
    }
  }
}

void MapManager::MergeOldArrow(LocalMap* local_map, const Arrow& per_arrow,
                               const Arrow& map_arrow) {
  for (auto& arrow : local_map->arrows_) {
    if (arrow.track_id_ == map_arrow.track_id_) {
      if (!arrow.ismature_) {
        arrow.tracked_count_++;
        if (arrow.tracked_count_ >= 10) {
          arrow.ismature_ = true;
        }
      }
      arrow.confidence_ = per_arrow.confidence_;
      arrow.type_ = per_arrow.type_;
      arrow.mid_point_ =
          0.5 * per_arrow.mid_point_ + 0.5 * map_arrow.mid_point_;
      arrow.length_ = 0.5 * per_arrow.length_ + 0.5 * map_arrow.length_;
      arrow.width_ = 0.5 * per_arrow.width_ + 0.5 * map_arrow.width_;
      LaneLine left_lane_line;
      LaneLine right_lane_line;
      for (const auto& laneline : local_map->lane_lines_) {
        if (laneline.is_after_stop_line_) {
          continue;
        }
        if (laneline.lanepos_ == -1) {
          left_lane_line = laneline;
        }
        if (laneline.lanepos_ == 1) {
          right_lane_line = laneline;
        }
      }
      if (left_lane_line.lanepos_ != -1 || right_lane_line.lanepos_ != 1 ||
          left_lane_line.c1_ == 1000 || right_lane_line.c1_ == 1000) {
        arrow.heading_ = per_arrow.heading_;
      } else {
        std::vector<Eigen::Vector3d> left_points;
        std::vector<Eigen::Vector3d> right_points;
        left_points.emplace_back(arrow.mid_point_.x() - 1,
                                 CommonUtil::CalCubicCurveY(
                                     left_lane_line, arrow.mid_point_.x() - 1),
                                 0);
        left_points.emplace_back(
            arrow.mid_point_.x(),
            CommonUtil::CalCubicCurveY(left_lane_line, arrow.mid_point_.x()),
            0);
        right_points.emplace_back(
            arrow.mid_point_.x() - 1,
            CommonUtil::CalCubicCurveY(right_lane_line,
                                       arrow.mid_point_.x() - 1),
            0);
        right_points.emplace_back(
            arrow.mid_point_.x(),
            CommonUtil::CalCubicCurveY(right_lane_line, arrow.mid_point_.x()),
            0);
        arrow.heading_ =
            CommonUtil::CalMainLaneHeading(left_points, right_points);
      }
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
        zebra_crossing.tracked_count_++;
        if (zebra_crossing.tracked_count_ >= 10) {
          zebra_crossing.ismature_ = true;
        }
      }
      zebra_crossing.confidence_ = per_zebra_crossing.confidence_;
      zebra_crossing.mid_point_ = 0.5 * per_zebra_crossing.mid_point_ +
                                  0.5 * map_zebra_crossing.mid_point_;
      zebra_crossing.length_ =
          0.5 * per_zebra_crossing.length_ + 0.5 * map_zebra_crossing.length_;
      zebra_crossing.width_ =
          0.5 * per_zebra_crossing.width_ + 0.5 * map_zebra_crossing.width_;
      LaneLine before_left_lane_line;
      LaneLine before_right_lane_line;
      LaneLine after_left_lane_line;
      LaneLine after_right_lane_line;
      for (const auto& laneline : local_map->lane_lines_) {
        if (!laneline.is_after_stop_line_) {
          if (laneline.lanepos_ == -1) {
            before_left_lane_line = laneline;
          } else if (laneline.lanepos_ == 1) {
            before_right_lane_line = laneline;
          }
        } else if (laneline.is_after_stop_line_) {
          if (laneline.lanepos_ == -1) {
            after_left_lane_line = laneline;
          } else if (laneline.lanepos_ == 1) {
            after_right_lane_line = laneline;
          }
        }
      }
      double before_dis = FLT_MAX;
      double after_dis = FLT_MAX;
      if (before_left_lane_line.fit_points_.empty() &&
          !before_right_lane_line.fit_points_.empty()) {
        before_dis = fabs(zebra_crossing.mid_point_.x() -
                          before_right_lane_line.fit_points_.back().x());
      } else if (!before_left_lane_line.fit_points_.empty() &&
                 before_right_lane_line.fit_points_.empty()) {
        before_dis = fabs(zebra_crossing.mid_point_.x() -
                          before_left_lane_line.fit_points_.back().x());
      } else if (!before_left_lane_line.fit_points_.empty() &&
                 !before_right_lane_line.fit_points_.empty()) {
        before_dis = fabs(zebra_crossing.mid_point_.x() -
                          (before_left_lane_line.fit_points_.back().x() +
                           before_right_lane_line.fit_points_.back().x()) /
                              2);
      }
      if (after_left_lane_line.fit_points_.empty() &&
          !after_right_lane_line.fit_points_.empty()) {
        after_dis = fabs(zebra_crossing.mid_point_.x() -
                         after_right_lane_line.fit_points_.front().x());
      } else if (!after_left_lane_line.fit_points_.empty() &&
                 after_right_lane_line.fit_points_.empty()) {
        after_dis = fabs(zebra_crossing.mid_point_.x() -
                         after_left_lane_line.fit_points_.front().x());
      } else if (!after_left_lane_line.fit_points_.empty() &&
                 !after_right_lane_line.fit_points_.empty()) {
        after_dis = fabs(zebra_crossing.mid_point_.x() -
                         (after_left_lane_line.fit_points_.front().x() +
                          after_right_lane_line.fit_points_.front().x()) /
                             2);
      }
      LaneLine left_lane_line;
      LaneLine right_lane_line;
      HLOG_INFO << "before_dis " << before_dis;
      HLOG_INFO << "after_dis " << after_dis;
      if (before_dis < 10 && after_dis > 10) {
        left_lane_line = before_left_lane_line;
        right_lane_line = before_right_lane_line;
      } else if (before_dis > 10 && after_dis < 10) {
        left_lane_line = after_left_lane_line;
        right_lane_line = after_right_lane_line;
      }
      int left_size = static_cast<int>(left_lane_line.fit_points_.size());
      int right_size = static_cast<int>(right_lane_line.fit_points_.size());
      HLOG_INFO << "left_size " << left_size;
      HLOG_INFO << "right_size " << right_size;
      if (left_size < 2 || right_size < 2) {
        zebra_crossing.heading_ = 0.5 * per_zebra_crossing.heading_ +
                                  0.5 * map_zebra_crossing.heading_;
      } else {
        std::vector<Eigen::Vector3d> left_points;
        std::vector<Eigen::Vector3d> right_points;
        left_points.emplace_back(left_lane_line.fit_points_[left_size - 2].x(),
                                 left_lane_line.fit_points_[left_size - 2].y(),
                                 0);
        left_points.emplace_back(left_lane_line.fit_points_[left_size - 1].x(),
                                 left_lane_line.fit_points_[left_size - 1].y(),
                                 0);
        right_points.emplace_back(
            right_lane_line.fit_points_[right_size - 2].x(),
            right_lane_line.fit_points_[right_size - 2].y(), 0);
        right_points.emplace_back(
            right_lane_line.fit_points_[right_size - 1].x(),
            right_lane_line.fit_points_[right_size - 1].y(), 0);
        zebra_crossing.heading_ =
            CommonUtil::CalMainLaneHeading(left_points, right_points);
      }
      return;
    }
  }
}

void MapManager::UpdateLanepos(LocalMap* local_map) {
  std::map<double, LaneLine> before_left_lane_lines;
  std::map<double, LaneLine, std::greater<>> before_right_lane_lines;
  std::map<double, LaneLine> after_left_lane_lines;
  std::map<double, LaneLine, std::greater<>> after_right_lane_lines;
  std::vector<LaneLine> unknown_lanepos_lane_lines;
  StopLine nearby_stop_line;
  double min_x = FLT_MAX;
  bool has_nearby_stop_line = false;
  for (const auto& stop_line : local_map->stop_lines_) {
    if (!stop_line.ismature_) {
      continue;
    }
    if (stop_line.mid_point_.x() > -50 && stop_line.mid_point_.x() < 50 &&
        fabs(stop_line.mid_point_.x()) < min_x) {
      min_x = fabs(stop_line.mid_point_.x());
      nearby_stop_line = stop_line;
      has_nearby_stop_line = true;
    }
  }
  for (auto& lane_line : local_map->lane_lines_) {
    if (lane_line.points_.empty()) {
      continue;
    }
    if (lane_line.points_.back().x() - lane_line.points_.front().x() < 10 ||
        lane_line.points_.back().x() < -50 ||
        lane_line.points_.front().x() > 50 || !lane_line.ismature_) {
      lane_line.lanepos_ = LanePositionType::OTHER;
      unknown_lanepos_lane_lines.emplace_back(lane_line);
      continue;
    }
    if (lane_line.points_.back().x() < 0) {
      lane_line.c0_for_lanepos_ = lane_line.points_.back().y();
    } else if (lane_line.points_.front().x() > 0) {
      lane_line.c0_for_lanepos_ = lane_line.points_.front().y();
    } else {
      lane_line.c0_for_lanepos_ = lane_line.c0_;
    }
    lane_line.is_after_stop_line_ =
        has_nearby_stop_line &&
        lane_line.points_.front().x() > nearby_stop_line.mid_point_.x();
    HLOG_INFO << "has_nearby_stop_line " << has_nearby_stop_line;
    HLOG_INFO << " lane_line.track_id " << lane_line.track_id_ << " "
              << lane_line.is_after_stop_line_;
    HLOG_INFO << lane_line.points_.front().x() << " "
              << nearby_stop_line.mid_point_.x();
    if (lane_line.is_after_stop_line_) {
      if (lane_line.c0_for_lanepos_ > 0) {
        after_left_lane_lines[lane_line.c0_for_lanepos_] = lane_line;
      } else if (lane_line.c0_for_lanepos_ < 0) {
        after_right_lane_lines[lane_line.c0_for_lanepos_] = lane_line;
      }
    } else {
      if (lane_line.c0_for_lanepos_ > 0) {
        before_left_lane_lines[lane_line.c0_for_lanepos_] = lane_line;
      } else if (lane_line.c0_for_lanepos_ < 0) {
        before_right_lane_lines[lane_line.c0_for_lanepos_] = lane_line;
      }
    }
    HLOG_INFO << "lane_line.lanepos " << lane_line.lanepos_
              << "lane_line.c0_for_lanepos_ " << lane_line.c0_for_lanepos_;
  }
  local_map->lane_lines_.clear();
  local_map->lane_lines_ = unknown_lanepos_lane_lines;
  auto pre_it = before_left_lane_lines.begin();
  if (pre_it != before_left_lane_lines.end()) {
    pre_it->second.lanepos_ = static_cast<LanePositionType>(-1);
    local_map->lane_lines_.emplace_back(pre_it->second);
    int left_index = -2;
    for (auto it = std::next(pre_it); it != before_left_lane_lines.end();
         it++) {
      if (it->first - pre_it->first > 1) {
        it->second.lanepos_ = static_cast<LanePositionType>(left_index--);
        local_map->lane_lines_.emplace_back(it->second);
      } else {
        it->second.lanepos_ = pre_it->second.lanepos_;
        local_map->lane_lines_.emplace_back(it->second);
      }
      pre_it = it;
    }
  }
  pre_it = before_right_lane_lines.begin();
  if (pre_it != before_right_lane_lines.end()) {
    pre_it->second.lanepos_ = static_cast<LanePositionType>(1);
    local_map->lane_lines_.emplace_back(pre_it->second);
    int right_index = 2;
    for (auto it = std::next(pre_it); it != before_right_lane_lines.end();
         it++) {
      if (pre_it->first - it->first > 1) {
        it->second.lanepos_ = static_cast<LanePositionType>(right_index++);
        local_map->lane_lines_.emplace_back(it->second);
      } else {
        it->second.lanepos_ = pre_it->second.lanepos_;
        local_map->lane_lines_.emplace_back(it->second);
      }
      pre_it = it;
    }
  }
  pre_it = after_left_lane_lines.begin();
  if (pre_it != after_left_lane_lines.end()) {
    pre_it->second.lanepos_ = static_cast<LanePositionType>(-1);
    local_map->lane_lines_.emplace_back(pre_it->second);
    int left_index = -2;
    for (auto it = std::next(pre_it); it != after_left_lane_lines.end(); it++) {
      if (it->first - pre_it->first > 1) {
        it->second.lanepos_ = static_cast<LanePositionType>(left_index--);
        local_map->lane_lines_.emplace_back(it->second);
      } else {
        it->second.lanepos_ = pre_it->second.lanepos_;
        local_map->lane_lines_.emplace_back(it->second);
      }
      pre_it = it;
    }
  }
  pre_it = after_right_lane_lines.begin();
  if (pre_it != after_right_lane_lines.end()) {
    pre_it->second.lanepos_ = static_cast<LanePositionType>(1);
    local_map->lane_lines_.emplace_back(pre_it->second);
    int right_index = 2;
    for (auto it = std::next(pre_it); it != after_right_lane_lines.end();
         it++) {
      if (pre_it->first - it->first > 1) {
        it->second.lanepos_ = static_cast<LanePositionType>(right_index++);
        local_map->lane_lines_.emplace_back(it->second);
      } else {
        it->second.lanepos_ = pre_it->second.lanepos_;
        local_map->lane_lines_.emplace_back(it->second);
      }
      pre_it = it;
    }
  }
  std::map<double, LaneLine> left_edge_lines;
  std::map<double, LaneLine, std::greater<>> right_edge_lines;
  std::vector<LaneLine> unknown_lanepos_edge_lines;
  for (auto& edge_line : local_map->edge_lines_) {
    if (edge_line.points_.empty()) {
      continue;
    }
    if (edge_line.points_.back().x() - edge_line.points_.front().x() < 10 ||
        edge_line.points_.back().x() < -60 ||
        edge_line.points_.front().x() > 60 || !edge_line.ismature_) {
      edge_line.lanepos_ = LanePositionType::OTHER;
      unknown_lanepos_edge_lines.emplace_back(edge_line);
      continue;
    }
    if (edge_line.points_.back().x() < 0) {
      edge_line.c0_ = edge_line.points_.back().y();
    } else if (edge_line.points_.front().x() > 0) {
      edge_line.c0_ = edge_line.points_.front().y();
    } else {
      std::vector<Eigen::Vector3d> points;
      for (const auto& point : edge_line.points_) {
        if (point.x() > -10 && point.x() < 10) {
          points.emplace_back(point);
        }
      }
      if (points.size() < 4) {
        edge_line.lanepos_ = LanePositionType::OTHER;
        unknown_lanepos_lane_lines.emplace_back(edge_line);
        continue;
      }
      std::vector<double> c(4);
      CommonUtil::FitLaneLine(points, &c);
      edge_line.c0_ = c[0];
      edge_line.c1_ = c[1];
      edge_line.c2_ = c[2];
      edge_line.c3_ = c[3];
    }
    if (edge_line.c0_ > 0) {
      left_edge_lines[edge_line.c0_] = edge_line;
    } else if (edge_line.c0_ < 0) {
      right_edge_lines[edge_line.c0_] = edge_line;
    }
    HLOG_INFO << "edge_line.lanepos " << edge_line.lanepos_ << "edge_line.c0 "
              << edge_line.c0_;
  }
  local_map->edge_lines_.clear();
  local_map->edge_lines_ = unknown_lanepos_edge_lines;
  pre_it = left_edge_lines.begin();
  if (pre_it != left_edge_lines.end()) {
    pre_it->second.lanepos_ = static_cast<LanePositionType>(-1);
    local_map->edge_lines_.emplace_back(pre_it->second);
    int left_index = -2;
    for (auto it = std::next(pre_it); it != left_edge_lines.end(); it++) {
      if (it->first - pre_it->first > 3) {
        it->second.lanepos_ = static_cast<LanePositionType>(left_index--);
        local_map->edge_lines_.emplace_back(it->second);
      } else {
        it->second.lanepos_ = pre_it->second.lanepos_;
        local_map->edge_lines_.emplace_back(it->second);
      }
      pre_it = it;
    }
  }
  pre_it = right_edge_lines.begin();
  if (pre_it != right_edge_lines.end()) {
    pre_it->second.lanepos_ = static_cast<LanePositionType>(1);
    local_map->edge_lines_.emplace_back(pre_it->second);
    int right_index = 2;
    for (auto it = std::next(pre_it); it != right_edge_lines.end(); it++) {
      if (pre_it->first - it->first > 3) {
        it->second.lanepos_ = static_cast<LanePositionType>(right_index++);
        local_map->edge_lines_.emplace_back(it->second);
      } else {
        it->second.lanepos_ = pre_it->second.lanepos_;
        local_map->edge_lines_.emplace_back(it->second);
      }
      pre_it = it;
    }
  }
}

void MapManager::MergeLaneLine(LocalMap* local_map) {
  std::multimap<int, LaneLine> multimap;
  std::vector<LaneLine> unknown_lanepos_lane_lines;
  for (const auto& lane_line : local_map->lane_lines_) {
    if (lane_line.lanepos_ == LanePositionType::OTHER) {
      unknown_lanepos_lane_lines.emplace_back(lane_line);
      continue;
    }
    multimap.insert({static_cast<int>(lane_line.lanepos_), lane_line});
  }
  local_map->lane_lines_.clear();
  local_map->lane_lines_ = unknown_lanepos_lane_lines;
  for (const auto& element : multimap) {
    local_map->lane_lines_.emplace_back(element.second);
  }
  for (int i = 0; i < static_cast<int>(local_map->lane_lines_.size() - 1);
       i++) {
    if (local_map->lane_lines_[i].lanepos_ == LanePositionType::OTHER) {
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
        HLOG_INFO << "track_id " << cur_lane_line.track_id_ << ", "
                  << next_lane_line.track_id_;
        HLOG_INFO << "lanepos " << cur_lane_line.lanepos_ << ", "
                  << next_lane_line.lanepos_;
        HLOG_INFO << "condition1 ave_dis " << ave_dis << " n " << n;
        local_map->lane_lines_[i].need_delete_ = true;
        for (const auto& cur_point : cur_lane_line.points_) {
          if (cur_point.x() > next_back) {
            local_map->lane_lines_[i + 1].points_.emplace_back(cur_point);
          }
        }
      }
    } else if (condition2) {
      bool need_skip = false;
      for (auto& stop_line : local_map->stop_lines_) {
        if (fabs(cur_front - stop_line.mid_point_.x()) < 5.0 ||
            fabs(next_back - stop_line.mid_point_.x()) < 5.0) {
          need_skip = true;
          break;
        }
      }
      if (need_skip) {
        continue;
      }
      double ave_dis = fabs(cur_lane_line.points_.front().y() -
                            next_lane_line.points_.back().y());
      if (ave_dis < 1) {
        HLOG_INFO << "track_id " << cur_lane_line.track_id_ << ", "
                  << next_lane_line.track_id_;
        HLOG_INFO << "lanepos " << cur_lane_line.lanepos_ << ", "
                  << next_lane_line.lanepos_;
        HLOG_INFO << "condition2 ave_dis " << ave_dis;
        local_map->lane_lines_[i].need_delete_ = true;
        double x0 = next_lane_line.points_.back().x();
        double y0 = next_lane_line.points_.back().y();
        double x1 = cur_lane_line.points_.front().x();
        double y1 = cur_lane_line.points_.front().y();
        double k = (y1 - y0) / (x1 - x0);
        double x = x0;
        while (x < x1 - 1) {
          x = x + 1.0;
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
        HLOG_INFO << "track_id " << cur_lane_line.track_id_ << ", "
                  << next_lane_line.track_id_;
        HLOG_INFO << "lanepos " << cur_lane_line.lanepos_ << ", "
                  << next_lane_line.lanepos_;
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
      bool need_skip = false;
      for (auto& stop_line : local_map->stop_lines_) {
        if (fabs(cur_back - stop_line.mid_point_.x()) < 5.0 ||
            fabs(next_front - stop_line.mid_point_.x()) < 5.0) {
          need_skip = true;
          break;
        }
      }
      if (need_skip) {
        continue;
      }
      double ave_dis = fabs(cur_lane_line.points_.back().y() -
                            next_lane_line.points_.front().y());
      if (ave_dis < 1) {
        HLOG_INFO << "track_id " << cur_lane_line.track_id_ << ", "
                  << next_lane_line.track_id_;
        HLOG_INFO << "lanepos " << cur_lane_line.lanepos_ << ", "
                  << next_lane_line.lanepos_;
        HLOG_INFO << "condition4 ave_dis " << ave_dis;
        local_map->lane_lines_[i].need_delete_ = true;
        double x0 = cur_lane_line.points_.back().x();
        double y0 = cur_lane_line.points_.back().y();
        double x1 = next_lane_line.points_.front().x();
        double y1 = next_lane_line.points_.front().y();
        double k = (y1 - y0) / (x1 - x0);
        double x = x0;
        while (x < x1 - 1) {
          x = x + 1.0;
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
        HLOG_INFO << "track_id " << cur_lane_line.track_id_ << ", "
                  << next_lane_line.track_id_;
        HLOG_INFO << "lanepos " << cur_lane_line.lanepos_ << ", "
                  << next_lane_line.lanepos_;
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
        HLOG_INFO << "track_id " << cur_lane_line.track_id_ << ", "
                  << next_lane_line.track_id_;
        HLOG_INFO << "lanepos " << cur_lane_line.lanepos_ << ", "
                  << next_lane_line.lanepos_;
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

void MapManager::MatchLaneLine(
    const std::vector<LaneLine>& per_lane_lines,
    const std::vector<LaneLine>& map_lane_lines,
    std::vector<LaneLineMatchInfo>* lane_line_matches) {
  std::vector<LaneLine> map_lane_lines_tmp = map_lane_lines;
  for (const auto& per_lane_line : per_lane_lines) {
    bool per_has_matched = false;
    LaneLineMatchInfo match;
    HLOG_INFO << "per_lane_line.lanepos_ " << per_lane_line.lanepos_;
    double min_ave_dis = FLT_MAX;
    int map_lane_index = 0;
    for (int i = 0; i < static_cast<int>(map_lane_lines_tmp.size()); i++) {
      if (map_lane_lines_tmp[i].has_matched_) {
        continue;
      }
      double sum = 0.0;
      int n = 0;
      double linear_max_x =
          per_lane_line.end_point_x_ * 0.7 + per_lane_line.start_point_x_ * 0.3;
      double curve_max_x =
          per_lane_line.end_point_x_ * 0.5 + per_lane_line.start_point_x_ * 0.5;
      for (const auto& map_point : map_lane_lines_tmp[i].points_) {
        if ((per_lane_line.c2_ > 0.001 ? map_point.x() > curve_max_x
                                       : map_point.x() > linear_max_x) ||
            map_point.x() < per_lane_line.start_point_x_) {
          continue;
        }
        double per_y = CommonUtil::CalCubicCurveY(per_lane_line, map_point.x());
        sum += fabs(map_point.y() - per_y);
        n++;
      }
      if (n == 0) {
        continue;
      }
      double ave_dis = sum / n;
      HLOG_INFO << "map_lane_lines_tmp[i].lanepos_ "
                << map_lane_lines_tmp[i].lanepos_;
      HLOG_INFO << "ave_dis " << ave_dis << " n " << n;
      if (ave_dis < min_ave_dis && n >= 3) {
        min_ave_dis = ave_dis;
        map_lane_index = i;
      }
    }
    if (min_ave_dis < 0.5) {
      match.per_lane_line_ = per_lane_line;
      match.map_lane_line_ = map_lane_lines_tmp[map_lane_index];
      match.update_type_ = ObjUpdateType::MERGE_OLD;
      per_has_matched = true;
      map_lane_lines_tmp[map_lane_index].has_matched_ = true;
    }
    if (!per_has_matched) {
      match.per_lane_line_ = per_lane_line;
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    lane_line_matches->emplace_back(match);
  }
}

void MapManager::MatchEdgeLine(
    const std::vector<LaneLine>& per_edge_lines,
    const std::vector<LaneLine>& map_edge_lines,
    std::vector<EdgeLineMatchInfo>* edge_line_matches) {
  std::vector<LaneLine> map_edge_lines_tmp = map_edge_lines;
  for (const auto& per_edge_line : per_edge_lines) {
    ObjUpdateType type = ObjUpdateType::ADD_NEW;
    bool per_has_matched = false;
    EdgeLineMatchInfo match;
    HLOG_INFO << "per_edge_line.lanepos_ " << per_edge_line.lanepos_;
    double min_ave_dis = 0.5;
    int map_lane_index = 0;
    for (int i = 0; i < static_cast<int>(map_edge_lines_tmp.size()); i++) {
      if (map_edge_lines_tmp[i].has_matched_) {
        continue;
      }
      double sum = 0.0;
      int n = 0;
      for (const auto& map_point : map_edge_lines_tmp[i].points_) {
        if (map_point.x() > per_edge_line.end_point_x_ ||
            map_point.x() < per_edge_line.start_point_x_) {
          continue;
        }
        double per_y = CommonUtil::CalCubicCurveY(per_edge_line, map_point.x());
        sum += fabs(map_point.y() - per_y);
        n++;
      }
      double ave_dis = sum / n;
      HLOG_INFO << "map_edge_line.lanepos_ " << map_edge_lines_tmp[i].lanepos_;
      HLOG_INFO << "ave_dis " << ave_dis << " n " << n;
      if (ave_dis < min_ave_dis && n >= 3) {
        min_ave_dis = ave_dis;
        map_lane_index = i;
      }
    }
    if (min_ave_dis < 0.5) {
      match.per_edge_line_ = per_edge_line;
      match.map_edge_line_ = map_edge_lines_tmp[map_lane_index];
      match.update_type_ = ObjUpdateType::MERGE_OLD;
      per_has_matched = true;
      map_edge_lines_tmp[map_lane_index].has_matched_ = true;
    }
    if (!per_has_matched) {
      match.per_edge_line_ = per_edge_line;
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    edge_line_matches->emplace_back(match);
  }
}

void MapManager::MatchStopLine(
    const std::vector<StopLine>& per_stop_lines,
    const std::vector<StopLine>& map_stop_lines,
    std::vector<StopLineMatchInfo>* stop_line_matches) {
  std::vector<StopLine> map_stop_lines_tmp = map_stop_lines;
  for (const auto& per_stop_line : per_stop_lines) {
    if (per_stop_line.mid_point_.x() < 10) {
      continue;
    }

    bool per_has_matched = false;
    StopLineMatchInfo match;
    double min_x_dis = FLT_MAX;
    int map_stop_line_index = 0;
    for (int i = 0; i < static_cast<int>(map_stop_lines_tmp.size()); i++) {
      if (map_stop_lines_tmp[i].has_matched_) {
        continue;
      }
      if (fabs(per_stop_line.mid_point_.x() -
               map_stop_lines_tmp[i].mid_point_.x()) < min_x_dis) {
        min_x_dis = fabs(per_stop_line.mid_point_.x() -
                         map_stop_lines_tmp[i].mid_point_.x());
        map_stop_line_index = i;
      }
    }
    if (min_x_dis < 5.0) {
      match.per_stop_line_ = per_stop_line;
      match.map_stop_line_ = map_stop_lines_tmp[map_stop_line_index];
      match.update_type_ = ObjUpdateType::MERGE_OLD;
      per_has_matched = true;
      map_stop_lines_tmp[map_stop_line_index].has_matched_ = true;
    }
    if (!per_has_matched) {
      match.per_stop_line_ = per_stop_line;
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    stop_line_matches->emplace_back(match);
  }
}

void MapManager::MatchArrow(const std::vector<Arrow>& per_arrows,
                            const std::vector<Arrow>& map_arrows,
                            std::vector<ArrowMatchInfo>* arrow_matches) {
  std::vector<Arrow> map_arrows_tmp = map_arrows;
  for (const auto& per_arrow : per_arrows) {
    if (per_arrow.mid_point_.x() < 10) {
      continue;
    }
    ObjUpdateType type = ObjUpdateType::ADD_NEW;
    bool per_has_matched = false;
    ArrowMatchInfo match;
    double min_x_dis = FLT_MAX;
    int map_arrow_index = 0;
    for (int i = 0; i < static_cast<int>(map_arrows_tmp.size()); i++) {
      if (map_arrows_tmp[i].has_matched_) {
        continue;
      }
      if (fabs(per_arrow.mid_point_.x() - map_arrows_tmp[i].mid_point_.x()) <
              min_x_dis &&
          fabs(per_arrow.mid_point_.y() - map_arrows_tmp[i].mid_point_.y()) <
              2) {
        min_x_dis =
            fabs(per_arrow.mid_point_.x() - map_arrows_tmp[i].mid_point_.x());
        map_arrow_index = i;
      }
      if (min_x_dis < 10.0) {
        match.per_arrow_ = per_arrow;
        match.map_arrow_ = map_arrows_tmp[map_arrow_index];
        match.update_type_ = ObjUpdateType::MERGE_OLD;
        per_has_matched = true;
        map_arrows_tmp[map_arrow_index].has_matched_ = true;
      }
    }
    if (!per_has_matched) {
      match.per_arrow_ = per_arrow;
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    arrow_matches->emplace_back(match);
  }
}

void MapManager::MatchZebraCrossing(
    const std::vector<ZebraCrossing>& per_zebra_crossings,
    const std::vector<ZebraCrossing>& map_zebra_crossings,
    std::vector<ZebraCrossingMatchInfo>* zebra_crossing_matches) {
  std::vector<ZebraCrossing> map_zebra_crossings_tmp = map_zebra_crossings;
  for (const auto& per_zebra_crossing : per_zebra_crossings) {
    if (per_zebra_crossing.mid_point_.x() < 10) {
      continue;
    }
    bool per_has_matched = false;
    ZebraCrossingMatchInfo match;
    double min_x_dis = FLT_MAX;
    int map_zebra_crossing_index = 0;
    for (int i = 0; i < static_cast<int>(map_zebra_crossings_tmp.size()); i++) {
      if (map_zebra_crossings_tmp[i].has_matched_) {
        continue;
      }
      if (fabs(per_zebra_crossing.mid_point_.x() -
               map_zebra_crossings_tmp[i].mid_point_.x()) < min_x_dis) {
        min_x_dis = fabs(per_zebra_crossing.mid_point_.x() -
                         map_zebra_crossings_tmp[i].mid_point_.x());
        map_zebra_crossing_index = i;
      }
      if (min_x_dis < 5.0) {
        match.per_zebra_crossing_ = per_zebra_crossing;
        match.map_zebra_crossing_ =
            map_zebra_crossings_tmp[map_zebra_crossing_index];
        match.update_type_ = ObjUpdateType::MERGE_OLD;
        per_has_matched = true;
        map_zebra_crossings_tmp[map_zebra_crossing_index].has_matched_ = true;
      }
    }
    if (!per_has_matched) {
      match.per_zebra_crossing_ = per_zebra_crossing;
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    zebra_crossing_matches->emplace_back(match);
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
