/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei/zhaohaowu
 *Date: 2023-09-01
 *****************************************************************************/
#include "modules/local_mapping/lib/utils/map_manager.h"

#include "modules/local_mapping/lib/utils/common.h"

namespace hozon {
namespace mp {
namespace lm {

void MapManager::Init() {
  /*
  int ret = util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(
      ktopic_lane_);
  if (ret < 0) {
    std::cout << "RvizAgent register " << ktopic_lane_ << " failed"
              << std::endl;
  }

  ret = util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(
      ktopic_cur_lane_);
  if (ret < 0) {
    std::cout << "RvizAgent register " << ktopic_cur_lane_ << " failed"
              << std::endl;
  }
  */
}

void MapManager::CutLocalMap(const double& length_x, const double& length_y) {
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    for (size_t j = 0; j < local_map_.local_map_lane_[i].points_.size(); ++j) {
      if (local_map_.local_map_lane_[i].points_[j].x() < -length_x ||
          local_map_.local_map_lane_[i].points_[j].x() > length_x ||
          local_map_.local_map_lane_[i].points_[j].y() < -length_y ||
          local_map_.local_map_lane_[i].points_[j].y() > +length_y) {
        local_map_.local_map_lane_[i].points_.erase(
            local_map_.local_map_lane_[i].points_.begin() + j);
        --j;
      }
    }
    for (size_t j = 0; j < local_map_.local_map_lane_[i].fit_points_.size();
         ++j) {
      if (local_map_.local_map_lane_[i].fit_points_[j].x() < -length_x ||
          local_map_.local_map_lane_[i].fit_points_[j].x() > +length_x ||
          local_map_.local_map_lane_[i].fit_points_[j].y() < -length_y ||
          local_map_.local_map_lane_[i].fit_points_[j].y() > +length_y) {
        local_map_.local_map_lane_[i].fit_points_.erase(
            local_map_.local_map_lane_[i].fit_points_.begin() + j);
        --j;
      }
    }
    if (local_map_.local_map_lane_[i].points_.size() == 0) {
      local_map_.local_map_lane_.erase(local_map_.local_map_lane_.begin() + i);
      --i;
    }
  }
}

void MapManager::AddLane(const LocalMapLane& lane) {
  local_map_.local_map_lane_.emplace_back(lane);
}

void MapManager::AddEdge(const LocalMapLane& edge) {
  local_map_.local_map_edge_.emplace_back(edge);
}

void MapManager::UpdateLane(const Sophus::SE3d& T_C_L) {
  Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
  T << T_C_L.matrix()(0, 0), T_C_L.matrix()(0, 1), T_C_L.matrix()(0, 3),
      T_C_L.matrix()(1, 0), T_C_L.matrix()(1, 1), T_C_L.matrix()(1, 3), 0, 0, 1;
  for (auto& lane : local_map_.local_map_lane_) {
    for (auto& point : lane.points_) {
      // point = T_C_L * point;
      point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
    for (auto& point : lane.fit_points_) {
      // point = T_C_L * point;
      point.x() = T(0, 0) * point.x() + T(0, 1) * point.y() + T(0, 2);
      point.y() = T(1, 0) * point.x() + T(1, 1) * point.y() + T(1, 2);
    }
  }
}

void MapManager::UpdateTimestamp(const double& timestamp) {
  local_map_.timestamp = timestamp;
}

void MapManager::AppendOldLanePoints(
    const int& id, const std::vector<Eigen::Vector3d>& points) {
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    if (local_map_.local_map_lane_[i].track_id_ == id) {
      for (size_t j = 0; j < points.size(); ++j) {
        if (local_map_.local_map_lane_[i].points_.empty()) {
          local_map_.local_map_lane_[i].points_.push_back(points[j]);
        } else {
          auto tmp = local_map_.local_map_lane_[i].points_.back();
          if (points[j].x() - tmp.x() > 0.8) {
            local_map_.local_map_lane_[i].points_.push_back(points[j]);
          }
        }
      }
      return;
    }
  }
}

void MapManager::CreateTrackVehicleLane(const int& id) {
  std::vector<Eigen::Vector3d> vehicle_points;
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    if (local_map_.local_map_lane_[i].track_id_ == id) {
      local_map_.local_map_lane_[i].vehicle_lane_param_ =
          std::make_shared<LaneCubicSpline>();
      for (const auto& v_point : local_map_.local_map_lane_[i].points_) {
        if (v_point.x() > 0.0 && v_point.x() < 100.0) {
          vehicle_points.emplace_back(v_point);
        }
      }
      if (vehicle_points.size() < 4) {
        HLOG_ERROR
            << "track points too small cant not create track vehicle lane";
        return;
      }
      CommonUtil::FitEKFLane(vehicle_points,
                             local_map_.local_map_lane_[i].vehicle_lane_param_);
      auto param = local_map_.local_map_lane_[i].vehicle_lane_param_;
      HLOG_DEBUG << "FitEKFLane: " << param->c0_ << ", " << param->c1_ << ", "
                 << param->c2_ << ", " << param->c3_ << ", "
                 << param->start_point_x_ << ", " << param->end_point_x_;
    }
  }
}

void MapManager::CreateNewTrackVehicleLane(
    std::shared_ptr<const Lane> cur_lane) {
  size_t map_lane_size = local_map_.local_map_lane_.size();
  local_map_.local_map_lane_[map_lane_size - 1].vehicle_lane_param_ =
      std::make_shared<LaneCubicSpline>();
  std::shared_ptr<LaneCubicSpline> vehicle_param =
      local_map_.local_map_lane_[map_lane_size - 1].vehicle_lane_param_;
  HLOG_DEBUG << "frame_lane_: " << cur_lane->lane_fit_a_ << ", "
             << cur_lane->lane_fit_b_ << ", " << cur_lane->lane_fit_c_ << ", "
             << cur_lane->lane_fit_d_ << ", " << cur_lane->x_start_vrf_ << ", "
             << cur_lane->x_end_vrf_;
  *vehicle_param = CommonUtil::MapVehicleLaneTolane(cur_lane);
}

double MapManager::CreateNewLane(const std::vector<Eigen::Vector3d>& points) {
  static int max_id = -1;
  max_id++;
  LocalMapLane lane;
  lane.track_id_ = max_id;
  lane.points_ = points;
  lane.need_fit_ = true;
  local_map_.local_map_lane_.emplace_back(lane);
  return max_id;
}

void MapManager::CreateNewLane(const std::vector<Eigen::Vector3d>& points,
                               const int& lane_id) {
  LocalMapLane lane;
  lane.track_id_ = lane_id;
  lane.points_ = points;
  lane.need_fit_ = true;
  local_map_.local_map_lane_.emplace_back(lane);
}

void MapManager::AppendEdgePoints(const int& id,
                                  const std::vector<Eigen::Vector3d>& points) {
  for (int i = 0; i < local_map_.local_map_edge_.size(); ++i) {
    if (local_map_.local_map_edge_[i].track_id_ == id) {
      local_map_.local_map_edge_[i].points_ = points;
      return;
    }
  }
  LocalMapLane lane;
  lane.track_id_ = id;
  lane.points_ = points;
  local_map_.local_map_edge_.emplace_back(lane);
}

void MapManager::DeleteLanePoints(const int& id, const double& min_dis) {
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    if (local_map_.local_map_lane_[i].track_id_ == id) {
      for (size_t j = 0; j < local_map_.local_map_lane_[i].points_.size();
           ++j) {
        Eigen::Vector3d p_v(local_map_.local_map_lane_[i].points_[j].x(),
                            local_map_.local_map_lane_[i].points_[j].y(),
                            local_map_.local_map_lane_[i].points_[j].z());
        if (p_v.x() > min_dis) {
          local_map_.local_map_lane_[i].points_.erase(
              local_map_.local_map_lane_[i].points_.begin() + j);
          --j;
        }
      }
      for (size_t j = 0; j < local_map_.local_map_lane_[i].fit_points_.size();
           ++j) {
        Eigen::Vector3d p_v(local_map_.local_map_lane_[i].fit_points_[j].x(),
                            local_map_.local_map_lane_[i].fit_points_[j].y(),
                            local_map_.local_map_lane_[i].fit_points_[j].z());
        if (p_v.x() > min_dis) {
          local_map_.local_map_lane_[i].fit_points_.erase(
              local_map_.local_map_lane_[i].fit_points_.begin() + j);
          --j;
        }
      }
      break;
    }
  }
}

void MapManager::GetLanes(std::shared_ptr<std::vector<LocalMapLane>> lane) {
  lane->clear();
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    if (local_map_.local_map_lane_[i].points_.size() > 0) {
      lane->emplace_back(local_map_.local_map_lane_[i]);
    }
  }
}

void MapManager::GetEdges(std::shared_ptr<std::vector<LocalMapLane>> edge) {
  *edge = local_map_.local_map_edge_;
}

void MapManager::GetLocalMap(std::shared_ptr<LocalMap> local_map) {
  *local_map = local_map_;
}

double MapManager::GetTimestamp() { return local_map_.timestamp; }

void GetLanePts(const LaneCubicSpline& cubic_curve,
                std::shared_ptr<std::vector<Eigen::Vector3d>> pts) {
  double a = cubic_curve.c0_;
  double b = cubic_curve.c1_;
  double c = cubic_curve.c2_;
  double d = cubic_curve.c3_;
  double start_x = cubic_curve.start_point_x_;
  double end_x = cubic_curve.end_point_x_;

  const float gap = 1.0;  // meter
  float fac = 1.0;
  for (size_t i = 0;; ++i) {
    float curr_x = start_x + static_cast<float>(i) * gap * fac;
    if ((fac > 0 && curr_x >= end_x) || (fac < 0 && curr_x <= end_x)) {
      break;
    }
    float curr_y =
        a * curr_x * curr_x * curr_x + b * curr_x * curr_x + c * curr_x + d;
    Eigen::Vector3d pt(curr_x, curr_y, 0);
    pts->push_back(pt);
  }
}

void SetPoint(const std::vector<LocalMapLane>& lines,
              std::shared_ptr<std::vector<Eigen::Vector3d>> pts) {
  pts->clear();
  for (auto& line : lines) {
    for (auto& cubic_curve : line.lane_param_) {
      GetLanePts(cubic_curve, pts);
    }
  }
}

void MapManager::PubPoints(const std::vector<Eigen::Vector3d>& points,
                           uint64_t sec, uint64_t nsec,
                           const std::string topic) {
  /*
  if (util::RvizAgent::Instance().Ok()) {
    adsfi_proto::viz::PointCloud lane_points;
    static uint32_t seq = 0;
    size_t curr_seq = seq++;

    lane_points.mutable_header()->set_seq(curr_seq);
    lane_points.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_points.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_points.mutable_header()->set_frameid("localmap");

    auto* channels = lane_points.add_channels();
    channels->set_name("rgb");

    for (auto p : points) {
      auto* points_ = lane_points.add_points();
      points_->set_x(p.x());
      points_->set_y(p.y());
      points_->set_z(p.z());
    }

    // int ret = util::RvizAgent::Instance().Publish(topic, lane_points);
    // if (ret < 0) {
    //   std::cout << "pub points error" << std::endl;
    // }
  }
  */
}

void MapManager::PubLocalMap(uint64_t sec, uint64_t nsec) {
  std::shared_ptr<std::vector<LocalMapLane>> lanes =
      std::make_shared<std::vector<LocalMapLane>>();
  GetLanes(lanes);
  std::shared_ptr<std::vector<Eigen::Vector3d>> lane_pts =
      std::make_shared<std::vector<Eigen::Vector3d>>();
  SetPoint(*lanes, lane_pts);

  PubPoints(*lane_pts, sec, nsec, ktopic_lane_);
}

void MapManager::PubLines(std::vector<LocalMapLane> lanes, uint64_t sec,
                          uint64_t nsec) {
  std::shared_ptr<std::vector<Eigen::Vector3d>> lane_pts =
      std::make_shared<std::vector<Eigen::Vector3d>>();
  SetPoint(lanes, lane_pts);

  PubPoints(*lane_pts, sec, nsec, ktopic_cur_lane_);
}

void MapManager::SetLaneProperty(int lane_id,
                                 const std::shared_ptr<const Lane> frame_lane) {
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    if (local_map_.local_map_lane_[i].track_id_ == lane_id) {
      local_map_.local_map_lane_[i].pos_type_ = frame_lane->pos_type_;
      break;
    }
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
