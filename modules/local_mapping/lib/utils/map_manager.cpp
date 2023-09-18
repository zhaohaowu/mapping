/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei/zhaohaowu
 *Date: 2023-09-01
 *****************************************************************************/
#include "modules/local_mapping/lib/utils/map_manager.h"

DEFINE_string(viz_addr, "ipc:///tmp/rviz_agent_local_map",
              "RvizAgent working address, may like "
              "ipc:///tmp/sample_rviz_agent or "
              "inproc://sample_rviz_agent or "
              "tcp://127.0.0.1:9100");

namespace hozon {
namespace mp {
namespace lm {

void MapManager::Init() {
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
}

void MapManager::CutLocalMap(const double& length_x, const double& length_y,
                             const Eigen::Vector3d& p_v) {
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    for (size_t j = 0; j < local_map_.local_map_lane_[i].points_.size(); ++j) {
      if (local_map_.local_map_lane_[i].points_[j].x() < p_v.x() - length_x ||
          local_map_.local_map_lane_[i].points_[j].x() > p_v.x() + length_x ||
          local_map_.local_map_lane_[i].points_[j].y() < p_v.y() - length_y ||
          local_map_.local_map_lane_[i].points_[j].y() > p_v.y() + length_y) {
        local_map_.local_map_lane_[i].points_.erase(
            local_map_.local_map_lane_[i].points_.begin() + j);
        --j;
      }
    }
  }
}

void MapManager::AddLane(const LocalMapLane& lane) {
  local_map_.local_map_lane_.emplace_back(lane);
}

void MapManager::AddEdge(const LocalMapLane& edge) {
  local_map_.local_map_edge_.emplace_back(edge);
}

void MapManager::UpdateLane(const LocalMapLane& lane) {
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    if (local_map_.local_map_lane_[i].track_id_ == lane.track_id_) {
      local_map_.local_map_lane_[i] = lane;
      break;
    }
  }
}

void MapManager::UpdateEdge(const LocalMapLane& edge) {
  for (size_t i = 0; i < local_map_.local_map_edge_.size(); ++i) {
    if (local_map_.local_map_edge_[i].track_id_ == edge.track_id_) {
      local_map_.local_map_edge_[i] = edge;
      break;
    }
  }
}

void MapManager::AppendOldLanePoints(
    const int& id, const std::vector<Eigen::Vector3d>& points) {
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    if (local_map_.local_map_lane_[i].track_id_ == id) {
      for (size_t j = 0; j < points.size(); ++j) {
        local_map_.local_map_lane_[i].points_.push_back(points[j]);
      }
      return;
    }
  }
}

void MapManager::AppendNewLanePoints(
    const std::vector<Eigen::Vector3d>& points) {
  // static int max_id = -1;
  // for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
  //   if (local_map_.local_map_lane_[i].track_id_ > max_id) {
  //     max_id = local_map_.local_map_lane_[i].track_id_;
  //   }
  // }
  static int max_id = 0;
  LocalMapLane lane;
  lane.track_id_ = max_id;
  lane.points_ = points;
  local_map_.local_map_lane_.emplace_back(lane);
  max_id++;
}

void MapManager::AppendEdgePoints(const int& id,
                                  const std::vector<Eigen::Vector3d>& points) {
  for (size_t i = 0; i < local_map_.local_map_edge_.size(); ++i) {
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

void MapManager::DeleteLanePoints(const int& id, const Eigen::Matrix4d& T_W_V,
                                  const double& min_dis) {
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    if (local_map_.local_map_lane_[i].track_id_ == id) {
      for (size_t j = 0; j < local_map_.local_map_lane_[i].points_.size();
           ++j) {
        Eigen::Vector4d p_w(local_map_.local_map_lane_[i].points_[j].x(),
                            local_map_.local_map_lane_[i].points_[j].y(),
                            local_map_.local_map_lane_[i].points_[j].z(), 1);
        Eigen::Vector4d p_v = T_W_V * p_w;
        if (p_v.x() > min_dis) {
          local_map_.local_map_lane_[i].points_.erase(
              local_map_.local_map_lane_[i].points_.begin() + j);
          --j;
        }
      }
      break;
    }
  }
}

void MapManager::GetLanes(std::shared_ptr<std::vector<LocalMapLane>> lane) {
  for (size_t i = 0; i < local_map_.local_map_lane_.size(); ++i) {
    if (local_map_.local_map_lane_[i].points_.size() > 20) {
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

    int ret = util::RvizAgent::Instance().Publish(topic, lane_points);
    if (ret < 0) {
      std::cout << "pub points error" << std::endl;
    }
  }
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

}  // namespace lm
}  // namespace mp
}  // namespace hozon
