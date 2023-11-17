/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： local_map_provider.cc
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#include "map_fusion/local_map_provider/local_map_provider.h"
#include <gflags/gflags.h>

#include <mutex>

#include "util/temp_log.h"

// NOLINTBEGIN
DEFINE_string(viz_addr, "",
              "RvizAgent working address, may like "
              "ipc:///tmp/sample_rviz_agent or "
              "inproc://sample_rviz_agent or "
              "tcp://127.0.0.1:9100");
DEFINE_bool(viz, false, "enable rivz or not");
// NOLINTEND

namespace hozon {
namespace mp {
namespace mf {

LocalMapProvider::~LocalMapProvider() {
  // if (!FLAGS_viz_addr.empty()) {
  //   RVIZ_AGENT.Term();
  // }
}

int LocalMapProvider::Init() {
  // if (!FLAGS_viz_addr.empty()) {
  //   int ret = RVIZ_AGENT.Init(FLAGS_viz_addr);
  //   if (ret < 0) {
  //     HLOG_WARN << "RvizAgent init failed";
  //   }
  //   if (!RVIZ_AGENT.Ok()) {
  //     HLOG_WARN << "RvizAgent start failed";
  //   } else {
  //     ret = RVIZ_AGENT.Register<adsfi_proto::viz::TransformStamped>(
  //         kTopicLocalMapProviderTf);
  //     if (ret < 0) {
  //       HLOG_WARN << "RvizAgent register " << kTopicLocalMapProviderTf
  //                 << " failed";
  //     }

  //     ret =
  //     RVIZ_AGENT.Register<adsfi_proto::viz::Path>(kTopicLocalMapLocation); if
  //     (ret < 0) {
  //       HLOG_WARN << "RvizAgent register " << kTopicLocalMapLocation
  //                 << " failed";
  //     }

  //     ret = RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(
  //         kTopicLocalMapProviderLaneLine);
  //     if (ret < 0) {
  //       HLOG_WARN << "RvizAgent register " << kTopicLocalMapProviderLaneLine
  //                 << " failed";
  //     }

  //     ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(
  //         KTopicLocalMapProviderMap);
  //     if (ret < 0) {
  //       HLOG_WARN << "RvizAgent register " << KTopicLocalMapProviderMap
  //                 << " failed";
  //     }
  //   }
  // }
  local_map_ = std::make_shared<hozon::mapping::LocalMap>();
  local_map_write_ = std::make_shared<hozon::mapping::LocalMap>();
  return 0;
}

void LocalMapProvider::OnInsNodeInfo(
    const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
  // ins fusion输出的的位置和姿态
  Eigen::Vector3d pose(msg->pos_gcj02().x(), msg->pos_gcj02().y(),
                       msg->pos_gcj02().z());

  Eigen::Quaterniond q_W_V(msg->quaternion().w(), msg->quaternion().x(),
                           msg->quaternion().y(), msg->quaternion().z());

  // HLOG_ERROR << "q_W_V: " << q_W_V.w() << ", " << q_W_V.x() << ", " <<
  // q_W_V.y()
  //            << ", " << q_W_V.z();

  q_W_V_ = q_W_V;
  q_W_V_.normalize();

  if (!init_) {
    HLOG_INFO << "ref_point_ = pose";
    ref_point_ = pose;
  }
  init_ = true;
  enu_ = util::Geo::Gcj02ToEnu(pose, ref_point_);
  // HLOG_ERROR << "enu_: " << enu_.x() << ", " << enu_.y() << ", " << enu_.z();

  // VizLocation(enu_, q_W_V_, msg->header().publish_stamp());
}

void LocalMapProvider::OnLocation(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  // static double last_stamp = -1;
  // auto curr_stamp =
  //     msg->header().timestamp().sec() + msg->header().timestamp().nsec() *
  //     1e-9;
  // if (curr_stamp > last_stamp) {
  // location的位置和姿态
  Eigen::Vector3d pose(msg->pose().gcj02().x(), msg->pose().gcj02().y(),
                       msg->pose().gcj02().z());

  Eigen::Quaterniond q_W_V(
      msg->pose().quaternion().w(), msg->pose().quaternion().x(),
      msg->pose().quaternion().y(), msg->pose().quaternion().z());

  // HLOG_ERROR << "q_W_V: " << q_W_V.w() << ", " << q_W_V.x() << ", " <<
  // q_W_V.y()
  //            << ", " << q_W_V.z();

  q_W_V_ = q_W_V;
  q_W_V_.normalize();

  if (!init_) {
    HLOG_INFO << "ref_point_ = pose";
    ref_point_ = pose;
  }
  init_ = true;
  // enu_ = util::Geo::Gcj02ToEnu(pose, ref_point_);
  enu_ = pose;
  // HLOG_ERROR << "enu_: " << enu_.x() << ", " << enu_.y() << ", " << enu_.z();

  // VizLocation(enu_, q_W_V_, msg->header().publish_stamp());
  // }
  // last_stamp = curr_stamp;
}

void LocalMapProvider::OnLaneLine(
    const std::shared_ptr<hozon::perception::TransportElement>& msg) {
  if (!init_) {
    return;
  }

  // 可视化感知结果
  // std::vector<Eigen::Vector3d> lane_points;
  // SetLaneLine(&lane_points, msg);
  // VizLaneLine(lane_points, msg->header().publish_stamp());

  std::lock_guard<std::mutex> lock(map_mtx_);

  // 时间戳每次都更新
  auto msg_timestamp = msg->header().publish_stamp();
  local_map_->mutable_header()->set_publish_stamp(msg_timestamp);

  float fac = 1.0;
  const float gap = 1.0;  // meter

  Eigen::Matrix3d rotate_V = q_W_V_.toRotationMatrix();

  if (!flag_ || (enu_ - pos_enu_add_lane_).norm() >= 50) {
    for (const auto& i : msg->lane()) {
      if (i.lanepos() >= 6) {
        continue;
      }
      auto lanes = local_map_->add_lanes();
      lanes->set_track_id(i.lanepos());
      for (const auto& j : i.lane_param().cubic_curve_set()) {
        auto start_x = j.start_point_x();
        auto end_x = j.end_point_x();
        auto c3 = j.c3();
        auto c2 = j.c2();
        auto c1 = j.c1();
        auto c0 = j.c0();
        // auto lane_cls = i.lanetype();

        // HLOG_ERROR << msg->header().seq()
        //            << "lane id: " << std::to_string(i.lane_id());
        // HLOG_ERROR << msg->header().seq()
        //            << "lane type: " << std::to_string(i.type());
        // HLOG_ERROR << "---xxxxxxxxxxxxxxxxxxxxxxx";

        // 目前lane的id有重复(300-0)
        if (start_x == end_x) {
          continue;
        }
        if (start_x == 0 && end_x == 0) {
          continue;
        }
        if (c0 == 0 && c1 == 0 && c2 == 0 && c3 == 0) {
          continue;
        }
        pos_enu_add_lane_ = enu_;
        for (size_t j = 0;; ++j) {
          float curr_x = start_x + static_cast<float>(j) * gap * fac;
          if (curr_x >= end_x) {
            break;
          }
          float curr_y = c3 * curr_x * curr_x * curr_x + c2 * curr_x * curr_x +
                         c1 * curr_x + c0;
          Eigen::Vector3d pos_lane;
          pos_lane << static_cast<double>(curr_x), static_cast<double>(curr_y),
              enu_.z();
          Eigen::Vector3d pos_enu_lane = rotate_V * pos_lane + enu_;
          auto pt = lanes->add_points();
          pt->set_x(pos_enu_lane.x());
          pt->set_y(pos_enu_lane.y());
          pt->set_z(pos_enu_lane.z());
        }
      }
      // 裁剪line
      if (local_map_->lanes().size() > 12) {
        local_map_->mutable_lanes()->DeleteSubrange(0, 1);
      }
    }
    flag_ = true;
  }

  // {
  //   std::lock_guard<std::mutex> lock(map_mtx_);
  //   local_map_write_ = std::make_shared<hozon::mapping::LocalMap>();
  //   // local_map_write_ = local_map_;
  //   local_map_write_->CopyFrom(*local_map_);
  // }

  // 可视化地图
  // VizLocalMap(local_map_);
}
void LocalMapProvider::OnRoadEdge(
    const std::shared_ptr<hozon::perception::TransportElement>& msg) {}

std::shared_ptr<hozon::mapping::LocalMap> LocalMapProvider::GetLocalMap() {
  std::lock_guard<std::mutex> lock(map_mtx_);
  auto ptr = std::make_shared<hozon::mapping::LocalMap>();
  ptr->CopyFrom(*local_map_);
  return ptr;
}

void LocalMapProvider::VizLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers;

  for (const auto& i : local_map->lanes()) {
    adsfi_proto::viz::Marker marker;
    LaneLineToMarker(local_map->header().publish_stamp(), i, &marker);
    if (!marker.points().empty()) {
      markers.add_markers()->CopyFrom(marker);
    }
  }

  RVIZ_AGENT.Publish(KTopicLocalMapProviderMap, markers);
}

void LocalMapProvider::SetLaneLine(
    std::vector<Eigen::Vector3d>* points,
    const std::shared_ptr<hozon::perception::TransportElement>& msg) {
  points->clear();
  float fac = 1.0;
  const float gap = 1.0;  // meter

  Eigen::Matrix3d rotate_V = q_W_V_.toRotationMatrix();

  for (const auto& i : msg->lane()) {
    for (const auto& j : i.lane_param().cubic_curve_set()) {
      auto start_x = j.start_point_x();
      auto end_x = j.end_point_x();
      auto c3 = j.c3();
      auto c2 = j.c2();
      auto c1 = j.c1();
      auto c0 = j.c0();
      // auto lane_cls = i.lanetype();

      if (i.track_id() > 100) {
        continue;
      }
      if (start_x == end_x) {
        continue;
      }
      if (start_x == 0 && end_x == 0) {
        continue;
      }
      if (c0 == 0 && c1 == 0 && c2 == 0 && c3 == 0) {
        continue;
      }

      for (size_t j = 0;; ++j) {
        float curr_x = start_x + static_cast<float>(j) * gap * fac;
        if (curr_x >= end_x) {
          break;
        }
        float curr_y = c3 * curr_x * curr_x * curr_x + c2 * curr_x * curr_x +
                       c1 * curr_x + c0;
        Eigen::Vector3d pos_lane;
        pos_lane << static_cast<double>(curr_x), static_cast<double>(curr_y),
            enu_.z();
        Eigen::Vector3d pos_enu_lane = rotate_V * pos_lane + enu_;
        points->emplace_back(pos_enu_lane);
      }
    }
  }

  if (points->size() > 500) {
    points->erase(points->begin() + 500, points->end());
  }
}

void LocalMapProvider::VizLaneLine(const std::vector<Eigen::Vector3d>& points,
                                   const double stamp) {
  if (RVIZ_AGENT.Ok()) {
    adsfi_proto::viz::PointCloud lane_points;
    static uint32_t seq = 0;
    int curr_seq = seq++;

    lane_points.mutable_header()->set_seq(curr_seq);
    lane_points.mutable_header()->mutable_timestamp()->set_sec(stamp);
    lane_points.mutable_header()->set_frameid("map");

    auto* channels = lane_points.add_channels();
    channels->set_name("rgb");

    for (auto p : points) {
      auto* points_ = lane_points.add_points();
      points_->set_x(p.x());
      points_->set_y(p.y());
      points_->set_z(p.z());
    }

    RVIZ_AGENT.Publish(kTopicLocalMapProviderLaneLine, lane_points);
  }
}

void LocalMapProvider::VizLocation(const Eigen::Vector3d& pose,
                                   const Eigen::Quaterniond& q_W_V,
                                   const double stamp) {
  if (RVIZ_AGENT.Ok()) {
    static uint32_t seq = 0;
    int curr_seq = seq++;
    adsfi_proto::viz::TransformStamped geo_tf;
    geo_tf.mutable_header()->set_seq(curr_seq);
    geo_tf.mutable_header()->mutable_timestamp()->set_sec(stamp);
    geo_tf.mutable_header()->set_frameid("map");
    geo_tf.set_child_frame_id("vehicle");
    geo_tf.mutable_transform()->mutable_translation()->set_x(pose.x());
    geo_tf.mutable_transform()->mutable_translation()->set_y(pose.y());
    geo_tf.mutable_transform()->mutable_translation()->set_z(pose.z());
    geo_tf.mutable_transform()->mutable_rotation()->set_x(q_W_V.x());
    geo_tf.mutable_transform()->mutable_rotation()->set_y(q_W_V.y());
    geo_tf.mutable_transform()->mutable_rotation()->set_z(q_W_V.z());
    geo_tf.mutable_transform()->mutable_rotation()->set_w(q_W_V.w());
    RVIZ_AGENT.Publish(kTopicLocalMapProviderTf, geo_tf);

    auto* location_pose = location_path_.add_poses();

    location_path_.mutable_header()->set_seq(curr_seq);
    location_path_.mutable_header()->mutable_timestamp()->set_sec(stamp);
    location_path_.mutable_header()->set_frameid("map");

    location_pose->mutable_header()->set_seq(curr_seq);
    location_pose->mutable_header()->mutable_timestamp()->set_sec(stamp);
    location_pose->mutable_header()->set_frameid("map");
    location_pose->mutable_pose()->mutable_position()->set_x(pose.x());
    location_pose->mutable_pose()->mutable_position()->set_y(pose.y());
    location_pose->mutable_pose()->mutable_position()->set_z(pose.z());

    location_pose->mutable_pose()->mutable_orientation()->set_w(q_W_V.w());
    location_pose->mutable_pose()->mutable_orientation()->set_x(q_W_V.x());
    location_pose->mutable_pose()->mutable_orientation()->set_y(q_W_V.y());
    location_pose->mutable_pose()->mutable_orientation()->set_z(q_W_V.z());

    if (location_path_.poses().size() > 200) {
      location_path_.mutable_poses()->DeleteSubrange(0, 1);
    }
    RVIZ_AGENT.Publish(kTopicLocalMapLocation, location_path_);
  }
}

void LocalMapProvider::LaneLineToMarker(
    const double stamp, const hozon::mapping::LaneInfo& lane_line,
    adsfi_proto::viz::Marker* marker) {
  static int id = 0;
  marker->Clear();
  marker->mutable_header()->set_frameid("map");
  marker->mutable_header()->mutable_timestamp()->set_sec(stamp);
  marker->mutable_header()->mutable_timestamp()->set_nsec(stamp);
  marker->set_ns("ns_local_map_lane");
  marker->set_id(id++);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker->mutable_pose()->mutable_position()->set_x(0);
  marker->mutable_pose()->mutable_position()->set_y(0);
  marker->mutable_pose()->mutable_position()->set_z(0);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  marker->mutable_scale()->set_x(0.2);
  marker->mutable_scale()->set_y(0.2);
  marker->mutable_scale()->set_z(0.2);
  marker->mutable_lifetime()->set_sec(1);
  marker->mutable_lifetime()->set_nsec(0);

  adsfi_proto::viz::ColorRGBA color;
  color.set_a(1.0);
  color.set_r(1.0);
  color.set_g(0.65);
  color.set_b(0);
  marker->mutable_color()->CopyFrom(color);
  for (const auto& point : lane_line.points()) {
    auto pt = marker->add_points();
    pt->set_x(point.x());
    pt->set_y(point.y());
    pt->set_z(point.z());
  }

  if (marker->points().empty()) {
    HLOG_WARN << "empty lane line";
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
