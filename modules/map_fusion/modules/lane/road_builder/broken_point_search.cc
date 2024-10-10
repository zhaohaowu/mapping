/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： broken_point_search.cc
 *   author     ： cuijiayu
 *   date       ： 2024.08
 ******************************************************************************/

#include "modules/map_fusion/modules/lane/road_builder/broken_point_search.h"

#include <algorithm>

using hozon::common::math::Vec2d;

namespace hozon {
namespace mp {
namespace mf {

bool BrokenPointSearch::Init() {
  detect_cut_pt_ = std::make_shared<DetectCutPt>();
  return true;
}

void BrokenPointSearch::Clear() {
  lines_.clear();
  cutpoints_.clear();
}

bool BrokenPointSearch::SearchCtp(
    const std::shared_ptr<std::vector<KinePosePtr>>& path,
    const KinePosePtr& curr_pose, const ElementMap::Ptr& ele_map) {
  HLOG_ERROR << "BrokenPointSearch::SearchCtp";
  if (path == nullptr || curr_pose == nullptr || ele_map == nullptr) {
    HLOG_FATAL << "input nullptr";
    return false;
  }
  float lane_line_interp_dist = 0;  // 默认为0，后期改成从config读
  curr_pose_ = curr_pose;
  std::deque<Line::Ptr> lines;
  Sophus::SE3d Twv;
  std::vector<LaneLine::Ptr> lanelines;
  // lane_line_interp_dist可以设为-1，当前上游点间隔已经是1m，这里不用插值出更细的点
  RetrieveBoundaries(ele_map, lane_line_interp_dist, &lines);
  lines_ = lines;
  PoseLineAdapter2Cpt(lines, &Twv, &lanelines);
  cutpoints_.clear();
  if (detect_cut_pt_->DetectCutPoint(path, Twv, lanelines, curr_pose_,
                                     &cutpoints_) != SMStatus::SUCCESS) {
    HLOG_ERROR << "detect failed!!!";
    return false;
  }

  return true;
}

std::vector<CutPoint> BrokenPointSearch::GetCutPoints() {
  std::vector<CutPoint> cut_points;
  cut_points.reserve(cutpoints_.size());
  for (const auto& ctp : cutpoints_) {
    cut_points.emplace_back(ctp);
  }
  return cut_points;
}

std::deque<Line::Ptr> BrokenPointSearch::GetLines() {
  std::deque<Line::Ptr> lines;
  for (const auto& l : lines_) {
    lines.emplace_back(l);
  }
  return lines;
}

void BrokenPointSearch::ComputeLineHeading(const Line::Ptr& line) {
  // 计算末端平均heading
  double heading = 0;
  std::vector<double> thetas;
  for (int i = static_cast<int>(line->pts.size()) - 1; i > 0; i--) {
    int j = i - 1;
    for (; j >= 0; j--) {
      const Eigen::Vector2f pb(line->pts[i].pt[0], line->pts[i].pt[1]);
      const Eigen::Vector2f pa(line->pts[j].pt[0], line->pts[j].pt[1]);
      Eigen::Vector2f v = pb - pa;
      if (v.norm() > 5.0) {
        double theta = atan2(v.y(), v.x());
        thetas.emplace_back(theta);
        i = j;
        break;
      }
    }
    if (thetas.size() >= 1) {
      break;
    }
  }
  heading = thetas.empty() ? 0.0
                           : std::accumulate(thetas.begin(), thetas.end(), 0.) /
                                 thetas.size();
  if (line->pts.size() < 2) {
    line->mean_end_heading = 0;
    line->mean_end_heading_std_dev = 0;
    line->pred_end_heading = std::make_tuple(0., 0., 0.);
  } else {
    std::vector<Vec2d> points;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    double kappa = 0.;
    double dkappa = 0.;
    Vec2d dist_point;
    for (const auto& point : line->pts) {
      Vec2d temp_point;
      temp_point.set_x(dist_point.x() - point.pt.x());
      temp_point.set_y(dist_point.y() - point.pt.y());
      if (temp_point.Length() >= 0.4) {
        points.emplace_back(point.pt.x(), point.pt.y());
      }
      dist_point.set_x(point.pt.x());
      dist_point.set_y(point.pt.y());
    }
    std::vector<double> fit_result;
    std::vector<Vec2d> fit_points;

    if (points.size() >= 10) {
      int fit_num = std::min(50, static_cast<int>(points.size()));
      std::copy(points.rbegin(), points.rbegin() + fit_num,
                std::back_inserter(fit_points));
      math::FitLaneLinePoint(fit_points, &fit_result);
      std::vector<Vec2d> cmp_points;
      std::copy(points.rbegin(), points.rbegin() + 10,
                std::back_inserter(cmp_points));
      math::ComputeDiscretePoints(cmp_points, fit_result, &kappas, &dkappas);
      // 整体平均
      kappa = kappas.empty() ? 0.0
                             : std::accumulate(kappas.begin() + 3,
                                               kappas.begin() + 8, 0.) /
                                   5.;
      dkappa = dkappas.empty() ? 0.0
                               : std::accumulate(dkappas.begin() + 3,
                                                 dkappas.begin() + 8, 0.) /
                                     5.;
      kappa = kappa / 8.;
      dkappa = dkappa / 8.;
    }
    line->mean_end_interval = 1.0;
    line->mean_end_heading = heading;
    line->mean_end_heading_std_dev = 0;
    line->pred_end_heading = std::make_tuple(heading, kappa, dkappa);
    HLOG_DEBUG << "line id:" << line->id << "," << line->mean_end_heading << ","
               << line->mean_end_interval << "," << kappas.size();
    HLOG_DEBUG << "line id:" << line->id << "," << line->mean_end_heading << ","
               << line->mean_end_interval << "," << kappas.size();
  }
}

// 从原始ElementMap里提取出车道线：
// 1.将ElementMap里点按原顺序保存到队列；
// 2.interp_dist为插值间距，当>0时按此间距对车道线点进行插值；
// 3.计算每根线的末端平均heading和平均点间距，用于后面预测.
void BrokenPointSearch::RetrieveBoundaries(const ElementMap::Ptr& ele_map,
                                           float interp_dist,
                                           std::deque<Line::Ptr>* lines) {
  HLOG_DEBUG << "RetrieveBoundaries" << ele_map->lane_boundaries.size();
  if (ele_map == nullptr || lines == nullptr) {
    return;
  }

  bool need_interp = (interp_dist > 0);

  for (const auto& bound_pair : ele_map->lane_boundaries) {
    const auto& bound = bound_pair.second;
    if (bound == nullptr) {
      HLOG_ERROR << "found nullptr boundary";
      continue;
    }
    if (bound->nodes.empty()) {
      continue;
    }
    auto line = std::make_shared<Line>();
    line->id = bound->id;
    line->type = bound->linetype;
    line->color = bound->color;
    line->isego = bound->is_ego;
    line->lanepos = bound->lanepos;
    line->is_near_road_edge = bound->is_near_road_edge;
    for (const auto& delete_id : bound->delete_ids) {
      line->deteled_ids.emplace_back(delete_id);
    }
    Eigen::Vector3f last_raw(0, 0, 0);
    for (const auto& node : bound->nodes) {
      if (node == nullptr) {
        HLOG_ERROR << "found nullptr node";
        continue;
      }
      const auto& curr_raw = node->point;
      if (need_interp && !line->pts.empty()) {
        // 方向向量
        Eigen::Vector3f n = curr_raw - last_raw;
        float dist = n.norm();
        if (dist > interp_dist) {
          n.normalize();
          int interp_cnt = static_cast<int>(dist / interp_dist);
          for (int i = 0; i < interp_cnt; ++i) {
            Eigen::Vector3f interp_pt = last_raw + (i + 1) * n;
            Point pt(PointType::INTERPOLATED, interp_pt.x(), interp_pt.y(),
                     interp_pt.z());
            line->pts.emplace_back(pt);
          }
        }
      }
      Point pt(PointType::RAW, curr_raw.x(), curr_raw.y(), curr_raw.z());
      line->pts.emplace_back(pt);
      last_raw = curr_raw;
    }
    if (line->pts.empty()) {
      continue;
    }
    ComputeLineHeading(line);
    lines->emplace_back(line);
  }
}

bool BrokenPointSearch::PoseLineAdapter2Cpt(
    const std::deque<Line::Ptr>& lines, Sophus::SE3d* Twv,
    std::vector<LaneLine::Ptr>* lanelines) {
  // convert Pose 2 Sophus::SE3d Twv
  HLOG_DEBUG << "PoseLineAdapter2Cpt!!!!!!!!!!";
  Sophus::Vector3f translation_raw(curr_pose_->pos.x(), curr_pose_->pos.y(),
                                   curr_pose_->pos.z());
  Sophus::Vector3d translation(static_cast<double>(translation_raw[0]),
                               static_cast<double>(translation_raw[1]),
                               static_cast<double>(translation_raw[2]));
  Eigen::Quaternionf quat_normalized_raw = curr_pose_->quat.normalized();
  Eigen::Quaterniond quat_normalized(
      static_cast<double>(quat_normalized_raw.w()),
      static_cast<double>(quat_normalized_raw.x()),
      static_cast<double>(quat_normalized_raw.y()),
      static_cast<double>(quat_normalized_raw.z()));
  Sophus::SO3d rotation(quat_normalized);
  Sophus::SE3d Twv_tmp(rotation, translation);
  (*Twv) = Twv_tmp;

  // convert gm::Line::Ptr to LaneLine::Ptr, deepcopy
  std::vector<LaneLine::Ptr> input_lines;
  if (lines.size() > 0) {
    input_lines.reserve(lines.size());
  } else {
    HLOG_WARN << "input lines size = 0, no need convert";
    return false;
  }

  for (const auto& line : lines) {
    if (line->id >= 1000) {
      continue;
    }
    LaneLine::Ptr tmp = std::make_shared<LaneLine>();
    tmp->SetId(line->id);
    tmp->SetType(line->type);
    std::vector<Point3D> points;
    // 把车道线转到local系
    for (const auto& pt : line->pts) {
      points.emplace_back((*Twv) * Point3D(static_cast<double>(pt.pt.x()),
                                           static_cast<double>(pt.pt.y()),
                                           static_cast<double>(pt.pt.z())));
    }
    tmp->SetPoints(points);
    input_lines.push_back(tmp);
  }
  (*lanelines) = input_lines;
  input_lines.clear();
  return true;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
