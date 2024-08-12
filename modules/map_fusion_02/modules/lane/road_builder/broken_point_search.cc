/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： broken_point_search.cc
 *   author     ： cuijiayu
 *   date       ： 2024.08
 ******************************************************************************/

#include "modules/map_fusion_02/modules/lane/road_builder/broken_point_search.h"

namespace hozon {
namespace mp {
namespace mf {

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

void BrokenPointSearch::GetCutPoints(std::vector<CutPoint>* cut_points) {
  if (cut_points == nullptr) {
    return;
  }
  cut_points->reserve(cutpoints_.size());
  for (const auto& ctp : cutpoints_) {
    cut_points->emplace_back(ctp);
  }
}

void BrokenPointSearch::GetLinse(std::deque<Line::Ptr>* lines) {
  if (lines == nullptr) {
    return;
  }
  for (const auto& l : lines_) {
    lines->emplace_back(l);
  }
}

// 从原始ElementMap里提取出车道线：
// 1.将ElementMap里点按原顺序保存到队列；
// 2.interp_dist为插值间距，当>0时按此间距对车道线点进行插值；
// 3.计算每根线的末端平均heading和平均点间距，用于后面预测.
void BrokenPointSearch::RetrieveBoundaries(const ElementMap::Ptr& ele_map,
                                           float interp_dist,
                                           std::deque<Line::Ptr>* lines) {
  HLOG_DEBUG << "RetrieveBoundaries";
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
