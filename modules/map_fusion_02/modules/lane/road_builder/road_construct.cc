/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_construct.cc
 *   author     ： cuijiayu
 *   date       ： 2024.08
 ******************************************************************************/
#include "modules/map_fusion_02/modules/lane/road_builder/road_construct.h"

#include "base/utils/log.h"
#include "data_manager/location_data_manager.h"

namespace hozon {
namespace mp {
namespace mf {

bool RoadConstruct::Init(const LaneFusionProcessOption& conf) {
  conf_ = conf;
  HLOG_INFO << "Road construct init";
  return true;
}

void RoadConstruct::Clear() {
  KDTrees_.clear();
  line_points_.clear();
  groups_.clear();
  path_in_curr_pose_.clear();
  distpoits_.clear();
  predict_line_params_.clear();
  unused_occ_roads_.clear();
  unused_occ_road_fitlines_.clear();
  unused_occ_ctps_.clear();
}

bool RoadConstruct::ConstructRoad(
    const std::vector<CutPoint>& cutpoints, std::deque<Line::Ptr> lines,
    const std::shared_ptr<std::vector<KinePosePtr>>& path,
    const KinePosePtr& curr_pose, const ElementMap::Ptr& ele_map) {
  BuildKDtrees(&lines);
  UpdatePathInCurrPose(*path, *curr_pose);
  BuildGroups(cutpoints, &lines, ele_map, &groups_);
  BuildOccGroups(ele_map, &groups_);
  SetBrokenId(&groups_);
  GenGroupName(ele_map, &groups_);
  GenRoadEdges(&groups_);
  RemoveInvalGroups(&groups_);
  return true;
}

// 获取内部Group
std::vector<Group::Ptr> RoadConstruct::GetGroups() {
  std::vector<Group::Ptr> groups;
  groups.reserve(groups_.size());
  for (const auto& grp : groups_) {
    groups.emplace_back(grp);
  }
  return groups;
}

std::vector<Eigen::Vector3f> RoadConstruct::GetDistPoints() {
  std::vector<Eigen::Vector3f> distpoints;
  distpoints.reserve(distpoits_.size());
  for (const auto& distp : distpoits_) {
    distpoints.emplace_back(distp);
  }
  return distpoints;
}

std::deque<Line::Ptr> RoadConstruct::GetFitOcc() {
  return unused_occ_road_fitlines_;
}

void RoadConstruct::BuildKDtrees(std::deque<Line::Ptr>* lines) {
  HLOG_DEBUG << "BuildKDtrees";
  // Check if there are any lines
  if (lines == nullptr) {
    return;
  }
  if (!KDTrees_.empty()) {
    KDTrees_.clear();
  }
  if (!line_points_.empty()) {
    line_points_.clear();
  }

  for (auto& line : *lines) {
    if (line->pts.empty()) {
      continue;
    }
    auto cv_points = std::make_shared<std::vector<cv::Point2f>>();
    for (auto& p : line->pts) {
      if (std::isnan(p.pt.x()) || std::isnan(p.pt.y())) {
        HLOG_DEBUG << "Nan points";
        break;
      }
      cv_points->emplace_back(p.pt.x(), p.pt.y());
    }
    cv::flann::KDTreeIndexParams index_params(1);
    auto kdtree = std::make_shared<cv::flann::Index>(
        cv::Mat(*cv_points).reshape(1), index_params);
    KDTrees_[line->id] = kdtree;
    line_points_[line->id] = cv_points;
    // HLOG_INFO << "build kdtree: " << line->id << ", size: " <<
    // cv_points->size();
  }
  return;
}

void RoadConstruct::UpdatePathInCurrPose(const std::vector<KinePosePtr>& path,
                                         const KinePose& curr_pose) {
  path_in_curr_pose_.clear();
  for (const auto& p : path) {
    if (p == nullptr) {
      HLOG_ERROR << "found nullptr pose in path";
      continue;
    }
    Eigen::Isometry3f T_local_v1;
    T_local_v1.setIdentity();
    T_local_v1.rotate(p->quat);
    T_local_v1.pretranslate(p->pos);

    Eigen::Isometry3f T_local_v2;
    T_local_v2.setIdentity();
    T_local_v2.rotate(curr_pose.quat);
    T_local_v2.pretranslate(curr_pose.pos);

    Eigen::Isometry3f T_v2_v1;
    T_v2_v1.setIdentity();
    T_v2_v1 = T_local_v2.inverse() * T_local_v1;
    // if (T_v2_v1.translation().x() < 0) {
    Pose pose;
    pose.stamp = p->stamp;
    pose.pos << T_v2_v1.translation().x(), T_v2_v1.translation().y(),
        T_v2_v1.translation().z();
    pose.quat = T_v2_v1.rotation();
    std::abs(pose.pos.x()) < 0.0001 ? pose.pos.x() = 0.0 : pose.pos.x();
    std::abs(pose.pos.y()) < 0.0001 ? pose.pos.y() = 0.0 : pose.pos.y();
    path_in_curr_pose_.emplace_back(pose);
    // }
  }
  // 自车原点也加进来
  // Pose pose;
  // pose.stamp = curr_pose.stamp;
  // pose.pos.setZero();
  // pose.quat.setIdentity();
  // path_in_curr_pose_.emplace_back(pose);
}

void RoadConstruct::BuildGroups(const std::vector<CutPoint>& cutpoints,
                                std::deque<Line::Ptr>* lines,
                                const ElementMap::Ptr& ele_map,
                                std::vector<Group::Ptr>* groups) {
  double stamp = ele_map->map_info.stamp;

  CreatGroupsFromCutPoints(cutpoints, groups);
  SplitPtsToGroup(lines, ele_map, groups);
  GenLanesInGroups(groups, ele_map, stamp);
}

void RoadConstruct::CreatGroupsFromCutPoints(
    const std::vector<CutPoint>& cutpoints, std::vector<Group::Ptr>* groups) {
  if (groups == nullptr) {
    HLOG_ERROR << "input nullptr";
    return;
  }
  // 用每一个cut point构建切分线
  std::vector<SliceLine> slice_lines;

  for (const auto& ctp : cutpoints) {
    if (ctp.GetType() == CutPointType::Unknown) {
      HLOG_ERROR << "found Unknown ctp";
      continue;
    }
    Point3D p = ctp.GetPoint();
    HLOG_DEBUG << "ctp: " << p.x << " " << p.y;
    std::vector<Point3D> extre_point = ctp.GetExtrePoint();
    Eigen::Vector3f po_veh(p.x, p.y, p.z);
    Eigen::Vector3f pr_veh(extre_point[0].x, extre_point[0].y, 0);
    Eigen::Vector3f pl_veh(extre_point[1].x, extre_point[1].y, 0);

    SliceLine slice;
    slice.po = po_veh;
    slice.pl = pl_veh;
    slice.pr = pr_veh;
    slice.cut_type = ctp.GetType();
    HLOG_DEBUG << "ctp type: " << ctp.GetType();
    slice_lines.emplace_back(slice);
  }

  if (slice_lines.size() < 2) {
    HLOG_ERROR << "no enough slice lines: " << slice_lines.size();
    return;
  }

  // 从SliceLines里创建出Group
  const auto slice_num = slice_lines.size();
  groups->clear();
  groups->reserve(slice_num);
  for (size_t i = 0; i < slice_num - 1; ++i) {
    auto grp = std::make_shared<Group>();
    grp->start_slice = slice_lines.at(i);
    grp->end_slice = slice_lines.at(i + 1);
    grp->str_id.clear();
    groups->emplace_back(grp);
  }
  HLOG_DEBUG << "groups size: " << groups->size();
}

void RoadConstruct::SplitLinesToGroup(std::deque<Line::Ptr>* lines,
                                      const Group::Ptr& grp) {
  // 生成LineSegments
  const auto& start_slice = grp->start_slice;
  Eigen::Vector2f start_po = start_slice.po.head<2>();
  Eigen::Vector2f start_pl = start_slice.pl.head<2>();
  const auto& end_slice = grp->end_slice;
  Eigen::Vector2f end_po = end_slice.po.head<2>();
  Eigen::Vector2f end_pl = end_slice.pl.head<2>();

  for (auto& line : *lines) {
    // if (!line->isego) {
    //   continue;
    // }
    auto line_seg = std::make_shared<LineSegment>();
    line_seg->id = line->id;
    line_seg->type = line->type;
    line_seg->color = line->color;
    line_seg->lanepos = line->lanepos;
    line_seg->isego = line->isego;
    line_seg->is_near_road_edge = line->is_near_road_edge;

    line_seg->mean_end_heading = line->mean_end_heading;
    line_seg->pred_end_heading = line->pred_end_heading;
    line_seg->mean_end_heading_std_dev = line->mean_end_heading_std_dev;
    line_seg->mean_end_interval = line->mean_end_interval;
    for (const auto& delete_id : line->deteled_ids) {
      line_seg->deteled_ids.emplace_back(delete_id);
    }
    while (!line->pts.empty()) {
      Eigen::Vector2f front_pt_xy = line->pts.front().pt.head<2>();
      // 第一个点已经在end_slice的右边，直接跳过这条线
      if (PointInVectorSide(end_po, end_pl, front_pt_xy) > 0) {
        // HLOG_DEBUG << "break!!!";
        break;
      }
      // 第一个点在end_slice的左边，并且在start_slice的右边，说明属于此group，加进来
      if (PointInVectorSide(start_po, start_pl, front_pt_xy) >= 0) {
        line_seg->pts.emplace_back(line->pts.front());
      }
      // 把处理完的第一个点pop掉，防止后面重复处理
      line->pts.pop_front();
    }
    if (line_seg->pts.size() < 2) {
      continue;
    }

    //! TBD: 这里用首尾两点求中值，后续考虑对所有点取平均值得到中心点
    // line_seg->center =
    //     (line_seg->pts.front().pt + line_seg->pts.back().pt) * 0.5;
    int size_line_seg = static_cast<int>(line_seg->pts.size());
    Eigen::Vector3f point_tmp(0.0, 0.0, 0.0);
    for (int i = 0; i < size_line_seg; ++i) {
      point_tmp += line_seg->pts[i].pt;
    }
    line_seg->center = point_tmp / size_line_seg;
    // auto dist =
    //     PointToVectorDist(start_slice.po, end_slice.po, line_seg->center);
    // auto dist = line_seg->center[1];  // !TBD: 暂时用y坐标代替距离
    auto dist = Dist2Path(line_seg->center);

    line_seg->dist_to_path = dist;
    grp->line_segments.emplace_back(line_seg);
  }
  // 按与path的距离从大到小排序，得到的结果就是所有线都是从左到右排序号
  std::sort(grp->line_segments.begin(), grp->line_segments.end(),
            [](const LineSegment::Ptr& a, const LineSegment::Ptr& b) {
              return a->dist_to_path > b->dist_to_path;
            });
}

void RoadConstruct::SplitOccsToGroup(
    const std::map<Id, OccRoad::Ptr>& occ_roads, const Group::Ptr& grp) {
  // 生成OccSegments
  const auto& start_slice = grp->start_slice;
  Eigen::Vector2f start_po = start_slice.po.head<2>();
  Eigen::Vector2f start_pl = start_slice.pl.head<2>();
  const auto& end_slice = grp->end_slice;
  Eigen::Vector2f end_po = end_slice.po.head<2>();
  Eigen::Vector2f end_pl = end_slice.pl.head<2>();

  if (occ_roads.empty()) {
    HLOG_DEBUG << "occ_roads is empty";
    return;
  } else {
    HLOG_DEBUG << "occ_roads size: " << occ_roads.size();
    double dist_sum = 0.0;
    double avg_dist = 0.0;

    for (const auto& occ : occ_roads) {
      auto occ_seg = std::make_shared<EdgeSegment>();
      auto& track_id = occ.first;
      auto& occ_lane = occ.second;
      occ_seg->id = track_id;
      if (static_cast<int>(occ_lane->road_points.size()) < 10) {
        continue;
      } else {
        while (!occ_lane->road_points.empty()) {
          Eigen::Vector2f front_pt_xy =
              occ_lane->road_points.front().head<2>().cast<float>();
          // 第一个点已经在end_slice的右边，直接跳过这条线
          if (PointInVectorSide(end_po, end_pl, front_pt_xy) > 0) {
            HLOG_DEBUG << "break!!!";
            break;
          }
          Point point_temp{
              RAW, static_cast<float>(occ_lane->road_points.front().x()),
              static_cast<float>(occ_lane->road_points.front().y()),
              static_cast<float>(occ_lane->road_points.front().z())};
          // 第一个点在end_slice的左边，并且在start_slice的右边，说明属于此group，加进来
          if (PointInVectorSide(start_po, start_pl, front_pt_xy) >= 0) {
            occ_seg->pts.emplace_back(point_temp);
          }
          // 把处理完的第一个点pop掉，防止后面重复处理
          occ_lane->road_points.erase(occ_lane->road_points.begin());
        }
        HLOG_DEBUG << "occ_seg->pts.size():" << occ_seg->pts.size();
        if (static_cast<int>(occ_seg->pts.size()) < 1) {
          continue;
        }
      }
      auto occ_length = Dist(occ_seg->pts.front().pt, occ_seg->pts.back().pt);
      auto grp_length = Dist(grp->start_slice.po, grp->end_slice.po);
      if (occ_length < 8.0 || occ_length < (grp_length * 0.3)) {
        continue;
      }

      int size_occ_seg = static_cast<int>(occ_seg->pts.size());
      Eigen::Vector3f point_tmp(0.0, 0.0, 0.0);
      for (int i = 0; i < size_occ_seg; ++i) {
        point_tmp += occ_seg->pts[i].pt;
      }
      occ_seg->center = point_tmp / size_occ_seg;
      // auto dist =
      //     PointToVectorDist(start_slice.po, end_slice.po, occ_seg->center);
      // auto dist = occ_seg->center[1];  // !TBD: 暂时用y坐标代替距离
      auto dist = Dist2Path(occ_seg->center);
      HLOG_DEBUG << "occ dist: " << dist;
      dist_sum += dist;

      occ_seg->dist_to_path = dist;
      occ_seg->road_edge_type = OCC;

      grp->occ_segments.emplace_back(occ_seg);
      HLOG_DEBUG << "grp->occ_segments.size():" << grp->occ_segments.size();
    }

    if (grp->occ_segments.empty()) {
      return;
    }

    avg_dist = dist_sum / static_cast<double>(grp->occ_segments.size());

    if (static_cast<int>(grp->occ_segments.size()) < 2) {
      auto& o_s = grp->occ_segments.front();
      if (o_s->dist_to_path < 0) {
        o_s->is_right = true;
      } else {
        o_s->is_left = true;
      }
    } else {
      for (auto& occ_seg : grp->occ_segments) {
        if (occ_seg->dist_to_path < avg_dist) {
          occ_seg->is_right = true;
          HLOG_DEBUG << "occ_seg is_right";
        } else {
          occ_seg->is_left = true;
          HLOG_DEBUG << "occ_seg is_left";
        }
      }
    }
  }

  std::sort(grp->occ_segments.begin(), grp->occ_segments.end(),
            [](const LineSegment::Ptr& a, const LineSegment::Ptr& b) {
              return a->dist_to_path > b->dist_to_path;
            });
}

void RoadConstruct::SplitModelEdgesToGroup(
    const std::map<Id, RoadEdge::Ptr>& road_edges, const Group::Ptr& grp) {
  // 生成ModelEdgesSegments
  const auto& start_slice = grp->start_slice;
  Eigen::Vector2f start_po = start_slice.po.head<2>();
  Eigen::Vector2f start_pl = start_slice.pl.head<2>();
  const auto& end_slice = grp->end_slice;
  Eigen::Vector2f end_po = end_slice.po.head<2>();
  Eigen::Vector2f end_pl = end_slice.pl.head<2>();

  if (road_edges.empty()) {
    HLOG_DEBUG << "road_edges is empty";
    return;
  } else {
    HLOG_DEBUG << "road_edges size: " << road_edges.size();
    double dist_sum = 0.0;
    double avg_dist = 0.0;

    for (const auto& model_edge : road_edges) {
      auto model_edge_seg = std::make_shared<EdgeSegment>();
      auto& track_id = model_edge.first;
      auto& edge = model_edge.second;
      model_edge_seg->id = track_id;
      if (edge->points.size() < 10) {
        continue;
      } else {
        while (!edge->points.empty()) {
          Eigen::Vector2f front_pt_xy =
              edge->points.front().head<2>().cast<float>();
          // 第一个点已经在end_slice的右边，直接跳过这条线
          if (PointInVectorSide(end_po, end_pl, front_pt_xy) > 0) {
            HLOG_DEBUG << "break!!!";
            break;
          }
          Point point_temp{RAW, static_cast<float>(edge->points.front().x()),
                           static_cast<float>(edge->points.front().y()),
                           static_cast<float>(edge->points.front().z())};
          // 第一个点在end_slice的左边，并且在start_slice的右边，说明属于此group，加进来
          if (PointInVectorSide(start_po, start_pl, front_pt_xy) >= 0) {
            model_edge_seg->pts.emplace_back(point_temp);
          }
          // 把处理完的第一个点pop掉，防止后面重复处理
          edge->points.erase(edge->points.begin());
        }
        HLOG_DEBUG << "model_edge_seg->points.size():"
                   << model_edge_seg->pts.size();
        if (model_edge_seg->pts.size() < 1) {
          continue;
        }
      }
      auto model_length =
          Dist(model_edge_seg->pts.front().pt, model_edge_seg->pts.back().pt);
      auto grp_length = Dist(grp->start_slice.po, grp->end_slice.po);
      if (model_length < 8.0 || model_length < (grp_length * 0.3)) {
        continue;
      }

      int size_occ_seg = static_cast<int>(model_edge_seg->pts.size());
      Eigen::Vector3f point_tmp(0.0, 0.0, 0.0);
      for (int i = 0; i < size_occ_seg; ++i) {
        point_tmp += model_edge_seg->pts[i].pt;
      }
      model_edge_seg->center = point_tmp / size_occ_seg;
      // auto dist = PointToVectorDist(start_slice.po, end_slice.po,
      //                               model_edge_seg->center);
      // auto dist = model_edge_seg->center[1];  // !TBD: 暂时用y坐标代替距离
      auto dist = Dist2Path(model_edge_seg->center);
      HLOG_DEBUG << "model dist: " << dist;
      dist_sum += dist;

      model_edge_seg->dist_to_path = dist;
      model_edge_seg->road_edge_type = MODEL;
      grp->model_edge_segments.emplace_back(model_edge_seg);
      HLOG_DEBUG << "grp->model_edge_segments.size():"
                 << grp->model_edge_segments.size();
    }

    if (grp->model_edge_segments.empty()) {
      return;
    }

    avg_dist = dist_sum / static_cast<double>(grp->model_edge_segments.size());

    if (static_cast<int>(grp->model_edge_segments.size()) < 2) {
      auto& m_e_s = grp->model_edge_segments.front();
      if (m_e_s->dist_to_path < 0) {
        m_e_s->is_right = true;
      } else {
        m_e_s->is_left = true;
      }
    } else {
      for (auto& model_edge_seg : grp->model_edge_segments) {
        if (model_edge_seg->dist_to_path < avg_dist) {
          model_edge_seg->is_right = true;
          HLOG_DEBUG << "model_edge_seg is_right";
        } else {
          model_edge_seg->is_left = true;
          HLOG_DEBUG << "model_edge_seg is_left";
        }
      }
    }
  }

  std::sort(grp->model_edge_segments.begin(), grp->model_edge_segments.end(),
            [](const LineSegment::Ptr& a, const LineSegment::Ptr& b) {
              return a->dist_to_path > b->dist_to_path;
            });
}

void RoadConstruct::SplitPtsToGroup(std::deque<Line::Ptr>* lines,
                                    const ElementMap::Ptr& ele_map,
                                    std::vector<Group::Ptr>* groups) {
  for (auto& grp : *groups) {
    grp->line_segments.clear();
    grp->occ_segments.clear();
    grp->model_edge_segments.clear();

    SplitLinesToGroup(lines, grp);
    ExtendLineToAlignSlice(grp);
    SplitOccsToGroup(ele_map->occ_roads, grp);
    SplitModelEdgesToGroup(ele_map->road_edges, grp);

    HLOG_DEBUG << "grp->line_segments.size(): " << grp->line_segments.size();
    HLOG_DEBUG << "grp->occ_segments.size(): " << grp->occ_segments.size();
    HLOG_DEBUG << "grp->model_edge_segments.size(): "
               << grp->model_edge_segments.size();
  }
}

void RoadConstruct::ExtendLineToAlignSlice(const Group::Ptr& grp) {
  auto start_slice = grp->start_slice;
  auto end_slice = grp->end_slice;
  HLOG_DEBUG << "start_slice.po: " << start_slice.po.x() << ", "
             << start_slice.po.y();
  HLOG_DEBUG << "end_slice.po: " << end_slice.po.x() << ", "
             << end_slice.po.y();

  // 向前
  for (auto& line_seg : grp->line_segments) {
    HLOG_DEBUG << "line_seg->id: " << line_seg->id;
    HLOG_DEBUG << "line_seg->pts.size(): " << line_seg->pts.size();
    if (line_seg->pts.empty()) {
      HLOG_INFO << "line_seg->pts is empty";
      continue;
    }
    const auto dist_to_start_slice = PointToVectorDist(
        start_slice.po, start_slice.pl, line_seg->pts.front().pt);
    const auto dist_to_end_slice =
        PointToVectorDist(end_slice.po, end_slice.pl, line_seg->pts.back().pt);
    if (dist_to_start_slice > 10 || dist_to_end_slice > 10) {
      continue;
    }
    if (dist_to_start_slice > 0.4) {
      auto front_p = line_seg->pts.front().pt;
      int index = 1;
      while (index < static_cast<int>(line_seg->pts.size()) &&
             Dist(line_seg->pts[index].pt, front_p) < 5.0) {
        index++;
      }
      auto next_p = line_seg->pts[index].pt;
      auto unit = (front_p - next_p).normalized();
      int cnt = 0;
      int siz = static_cast<int>(line_seg->pts.size());
      while (PointToVectorDist(start_slice.po, start_slice.pr,
                               line_seg->pts.front().pt) > 0.4 &&
             PointInVectorSide(
                 Eigen::Vector2f(start_slice.po.head<2>()),
                 Eigen::Vector2f(start_slice.pl.head<2>()),
                 Eigen::Vector2f(line_seg->pts.front().pt.head<2>())) > 0) {
        HLOG_DEBUG << "start dist: "
                   << PointToVectorDist(start_slice.po, start_slice.pr,
                                        line_seg->pts.front().pt);
        cnt++;
        if (cnt > siz) {
          break;
          HLOG_ERROR << "front cnt > siz!!!";
        }
        auto start_point = line_seg->pts.front().pt;
        Point t(VIRTUAL, start_point.x() + unit.x(), start_point.y() + unit.y(),
                static_cast<float>(0.0));
        line_seg->pts.insert(line_seg->pts.begin(), t);
        HLOG_DEBUG << "insert start point: " << t.pt.x() << " " << t.pt.y();
      }
    }
    // 向后
    if (PointToVectorDist(end_slice.po, end_slice.pl, line_seg->pts.back().pt) >
        0.4) {
      auto back_p = line_seg->pts.back().pt;
      int index = static_cast<int>(line_seg->pts.size()) - 1;
      while (index > 0 && Dist(line_seg->pts[index].pt, back_p) < 5.0) {
        index--;
      }
      auto next_p = line_seg->pts[index].pt;
      auto unit = (back_p - next_p).normalized();
      int cnt = 0;
      int siz = static_cast<int>(line_seg->pts.size());
      while (PointToVectorDist(end_slice.po, end_slice.pl,
                               line_seg->pts.back().pt) > 0.4 &&
             PointInVectorSide(
                 Eigen::Vector2f(end_slice.po.head<2>()),
                 Eigen::Vector2f(end_slice.pl.head<2>()),
                 Eigen::Vector2f(line_seg->pts.back().pt.head<2>())) < 0) {
        HLOG_DEBUG << "end dist: "
                   << PointToVectorDist(end_slice.po, end_slice.pl,
                                        line_seg->pts.back().pt);
        cnt++;
        if (cnt > siz) {
          break;
          HLOG_ERROR << "back cnt > siz!!!";
        }
        auto end_point = line_seg->pts.back().pt;
        Point t(VIRTUAL, end_point.x() + unit.x(), end_point.y() + unit.y(),
                static_cast<float>(0.0));
        line_seg->pts.insert(line_seg->pts.end(), t);
        HLOG_DEBUG << "insert end point: " << t.pt.x() << " " << t.pt.y();
      }
    }
  }
}

void RoadConstruct::GenRoadEdges(std::vector<Group::Ptr>* groups) {
  HLOG_DEBUG << "GenRoadEdges";

  if (groups == nullptr) {
    HLOG_ERROR << "groups is null";
    return;
  }

  if (groups->empty()) {
    HLOG_ERROR << "groups is empty";
    return;
  }

  for (auto& grp : *groups) {
    HLOG_DEBUG << "grp name: " << grp->str_id;

    grp->road_edges.clear();
    RoadEdge::Ptr road_edge_left = std::make_shared<RoadEdge>();
    RoadEdge::Ptr road_edge_right = std::make_shared<RoadEdge>();

    if (grp->line_segments.empty() && grp->occ_segments.empty() &&
        grp->model_edge_segments.empty()) {
      HLOG_INFO << "all segments is empty";
      continue;
    }

    UpdateRoadEdgeWithOccs(grp, &road_edge_left, &road_edge_right);
    UpdateRoadEdgeWithModelEdges(grp, &road_edge_left, &road_edge_right);
    UpdateRoadEdgeWithLines(grp, &road_edge_left, &road_edge_right);

    HLOG_DEBUG << "road edge info: ";
    HLOG_DEBUG << "road_edge_left: \t" << "id: " << road_edge_left->id << " \t"
               << "is_left: " << road_edge_left->is_left << " \t"
               << "road_edge_type: " << road_edge_left->road_edge_type << " \t"
               << "dist_to_path: " << road_edge_left->dist_to_path << " \t"
               << "points: " << road_edge_left->points.size();

    HLOG_DEBUG << "road_edge_right: \t" << "id: " << road_edge_right->id
               << " \t" << "is_right: " << road_edge_right->is_right << " \t"
               << "road_edge_type: " << road_edge_right->road_edge_type << " \t"
               << "dist_to_path: " << road_edge_right->dist_to_path << " \t"
               << "points: " << road_edge_right->points.size();

    grp->road_edges.emplace_back(road_edge_left);
    grp->road_edges.emplace_back(road_edge_right);
  }
}

void RoadConstruct::UpdateRoadEdgeWithOccs(const Group::Ptr& grp,
                                           RoadEdge::Ptr* road_edge_left,
                                           RoadEdge::Ptr* road_edge_right) {
  if (conf_.use_occ) {
    if (grp->occ_segments.empty()) {
      HLOG_DEBUG << "occ_segments is empty";
    } else {
      HLOG_DEBUG << "occ_segments size: " << grp->occ_segments.size();

      RoadEdge::Ptr OCC_road_edge_left = std::make_shared<RoadEdge>();
      RoadEdge::Ptr OCC_road_edge_right = std::make_shared<RoadEdge>();

      FindNearestCaditate(grp->occ_segments, &OCC_road_edge_left,
                          &OCC_road_edge_right);

      UpdateWithCandidate(road_edge_left, road_edge_right, OCC_road_edge_left,
                          OCC_road_edge_right);
    }
  }
}

void RoadConstruct::UpdateRoadEdgeWithModelEdges(
    const Group::Ptr& grp, RoadEdge::Ptr* road_edge_left,
    RoadEdge::Ptr* road_edge_right) {
  if (grp->model_edge_segments.empty()) {
    HLOG_DEBUG << "model_edge_segments is empty";
  } else {
    HLOG_DEBUG << "model_edge_segments size: "
               << grp->model_edge_segments.size();

    RoadEdge::Ptr model_road_edge_left = std::make_shared<RoadEdge>();
    RoadEdge::Ptr model_road_edge_right = std::make_shared<RoadEdge>();

    FindNearestCaditate(grp->model_edge_segments, &model_road_edge_left,
                        &model_road_edge_right);

    UpdateWithCandidate(road_edge_left, road_edge_right, model_road_edge_left,
                        model_road_edge_right);
  }
}

void RoadConstruct::UpdateRoadEdgeWithLines(const Group::Ptr& grp,
                                            RoadEdge::Ptr* road_edge_left,
                                            RoadEdge::Ptr* road_edge_right) {
  if (grp->line_segments.empty()) {
    HLOG_DEBUG << "line_segments is empty";
  } else {
    HLOG_DEBUG << "line_segments size: " << grp->line_segments.size();
    RoadEdge::Ptr line_road_edge_left = std::make_shared<RoadEdge>();
    RoadEdge::Ptr line_road_edge_right = std::make_shared<RoadEdge>();
    // left
    line_road_edge_left->id = grp->line_segments.front()->id;
    line_road_edge_left->is_left = true;
    line_road_edge_left->road_edge_type = LINE;
    line_road_edge_left->dist_to_path =
        grp->line_segments.front()->dist_to_path;
    for (const auto& p : grp->line_segments.front()->pts) {
      line_road_edge_left->points.emplace_back(p.pt);
    }
    HLOG_DEBUG << "line_road_edge_left->dist_to_path: "
               << line_road_edge_left->dist_to_path;

    if (static_cast<int>(grp->line_segments.size()) > 1) {
      // right
      line_road_edge_right->id = grp->line_segments.back()->id;
      line_road_edge_right->is_right = true;
      line_road_edge_right->road_edge_type = LINE;
      line_road_edge_right->dist_to_path =
          grp->line_segments.back()->dist_to_path;
      for (const auto& p : grp->line_segments.back()->pts) {
        line_road_edge_right->points.emplace_back(p.pt);
      }
      HLOG_DEBUG << "line_road_edge_right->dist_to_path: "
                 << line_road_edge_right->dist_to_path;
    }
    // update
    UpdateWithCandidate(road_edge_left, road_edge_right, line_road_edge_left,
                        line_road_edge_right);
  }
}

void RoadConstruct::FindNearestCaditate(
    const std::vector<EdgeSegment::Ptr>& edge_segments,
    RoadEdge::Ptr* candidate_road_edge_left,
    RoadEdge::Ptr* candidate_road_edge_right) {
  for (const auto& seg : edge_segments) {
    RoadEdge::Ptr road_edge_temp = std::make_shared<RoadEdge>();
    road_edge_temp->id = seg->id;
    road_edge_temp->road_edge_type = seg->road_edge_type;
    road_edge_temp->dist_to_path = seg->dist_to_path;
    HLOG_DEBUG << "road_edge_temp->dist_to_path: "
               << road_edge_temp->dist_to_path;

    for (const auto& p : seg->pts) {
      road_edge_temp->points.emplace_back(p.pt);
    }

    if (seg->is_left) {
      road_edge_temp->is_left = true;
      if ((*candidate_road_edge_left)->points.empty()) {
        HLOG_DEBUG << "First !!! candidate_road_edge_left->points.empty()";
        (*candidate_road_edge_left) = road_edge_temp;
      } else {
        HLOG_DEBUG << "candidate_road_edge_left->dist_to_path: "
                   << (*candidate_road_edge_left)->dist_to_path
                   << "  road_edge_temp->dist_to_path: "
                   << road_edge_temp->dist_to_path;
        (*candidate_road_edge_left) =
            (*candidate_road_edge_left)->dist_to_path <
                    road_edge_temp->dist_to_path
                ? (*candidate_road_edge_left)
                : road_edge_temp;
        if ((*candidate_road_edge_left)->dist_to_path >
            road_edge_temp->dist_to_path) {
          HLOG_DEBUG << "update left "
                     << (*candidate_road_edge_left)->road_edge_type;
        }
      }
    }
    if (seg->is_right) {
      road_edge_temp->is_right = true;
      if ((*candidate_road_edge_right)->points.empty()) {
        HLOG_DEBUG << "First !!! candidate_road_edge_right->points.empty()";
        (*candidate_road_edge_right) = road_edge_temp;
      } else {
        HLOG_DEBUG << "candidate_road_edge_right->dist_to_path: "
                   << (*candidate_road_edge_right)->dist_to_path
                   << "  road_edge_temp->dist_to_path: "
                   << road_edge_temp->dist_to_path;
        (*candidate_road_edge_right) =
            (*candidate_road_edge_right)->dist_to_path >
                    road_edge_temp->dist_to_path
                ? (*candidate_road_edge_right)
                : road_edge_temp;
        if ((*candidate_road_edge_right)->dist_to_path <
            road_edge_temp->dist_to_path) {
          HLOG_DEBUG << "update right "
                     << (*candidate_road_edge_right)->road_edge_type;
        }
      }
    }
  }
}

void RoadConstruct::UpdateWithCandidate(
    RoadEdge::Ptr* road_edge_left, RoadEdge::Ptr* road_edge_right,
    const RoadEdge::Ptr& candidate_road_edge_left,
    const RoadEdge::Ptr& candidate_road_edge_right) {
  if ((*road_edge_left)->points.empty()) {
    HLOG_DEBUG << "first update left road edge";
    (*road_edge_left) = candidate_road_edge_left;
  } else {
    HLOG_DEBUG << "road_edge_left->dist_to_path: "
               << (*road_edge_left)->dist_to_path;
    HLOG_DEBUG << "candidate_road_edge_left->dist_to_path: "
               << candidate_road_edge_left->dist_to_path;
    if (candidate_road_edge_left->dist_to_path != -1000) {
      auto dist_left = candidate_road_edge_left->dist_to_path -
                       (*road_edge_left)->dist_to_path;
      if (candidate_road_edge_left->road_edge_type != LINE) {
        (*road_edge_left) =
            dist_left > 0.0 ? (*road_edge_left) : candidate_road_edge_left;
        if (dist_left < 0.0) {
          HLOG_DEBUG << "update left "
                     << candidate_road_edge_left->road_edge_type;
        }
      } else {
        if (dist_left < 0 && (dist_left > -1.0 || dist_left < -20.0)) {
          (*road_edge_left) = candidate_road_edge_left;
          HLOG_DEBUG << "line update left "
                     << candidate_road_edge_left->road_edge_type;
        }
      }
    }
  }

  if ((*road_edge_right)->points.empty()) {
    HLOG_DEBUG << "first update right road edge";
    (*road_edge_right) = candidate_road_edge_right;
  } else {
    HLOG_DEBUG << "road_edge_right->dist_to_path: "
               << (*road_edge_right)->dist_to_path;
    HLOG_DEBUG << "candidate_road_edge_right->dist_to_path: "
               << candidate_road_edge_right->dist_to_path;
    if (candidate_road_edge_right->dist_to_path != -1000) {
      auto dist_right = candidate_road_edge_right->dist_to_path -
                        (*road_edge_right)->dist_to_path;
      if (candidate_road_edge_right->road_edge_type != LINE) {
        (*road_edge_right) =
            dist_right < 0.0 ? (*road_edge_right) : candidate_road_edge_right;
        if (dist_right > 0.0) {
          HLOG_DEBUG << "update right "
                     << candidate_road_edge_right->road_edge_type;
        }
      } else {
        if (dist_right > 0 && (dist_right < 1.0 || dist_right > 20.0)) {
          (*road_edge_right) = candidate_road_edge_right;
          HLOG_DEBUG << "line update right "
                     << candidate_road_edge_right->road_edge_type;
        }
      }
    }
  }
}

void RoadConstruct::GenLanesInGroups(std::vector<Group::Ptr>* groups,
                                     const ElementMap::Ptr& ele_map,
                                     double stamp) {
  for (auto& grp : *groups) {
    GenGroupAllLanes(grp);
    FilterGroupBadLane(grp);
    // MatchLRLane(grp);
    EgoLineTrajectory(grp, ele_map);
  }
  //   |   |
  //   *   *
  //   |   |     Group
  //   *   *
  //   (中断)
  // - - - - -  SliceLine
  //   (中断)
  //   *   *
  //   |   |     Group
  //   *   *
  //   |   |
  // 对上面相邻的Group，SliceLine切分处会缺失点，导致前一个group里的线与后一个group里的线断开了，
  // 这里对于前一个group里每根线，从后一个group查找是否存在，如果存在就将后一个group那根线的front
  // 直接加到前一个group的back
  OptiPreNextLaneBoundaryPoint(groups);
  // 生成车道中心线
  GenLaneCenterLine(groups);

  // 删除脑部多的ego_group，青鸾号:1273597
  // EraseEgoGroupWithNoEgoLane(groups);

  // 删除空的group数据
  int before_remove_grp_nums = static_cast<int>(groups->size());
  // RemoveNullGroup(groups);
  int after_remove_grp_nums = static_cast<int>(groups->size());
  if (after_remove_grp_nums != before_remove_grp_nums) {
    HLOG_WARN << "*********[CrossWalk]*******delete null group nums: "
              << after_remove_grp_nums - before_remove_grp_nums;
  }

  HLOG_DEBUG << "current stamp is:" << std::to_string(stamp);

  // SmoothCenterline(groups);
}

void RoadConstruct::GenGroupAllLanes(const Group::Ptr& grp) {
  // 生成Lane
  const auto line_seg_num = grp->line_segments.size();
  if (line_seg_num > 1) {
    // 按边线距离生成Lane
    for (size_t i = 0; i < line_seg_num - 1; ++i) {
      auto& left_line = grp->line_segments.at(i);
      auto& right_line = grp->line_segments.at(i + 1);
      int l_size = static_cast<int>(left_line->pts.size());
      if (l_size == 0) {
        continue;
      }
      auto& left_distp = left_line->pts[static_cast<int>(l_size * 0.5)].pt;
      auto right_center = right_line->center;
      //! 商汤切分点情况下，由于center点是用首尾两点求中值，如果在弯道，该点会在车道外，导致计算距离错误
      auto dist = DistByKDtree(left_distp, *right_line);
      // HLOG_INFO << "dist: " << dist;
      if (dist < conf_.min_lane_width) {
        continue;
      }

      if (dist > conf_.max_lane_width) {
        continue;
      }
      auto lane = std::make_shared<Lane>();
      lane->left_boundary = left_line;
      lane->right_boundary = right_line;
      grp->lanes.emplace_back(lane);
    }
  }
}

void RoadConstruct::FilterGroupBadLane(const Group::Ptr& grp) {
  auto temp_lanes = grp->lanes;
  grp->lanes.clear();
  for (auto& lane : temp_lanes) {
    if (lane != nullptr && lane->left_boundary != nullptr &&
        lane->left_boundary->pts.size() > 1 &&
        lane->right_boundary != nullptr &&
        lane->right_boundary->pts.size() > 1) {
      grp->lanes.emplace_back(lane);
    }
  }
}

void RoadConstruct::GenGroupName(const ElementMap::Ptr& ele_map,
                                 std::vector<Group::Ptr>* groups) {
  int grp_idx = -1;
  double stamp = ele_map->map_info.stamp;
  for (auto& grp : *groups) {
    grp_idx++;
    if (!grp->lanes.empty()) {
      std::string lane_names;

      for (auto& lane : grp->lanes) {
        lane->str_id = std::to_string(lane->left_boundary->id) + "_" +
                       std::to_string(lane->right_boundary->id);
        lane->lanepos_id = std::to_string(lane->left_boundary->lanepos) + "_" +
                           std::to_string(lane->right_boundary->lanepos);
        if (!lane_names.empty()) {
          lane_names += "|";
        }
        lane_names += lane->str_id;
      }

      grp->str_id =
          std::string("G") + std::to_string(grp_idx) + "-" + lane_names;
      grp->stamp = stamp;

      for (auto& lane : grp->lanes) {
        lane->str_id_with_group = grp->str_id + ":" + lane->str_id;
        // HLOG_ERROR << "lane->str_id_with_group: " << lane->str_id_with_group;
      }
    } else {
      // HLOG_INFO << "no lane in grp";
      grp->str_id = std::string("G") + std::to_string(grp_idx) + "-no_lane";
      grp->stamp = stamp;
      if (!grp->line_segments.empty()) {
        for (auto& line_seg : grp->line_segments) {
          grp->str_id += "_" + std::to_string(line_seg->id);
        }
      } else if (!grp->occ_segments.empty()) {
        for (auto& occ_seg : grp->occ_segments) {
          grp->str_id += "_" + std::to_string(occ_seg->id);
        }
      }
      // HLOG_INFO << "grp->str_id: " << grp->str_id;
    }
  }
}

void RoadConstruct::EgoLineTrajectory(const Group::Ptr& grp,
                                      const ElementMap::Ptr& ele_map) {
  int line1_id = -200, line2_id = -200, near_line = -200;
  int flag = 0;
  if (grp->end_slice.po.x() > -5 && grp->start_slice.po.x() < 5) {
    if (grp->line_segments.size() > 1) {
      for (int i = 0; i < static_cast<int>(grp->line_segments.size()) - 1;
           ++i) {
        if ((grp->line_segments[i]->dist_to_path *
                 grp->line_segments[i + 1]->dist_to_path <
             0.1) &&
            fabs(grp->line_segments[i]->dist_to_path) < 3.75 &&
            fabs(grp->line_segments[i + 1]->dist_to_path) < 3.75) {
          line1_id = grp->line_segments[i]->id;
          line2_id = grp->line_segments[i + 1]->id;
          near_line = (abs(grp->line_segments[i]->dist_to_path) <
                       abs(grp->line_segments[i + 1]->dist_to_path))
                          ? line1_id
                          : line2_id;
          flag = 1;
          break;
        }
      }
    }
  }

  if (line1_id == -200 && line2_id == -200) {
    ego_line_exist_ = false;
  } else {
    ego_line_exist_ = true;
    FitLaneline(ele_map, line1_id, line2_id, near_line);
  }
  LOCATION_MANAGER->SetEgoLane(line1_id, line2_id);
  HLOG_DEBUG << "ego line id:" << line1_id << "," << line2_id;
}

void RoadConstruct::SetBrokenId(std::vector<Group::Ptr>* groups) {
  if (groups->empty()) {
    HLOG_ERROR << "Wrong!! groups is empty!";
    return;
  }
  if (static_cast<int>(groups->size()) < 2) {
    groups->front()->broken_id = 0;
    return;
  } else {
    int broken_idx = -1;
    groups->front()->broken_id = 0;

    auto last_end_slice = groups->front()->end_slice;
    for (auto& grp : *groups) {
      if (grp->start_slice.po == last_end_slice.po) {
        grp->broken_id = broken_idx;
      } else {
        ++broken_idx;
        grp->broken_id = broken_idx;
      }
      last_end_slice = grp->end_slice;
    }
  }
}

float RoadConstruct::DistByKDtree(const Eigen::Vector3f& ref_point,
                                  const LineSegment& LineSegment) {
  distpoits_.push_back(ref_point);
  int id = LineSegment.id;
  if (KDTrees_[id] == nullptr) {
    return 0.0;
  }

  float ref_x = static_cast<float>(ref_point.x());
  float ref_y = static_cast<float>(ref_point.y());
  if (std::isnan(ref_x) || std::isnan(ref_y)) {
    return 0.0;
  }

  const int dim = 1;
  std::vector<int> nearest_index(dim);
  std::vector<float> nearest_dist(dim);
  std::vector<float> query_point = {ref_x, ref_y};
  auto kdtree_tofind = KDTrees_[id];
  auto line_tofind = *line_points_[id];

  kdtree_tofind->knnSearch(query_point, nearest_index, nearest_dist, dim,
                           cv::flann::SearchParams(-1));

  Eigen::Vector3f tar_point;
  tar_point.x() = line_tofind[nearest_index[0]].x;
  tar_point.y() = line_tofind[nearest_index[0]].y;
  distpoits_.push_back(tar_point);

  if (line_tofind.size() == 1) {
    return nearest_dist[0];
  } else {
    int id_next = 0;
    if (nearest_index[0] < static_cast<int>(line_tofind.size()) - 1) {
      id_next = nearest_index[0] + 1;
    } else {
      id_next = nearest_index[0] - 1;
    }
    Eigen::Vector3f tar_point_next;
    tar_point_next.x() = line_tofind[id_next].x;
    tar_point_next.y() = line_tofind[id_next].y;
    distpoits_.push_back(tar_point_next);

    return GetDistPointLane(ref_point, tar_point, tar_point_next);
  }
}

float RoadConstruct::GetDistPointLane(const Eigen::Vector3f& point_a,
                                      const Eigen::Vector3f& point_b,
                                      const Eigen::Vector3f& point_c) {
  Eigen::Vector2f A(point_a.x(), point_a.y());
  Eigen::Vector2f B(point_b.x(), point_b.y());
  Eigen::Vector2f C(point_c.x(), point_c.y());
  // 以B为起点计算向量BA 在向量BC上的投影
  Eigen::Vector2f BC = C - B;
  Eigen::Vector2f BA = A - B;

  if (abs(BC.norm()) < 0.0001) {
    return abs(BA.y());
  }

  float dist_proj = BA.dot(BC) / BC.norm();
  // 计算点到直线的距离
  float point_dist =
      static_cast<float>(sqrt(pow(BA.norm(), 2) - pow(dist_proj, 2)));
  return point_dist;
}

void RoadConstruct::FitLaneline(const ElementMap::Ptr& ele_map, int id_1,
                                int id_2, int near_line) {
  predict_line_params_.clear();
  int size1_s = -1, size2_s = -1, size1_num = 0, size2_num = 0;
  for (const auto& node : ele_map->lane_boundaries[id_1]->nodes) {
    if (node->point.x() > -15 && node->point.x() < 15) {
      if (size1_s == -1) {
        size1_s = node->id;
      }
      size1_num++;
    } else if (node->point.x() > 15) {
      break;
    }
  }
  for (const auto& node : ele_map->lane_boundaries[id_2]->nodes) {
    if (node->point.x() > -15 && node->point.x() < 15) {
      if (size2_s == -1) {
        size2_s = node->id;
      }
      size2_num++;
    } else if (node->point.x() > 15) {
      break;
    }
  }
  if (size1_num < 8 && size2_num < 8) {
    ego_line_exist_ = false;
    return;
  }
  if (id_1 == near_line && size1_num > 4) {
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(size1_num, 4);
    Eigen::VectorXd x(size1_num);
    Eigen::VectorXd b(size1_num);
    for (int i = 0; i < size1_num; i++) {
      double xi = ele_map->boundary_nodes[size1_s + i]->point.x();
      double yi = ele_map->boundary_nodes[size1_s + i]->point.y();
      A(i, 0) = 1.0;
      A(i, 1) = xi;
      A(i, 2) = xi * xi;
      A(i, 3) = xi * xi * xi;
      b[i] = yi;
    }
    x = (A.transpose() * A).inverse() * A.transpose() * b;
    predict_line_params_ = {x[0], x[1], x[2], x[3]};
  } else {
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(size2_num, 4);
    Eigen::VectorXd x(size2_num);
    Eigen::VectorXd b(size2_num);
    for (int i = 0; i < size2_num; i++) {
      double xi = ele_map->boundary_nodes[size2_s + i]->point.x();
      double yi = ele_map->boundary_nodes[size2_s + i]->point.y();
      A(i, 0) = 1.0;
      A(i, 1) = xi;
      A(i, 2) = xi * xi;
      A(i, 3) = xi * xi * xi;
      b[i] = yi;
    }
    x = (A.transpose() * A).inverse() * A.transpose() * b;
    predict_line_params_ = {x[0], x[1], x[2], x[3]};
  }
  // HLOG_ERROR << "predict_line_params_ = " << predict_line_params_[0] << "  "
  //            << predict_line_params_[1] << "  " << predict_line_params_[2]
  //            << "  " << predict_line_params_[3];
}

bool RoadConstruct::FilterGroupBadLane(
    const std::vector<Lane::Ptr>& possible_lanes, const Group::Ptr& grp) {
  // 过滤掉边线只有一个点的lane
  std::vector<Lane::Ptr> valid_lanes;
  for (const auto& lane : possible_lanes) {
    if (lane != nullptr && lane->left_boundary != nullptr &&
        lane->left_boundary->pts.size() > 1 &&
        lane->right_boundary != nullptr &&
        lane->right_boundary->pts.size() > 1) {
      grp->lanes.emplace_back(lane);
    }
  }
  return true;
}

bool RoadConstruct::MatchLRLane(const Group::Ptr& grp) {
  if (grp->lanes.size() > 1) {
    for (int i = 0; i < static_cast<int>(grp->lanes.size()) - 1; ++i) {
      auto& curr = grp->lanes.at(i);
      for (size_t j = i + 1; j < grp->lanes.size(); ++j) {
        const auto& right = grp->lanes.at(j);
        curr->right_lane_str_id_with_group.emplace_back(
            right->str_id_with_group);
      }
    }
    for (int i = static_cast<int>(grp->lanes.size()) - 1; i > 0; --i) {
      auto& curr = grp->lanes.at(i);
      for (int j = i - 1; j >= 0; --j) {
        const auto& left = grp->lanes.at(j);
        curr->left_lane_str_id_with_group.emplace_back(left->str_id_with_group);
      }
    }
  }
  return true;
}

bool RoadConstruct::OptiPreNextLaneBoundaryPoint(
    std::vector<Group::Ptr>* groups) {
  for (int i = 0; i < static_cast<int>(groups->size()) - 1; ++i) {
    auto& curr_grp = groups->at(i);
    auto& next_grp = groups->at(i + 1);
    std::map<Id, LineSegment::Ptr> curr_lines;
    std::map<Id, LineSegment::Ptr> next_lines;
    for (auto& lanes : curr_grp->lanes) {
      if (lanes->left_boundary != nullptr) {
        curr_lines.insert_or_assign(lanes->left_boundary->id,
                                    lanes->left_boundary);
      }
      if (lanes->right_boundary != nullptr) {
        curr_lines.insert_or_assign(lanes->right_boundary->id,
                                    lanes->right_boundary);
      }
    }
    for (auto& lanes : next_grp->lanes) {
      if (lanes->left_boundary != nullptr) {
        next_lines.insert_or_assign(lanes->left_boundary->id,
                                    lanes->left_boundary);
      }
      if (lanes->right_boundary != nullptr) {
        next_lines.insert_or_assign(lanes->right_boundary->id,
                                    lanes->right_boundary);
      }
    }
    for (auto& curr_line : curr_lines) {
      Id line_id = curr_line.first;
      if (curr_line.second->pts.empty()) {
        continue;
      }
      if (next_lines.find(line_id) != next_lines.end()) {
        auto& next_line = next_lines[line_id];
        if (next_line->pts.empty()) {
          continue;
        }
        if ((curr_line.second->pts.back().pt - next_line->pts.front().pt)
                .norm() > 10.0) {
          continue;
        }
        curr_line.second->pts.emplace_back(next_line->pts.front());
      }
    }
  }
  return true;
}

bool RoadConstruct::GenLaneCenterLine(std::vector<Group::Ptr>* groups) {
  for (auto& grp : *groups) {
    for (auto& lane : grp->lanes) {
      FitCenterLine(lane);
    }
  }

  return true;
}

int RoadConstruct::FindEgoGroup(std::vector<Group::Ptr>* groups) {
  int index = -1;
  for (auto& grp : *groups) {
    index++;
    if (grp->start_slice.po.x() > 0.0) {
      // 前一个group是当前group或者没有找到当前group，需要再进行判断
      return -1;
    }
    if (grp->end_slice.po.x() > 0.0) {
      // 当前group是
      return index;
    }
  }
  return -1;
}

void RoadConstruct::EraseEgoGroupWithNoEgoLane(
    std::vector<Group::Ptr>* groups) {
  int index = FindEgoGroup(groups);
  if (index == -1 || static_cast<int>(groups->size()) <= index) {
    // 没找到自车所在group
    if (groups->empty()) {
      return;
    }
    auto curr_group = groups->back();

    auto curr_grp_start_slice = curr_group->start_slice;
    auto curr_grp_end_slice = curr_group->end_slice;

    Eigen::Vector2f curr_end_pl(curr_grp_end_slice.pl.x(),
                                curr_grp_end_slice.pl.y());
    Eigen::Vector2f curr_end_pr(curr_grp_end_slice.pr.x(),
                                curr_grp_end_slice.pr.y());
    Pose nearest;
    nearest.stamp = -1;
    float min_dist = FLT_MAX;
    for (const auto& p : path_in_curr_pose_) {
      Eigen::Vector2f pt(p.pos.x(), p.pos.y());
      float dist = PointToVectorDist(curr_end_pl, curr_end_pr, pt);
      if (dist < min_dist) {
        min_dist = dist;
        nearest = p;
        nearest.stamp = 0;
      }
    }

    // 找到与历史车辆位置最接近的curr_lane作为当前所在lane
    min_dist = FLT_MAX;  // min_dist = FLT_MAX;
    if (nearest.stamp >= 0) {
      Eigen::Vector3f temp_pt(1, 0, 0);
      // 转到当前车体系下
      Eigen::Vector3f temp_pt_curr_veh = nearest.quat * temp_pt + nearest.pos;
      Eigen::Vector2f p1(nearest.pos.x(), nearest.pos.y());
      Eigen::Vector2f p0(temp_pt_curr_veh.x(), temp_pt_curr_veh.y());
      for (auto& curr_lane : curr_group->lanes) {
        Eigen::Vector2f pt(curr_lane->center_line_pts.back().pt.x(),
                           curr_lane->center_line_pts.back().pt.y());
        // 把最近的centerline添加进来，要不然太远了角度有问题
        float pt_dis = (pt - p0).norm();
        for (auto it = curr_lane->center_line_pts.rbegin();
             it != curr_lane->center_line_pts.rend(); ++it) {
          Eigen::Vector2f point_center(it->pt.x(), it->pt.y());
          if ((point_center - p0).norm() > pt_dis) {
            break;
          }
          pt = point_center;
          pt_dis = (point_center - p0).norm();
        }
        float dist = PointToVectorDist(p0, p1, pt);
        min_dist = std::min(dist, min_dist);
      }
    }
    if (min_dist > conf_.max_lane_width) {
      groups->pop_back();
    }
    return;
  }
  auto& ego_group = groups->at(index);
  if (static_cast<int>(groups->size()) > index + 1 &&
      ego_group->end_slice.po.x() < 0.0 &&
      groups->at(index + 1)->start_slice.po.x() > 0.0) {
    // 没找到自车所在group
    return;
  }
  if (static_cast<int>(groups->size()) == index + 1 &&
      ego_group->end_slice.po.x() < 0.0) {
    return;
  }
  // 判断是否有自车道和邻车道
  for (auto& lane : ego_group->lanes) {
    if (lane->center_line_pts.size() < 1) {
      continue;
    }
    int index = -1;  // centerpoint_index;
    float best_dis = FLT_MAX;
    for (int i = 0; i < lane->center_line_pts.size(); ++i) {
      float dis = lane->center_line_pts[i].pt.norm();
      if (dis < best_dis) {
        index = i;
        best_dis = dis;
      } else {
        // 点都是顺序排列的，所以如果距离变大的话就可以退出了
        break;
      }
    }
    if (index == -1) {
      continue;
    }
    if (lane->center_line_pts[index].pt.norm() < conf_.max_lane_width) {
      return;
    }
  }
  groups->erase(groups->begin() + index);
}

void RoadConstruct::FitCenterLine(const Lane::Ptr& lane) {
  std::vector<Vec2d> left_pts;
  std::vector<Vec2d> right_pts;
  left_pts.reserve(lane->left_boundary->pts.size());
  right_pts.reserve(lane->right_boundary->pts.size());
  for (const auto& pt : lane->left_boundary->pts) {
    left_pts.emplace_back(Vec2d(pt.pt.x(), pt.pt.y()));
  }
  for (const auto& pt : lane->right_boundary->pts) {
    right_pts.emplace_back(Vec2d(pt.pt.x(), pt.pt.y()));
  }
  std::vector<Vec2d> cent_pts;
  math::GenerateCenterPoint(left_pts, right_pts, &cent_pts);
  for (const auto& pt : cent_pts) {
    Point center_p(RAW, static_cast<float>(pt.x()), static_cast<float>(pt.y()),
                   0);
    lane->center_line_pts.emplace_back(center_p);
  }
  if (lane->center_line_pts.size() > 3) {
    lane->center_line_param = math::FitLaneline(lane->center_line_pts);
    lane->center_line_param_front =
        math::FitLanelinefront(lane->center_line_pts);
  }
}

void RoadConstruct::RemoveNullGroup(std::vector<Group::Ptr>* groups) {
  for (size_t grp_idx = 0; grp_idx < static_cast<int>(groups->size());
       ++grp_idx) {
    auto& curr_group = groups->at(grp_idx);
    HLOG_DEBUG << "REMOVE1 curr_group->lanes.size()"
               << curr_group->lanes.size();
    if (curr_group->lanes.size() < 1) {
      groups->erase(groups->begin() + static_cast<int>(grp_idx));
    }
  }
}

void RoadConstruct::SmoothCenterline(std::vector<Group::Ptr>* groups) {
  if (groups->empty()) {
    return;
  }
  std::unordered_map<std::string, int>
      lane_grp_index;  // lane所在对应group的index
  for (auto& grp : *groups) {
    for (int index = 0; index < static_cast<int>(grp->lanes.size()); ++index) {
      lane_grp_index[grp->lanes[index]->str_id_with_group] = index;
    }
  }
  // forbiden lane->next_lane_str_id_with_group and
  // lane->prev_lane_str_id_with_group don't exist grouplane
  for (auto& grp : *groups) {
    for (auto lane : grp->lanes) {
      for (int index =
               static_cast<int>(lane->next_lane_str_id_with_group.size()) - 1;
           index >= 0; --index) {
        if (lane_grp_index.find(lane->next_lane_str_id_with_group[index]) ==
            lane_grp_index.end()) {
          lane->next_lane_str_id_with_group.erase(
              lane->next_lane_str_id_with_group.begin() + index);
        }
      }
      for (int index =
               static_cast<int>(lane->prev_lane_str_id_with_group.size()) - 1;
           index >= 0; --index) {
        if (lane_grp_index.find(lane->prev_lane_str_id_with_group[index]) ==
            lane_grp_index.end()) {
          lane->prev_lane_str_id_with_group.erase(
              lane->prev_lane_str_id_with_group.begin() + index);
        }
      }
    }
  }
  for (int i = 0; i < static_cast<int>(groups->size()) - 1; ++i) {
    auto& curr_grp = groups->at(i);
    auto& next_grp = groups->at(i + 1);
    // 过路口不平滑
    if (curr_grp->str_id.find("V") < curr_grp->str_id.length()) {
      continue;
    }
    for (auto lane : curr_grp->lanes) {
      if (lane->is_smooth || lane->next_lane_str_id_with_group.empty()) {
        continue;
      }
      // 前后继直接相连不平滑
      if (lane->next_lane_str_id_with_group.size() == 1) {
        int index_next_ = lane_grp_index[lane->next_lane_str_id_with_group[0]];
        if (next_grp->lanes[index_next_]->prev_lane_str_id_with_group.size() ==
            1) {
          continue;
        }
      }
      if (lane->next_lane_str_id_with_group.size() == 1) {
        // 从后往前滑动窗口
        //! TD:后续用距离不用点
        //! TBD:每个点计算曲率，抑制最大曲率
        int lane_index = lane_grp_index[lane->next_lane_str_id_with_group[0]];
        std::vector<Point> centerpt;
        auto& cur_centerline = lane->center_line_pts;
        int crr_size = static_cast<int>(cur_centerline.size());
        if (crr_size >= 10) {
          centerpt.insert(centerpt.begin(), cur_centerline.end() - 10,
                          cur_centerline.end());
        } else {
          centerpt.insert(centerpt.begin(), cur_centerline.begin(),
                          cur_centerline.end());
          int ctpt_size = static_cast<int>(centerpt.size());
          if (lane->prev_lane_str_id_with_group.size() == 1) {
            int pre_index =
                lane_grp_index[lane->prev_lane_str_id_with_group[0]];
            auto& prev_centerline =
                groups->at(i - 1)->lanes[pre_index]->center_line_pts;
            if (static_cast<int>(prev_centerline.size()) >= 10 - ctpt_size) {
              centerpt.insert(centerpt.begin(),
                              prev_centerline.end() - (10 - ctpt_size),
                              prev_centerline.end());
            } else {
              centerpt.insert(centerpt.begin(), prev_centerline.begin(),
                              prev_centerline.end());
            }
          }
        }
        std::vector<Point> res = SigmoidFunc(centerpt, 5.0);
        int res_size = static_cast<int>(res.size());
        if (crr_size >= res_size) {
          std::copy(res.begin(), res.end(),
                    lane->center_line_pts.end() - res_size);
          if ((lane->center_line_pts.back().pt -
               next_grp->lanes[lane_index]->center_line_pts[0].pt)
                  .norm() > 0.1) {
            lane->center_line_pts.emplace_back(
                next_grp->lanes[lane_index]->center_line_pts[0]);
          }
        } else {
          std::copy(res.end() - crr_size, res.end(),
                    lane->center_line_pts.begin());
          int pre_index = lane_grp_index[lane->prev_lane_str_id_with_group[0]];
          auto& prev_centerline =
              groups->at(i - 1)->lanes[pre_index]->center_line_pts;
          std::copy(res.begin(), res.end() - crr_size,
                    prev_centerline.end() - (res_size - crr_size));
          if ((lane->center_line_pts.back().pt -
               next_grp->lanes[lane_index]->center_line_pts[0].pt)
                  .norm() > 0.1) {
            lane->center_line_pts.emplace_back(
                next_grp->lanes[lane_index]->center_line_pts[0]);
          }
          if ((prev_centerline.back().pt - lane->center_line_pts[0].pt).norm() >
              0.1) {
            prev_centerline.emplace_back(lane->center_line_pts[0]);
          }
        }
      } else {
        // 从前往后滑动
        // 一根车道最多两根后继
        std::vector<Point> next_centerpt;
        int lane_index_next_f =
            lane_grp_index[lane->next_lane_str_id_with_group[0]];
        auto& next_first_lane = next_grp->lanes[lane_index_next_f];
        auto& lane_next_f_centerline =
            next_grp->lanes[lane_index_next_f]->center_line_pts;
        int next_f_ctl_size = static_cast<int>(lane_next_f_centerline.size());
        if (next_f_ctl_size >= 10) {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_f_centerline.begin(),
                               lane_next_f_centerline.begin() + 10);
        } else {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_f_centerline.begin(),
                               lane_next_f_centerline.end());
          int ctpt_size = static_cast<int>(next_centerpt.size());

          if (next_first_lane->next_lane_str_id_with_group.size() == 1) {
            int next_next_index =
                lane_grp_index[next_first_lane->next_lane_str_id_with_group[0]];
            auto& next_next_ctline =
                groups->at(i + 2)->lanes[next_next_index]->center_line_pts;
            if (static_cast<int>(next_next_ctline.size()) >= 10 - ctpt_size) {
              next_centerpt.insert(next_centerpt.end(),
                                   next_next_ctline.begin(),
                                   next_next_ctline.begin() + (10 - ctpt_size));

            } else {
              next_centerpt.insert(next_centerpt.end(),
                                   next_next_ctline.begin(),
                                   next_next_ctline.end());
            }
          }
        }
        std::vector<Point> first_ctl = SigmoidFunc(next_centerpt, 5.0);
        int first_ctl_size = static_cast<int>(first_ctl.size());
        if (next_f_ctl_size >= first_ctl_size) {
          std::copy(first_ctl.begin(), first_ctl.end(),
                    lane_next_f_centerline.begin());
          if ((lane_next_f_centerline[0].pt - lane->center_line_pts.back().pt)
                  .norm() > 0.1) {
            lane_next_f_centerline.insert(lane_next_f_centerline.begin(),
                                          lane->center_line_pts.back());
          }
        } else {
          std::copy(first_ctl.begin(), first_ctl.begin() + next_f_ctl_size,
                    lane_next_f_centerline.begin());
          int next_next_index =
              lane_grp_index[next_first_lane->next_lane_str_id_with_group[0]];
          auto& next_next_ctline =
              groups->at(i + 2)->lanes[next_next_index]->center_line_pts;
          std::copy(first_ctl.begin() + next_f_ctl_size, first_ctl.end(),
                    next_next_ctline.begin());
          if ((lane_next_f_centerline[0].pt - lane->center_line_pts.back().pt)
                  .norm() > 0.1) {
            lane_next_f_centerline.insert(lane_next_f_centerline.begin(),
                                          lane->center_line_pts.back());
          }
          if ((lane_next_f_centerline.back().pt - next_next_ctline[0].pt)
                  .norm() > 0.1) {
            next_next_ctline.insert(next_next_ctline.begin(),
                                    lane_next_f_centerline.back());
          }
        }

        next_centerpt.clear();
        int lane_index_next_s =
            lane_grp_index[lane->next_lane_str_id_with_group[1]];
        auto& next_second_lane = next_grp->lanes[lane_index_next_s];
        auto& lane_next_s_centerline =
            next_grp->lanes[lane_index_next_s]->center_line_pts;
        int next_s_ctl_size = static_cast<int>(lane_next_s_centerline.size());
        if (next_s_ctl_size >= 10) {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_s_centerline.begin(),
                               lane_next_s_centerline.begin() + 10);
        } else {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_s_centerline.begin(),
                               lane_next_s_centerline.end());
          int ctpt_size = static_cast<int>(next_centerpt.size());
          if (next_second_lane->next_lane_str_id_with_group.size() == 1) {
            int next_next_index =
                lane_grp_index[next_second_lane
                                   ->next_lane_str_id_with_group[0]];
            auto& next_next_ctline =
                groups->at(i + 2)->lanes[next_next_index]->center_line_pts;
            if (static_cast<int>(next_next_ctline.size()) >= 10 - ctpt_size) {
              next_centerpt.insert(next_centerpt.end(),
                                   next_next_ctline.begin(),
                                   next_next_ctline.begin() + (10 - ctpt_size));
            } else {
              next_centerpt.insert(next_centerpt.end(),
                                   next_next_ctline.begin(),
                                   next_next_ctline.end());
            }
          }
        }
        std::vector<Point> second_ctl = SigmoidFunc(next_centerpt, 5.0);
        int second_ctl_size = static_cast<int>(second_ctl.size());
        if (next_s_ctl_size >= second_ctl_size) {
          std::copy(second_ctl.begin(), second_ctl.end(),
                    lane_next_s_centerline.begin());
          if ((lane_next_s_centerline[0].pt - lane->center_line_pts.back().pt)
                  .norm() > 0.1) {
            lane_next_s_centerline.insert(lane_next_s_centerline.begin(),
                                          lane->center_line_pts.back());
          }
        } else {
          std::copy(second_ctl.begin(), second_ctl.begin() + next_s_ctl_size,
                    lane_next_s_centerline.begin());
          int next_next_index =
              lane_grp_index[next_second_lane->next_lane_str_id_with_group[0]];
          auto& next_next_line =
              groups->at(i + 2)->lanes[next_next_index]->center_line_pts;
          std::copy(second_ctl.begin() + next_s_ctl_size, second_ctl.end(),
                    next_next_line.begin());
          if ((lane_next_s_centerline[0].pt - lane->center_line_pts.back().pt)
                  .norm() > 0.1) {
            lane_next_s_centerline.insert(lane_next_s_centerline.begin(),
                                          lane->center_line_pts.back());
          }
          if ((next_next_line[0].pt - lane_next_s_centerline.back().pt).norm() >
              0.1) {
            next_next_line.insert(next_next_line.begin(),
                                  lane_next_s_centerline.back());
          }
        }
      }
    }
  }
}

std::vector<Point> RoadConstruct::SigmoidFunc(
    const std::vector<Point>& centerline, float sigma) {
  int size_ct = static_cast<int>(centerline.size());
  // 中心点数目少于两个点不拟合
  if (centerline.size() < 2) {
    return centerline;
  }
  float dis_x = centerline.back().pt.x() - centerline.front().pt.x();
  float dis_y = centerline.back().pt.y() - centerline.front().pt.y();

  // 距离过小不拟合
  if (dis_x < 0.1) {
    return centerline;
  }
  std::vector<Point> center;
  center.emplace_back(centerline[0]);

  for (int i = 1; i < static_cast<int>(centerline.size()) - 1; ++i) {
    Point center_p(VIRTUAL, 0.0, 0.0, 0.0);
    if (center.back().pt.x() < centerline[i + 1].pt.x()) {
      center_p.pt.x() = centerline[i + 1].pt.x();
      float deta_x = centerline[i + 1].pt.x() - center[0].pt.x();
      // 1/(1+e^(-x)) sigmoid函数
      // 把x的范围圈定在[-sigma,sigma]范围内，deta_x的范围是(0,dis_x]
      // y的范围是(centerline[0].pt.y(),centerline[0].y()+dis_y)
      center_p.pt.y() =
          centerline[0].pt.y() +
          dis_y / (1 + exp(-(2 * deta_x - dis_x) / dis_x * sigma));
    } else {
      float x_to_end = centerline.back().pt.x() - center.back().pt.x();
      float interval = x_to_end / static_cast<float>(centerline.size() - i - 1);
      center_p.pt.x() = center.back().pt.x() + interval;
      float deta_x = center_p.pt.x() - center[0].pt.x();
      center_p.pt.y() =
          centerline[0].pt.y() +
          dis_y / (1 + exp(-(2 * deta_x - dis_x) / dis_x * sigma));
    }
    center.emplace_back(center_p);
  }
  center.emplace_back(centerline.back());
  return center;
}

float RoadConstruct::Dist2Path(const Eigen::Vector3f& point) {
  if (path_in_curr_pose_.size() < 10) {
    return point.y();
  }
  auto tar_path = path_in_curr_pose_.begin()->pos;
  auto tar_point_next = path_in_curr_pose_.rbegin()->pos;

  Eigen::Vector2f A(point.x(), point.y());
  Eigen::Vector2f B(tar_path.x(), tar_path.y());
  Eigen::Vector2f C(tar_point_next.x(), tar_point_next.y());
  // 以B为起点计算向量BA 在向量BC上的投影
  Eigen::Vector2f BC = C - B;
  Eigen::Vector2f BA = A - B;

  if (abs(BC.norm()) < 0.0001) {
    return abs(BA.y());
    HLOG_DEBUG << "abs(BA.y()):" << abs(BA.y());
  }

  float dist_proj = BA.dot(BC) / BC.squaredNorm();

  Eigen::Vector2f proj_p = B + dist_proj * BC;
  float dist = A.y() > 0 ? (A - proj_p).norm() : -(A - proj_p).norm();
  return dist;
}

void RoadConstruct::BuildOccGroups(const ElementMap::Ptr& ele_map,
                                   std::vector<Group::Ptr>* groups) {
  FitUnusedOccRoads(ele_map);
  SearchOccCutPoints();
  // for (auto p : unused_occ_ctps_) {
  //   HLOG_INFO << "unused_occ_ctps_: id: " << p.GetId()
  //             << " x: " << p.GetPoint().x << " y: " << p.GetPoint().y
  //             << " ex start: " << p.GetExtrePoint().front().x << " "
  //             << p.GetExtrePoint().front().y
  //             << " ex end: " << p.GetExtrePoint().back().x << " "
  //             << p.GetExtrePoint().back().y;
  // }
  std::vector<Group::Ptr> occ_groups;
  CreatGroupsFromCutPoints(unused_occ_ctps_, &occ_groups);
  for (auto& grp : occ_groups) {
    SplitOccsToGroup(unused_occ_roads_, grp);
    if (static_cast<int>(grp->occ_segments.size()) > 1) {
      groups->emplace_back(grp);
    }
  }
}

void RoadConstruct::FitUnusedOccRoads(const ElementMap::Ptr& ele_map) {
  HLOG_DEBUG << "FitUnusedOccRoads";
  auto& occr = ele_map->occ_roads;
  for (auto& occ : occr) {
    auto& occ_line = occ.second;
    if (static_cast<int>(occ_line->road_points.size()) >= 10) {
      unused_occ_roads_.emplace(occ.first, occ_line);
    }
  }

  double good_k = 0;
  double good_b = 0;
  double good_y = 0;
  double max_r_squared = DBL_MIN;
  int good_id = 0;

  // 选择拟合度最好的线
  CheckBestOccRoad(good_k, good_b, good_y, max_r_squared, good_id);
  HLOG_DEBUG << "good_id: " << good_id << " good_k: " << good_k
             << " good_b: " << good_b << " max_r_squared: " << max_r_squared;

  // 生成新的点
  GenNewOccRoads(good_k, good_b, good_y, good_id);
  HLOG_DEBUG << "unused_occ_road_fitlines_.size():"
             << unused_occ_road_fitlines_.size();
}

void RoadConstruct::CheckBestOccRoad(double& good_k, double& good_b,
                                     double& good_y, double& max_r_squared,
                                     int& good_id) {
  for (const auto& occr : unused_occ_roads_) {
    double k = 0;
    double b = 0;
    double r_squared = 0;
    int curr_size = static_cast<int>(occr.second->road_points.size());
    if (curr_size < 10) {
      continue;
    }
    FitLineTLS(occr.second->road_points, k, b, r_squared);
    if (r_squared > max_r_squared) {
      max_r_squared = r_squared;
      good_k = k;
      good_b = b;
      good_id = occr.first;
      good_y = occr.second->road_points[static_cast<int>(curr_size / 2)].y();
    }
  }
}

void RoadConstruct::FitLineTLS(const std::vector<Eigen::Vector3d>& points,
                               double& k, double& b, double& r_squared) {
  if (static_cast<int>(points.size()) < 2) {
    HLOG_DEBUG << "fit occ points.size() < 2";
    return;
  }

  // 1. 计算均值
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto& point : points) {
    mean.x() += point.x();
    mean.y() += point.y();
  }
  mean /= static_cast<double>(points.size());

  // 2. 计算方差
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto& point : points) {
    Eigen::Vector2d centered = Eigen::Vector2d::Zero();
    centered.x() = point.x() - mean.x();
    centered.y() = point.y() - mean.y();
    covariance += centered * centered.transpose();
  }

  // 3. 特征值分解
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  Eigen::Vector2d direction =
      solver.eigenvectors()
          .col(1)
          .normalized();  // 拿到最小特征值对应的特征向量

  // 4. 计算直线的斜率 k 和截距 b
  k = direction.y() / direction.x();  // 方向向量的斜率
  b = mean.y() - k * mean.x();        // 通过均值点计算截距

  // 5. 计算拟合度（R²）和残差平方和（RSS）
  double total_sum_squares = 0.0;     // 总平方和
  double residual_sum_squares = 0.0;  // 残差平方和
  double mean_y = mean.y();

  for (const auto& point : points) {
    double y_actual = point.y();
    double y_predicted = k * point.x() + b;
    total_sum_squares += (y_actual - mean_y) * (y_actual - mean_y);
    residual_sum_squares += (y_actual - y_predicted) * (y_actual - y_predicted);
  }

  r_squared = 1.0 - (residual_sum_squares / total_sum_squares);  // R² 公式
}

void RoadConstruct::GenNewOccRoads(double& good_k, double& good_b,
                                   double& good_y, int& good_id) {
  std::vector<Eigen::Vector3d> new_points;
  for (const auto& occr : unused_occ_roads_) {
    if (occr.first == good_id) {
      HLOG_DEBUG << "occr.first == good_id";
      for (const auto& point : occr.second->road_points) {
        double yy = good_k * point.x() + good_b;
        new_points.emplace_back(point.x(), yy, point.z());
        // HLOG_INFO << "key_line: point.x(): " << point.x() << " yy: " << yy;
      }
    } else {
      HLOG_DEBUG << "occr.first != good_id";
      int psize = static_cast<int>(occr.second->road_points.size());
      Eigen::Vector3d key_p = Eigen::Vector3d::Zero();
      // HLOG_INFO << "occr.second->road_points[psize / 2].y()  "
      //           << occr.second->road_points[psize / 2].y() << " good_y  "
      //           << good_y;
      key_p = *std::min_element(
          occr.second->road_points.begin(), occr.second->road_points.end(),
          [this](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            Eigen::Vector3f af = a.cast<float>();
            Eigen::Vector3f bf = b.cast<float>();
            float a_dist = std::fabs(this->Dist2Path(af));
            float b_dist = std::fabs(this->Dist2Path(bf));
            return a_dist < b_dist;
          });

      double b = key_p.y() - good_k * key_p.x();
      for (const auto& point : occr.second->road_points) {
        double yy = good_k * point.x() + b;
        new_points.emplace_back(point.x(), yy, point.z());
        // HLOG_INFO << "key_line: point.x(): " << point.x() << " yy: " << yy;
      }
    }

    // HLOG_DEBUG << "new_points.size():" << new_points.size();
    Line::Ptr occ_line = std::make_shared<Line>();
    for (const auto& point : new_points) {
      occ_line->pts.emplace_back(RAW, static_cast<float>(point.x()),
                                 static_cast<float>(point.y()),
                                 static_cast<float>(point.z()));
    }
    occ_line->id = occr.first;
    unused_occ_road_fitlines_.emplace_back(occ_line);
    new_points.clear();
  }
}

void RoadConstruct::SearchOccCutPoints() {
  int idx = 1000;
  for (const auto& occ : unused_occ_road_fitlines_) {
    Eigen::Vector3f dir(0.0, 0.0, 0.0);
    dir = (occ->pts.back().pt - occ->pts.front().pt);
    Eigen::Vector3d dir_n = dir.cast<double>().normalized();
    Eigen::Matrix3d R =
        Eigen::AngleAxisd(90.0 * M_PI / 180.0, Eigen::Vector3d(0, 0, 1))
            .toRotationMatrix();
    Eigen::Vector3d extend_dir = (R * dir_n).normalized();

    Eigen::Vector3d p_f = Eigen::Vector3d::Zero();
    p_f = occ->pts.front().pt.cast<double>();

    CutPoint ctp_f(idx++, p_f, CutPointType::Broken_Start, occ->id, extend_dir);
    unused_occ_ctps_.push_back(ctp_f);

    Eigen::Vector3d p_b = Eigen::Vector3d::Zero();
    p_b = occ->pts.back().pt.cast<double>();

    CutPoint ctp_b(idx++, p_b, CutPointType::Broken, occ->id, extend_dir);
    unused_occ_ctps_.push_back(ctp_b);
  }
}

void RoadConstruct::RemoveInvalGroups(std::vector<Group::Ptr>* groups) {
  if (groups->empty()) {
    return;
  }
  int group_size = static_cast<int>(groups->size());
  for (int i = 0; i < group_size; i++) {
    if (CheckRoadInval(groups, i) || CheckMidGroupLaneInval(groups, i)) {
      groups->erase(groups->begin() + i);
      group_size--;
      continue;
    }
  }
}

bool RoadConstruct::CheckRoadInval(std::vector<Group::Ptr>* groups,
                                   const int& i) {
  auto grp = groups->at(i);
  if (static_cast<int>(grp->road_edges.size()) < 2) {
    groups->erase(groups->begin() + i);
    return true;
  }

  bool have_left_edge = false;
  bool have_right_edge = false;
  RoadEdge::Ptr left_edge = std::make_shared<RoadEdge>();
  RoadEdge::Ptr right_edge = std::make_shared<RoadEdge>();
  for (auto& road_edge : grp->road_edges) {
    if (road_edge->is_left) {
      have_left_edge = true;
      left_edge = road_edge;
    } else if (road_edge->is_right) {
      have_right_edge = true;
      right_edge = road_edge;
    }
  }
  if (!have_left_edge || !have_right_edge) {
    groups->erase(groups->begin() + i);
    return true;
  } else if (std::fabs(left_edge->dist_to_path - right_edge->dist_to_path) <
             3.0) {
    groups->erase(groups->begin() + i);
    return true;
  }
  return false;
}

bool RoadConstruct::CheckMidGroupLaneInval(std::vector<Group::Ptr>* groups,
                                           const int& i) {
  auto grp = groups->at(i);

  if (0 < i && i < (static_cast<int>(groups->size()) - 1)) {
    if (!groups->at(i - 1)->line_segments.empty() &&
        !groups->at(i + 1)->line_segments.empty() && grp->lanes.empty()) {
      return true;
    }
  }
  return false;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
