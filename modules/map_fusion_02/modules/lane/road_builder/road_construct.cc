/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_construct.cc
 *   author     ： cuijiayu
 *   date       ： 2024.08
 ******************************************************************************/
#include "modules/map_fusion_02/modules/lane/road_builder/road_construct.h"

namespace hozon {
namespace mp {
namespace mf {

void RoadConstruct::Init(const LaneFusionProcessOption& conf) {
  conf_ = conf;
  HLOG_INFO << "Road construct init";
}

bool RoadConstruct::ConstructLane(
    const std::vector<CutPoint>& cutpoints, std::deque<Line::Ptr> lines,
    const std::shared_ptr<std::vector<KinePosePtr>>& path,
    const KinePosePtr& curr_pose, const ElementMap::Ptr& ele_map) {
  BuildKDtrees(&lines);
  UpdatePathInCurrPose(*path, *curr_pose);
  BuildGroupSegments(cutpoints, &lines, &group_segments_, ele_map);

  BuildGroups(ele_map, group_segments_, &groups_);
  return true;
}

// 获取内部Group
void RoadConstruct::GetGroups(std::vector<Group::Ptr>* groups) {
  if (groups == nullptr) {
    return;
  }
  groups->reserve(groups_.size());
  for (const auto& grp : groups_) {
    groups->emplace_back(grp);
  }
}

void RoadConstruct::GetDistPoints(std::vector<Eigen::Vector3f>* distpoints) {
  if (distpoints == nullptr) {
    return;
  }
  distpoints->reserve(distpoits_.size());
  for (const auto& distp : distpoits_) {
    distpoints->emplace_back(distp);
  }
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
    if (T_v2_v1.translation().x() < 0) {
      Pose pose;
      pose.stamp = p->stamp;
      pose.pos << T_v2_v1.translation().x(), T_v2_v1.translation().y(),
          T_v2_v1.translation().z();
      pose.quat = T_v2_v1.rotation();
      path_in_curr_pose_.emplace_back(pose);
    }
  }
  // 自车原点也加进来
  Pose pose;
  pose.stamp = curr_pose.stamp;
  pose.pos.setZero();
  pose.quat.setIdentity();
  path_in_curr_pose_.emplace_back(pose);
}

void RoadConstruct::BuildGroupSegments(
    const std::vector<CutPoint>& cutpoints, std::deque<Line::Ptr>* lines,
    std::vector<GroupSegment::Ptr>* group_segments,
    const ElementMap::Ptr& ele_map) {
  HLOG_DEBUG << "BuildGroupSegments ctp";

  CreateGroupSegFromCutPoints(cutpoints, group_segments);
  SplitPtsToGroupSeg(lines, group_segments);
  GenLaneSegInGroupSeg(group_segments);
  EgoLineTrajectory(group_segments, ele_map);
}

void RoadConstruct::CreateGroupSegFromCutPoints(
    const std::vector<CutPoint>& cutpoints,
    std::vector<GroupSegment::Ptr>* segments) {
  if (segments == nullptr) {
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
    HLOG_DEBUG << "ctp: " << p.x << p.y << p.z;
    std::vector<Point3D> extre_point = ctp.GetExtrePoint();
    Eigen::Vector3f po_veh(p.x, p.y, p.z);
    Eigen::Vector3f pr_veh(extre_point[0].x, extre_point[0].y, 0);
    Eigen::Vector3f pl_veh(extre_point[1].x, extre_point[1].y, 0);

    SliceLine slice;
    slice.po = po_veh;
    slice.pl = pl_veh;
    slice.pr = pr_veh;
    slice_lines.emplace_back(slice);
  }

  if (slice_lines.size() < 2) {
    HLOG_ERROR << "no enough slice lines: " << slice_lines.size();
    return;
  }

  // 从SliceLines里创建出GroupSegments
  const auto slice_num = slice_lines.size();
  segments->clear();
  segments->reserve(slice_num);
  for (size_t i = 0; i < slice_num - 1; ++i) {
    auto seg = std::make_shared<GroupSegment>();
    seg->start_slice = slice_lines.at(i);
    seg->end_slice = slice_lines.at(i + 1);
    seg->str_id.clear();
    segments->emplace_back(seg);
  }
  HLOG_DEBUG << "segments size: " << segments->size();
}

// 将所有车道线点切分到所属GroupSegment：对每根线的点从front到back，判断是否在GroupSegment
// 范围内
void RoadConstruct::SplitPtsToGroupSeg(
    std::deque<Line::Ptr>* lines, std::vector<GroupSegment::Ptr>* segments) {
  // 将线分割并填充到GroupSegment
  for (auto& seg : *segments) {
    // 生成LineSegments
    const auto& start_slice = seg->start_slice;
    Eigen::Vector2f start_po = start_slice.po.head<2>();
    Eigen::Vector2f start_pl = start_slice.pl.head<2>();
    const auto& end_slice = seg->end_slice;
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
          break;
        }
        // 第一个点在end_slice的左边，并且在start_slice的右边，说明属于此segment，加进来
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
      int size_line_seg = line_seg->pts.size();
      Eigen::Vector3f point_tmp(0.0, 0.0, 0.0);
      for (int i = 0; i < size_line_seg; ++i) {
        point_tmp += line_seg->pts[i].pt;
      }
      line_seg->center = point_tmp / size_line_seg;
      auto dist =
          PointToVectorDist(start_slice.po, end_slice.po, line_seg->center);
      Eigen::Vector2f center_xy = line_seg->center.head<2>();
      // 在path右边，距离设为负值
      if (PointInVectorSide(start_po, end_po, center_xy) > 0) {
        dist = -1 * dist;
      }
      line_seg->dist_to_path = dist;
      seg->line_segments.emplace_back(line_seg);
    }
  }
}

// 从GroupSegment包含的所有线中，聚合出一个个小的LaneSegment
void RoadConstruct::GenLaneSegInGroupSeg(
    std::vector<GroupSegment::Ptr>* segments) {
  std::string last_seg_id;
  for (auto& seg : *segments) {
    // 生成LaneSegments
    const auto line_seg_num = seg->line_segments.size();
    if (line_seg_num > 1) {
      // 按与path的距离从大到小排序，得到的结果就是所有线都是从左到右排序号
      std::sort(seg->line_segments.begin(), seg->line_segments.end(),
                [](const LineSegment::Ptr& a, const LineSegment::Ptr& b) {
                  return a->dist_to_path > b->dist_to_path;
                });
      // 按边线距离生成LaneSegment
      for (size_t i = 0; i < line_seg_num - 1; ++i) {
        auto& left_line = seg->line_segments.at(i);
        auto& right_line = seg->line_segments.at(i + 1);
        auto& left_center = left_line->center;
        int l_size = left_line->pts.size();
        if (l_size == 0) {
          continue;
        }
        auto& left_distp = left_line->pts[static_cast<int>(l_size * 0.5)].pt;
        auto right_center = right_line->center;
        //! 商汤切分点情况下，由于center点是用首尾两点求中值，如果在弯道，该点会在车道外，导致计算距离错误
        auto dist = DistByKDtree(left_distp, *right_line);
        if (dist < conf_.min_lane_width) {
          continue;
        }

        // if (dist < conf_.min_lane_width || dist > conf_.max_lane_width) {
        //   continue;
        // }
        if (dist > conf_.max_lane_width) {
          continue;
        }
        auto lane_seg = std::make_shared<LaneSegment>();
        lane_seg->left_boundary = left_line;
        lane_seg->right_boundary = right_line;
        // 将左右边线id连起来，作为LaneSegment的str_id，
        // 这里使用边线id组成字符串id来标识lane，可以方便后续比对lane与lane是否共线
        lane_seg->str_id = std::to_string(left_line->id) + "_" +
                           std::to_string(right_line->id);
        lane_seg->lanepos_id = std::to_string(left_line->lanepos) + "_" +
                               std::to_string(right_line->lanepos);
        seg->lane_segments.emplace_back(lane_seg);
      }
    }

    // 将所有LaneSegment的str_id连起来，作为GroupSegment的str_id，
    // 这里GroupSegment的字符串id包含了所有内部lane
    // id信息，可以方便后续比对GroupSegment间 是否包含同样的lane
    for (const auto& lane_seg : seg->lane_segments) {
      if (!seg->str_id.empty()) {
        seg->str_id += "|";
      }
      seg->str_id += lane_seg->str_id;
    }
    last_seg_id = seg->str_id;
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
  float point_dist = sqrt(pow(BA.norm(), 2) - pow(dist_proj, 2));
  return point_dist;
}

void RoadConstruct::EgoLineTrajectory(
    std::vector<GroupSegment::Ptr>* grp_segment,
    const ElementMap::Ptr& ele_map) {
  int line1_id = -200, line2_id = -200, near_line = -200;
  for (auto& seg : *grp_segment) {
    int flag = 0;
    if (seg->end_slice.po.x() > -5 && seg->start_slice.po.x() < 5) {
      if (seg->line_segments.size() > 1) {
        for (int i = 0; i < static_cast<int>(seg->line_segments.size()) - 1;
             ++i) {
          if ((seg->line_segments[i]->dist_to_path *
                   seg->line_segments[i + 1]->dist_to_path <
               0.1) &&
              fabs(seg->line_segments[i]->dist_to_path < 3.75) &&
              fabs(seg->line_segments[i + 1]->dist_to_path < 3.75)) {
            line1_id = seg->line_segments[i]->id;
            line2_id = seg->line_segments[i + 1]->id;
            near_line = (abs(seg->line_segments[i]->dist_to_path) <
                         abs(seg->line_segments[i + 1]->dist_to_path))
                            ? line1_id
                            : line2_id;
            flag = 1;
            break;
          }
        }
      }
    }
    if (flag) {
      break;
    }
  }
  if (line1_id == -200 && line2_id == -200) {
    ego_line_exist_ = false;
  } else {
    ego_line_exist_ = true;
    FitLaneline(ele_map, line1_id, line2_id, near_line);
  }
  ego_line_id_.left_id = line1_id;
  ego_line_id_.right_id = line2_id;
  HLOG_DEBUG << "ego line id:" << ego_line_id_.left_id << ","
             << ego_line_id_.right_id;
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

// 将所有GroupSegments聚合成一个个Group
void RoadConstruct::BuildGroups(
    const ElementMap::Ptr& ele_map,
    const std::vector<GroupSegment::Ptr>& group_segments,
    std::vector<Group::Ptr>* groups) {
  HLOG_DEBUG << "BuildGroups";
  if (group_segments.empty() || groups == nullptr) {
    return;
  }

  double stamp = ele_map->map_info.stamp;
  UniteGroupSegmentsToGroups(stamp, group_segments, groups);

  const auto& occ_roads = ele_map->occ_roads;
  GenLanesInGroups(groups, occ_roads, stamp);
}

// 将若干个GroupSegment聚合成Group：
// 当前采用的策略是：相邻两个GroupSegment内包含的LaneSegment一样，那就可以聚合到一起；
// 即每个Group里所有GroupSegment包含的LaneSegment数目都相同，并且LaneSegment的
// str_id（由边线id组成）也相同.
void RoadConstruct::UniteGroupSegmentsToGroups(
    double stamp, const std::vector<GroupSegment::Ptr>& group_segments,
    std::vector<Group::Ptr>* groups) {
  // HLOG_ERROR << "group_segments.size() = " << group_segments.size();

  int grp_idx = -1;
  for (const auto& seg : group_segments) {
    if (seg->str_id == "") {
      continue;
    }
    if (!groups->empty() && seg->str_id == groups->back()->seg_str_id) {
      groups->back()->group_segments.emplace_back(seg);
      continue;
    }

    grp_idx += 1;
    auto grp = std::make_shared<Group>();
    grp->group_segments.emplace_back(seg);
    grp->seg_str_id = seg->str_id;
    grp->str_id = std::string("G") + std::to_string(grp_idx) +
                  std::string("-") + grp->seg_str_id;
    grp->stamp = stamp;
    groups->emplace_back(grp);
  }
}

// 把Group里一个个GroupSegment中包含的小的LaneSegment，纵向上聚合成一个个大的Lane，
// 并且生成出：左右关联、前后关联、远端预测线、中心线
void RoadConstruct::GenLanesInGroups(std::vector<Group::Ptr>* groups,
                                     std::map<Id, OccRoad::Ptr> occ_roads,
                                     double stamp) {
  if (groups->size() < 1) {
    return;
  }
  HLOG_DEBUG << "GenLanesInGroups";
  for (auto& grp : *groups) {
    std::vector<Lane::Ptr> possible_lanes;
    possible_lanes.clear();
    // 收集可能的lane
    CollectGroupPossibleLanes(grp, &possible_lanes);
    // 过滤掉边线只有一个点的lane
    FilterGroupBadLane(possible_lanes, grp);

    MatchLRLane(grp);
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

  // 删除空的group数据
  int before_remove_grp_nums = groups->size();
  RemoveNullGroup(groups);
  int after_remove_grp_nums = groups->size();
  if (after_remove_grp_nums != before_remove_grp_nums) {
    HLOG_WARN << "*********[CrossWalk]*******delete null group nums: "
              << after_remove_grp_nums - before_remove_grp_nums;
  }

  HLOG_DEBUG << "current stamp is:" << std::to_string(stamp);

  SmoothCenterline(groups);
}

void RoadConstruct::CollectGroupPossibleLanes(
    const Group::Ptr& grp, std::vector<Lane::Ptr>* possible_lanes) {
  for (const auto& grp_seg : grp->group_segments) {
    if (possible_lanes->empty()) {
      for (const auto& lane_seg : grp_seg->lane_segments) {
        auto lane = std::make_shared<Lane>();
        lane->str_id = lane_seg->str_id;
        lane->lanepos_id = lane_seg->lanepos_id;
        lane->str_id_with_group = grp->str_id + ":" + lane->str_id;
        lane->left_boundary = std::make_shared<LineSegment>();
        lane->left_boundary->id = lane_seg->left_boundary->id;
        lane->left_boundary->lanepos = lane_seg->left_boundary->lanepos;
        lane->left_boundary->type = lane_seg->left_boundary->type;
        lane->left_boundary->color = lane_seg->left_boundary->color;
        lane->left_boundary->isego = lane_seg->left_boundary->isego;
        lane->left_boundary->is_near_road_edge =
            lane_seg->left_boundary->is_near_road_edge;
        for (const auto& delete_id : lane_seg->left_boundary->deteled_ids) {
          lane->left_boundary->deteled_ids.emplace_back(delete_id);
        }
        lane->left_boundary->mean_end_heading =
            lane_seg->left_boundary->mean_end_heading;
        lane->left_boundary->pred_end_heading =
            lane_seg->left_boundary->pred_end_heading;
        lane->left_boundary->mean_end_heading_std_dev =
            lane_seg->left_boundary->mean_end_heading_std_dev;
        lane->left_boundary->mean_end_interval =
            lane_seg->left_boundary->mean_end_interval;
        for (const auto& pt : lane_seg->left_boundary->pts) {
          lane->left_boundary->pts.emplace_back(pt);
        }
        lane->right_boundary = std::make_shared<LineSegment>();
        lane->right_boundary->id = lane_seg->right_boundary->id;
        lane->right_boundary->lanepos = lane_seg->right_boundary->lanepos;
        lane->right_boundary->type = lane_seg->right_boundary->type;
        lane->right_boundary->color = lane_seg->right_boundary->color;
        lane->right_boundary->isego = lane_seg->right_boundary->isego;
        lane->right_boundary->is_near_road_edge =
            lane_seg->right_boundary->is_near_road_edge;
        lane->right_boundary->mean_end_heading =
            lane_seg->right_boundary->mean_end_heading;
        lane->right_boundary->pred_end_heading =
            lane_seg->right_boundary->pred_end_heading;
        lane->right_boundary->mean_end_heading_std_dev =
            lane_seg->right_boundary->mean_end_heading_std_dev;
        lane->right_boundary->mean_end_interval =
            lane_seg->right_boundary->mean_end_interval;
        for (const auto& delete_id : lane_seg->right_boundary->deteled_ids) {
          lane->right_boundary->deteled_ids.emplace_back(delete_id);
        }
        for (const auto& pt : lane_seg->right_boundary->pts) {
          lane->right_boundary->pts.emplace_back(pt);
        }
        possible_lanes->emplace_back(lane);
      }
      continue;
    }

    const size_t grp_lane_num = possible_lanes->size();
    if (grp_lane_num != grp_seg->lane_segments.size()) {
      HLOG_ERROR << "something is wrong, group's lane num should be equal to "
                    "group segment's lane segment num";
      return;
    }

    for (size_t i = 0; i < grp_lane_num; ++i) {
      auto& exist_lane = possible_lanes->at(i);
      const auto& lane_seg = grp_seg->lane_segments.at(i);
      for (const auto& pt : lane_seg->left_boundary->pts) {
        exist_lane->left_boundary->pts.emplace_back(pt);
      }
      for (const auto& pt : lane_seg->right_boundary->pts) {
        exist_lane->right_boundary->pts.emplace_back(pt);
      }
    }
  }
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
    lane->center_line_param = FitLaneline(lane->center_line_pts);
    lane->center_line_param_front = FitLanelinefront(lane->center_line_pts);
  }
}

std::vector<double> RoadConstruct::FitLaneline(
    const std::vector<Point>& centerline) {
  int size_c = centerline.size();

  double k = 0.0, kk = 0.0;
  double y1 = centerline[size_c - 1].pt.y(), x1 = centerline[size_c - 1].pt.x();
  double y2 = centerline[size_c - 2].pt.y(), x2 = centerline[size_c - 2].pt.x();
  int idx = size_c - 2;
  while (sqrt(pow(y1 - y2, 2) + pow(x1 - x2, 2)) < 4 && idx > 0) {
    idx--;
    y2 = centerline[idx].pt.y();
    x2 = centerline[idx].pt.x();
  }
  if (idx < 0) {
    return {};
  } else {
    kk = (y2 - y1) / (x2 - x1);
    k = y1 - kk * x1;
  }
  return {k, kk};
}

std::vector<double> RoadConstruct::FitLanelinefront(
    const std::vector<Point>& centerline) {
  int size_c = centerline.size();

  double k = 0.0, kk = 0.0;
  double y1 = centerline[0].pt.y(), x1 = centerline[0].pt.x();
  double y2 = centerline[1].pt.y(), x2 = centerline[1].pt.x();
  int idx = 1;
  while (sqrt(pow(y1 - y2, 2) + pow(x1 - x2, 2)) < 4 && idx < size_c) {
    y2 = centerline[idx].pt.y();
    x2 = centerline[idx].pt.x();
    idx++;
  }
  if (idx >= size_c) {
    return {};
  } else {
    kk = (y2 - y1) / (x2 - x1);
    k = y1 - kk * x1;
  }
  return {k, kk};
}

void RoadConstruct::RemoveNullGroup(std::vector<Group::Ptr>* groups) {
  for (size_t grp_idx = 0; grp_idx < groups->size(); ++grp_idx) {
    auto& curr_group = groups->at(grp_idx);
    HLOG_DEBUG << "REMOVE1 curr_group->group_segments.size()"
               << curr_group->group_segments.size()
               << "REMOVE1 curr_group->lanes.size()"
               << curr_group->lanes.size();
    if (curr_group->group_segments.size() < 1 || curr_group->lanes.size() < 1) {
      HLOG_DEBUG << "REMOVE curr_group->group_segments.size()"
                 << curr_group->group_segments.size()
                 << "REMOVE curr_group->lanes.size()"
                 << curr_group->lanes.size();
      groups->erase(groups->begin() + grp_idx);
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
    for (int index = 0; index < grp->lanes.size(); ++index) {
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
        int crr_size = cur_centerline.size();
        if (crr_size >= 10) {
          centerpt.insert(centerpt.begin(), cur_centerline.end() - 10,
                          cur_centerline.end());
        } else {
          centerpt.insert(centerpt.begin(), cur_centerline.begin(),
                          cur_centerline.end());
          int ctpt_size = centerpt.size();
          if (lane->prev_lane_str_id_with_group.size() == 1) {
            int pre_index =
                lane_grp_index[lane->prev_lane_str_id_with_group[0]];
            auto& prev_centerline =
                groups->at(i - 1)->lanes[pre_index]->center_line_pts;
            if (prev_centerline.size() >= 10 - ctpt_size) {
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
        int res_size = res.size();
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
        int next_f_ctl_size = lane_next_f_centerline.size();
        if (next_f_ctl_size >= 10) {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_f_centerline.begin(),
                               lane_next_f_centerline.begin() + 10);
        } else {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_f_centerline.begin(),
                               lane_next_f_centerline.end());
          int ctpt_size = next_centerpt.size();

          if (next_first_lane->next_lane_str_id_with_group.size() == 1) {
            int next_next_index =
                lane_grp_index[next_first_lane->next_lane_str_id_with_group[0]];
            auto& next_next_ctline =
                groups->at(i + 2)->lanes[next_next_index]->center_line_pts;
            if (next_next_ctline.size() >= 10 - ctpt_size) {
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
        int first_ctl_size = first_ctl.size();
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
        int next_s_ctl_size = lane_next_s_centerline.size();
        if (next_s_ctl_size >= 10) {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_s_centerline.begin(),
                               lane_next_s_centerline.begin() + 10);
        } else {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_s_centerline.begin(),
                               lane_next_s_centerline.end());
          int ctpt_size = next_centerpt.size();
          if (next_second_lane->next_lane_str_id_with_group.size() == 1) {
            int next_next_index =
                lane_grp_index[next_second_lane
                                   ->next_lane_str_id_with_group[0]];
            auto& next_next_ctline =
                groups->at(i + 2)->lanes[next_next_index]->center_line_pts;
            if (next_next_ctline.size() >= 10 - ctpt_size) {
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
        int second_ctl_size = second_ctl.size();
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
  int size_ct = centerline.size();
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

}  // namespace mf
}  // namespace mp
}  // namespace hozon
