/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： group_map.cc
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#include "map_fusion/road_recognition/group_map.h"
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "base/utils/log.h"
#include "common/math/vec2d.h"
#include "map_fusion/fusion_common/calc_util.h"
#include "map_fusion/fusion_common/element_map.h"
// #include "util/common.h"
// #include "util/tic_toc.h"

namespace hozon {
namespace mp {
namespace mf {
namespace gm {

using ProtoBoundType = hozon::hdmap::LaneBoundaryType;
using hozon::common::math::double_type::Compare;

struct LaneWithNextLanes {
  Lane::Ptr lane = nullptr;
  std::vector<Lane::Ptr> next_lanes;
};

void GroupMap::Clear() {
  zebra_.clear();
  arrow_.clear();
  stopline_.clear();
  overlaps_.clear();
  groups_.clear();
  group_segments_.clear();
  ego_line_id_.left_id = -200;
  ego_line_id_.right_id = -200;
  history_best_lane_ = nullptr;
  ego_curr_lane_ = nullptr;
}

bool GroupMap::Build(const std::shared_ptr<std::vector<KinePose::Ptr>>& path,
                     const KinePose::Ptr& curr_pose,
                     const KinePose::Ptr& last_pose,
                     const em::ElementMap::Ptr& ele_map, IsCross* is_cross) {
  if (path == nullptr || curr_pose == nullptr || ele_map == nullptr) {
    HLOG_ERROR << "input nullptr";
    return false;
  }

  // - - - - - + - - - -              +-----
  // |   |   | . |                         |
  // - - - - - + - - - -                   |
  // |   |   | . |                         |
  // - - - - - + - - - -                   |
  // |   |   | . |                       Group
  // - - - - - + - - - -                   |
  //  \  |   | . |                         |
  // - - - - - + - - - -                   |
  //    \|   | . |                         |
  // - - - - - + - - - -              +-----
  //     |   | . |                         |
  // - - - - - + - - - -                   |
  //     |   | . |                       Group
  // - - - - - + - - - -                   |
  //     |   | . |                         |
  // - - - - - + - - - -              +-----
  //     |   | . |\                        |
  // - - - - - + - - - -                   |
  //     |   | ^ |  \                      |
  // - - - - - + - - - -                   |
  //     |   | . |   |                   Group
  // - - - - - + - - - -                   |
  //     |   | . |   |                     |
  // - - - - - + - - - -                   |
  //     |   | . |   |    GroupSegment     |
  // - - - - - + - - - -   SliceLine  +-----
  // 0.
  // 先对所有车道线的点进行等间距采样，间隔比path的点间距要小，比如path间距2m，这里车道线按1m间距采样；
  // 这里采样的目的是，防止原始车道线的点间距太大，导致下面的GroupSegment切分不到点；
  // 1. 生成所有GroupSegment：
  // 遍历path里每个pose，相邻两个pose组成一个GroupSegment；将每个pose的vehicle系下自车原点(0,
  // 0)、 左边点(0, 1)、右边点(0,
  // -1)根据当时的pose转到local系下；然后再根据curr_pose再转到当前 车体系下；
  // 2. 按顺序遍历每个GroupSegment，将所有线传递给每个GroupSegment用于构建：
  // 遍历每根线的每一个点，判断这个点是否在GroupSegment.start_slice的左侧，如果左侧就丢掉，继续下一个点；
  // 然后判断点是否在GroupSegment.end_slice的左侧或其上，如果是，说明此点在start_slice与end_slice之间，
  // 就把这个点加到LineSegment；
  // 遍历每个LineSegment，计算LineSegment的中点，然后计算中点到向量(start_slice.po,
  // end_slice.po)的距离dist_to_path，
  // 并且左边为正，右边为负，对所有LineSegment按dist_to_path从大到小重新排序，即将所有LineSegment从左到右排序了；
  // 遍历排序后的LineSegment，计算每相邻两个LineSegment.center的间距，如果超过一定阈值且小于一定阈值，那就认为可以组成
  // 一个LaneSegment，并把左右LineSegment分别填充到LaneSegment的左右边界；并且将左右LineSegment的id转为字符串并拼接，
  // 作为LaneSegment的str_id（比如"12_13"）；
  // 将GroupSegment里的所有LaneSegment的str_id拼接（比如"12_13|13_14|14_15"），作为GroupSegment的str_id；
  // 3. 聚合GroupSegment为Group：
  // 遍历所有GroupSegment，对于第一个GroupSegment，直接创建一个Group，并且将第一个GroupSegment直接加入到Group中，
  // 并且Group的str_id也赋值为这个GroupSegment的str_id；
  // 对于后面每个GroupSegment，如果str_id与最后一个Group的str_id相同，就把自己加入到最后一个Group；如果不相同就
  // 新创建一个Group，将自己加到这个Group中；直到遍历完所有GroupSegment，这样就得到所有Group；
  // 4. 遍历每个Group，把Group中GroupSegment聚合成多个Lane：
  // 遍历此Group中所有GroupSegment，对于第一个GroupSegment，其中每一个LaneSegment都创建出一个Lane，将LaneSegment的左右边界
  // 直接塞到Lane的左右边界，并且Lane的str_id直接使用LaneSegment的str_id，
  // str_id_with_group使用Group.str_id和LaneSegment.str_id进行拼接（比如"12_13|13_14|14_15:12_13"）；
  // 对于后面每个GroupSegment，按索引将每个LaneSegment的左右边界塞到对应Lane的左右边界；这样所有纵向的LaneSegment就聚合成单独的Lane了；
  // 聚合完Lane后，遍历所有Lane，填充每个Lane的left_lane_str_id_with_group和right_lane_str_id_with_group；
  // 对每个Lane，计算出中心线center_line_pts；
  // 5. 遍历每个Group，关联Lane的前后继：
  // 对相邻的两个Group，如果前面Group中的某个Lane的str_id在后面Group中存在，就分别设置它们的prev_lane_str_id_with_group
  // 和next_lane_str_id_with_group；
  // 6. TBD：分合流时的前后继，比如：
  // case 1:
  // 分流:  1->3, 1->4, 2->4, 2->5
  // 或合流: 3->1, 4->1, 4->2, 5->2
  // |   |   |   |
  // | 3 | 4 | 5 |
  // |   |   |   |
  // - - - - - - -
  //   |   |   |
  //   | 1 | 2 |
  //   |   |   |
  //
  // case 2:
  // 分流:  1->3, 1->4, 2->5
  // 或合流: 3->1, 4->1, 5->2
  // |   |   |   |
  // |   |   |   |
  //  \3 | 4 | 5 |
  //   \ |   |   |
  //     - - - - -
  //     |   |   |
  //     | 1 | 2 |
  //     |   |   |
  is_cross_ = *is_cross;
  HLOG_DEBUG << "IS_CROSS" << is_cross_.cross_after_lane_ << "  "
             << is_cross_.cross_before_lane_ << "  " << is_cross_.is_crossing_
             << " " << is_cross_.next_lane_left << " "
             << is_cross->next_lane_right;
  // 计算上一帧和这一帧pose的heading偏差
  if (last_pose != nullptr) {
    delta_pose_heading_ =
        math::CalculateHeading(last_pose->quat, curr_pose->quat);
  }
  curr_pose_ = curr_pose;
  std::deque<Line::Ptr> lines;
  // lane_line_interp_dist可以设为-1，当前上游点间隔已经是1m，这里不用插值出更细的点

  RetrieveBoundaries(ele_map, conf_.lane_line_interp_dist, &lines);

  UpdatePathInCurrPose(*path, *curr_pose);

  BuildGroupSegments(path, curr_pose, &lines, &group_segments_, ele_map);

  BuildGroups(ele_map, group_segments_, &groups_);

  is_cross->next_lane_left = is_cross_.next_lane_left;
  is_cross->next_lane_right = is_cross_.next_lane_right;
  is_cross->is_connect_ = is_cross_.is_connect_;
  is_cross->is_crossing_ = is_cross_.is_crossing_;
  is_cross->next_satisefy_lane_seg = is_cross_.next_satisefy_lane_seg;

  return true;
}

void GroupMap::SetCurrentRoadScene(const std::vector<Group::Ptr>* groups) {
  if (groups->size() == 0) {
    road_scene_ = RoadScene::BIG_JUNCTUIN;
    return;
  }

  int index = FindEgoGroup(groups);
  if (index >= 0 || index < groups->size()) {
    for (const auto& lane : groups->at(index)->lanes) {
      if (lane->left_boundary->type ==
              em::LaneType_INTERSECTION_VIRTUAL_MARKING ||
          lane->right_boundary->type ==
              em::LaneType_INTERSECTION_VIRTUAL_MARKING) {
        road_scene_ = RoadScene::SMALL_JUNCTION;
        return;
      }
    }
  }

  if (groups->size() == 1) {
    auto& last_group = groups->back();
    if (last_group->group_state == Group::GroupState::VIRTUAL ||
        last_group->group_segments.empty()) {
      road_scene_ = RoadScene::NON_JUNCTION;
      return;
    }
    Eigen::Vector2f next_start_pl(
        last_group->group_segments.front()->start_slice.pl.x(),
        last_group->group_segments.front()->start_slice.pl.y());
    Eigen::Vector2f next_start_pr(
        last_group->group_segments.front()->start_slice.pr.x(),
        last_group->group_segments.front()->start_slice.pr.y());

    Eigen::Vector2f next_start_po(
        last_group->group_segments.front()->start_slice.po.x(),
        last_group->group_segments.front()->start_slice.po.y());

    Eigen::Vector2f next_end_pl(
        last_group->group_segments.back()->start_slice.pl.x(),
        last_group->group_segments.back()->start_slice.pl.y());
    Eigen::Vector2f next_end_pr(
        last_group->group_segments.back()->start_slice.pr.x(),
        last_group->group_segments.back()->start_slice.pr.y());
    Eigen::Vector2f next_end_po(
        last_group->group_segments.back()->start_slice.po.x(),
        last_group->group_segments.back()->start_slice.po.y());

    Eigen::Vector2f ego_pos(0.0, 0.0);
    if (next_start_po.x() > 0.0) {
      road_scene_ = RoadScene::SMALL_JUNCTION;
      return;
    } else if (next_end_po.x() > big_junction_dis_thresh_) {
      road_scene_ = RoadScene::NON_JUNCTION;
      return;
    } else {
      road_scene_ = RoadScene::BIG_JUNCTUIN;
      return;
    }
  } else {
    // 存在两个及以上的group时， 判断最远的group是否在车后。
    auto& first_group = groups->front();
    auto& last_group = groups->back();
    if (last_group->group_state != Group::GroupState::VIRTUAL &&
        !last_group->group_segments.empty()) {
      Eigen::Vector2f next_end_pl(
          last_group->group_segments.back()->end_slice.pl.x(),
          last_group->group_segments.back()->end_slice.pl.y());
      Eigen::Vector2f next_end_pr(
          last_group->group_segments.back()->end_slice.pr.x(),
          last_group->group_segments.back()->end_slice.pr.y());
      Eigen::Vector2f next_end_po(
          last_group->group_segments.back()->end_slice.po.x(),
          last_group->group_segments.back()->end_slice.po.y());
      Eigen::Vector2f ego_pos(0.0, 0.0);

      // 如果路口出现在车子的后面，
      // if (PointInVectorSide(next_end_pr, next_end_pl, ego_pos) > 0) {
      //   return RoadScene::BIG_JUNCTUIN;
      // }

      if (next_end_po.x() < 0.0) {
        // HLOG_ERROR << "2222222222222222 BIG" << "pl x:" << next_end_pl.x()
        // << "pr x:" << next_end_pr.x()
        //  << ", dist value" << PointToVectorDist(next_end_pr, next_end_pl,
        //  ego_pos)
        //  << ", dist" <<int(PointToVectorDist(next_end_pr, next_end_pl,
        //  ego_pos) >
        //       big_junction_dis_thresh_) << ", side:" <<
        //       int(PointInVectorSide(next_end_pr, next_end_pl, ego_pos) <= 0);
        road_scene_ = RoadScene::BIG_JUNCTUIN;
        return;
      }
    }
    // 存在两个及以上的group时， 判断最近的group是否在车前。
    if (first_group->group_state != Group::GroupState::VIRTUAL &&
        !first_group->group_segments.empty()) {
      Eigen::Vector2f next_start_pl(
          first_group->group_segments.front()->start_slice.pl.x(),
          first_group->group_segments.front()->start_slice.pl.y());
      Eigen::Vector2f next_start_pr(
          first_group->group_segments.front()->start_slice.pr.x(),
          first_group->group_segments.front()->start_slice.pr.y());
      Eigen::Vector2f next_start_po(
          first_group->group_segments.front()->start_slice.po.x(),
          first_group->group_segments.front()->start_slice.po.y());
      Eigen::Vector2f ego_pos(0.0, 0.0);

      // 如果路口出现在车子的前面，
      // if (PointInVectorSide(next_start_pr, next_start_pl, ego_pos) < 0) {
      //   return RoadScene::SMALL_JUNCTION;
      // }
      if (next_start_po.x() > 0.0) {
        // HLOG_ERROR << "2222222222222222 SMALL" << "pl x:" <<
        // next_start_po.x();
        road_scene_ = RoadScene::SMALL_JUNCTION;
        return;
      }
    }

    // bool has_cw_forward_group = false;
    for (int grp_idx = 0; grp_idx < static_cast<int>(groups->size()) - 1;
         ++grp_idx) {
      auto& curr_group = groups->at(grp_idx);
      auto& next_group = groups->at(grp_idx + 1);
      // 路口新建出来的group，先跳过。
      if (curr_group->group_segments.empty() ||
          next_group->group_segments.empty() ||
          curr_group->group_state == Group::GroupState::VIRTUAL ||
          next_group->group_state == Group::GroupState::VIRTUAL) {
        continue;
      }

      Eigen::Vector2f next_start_pl(
          next_group->group_segments.front()->start_slice.pl.x(),
          next_group->group_segments.front()->start_slice.pl.y());
      Eigen::Vector2f next_start_pr(
          next_group->group_segments.front()->start_slice.pr.x(),
          next_group->group_segments.front()->start_slice.pr.y());
      Eigen::Vector2f next_start_po(
          next_group->group_segments.front()->start_slice.po.x(),
          next_group->group_segments.front()->start_slice.po.y());
      Eigen::Vector2f ego_pos(0.0, 0.0);
      Eigen::Vector2f next_end_pl(
          next_group->group_segments.back()->end_slice.pl.x(),
          next_group->group_segments.back()->end_slice.pl.y());
      Eigen::Vector2f next_end_pr(
          next_group->group_segments.back()->end_slice.pr.x(),
          next_group->group_segments.back()->end_slice.pr.y());
      Eigen::Vector2f next_end_po(
          next_group->group_segments.back()->end_slice.po.x(),
          next_group->group_segments.back()->end_slice.po.y());

      // 在车后面的， 直接跳过（前面已经判断过group全部位于车后的情况）
      // if (PointInVectorSide(next_end_pr, next_end_pl, ego_pos) > 0) {
      //   continue;
      // }
      if (next_end_po.x() < 0.0) {
        continue;
      }

      if (AreAdjacentLaneGroupsDisconnected(curr_group, next_group)) {
        if (!IsAngleOkOfCurGrpAndNextGrp(curr_group, next_group)) {
          // 路口转弯等场景则跳过。
          continue;
        }

        // 如果路口出现在车子的后面，
        // if (PointInVectorSide(next_start_pr, next_start_pl, ego_pos) < 0) {
        //   return RoadScene::SMALL_JUNCTION;
        // }

        if (next_start_po.x() > 0.0) {
          // HLOG_ERROR << "3333333333 SMALL" << "pl x:" << next_start_po.x();
          road_scene_ = RoadScene::SMALL_JUNCTION;
          return;
        }
      }
    }

    // 20米判断的逻辑
    if (last_group->group_state != Group::GroupState::VIRTUAL &&
        !last_group->group_segments.empty()) {
      Eigen::Vector2f next_end_pl(
          last_group->group_segments.back()->end_slice.pl.x(),
          last_group->group_segments.back()->end_slice.pl.y());
      Eigen::Vector2f next_end_pr(
          last_group->group_segments.back()->end_slice.pr.x(),
          last_group->group_segments.back()->end_slice.pr.y());
      Eigen::Vector2f next_end_po(
          last_group->group_segments.back()->end_slice.po.x(),
          last_group->group_segments.back()->end_slice.po.y());
      Eigen::Vector2f ego_pos(0.0, 0.0);

      // 设置阈值时，需要特别小心， 当pose出现预测偏差时，距离很可能会计算错误。
      // if (PointToVectorDist(next_end_pr, next_end_pl, ego_pos) >
      //         big_junction_dis_thresh_ &&
      //     PointInVectorSide(next_end_pr, next_end_pl, ego_pos) <= 0) {
      //   return RoadScene::NON_JUNCTION;
      // } else {
      //   HLOG_ERROR << "333333333333333333 BIG" << "pl x:" << next_end_pl.x()
      //   << "pr x:" << next_end_pr.x()
      //    << ", dist value" << PointToVectorDist(next_end_pr, next_end_pl,
      //    ego_pos)
      //    << ", dist" <<int(PointToVectorDist(next_end_pr, next_end_pl,
      //    ego_pos) >
      //         big_junction_dis_thresh_) << ", side:" <<
      //         int(PointInVectorSide(next_end_pr, next_end_pl, ego_pos) <= 0);
      //   return RoadScene::BIG_JUNCTUIN;
      // }

      if (next_end_po.x() > big_junction_dis_thresh_) {
        road_scene_ = RoadScene::NON_JUNCTION;
        return;
      } else {
        // HLOG_ERROR << "333333333333333333 BIG" << "pl x:" << next_end_pl.x()
        // << "pr x:" << next_end_pr.x()
        //  << ", dist value" << PointToVectorDist(next_end_pr, next_end_pl,
        //  ego_pos)
        //  << ", dist" <<int(PointToVectorDist(next_end_pr, next_end_pl,
        //  ego_pos) >
        //       big_junction_dis_thresh_) << ", side:" <<
        //       int(PointInVectorSide(next_end_pr, next_end_pl, ego_pos) <= 0);
        road_scene_ = RoadScene::BIG_JUNCTUIN;
        return;
      }
    }

    road_scene_ = RoadScene::NON_JUNCTION;
    return;
  }
}

// 从原始ElementMap里提取出车道线：
// 1.将ElementMap里点按原顺序保存到队列；
// 2.interp_dist为插值间距，当>0时按此间距对车道线点进行插值；
// 3.计算每根线的末端平均heading和平均点间距，用于后面预测.
void GroupMap::RetrieveBoundaries(const em::ElementMap::Ptr& ele_map,
                                  float interp_dist,
                                  std::deque<Line::Ptr>* lines) {
  HLOG_DEBUG << "RetrieveBoundaries";
  if (ele_map == nullptr || lines == nullptr) {
    return;
  }
  zebra_.clear();
  arrow_.clear();
  stopline_.clear();
  overlaps_.clear();
  bool need_interp = (interp_dist > 0);

  for (const auto& bound_pair : ele_map->boundaries) {
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
    em::Point last_raw(0, 0, 0);
    for (const auto& node : bound->nodes) {
      if (node == nullptr) {
        HLOG_ERROR << "found nullptr node";
        continue;
      }
      const auto& curr_raw = node->point;
      if (need_interp && !line->pts.empty()) {
        // 方向向量
        em::Point n = curr_raw - last_raw;
        float dist = n.norm();
        if (dist > interp_dist) {
          n.normalize();
          int interp_cnt = static_cast<int>(dist / interp_dist);
          for (int i = 0; i < interp_cnt; ++i) {
            em::Point interp_pt = last_raw + (i + 1) * n;
            Point pt(gm::INTERPOLATED, interp_pt.x(), interp_pt.y(),
                     interp_pt.z());
            line->pts.emplace_back(pt);
          }
        }
      }
      Point pt(gm::RAW, curr_raw.x(), curr_raw.y(), curr_raw.z());
      line->pts.emplace_back(pt);
      last_raw = curr_raw;
    }
    if (line->pts.empty()) {
      continue;
    }
    ComputeLineHeading(line);
    lines->emplace_back(line);
  }

  for (const auto& arrow : ele_map->arrows) {
    Arrow arw;
    arw.id = arrow.second->id;
    arw.type = arrow.second->type;
    arw.polygon = arrow.second->polygon;
    arrow_[arw.id] = std::make_shared<Arrow>(arw);
  }

  for (const auto& stopline : ele_map->stop_lines) {
    Stpl stpl;
    stpl.id = stopline.second->id;
    stpl.points = stopline.second->points;
    stopline_[stpl.id] = std::make_shared<Stpl>(stpl);
  }

  for (const auto& zebra : ele_map->cross_walks) {
    Zebra zbr;
    zbr.id = zebra.second->id;
    zbr.polygon = zebra.second->polygon;
    zebra_[zbr.id] = std::make_shared<Zebra>(zbr);
  }
}

void GroupMap::BuildKDtrees(std::deque<Line::Ptr>* lines) {
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

// 按轨迹方向生成所有GroupSegments
void GroupMap::BuildGroupSegments(
    const std::shared_ptr<std::vector<KinePose::Ptr>>& path,
    const KinePose::Ptr& curr_pose, std::deque<Line::Ptr>* lines,
    std::vector<GroupSegment::Ptr>* group_segments,
    const em::ElementMap::Ptr& ele_map) {
  HLOG_DEBUG << "BuildGroupSegments";
  if (path == nullptr || curr_pose == nullptr || lines == nullptr ||
      group_segments == nullptr) {
    return;
  }

  CreateGroupSegFromPath(path, *curr_pose, group_segments);

  SplitPtsToGroupSeg(lines, group_segments);

  GenLaneSegInGroupSeg(group_segments);

  EgoLineTrajectory(group_segments, ele_map);
}

void GroupMap::FitLaneline(const em::ElementMap::Ptr& ele_map, int id_1,
                           int id_2, int near_line) {
  predict_line_params_.clear();
  int size1_s = -1, size2_s = -1, size1_num = 0, size2_num = 0;
  for (const auto& node : ele_map->boundaries[id_1]->nodes) {
    if (node->point.x() > -15 && node->point.x() < 15) {
      if (size1_s == -1) {
        size1_s = node->id;
      }
      size1_num++;
    } else if (node->point.x() > 15) {
      break;
    }
  }
  for (const auto& node : ele_map->boundaries[id_2]->nodes) {
    if (node->point.x() > -15 && node->point.x() < 15) {
      if (size2_s == -1) {
        size2_s = node->id;
      }
      size2_num++;
    } else if (node->point.x() > 15) {
      break;
    }
  }
  //! TBD: 用三次来拟合线会更准确，目前只是二次的，待改进
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

void GroupMap::EgoLineTrajectory(std::vector<GroupSegment::Ptr>* grp_segment,
                                 const em::ElementMap::Ptr& ele_map) {
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

// 将所有GroupSegments聚合成一个个Group
void GroupMap::BuildGroups(const em::ElementMap::Ptr& ele_map,
                           std::vector<GroupSegment::Ptr> group_segments,
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

// 查找path中是否有掉头path_i，若有则删掉path_0到path_i
void GroupMap::ProcessUTurn(
    const std::shared_ptr<std::vector<KinePose::Ptr>>& path) {
  for (int i = static_cast<int>(path->size() - 1); i > 0; i--) {
    if ((*path)[i]->state == PoseState::UTURN) {
      path->erase(path->begin(), (path->begin() + i));
      break;
    }
  }
}

// 按轨迹方向计算出所有切分线，每两根相邻切分线组成一个GroupSegment
void GroupMap::CreateGroupSegFromPath(
    const std::shared_ptr<std::vector<KinePose::Ptr>>& path,
    const KinePose& curr_pose, std::vector<GroupSegment::Ptr>* segments) {
  if (segments == nullptr) {
    HLOG_ERROR << "input nullptr";
    return;
  }
  // 检查是否有掉头，并处理
  ProcessUTurn(path);

  // 将path里一个个pose转换成分割线SliceLine
  std::vector<SliceLine> slice_lines;
  Pose pose_local_to_veh = curr_pose.Inverse();
  for (auto& p : *path) {
    if (p == nullptr) {
      HLOG_ERROR << "found nullptr pose in path";
      continue;
    }
    em::Point pl_veh(0, 1, 0);
    pl_veh *= conf_.half_slice_length;
    em::Point pr_veh(0, -1, 0);
    pr_veh *= conf_.half_slice_length;
    em::Point pl_local = p->TransformPoint(pl_veh);
    em::Point pr_local = p->TransformPoint(pr_veh);
    em::Point po_local = p->pos;

    em::Point pl_curr_veh = pose_local_to_veh.TransformPoint(pl_local);
    em::Point pr_curr_veh = pose_local_to_veh.TransformPoint(pr_local);
    em::Point po_curr_veh = pose_local_to_veh.TransformPoint(po_local);
    SliceLine slice;
    slice.po = po_curr_veh;
    slice.pl = pl_curr_veh;
    slice.pr = pr_curr_veh;
    slice_lines.emplace_back(slice);
  }

  if (slice_lines.size() < 2) {
    HLOG_WARN << "no enough slice lines: " << slice_lines.size();
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
}

// 将所有车道线点切分到所属GroupSegment：对每根线的点从front到back，判断是否在GroupSegment
// 范围内
void GroupMap::SplitPtsToGroupSeg(std::deque<Line::Ptr>* lines,
                                  std::vector<GroupSegment::Ptr>* segments) {
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
      if (line_seg->pts.empty()) {
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

float GroupMap::DistByKDtree(const em::Point& ref_point,
                             const LineSegment& LineSegment) {
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

  em::Point tar_point;
  tar_point.x() = line_tofind[nearest_index[0]].x;
  tar_point.y() = line_tofind[nearest_index[0]].y;

  if (line_tofind.size() == 1) {
    return nearest_dist[0];
  } else {
    int id_next = 0;
    if (nearest_index[0] < static_cast<int>(line_tofind.size()) - 1) {
      id_next = nearest_index[0] + 1;
    } else {
      id_next = nearest_index[0] - 1;
    }
    em::Point tar_point_next;
    tar_point_next.x() = line_tofind[id_next].x;
    tar_point_next.y() = line_tofind[id_next].y;

    return GetDistPointLane(ref_point, tar_point, tar_point_next);
  }
}

float GroupMap::DistPointNew(const em::Point& ref_point,
                             const LineSegment& lineSegment) {
  if (std::isnan(ref_point.x()) || std::isnan(ref_point.y())) {
    return 0.0;
  }

  // 找到最近的点
  auto it = std::min_element(
      lineSegment.pts.begin(), lineSegment.pts.end(),
      [&ref_point](const gm::Point& a, const gm::Point& b) {
        return (ref_point - a.pt).norm() < (ref_point - b.pt).norm();
      });

  // 找到最近的点在向量中的位置
  size_t tar_idx = std::distance(lineSegment.pts.begin(), it);
  em::Point tar_point = lineSegment.pts[tar_idx].pt;
  if (lineSegment.pts.size() == 1) {
    return (ref_point - tar_point).norm();
  } else {
    int id_next = 0;
    // 获取后一个点
    if (tar_idx < lineSegment.pts.size() - 1) {
      id_next = tar_idx + 1;
    } else {
      // 获取前一个点
      id_next = tar_idx - 1;
    }
    em::Point tar_point_next = lineSegment.pts[id_next].pt;
    return GetDistPointLane(ref_point, tar_point, tar_point_next);
  }
}

float GroupMap::GetDistPointLane(const em::Point& point_a,
                                 const em::Point& point_b,
                                 const em::Point& point_c) {
  Eigen::Vector2f A(point_a.x(), point_a.y()), B(point_b.x(), point_b.y()),
      C(point_c.x(), point_c.y());
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

// 从GroupSegment包含的所有线中，聚合出一个个小的LaneSegment
void GroupMap::GenLaneSegInGroupSeg(std::vector<GroupSegment::Ptr>* segments) {
  std::string last_seg_id = "";
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
        auto right_center = right_line->center;
        auto dist = DistPointNew(left_center, *right_line);
        // HLOG_INFO << "1111 dist: " << dist;
        if (left_line->is_near_road_edge && right_line->is_near_road_edge &&
            line_seg_num > 5) {
          continue;
        }
        if (dist < conf_.min_lane_width ||
            ((left_line->is_near_road_edge || right_line->is_near_road_edge) &&
             dist < 2.5)) {
          // if (i == 0 && line_seg_num > 2) {
          //   std::string index =
          //       std::to_string(left_line->id) + "_" +
          //       std::to_string(seg->line_segments.at(i + 2)->id);
          //   if (last_seg_id.substr(0, index.length()) != index) {
          //     continue;
          //   }
          //   auto& right_line_2 = seg->line_segments.at(i + 2);
          //   auto& right_center_2 = right_line_2->center;
          //   auto dist2 = Dist(left_center, right_center_2);
          //   if (dist2 < conf_.min_lane_width || dist2 > conf_.max_lane_width)
          //   {
          //     continue;
          //   }
          //   right_line = right_line_2;
          //   right_center = right_center_2;
          //   i = i + 1;
          // } else if (i == line_seg_num - 2 && line_seg_num > 2) {
          //   std::string index = std::to_string(seg->line_segments.at(i)->id);
          //   std::string index2 =
          //       index + "_" + std::to_string(seg->line_segments.at(i +
          //       1)->id);
          //   auto& left_line_2 = seg->line_segments.at(i - 1);
          //   auto& left_center_2 = left_line_2->center;
          //   if ((last_seg_id.length() > index.length() &&
          //        last_seg_id.substr(last_seg_id.length() - index.length(),
          //                           index.length()) == index) ||
          //       (last_seg_id.length() > index2.length() &&
          //        last_seg_id.substr(last_seg_id.length() - index2.length(),
          //                           index2.length()) == index2)) {
          //     continue;
          //   }

          //   auto dist2 = Dist(left_center_2, right_center);
          //   auto dist1 = Dist(left_center_2, left_center);
          //   if (dist2 < conf_.min_lane_width || dist2 > conf_.max_lane_width)
          //   {
          //     continue;
          //   } else {
          //     auto lane_seg = std::make_shared<LaneSegment>();
          //     lane_seg->left_boundary = left_line_2;
          //     lane_seg->right_boundary = right_line;
          //     // 将左右边线id连起来，作为LaneSegment的str_id，
          //     //
          //     这里使用边线id组成字符串id来标识lane，可以方便后续比对lane与lane是否共线
          //     lane_seg->str_id = std::to_string(left_line_2->id) + "_" +
          //                        std::to_string(right_line->id);
          //     lane_seg->lanepos_id = std::to_string(left_line_2->lanepos) +
          //                            "_" +
          //                            std::to_string(right_line->lanepos);
          //     if (dist1 > conf_.min_lane_width &&
          //         dist1 < conf_.max_lane_width) {
          //       seg->lane_segments.back() = lane_seg;
          //     } else {
          //       seg->lane_segments.emplace_back(lane_seg);
          //     }
          //     break;
          //   }
          // } else {
          //   continue;
          // }
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

// 将若干个GroupSegment聚合成Group：
// 当前采用的策略是：相邻两个GroupSegment内包含的LaneSegment一样，那就可以聚合到一起；
// 即每个Group里所有GroupSegment包含的LaneSegment数目都相同，并且LaneSegment的
// str_id（由边线id组成）也相同.
//! TBD：当前聚合方法只考虑了lane的边线id，后续可增加根据lane边线的虚实类型进行聚合
void GroupMap::UniteGroupSegmentsToGroups(
    double stamp, std::vector<GroupSegment::Ptr> group_segments,
    std::vector<Group::Ptr>* groups) {
  // HLOG_ERROR << "group_segments.size() = " << group_segments.size();
  if (group_segments.size() > 1) {
    while (group_segments.size() > 1 &&
           group_segments[0]->str_id != group_segments[1]->str_id) {
      group_segments.erase(group_segments.begin());
    }
    if (group_segments.size() > 1) {
      for (int i = static_cast<int>(group_segments.size()) - 1; i > 0; i--) {
        if (group_segments[i]->str_id != group_segments[i - 1]->str_id) {
          group_segments.erase(group_segments.begin() + i);
        } else {
          break;
        }
      }
    }
  }
  if (group_segments.size() > 3) {
    for (size_t i = 1; i < group_segments.size() - 2; ++i) {
      if ((group_segments[i]->str_id != group_segments[i - 1]->str_id) &&
          ((group_segments[i]->str_id != group_segments[i + 1]->str_id))) {
        //    ||
        //  (group_segments[i]->str_id != group_segments[i + 2]->str_id)
        group_segments.erase(group_segments.begin() + i);
        i--;
      }
    }
  }

  // if (!group_segments.empty()) {
  //   HLOG_ERROR << "group_segments2.size() = " << group_segments.size();
  // }
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
  // HLOG_ERROR << "grp_idx = " << grp_idx
  //            << "groups->size() = " << groups->size();
  // for (auto grp : *groups) {
  //   HLOG_ERROR << "group name is " << grp->str_id;
  //   HLOG_ERROR << "group segment size is " << grp->group_segments.size();
  // }
  // if (grp_idx > 2) {
  //   for (int i = 1; i < static_cast<int>(groups->size()) - 1; ++i) {
  //     if (groups->at(i - 1)->seg_str_id == groups->at(i + 1)->seg_str_id &&
  //         groups->at(i)->group_segments.size() < 6) {
  //       groups->erase(groups->begin() + i);
  //       i--;
  //     }
  //   }
  // }
}

float GroupMap::CalculateDistPt(Lane::Ptr lane_in_next, Lane::Ptr lane_in_curr,
                                size_t sizet) {
  return pow(lane_in_next->center_line_pts[0].pt.y() -
                 lane_in_curr->center_line_pts[sizet - 1].pt.y(),
             2) +
         pow(lane_in_next->center_line_pts[0].pt.x() -
                 lane_in_curr->center_line_pts[sizet - 1].pt.x(),
             2);
}

float GroupMap::CalculatePoint2CenterLine(Lane::Ptr lane_in_next,
                                          Lane::Ptr lane_in_curr) {
  return abs(lane_in_next->center_line_pts[0].pt.y() -
             (lane_in_curr->center_line_param[0] +
              lane_in_curr->center_line_param[1] *
                  lane_in_next->center_line_pts[0].pt.x())) /
         sqrt(1 + pow(lane_in_curr->center_line_param[1], 2));
}

float GroupMap::Calculate2CenterlineAngle(Lane::Ptr lane_in_next,
                                          Lane::Ptr lane_in_curr,
                                          size_t sizet) {
  return atan((lane_in_next->center_line_pts[0].pt.y() -
               lane_in_curr->center_line_pts[sizet - 1].pt.y()) /
              (lane_in_next->center_line_pts[0].pt.x() -
               lane_in_curr->center_line_pts[sizet - 1].pt.x())) -
         atan(lane_in_curr->center_line_param[1]);
}

bool GroupMap::IsAccessLane(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next) {
  if (lane_in_curr->center_line_pts.empty() ||
      lane_in_next->center_line_pts.empty()) {
    return false;
  }

  // 从lane_in_next车道的中心线上找出一条大于2米长度的向量出来(从起点位置开始)。
  Eigen::Vector3f lane_in_next_norm(0.0, 0.0, 0.0);
  for (size_t next_lane_cp_idx = 1;
       next_lane_cp_idx < lane_in_next->center_line_pts.size();
       ++next_lane_cp_idx) {
    lane_in_next_norm = lane_in_next->center_line_pts[next_lane_cp_idx].pt -
                        lane_in_next->center_line_pts[0].pt;
    if (lane_in_next_norm.norm() >= 2.0) {
      break;
    }
  }

  if (lane_in_next_norm.norm() <= 2.0) {
    return false;
  }

  size_t sizet = lane_in_curr->center_line_pts.size();
  Eigen::Vector3f lane_curr2next_vec =
      lane_in_curr->center_line_pts[sizet - 1].pt -
      lane_in_next->center_line_pts[0].pt;
  lane_in_next_norm = lane_in_next_norm.normalized();
  // 判断两车道中心线的横向距离是否大于2.0, 如果小于2米，认为是可通行的。
  if ((lane_in_next_norm.cross(lane_curr2next_vec)).norm() <= 2.0) {
    return true;
  }

  // 判断curr点是否在next_lane的右侧
  bool is_right = (lane_in_next_norm.y() * lane_curr2next_vec.x() -
                   lane_in_next_norm.x() * lane_curr2next_vec.y()) > 0
                      ? true
                      : false;
  if (is_right &&
      (lane_in_next->right_boundary->type == em::LaneType_SOLID ||
       lane_in_next->right_boundary->type == em::LaneType_DOUBLE_SOLID ||
       lane_in_next->right_boundary->type ==
           em::LaneType_LEFT_SOLID_RIGHT_DASHED)) {
    HLOG_DEBUG << "NOT ACCESS RIGHT!";
    return false;
  } else if (!is_right &&
             (lane_in_next->left_boundary->type == em::LaneType_SOLID ||
              lane_in_next->left_boundary->type == em::LaneType_DOUBLE_SOLID ||
              lane_in_next->left_boundary->type ==
                  em::LaneType_RIGHT_SOLID_LEFT_DASHED)) {
    HLOG_DEBUG << "NOT ACCESS LEFT!";
    return false;
  }

  return true;
}

bool GroupMap::AreLaneConnect(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next) {
  if (!IsAccessLane(lane_in_curr, lane_in_next)) {
    return false;
  }
  size_t sizet = lane_in_curr->center_line_pts.size();
  float dis_pt = CalculateDistPt(lane_in_next, lane_in_curr, sizet);
  float angel_thresh{0.0};
  float dis_thresh{0.0};
  if (lane_in_curr->left_boundary->id == lane_in_next->right_boundary->id ||
      lane_in_curr->right_boundary->id == lane_in_next->left_boundary->id) {
    return false;
  }
  if (dis_pt < 8 || (lane_in_curr->lanepos_id == lane_in_next->lanepos_id &&
                     lane_in_curr->lanepos_id != "99_99")) {
    // HLOG_ERROR << "lane_in_curr=" << lane_in_curr->str_id_with_group
    //            << "   lane_in_next" << lane_in_next->str_id_with_group
    //            << "  dis_pt = " << dis_pt
    //            << " lane_in_curr->lanepos_id =" << lane_in_curr->lanepos_id
    //            << "  lane_in_next->lanepos_id = " <<
    //            lane_in_next->lanepos_id;
    return true;
  } else if (!lane_in_curr->center_line_param.empty() &&
             !lane_in_next->center_line_pts.empty()) {
    if (dis_pt > 10000) {
      // 差的太远不计算
      return false;
    }
    if (dis_pt > 100) {
      angel_thresh = 10;
      dis_thresh = 3;
    } else {
      angel_thresh = 25;
      dis_thresh = 1.8;
    }
    float dis = CalculatePoint2CenterLine(lane_in_next, lane_in_curr);
    float angle = Calculate2CenterlineAngle(lane_in_next, lane_in_curr, sizet);
    // HLOG_INFO << "angle = " << angle * 180 / pi_;
    // HLOG_INFO << "lane angle lane_in_next->center_line_pts[0].pt  "
    //           << lane_in_next->center_line_pts[0].pt.y() << "   "
    //           << lane_in_next->center_line_pts[0].pt.x();
    // HLOG_INFO << "lane_in_curr->center_line_pts[sizet - 1].pt  "
    //           << lane_in_curr->center_line_pts[sizet - 1].pt.y() << "   "
    //           << lane_in_curr->center_line_pts[sizet - 1].pt.x()
    //           << "  angle is "
    //           << atan((lane_in_next->center_line_pts[0].pt.y() -
    //                    lane_in_curr->center_line_pts[sizet - 1].pt.y()) /
    //                   (lane_in_next->center_line_pts[0].pt.x() -
    //                    lane_in_curr->center_line_pts[sizet - 1].pt.x())) *
    //                  180 / pi_;
    // HLOG_INFO << "lane_in_curr->center_line_param[1] is"
    //           << atan(lane_in_curr->center_line_param[1]) * 180 / pi_;
    // HLOG_INFO << "lane_in_curr=" << lane_in_curr->str_id_with_group
    //           << "   lane_in_next" << lane_in_next->str_id_with_group
    //           << "  dis2l = " << dis << "  dis thresh = " << dis_thresh
    //           << " angle = " << abs(angle) * 180 / pi_
    //           << "  angel_thresh = " << angel_thresh;
    if ((dis < dis_thresh && abs(angle) * 180 / pi_ < angel_thresh)) {
      // HLOG_INFO << "lane_in_curr=" << lane_in_curr->str_id_with_group
      //           << "   lane_in_next" << lane_in_next->str_id_with_group
      //           << "  dis2l = " << dis << "  dis thresh = " << dis_thresh
      //           << " angle = " << angle << "  angel_thresh = " <<
      //           angel_thresh;
      return true;
    }
  }
  // HLOG_INFO << "lane_in_curr=" << lane_in_curr->str_id_with_group;
  return false;
}

bool GroupMap::AreLaneConnect(
    Group::Ptr curr_group, Group::Ptr next_group, int i, int j,
    std::map<int, std::vector<int>>* curr_group_next_lane,
    std::map<int, std::vector<int>>* next_group_prev_lane) {
  auto& lane_in_curr = curr_group->lanes[i];
  auto& lane_in_next = next_group->lanes[j];
  if (!IsAccessLane(lane_in_curr, lane_in_next)) {
    return false;
  }
  size_t sizet = lane_in_curr->center_line_pts.size();
  float dis_pt = CalculateDistPt(lane_in_next, lane_in_curr, sizet);
  float angel_thresh{0.0};
  float dis_thresh{0.0};
  if (dis_pt < 8 || (lane_in_curr->lanepos_id == lane_in_next->lanepos_id &&
                     lane_in_curr->lanepos_id != "99_99")) {
    if (curr_group_next_lane->find(i) != curr_group_next_lane->end()) {
      curr_group_next_lane->at(i).emplace_back(j);
    } else {
      curr_group_next_lane->insert({i, {j}});
    }
    if (next_group_prev_lane->find(j) != next_group_prev_lane->end()) {
      next_group_prev_lane->at(j).emplace_back(i);
    } else {
      next_group_prev_lane->insert({j, {i}});
    }
    // HLOG_ERROR << "lane_in_curr=" << lane_in_curr->str_id_with_group
    //            << "   lane_in_next" << lane_in_next->str_id_with_group
    //            << "  dis_pt = " << dis_pt
    //            << " lane_in_curr->lanepos_id =" << lane_in_curr->lanepos_id
    //            << "  lane_in_next->lanepos_id = " <<
    //            lane_in_next->lanepos_id;

    return true;
  } else if (!lane_in_curr->center_line_param.empty() &&
             !lane_in_next->center_line_pts.empty()) {
    if (dis_pt > 10000) {
      // 差的太远不计算
      return false;
    }
    if (dis_pt > 100) {
      angel_thresh = 10;
      dis_thresh = 3;
    } else {
      angel_thresh = 25;
      dis_thresh = 1.8;
    }
    float dis = CalculatePoint2CenterLine(lane_in_next, lane_in_curr);
    float angle = Calculate2CenterlineAngle(lane_in_next, lane_in_curr, sizet);
    // HLOG_ERROR << "angle = "<<angle*180/pi_;
    if ((dis < dis_thresh && abs(angle) * 180 / pi_ < angel_thresh)) {
      //! TBD:
      //! 这里直接把后一个lane中心线的第一个点加到前一个lane中心线的末尾，
      //! 后续需要考虑某些异常情况，比如后一个lane中心线的第一个点在前一个lane中心线最后
      //! 一个点的后方，这样直连就导致整个中心线往后折返了；以及还要考虑横向偏移较大时不平
      //! 滑的问题
      if (curr_group_next_lane->find(i) != curr_group_next_lane->end()) {
        curr_group_next_lane->at(i).emplace_back(j);
      } else {
        curr_group_next_lane->insert({i, {j}});
      }
      if (next_group_prev_lane->find(j) != next_group_prev_lane->end()) {
        next_group_prev_lane->at(j).emplace_back(i);
      } else {
        next_group_prev_lane->insert({j, {i}});
      }
      // HLOG_ERROR << "lane_in_curr=" << lane_in_curr->str_id_with_group
      //            << "   lane_in_next" << lane_in_next->str_id_with_group
      //            << "  dis2l = " << dis << "  dis thresh = " << dis_thresh
      //            << " angle = " << angle << "  angel_thresh = " <<
      //            angel_thresh;

      return true;
    }
  }
  return false;
}

void GroupMap::NextGroupLaneConnect(
    Group::Ptr curr_group, Group::Ptr next_group,
    const std::map<int, std::vector<int>>& curr_group_next_lane) {
  for (const auto& iter : curr_group_next_lane) {
    auto& lane_in_curr = curr_group->lanes[iter.first];
    if (iter.second.size() == 1) {
      auto& lane_in_next = next_group->lanes[iter.second[0]];
      lane_in_curr->next_lane_str_id_with_group.emplace_back(
          lane_in_next->str_id_with_group);
      lane_in_next->prev_lane_str_id_with_group.emplace_back(
          lane_in_curr->str_id_with_group);
      if (lane_in_next->center_line_pts.size() > 2 &&
          lane_in_curr->center_line_pts.size() > 4) {
        // size_t cur_num = lane_in_curr->center_line_pts.size();
        // std::vector<Eigen::Vector3f> tmp_vec;
        // tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num - 5].pt);
        // tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num - 4].pt);
        // tmp_vec.emplace_back(lane_in_next->center_line_pts[0].pt);
        // tmp_vec.emplace_back(lane_in_next->center_line_pts[1].pt);
        // std::vector<Eigen::Vector3f> center_fit;
        // CatmullRom(tmp_vec, &center_fit, 4);
        // Point pt_center;
        // pt_center.type = gm::VIRTUAL;
        // pt_center.pt = center_fit[0];
        // lane_in_curr->center_line_pts[cur_num - 3] = pt_center;
        // pt_center.pt = center_fit[1];
        // lane_in_curr->center_line_pts[cur_num - 2] = pt_center;
        // pt_center.pt = center_fit[2];
        // lane_in_curr->center_line_pts[cur_num - 1] = pt_center;
        lane_in_curr->center_line_pts.erase(
            lane_in_curr->center_line_pts.end() - 3,
            lane_in_curr->center_line_pts.end());
        lane_in_curr->center_line_pts.emplace_back(
            lane_in_next->center_line_pts.front());
      } else if (!lane_in_next->center_line_pts.empty()) {
        lane_in_curr->center_line_pts.emplace_back(
            lane_in_next->center_line_pts.front());
      }
    } else if (iter.second.size() > 1) {
      for (const auto& next_idx : iter.second) {
        auto& lane_in_next = next_group->lanes[next_idx];
        lane_in_curr->next_lane_str_id_with_group.emplace_back(
            lane_in_next->str_id_with_group);
        lane_in_next->prev_lane_str_id_with_group.emplace_back(
            lane_in_curr->str_id_with_group);
        if (lane_in_next->center_line_pts.size() > 4 &&
            lane_in_curr->center_line_pts.size() > 2) {
          // size_t cur_num = lane_in_curr->center_line_pts.size();
          // std::vector<Eigen::Vector3f> tmp_vec;
          // tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num -
          // 2].pt); tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num
          // - 1].pt);
          // tmp_vec.emplace_back(lane_in_next->center_line_pts[3].pt);
          // tmp_vec.emplace_back(lane_in_next->center_line_pts[4].pt);
          // std::vector<Eigen::Vector3f> center_fit;
          // CatmullRom(tmp_vec, &center_fit, 4);
          // Point pt_center;
          // pt_center.type = gm::VIRTUAL;
          // pt_center.pt = center_fit[0];
          // lane_in_next->center_line_pts[0] = pt_center;
          // pt_center.pt = center_fit[1];
          // lane_in_next->center_line_pts[1] = pt_center;
          // pt_center.pt = center_fit[2];
          // lane_in_next->center_line_pts[2] = pt_center;
          // lane_in_next->center_line_pts.insert(
          //     lane_in_next->center_line_pts.begin(),
          //     lane_in_curr->center_line_pts.back());
          lane_in_next->center_line_pts.erase(
              lane_in_next->center_line_pts.begin(),
              lane_in_next->center_line_pts.begin() + 2);
          lane_in_next->center_line_pts[0] =
              lane_in_curr->center_line_pts.back();
        } else if (!lane_in_curr->center_line_pts.empty()) {
          lane_in_next->center_line_pts.insert(
              lane_in_next->center_line_pts.begin(),
              lane_in_curr->center_line_pts.back());
        }
      }
    }
  }
}

float GroupMap::LaneDist(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next) {
  size_t sizet = lane_in_curr->center_line_pts.size();
  Eigen::Vector3f lane_in_next_norm(0.0, 0.0, 0.0);
  size_t next_lane_cp_idx = 1;
  while (next_lane_cp_idx < lane_in_next->center_line_pts.size() &&
         lane_in_next_norm.norm() < 2.0) {
    lane_in_next_norm = lane_in_next->center_line_pts[next_lane_cp_idx].pt -
                        lane_in_next->center_line_pts[0].pt;
    next_lane_cp_idx++;
  }
  if (lane_in_next_norm.norm() > 2.0) {
    lane_in_next_norm = lane_in_next_norm.normalized();
    Eigen::Vector3f lane_curr_next_vec =
        lane_in_curr->center_line_pts[sizet - 1].pt -
        lane_in_next->center_line_pts[0].pt;
    float dis = (lane_curr_next_vec.cross(lane_in_next_norm)).norm();
    return dis;
  }
  return 10.0;
}

void GroupMap::FindGroupNextLane(Group::Ptr curr_group, Group::Ptr next_group) {
  // 如果next_group的lanes比curr_group的lanes多或者相等的话，currgroup->lane有一个后继
  // 如果next_group的lanes比curr_group的lanes少的话，next->lane有一个前驱
  // 如果左右车道线有一根的trackid相等的话，有连接关系

  // 用hash先存下关联关系
  std::map<int, std::vector<int>>
      curr_group_next_lane;  // 当前车道groupindex,后继在下个group的index
  std::map<int, std::vector<int>> next_group_prev_lane;
  if (curr_group->lanes.size() >= next_group->lanes.size()) {
    HLOG_DEBUG << "CUR >= NEXT";
    for (int i = 0; i < curr_group->lanes.size(); ++i) {
      auto& lane_in_curr = curr_group->lanes[i];
      if (lane_in_curr->next_lane_str_id_with_group.empty()) {
        int trackid_same_find = 0;
        for (int j = 0; j < next_group->lanes.size(); ++j) {
          auto& lane_in_next = next_group->lanes[j];
          if (lane_in_next->left_boundary->id ==
                  lane_in_curr->left_boundary->id ||
              lane_in_next->right_boundary->id ==
                  lane_in_curr->right_boundary->id) {
            curr_group_next_lane[i].emplace_back(j);
            next_group_prev_lane[j].emplace_back(i);
            trackid_same_find = 1;
            break;
          }
        }
        if (trackid_same_find == 0) {
          bool has_next_lane = false;
          for (int j = 0; j < next_group->lanes.size(); ++j) {
            has_next_lane =
                AreLaneConnect(curr_group, next_group, i, j,
                               &curr_group_next_lane, &next_group_prev_lane) ||
                has_next_lane;
          }
          if (!has_next_lane) {
            // 目前只考虑最左或最右线关联问题
            // 这里仅增加收缩的lane，目的是为了排除正常宽度的lane也被认为是merge的lane
            double curr_len = CalcLaneLength(lane_in_curr);
            bool shrink = IsLaneShrink(lane_in_curr);
            const float dis_thresh = 4.5;
            if (i == 0 && curr_len > kMergeLengthThreshold && shrink &&
                IsAccessLane(lane_in_curr, next_group->lanes[0]) &&
                LaneDist(lane_in_curr, next_group->lanes[0]) < dis_thresh &&
                CalcLaneLength(next_group->lanes[0]) > kMergeLengthThreshold) {
              curr_group_next_lane[i].emplace_back(0);
              next_group_prev_lane[0].emplace_back(i);
            } else if (i == static_cast<int>(curr_group->lanes.size()) - 1 &&
                       curr_len > kMergeLengthThreshold && shrink &&
                       IsAccessLane(lane_in_curr, next_group->lanes.back()) &&
                       LaneDist(lane_in_curr, next_group->lanes.back()) <
                           dis_thresh &&
                       CalcLaneLength(next_group->lanes.back()) >
                           kMergeLengthThreshold) {
              curr_group_next_lane[i].emplace_back(
                  static_cast<int>(next_group->lanes.size()) - 1);

              next_group_prev_lane
                  [static_cast<int>(static_cast<int>(next_group->lanes.size()) -
                                    1)]
                      .emplace_back(i);
            }
          }
        }
      }
    }
    for (auto& iter : curr_group_next_lane) {
      if (iter.second.size() > 1) {
        int best = iter.second[0];
        for (auto& next_lane_idx : iter.second) {
          if (best == next_lane_idx) {
            continue;
          }
          auto& lane_in_next = next_group->lanes[next_lane_idx];
          auto& lane_best = next_group->lanes[best];
          auto& lane_in_curr = curr_group->lanes[iter.first];
          if (next_group_prev_lane[next_lane_idx].size() == 1 &&
              next_group_prev_lane[best].size() > 1) {
            best = next_lane_idx;
          } else if ((next_group_prev_lane[next_lane_idx].size() > 1 &&
                      next_group_prev_lane[best].size() > 1) ||
                     (next_group_prev_lane[next_lane_idx].size() == 1 &&
                      next_group_prev_lane[best].size() == 1)) {
            size_t sizet = lane_in_curr->center_line_pts.size();
            if (!lane_in_curr->center_line_param.empty() &&
                !lane_in_next->center_line_param_front.empty() &&
                !lane_best->center_line_param_front.empty()) {
              float angle1 =
                  Calculate2CenterlineAngle(lane_in_next, lane_in_curr, sizet);
              float angle2 =
                  Calculate2CenterlineAngle(lane_best, lane_in_curr, sizet);
              float dis1 =
                  CalculatePoint2CenterLine(lane_in_next, lane_in_curr);
              float dis2 = CalculatePoint2CenterLine(lane_best, lane_in_curr);
              if (angle2 > angle1 + 1 * 180 / pi_ && dis2 > dis1 + 0.2) {
                best = next_lane_idx;
              }
            } else {
              float dis_pt = CalculateDistPt(lane_in_next, lane_in_curr, sizet);
              float dis_pt2 = CalculateDistPt(lane_best, lane_in_curr, sizet);
              if (dis_pt < dis_pt2) {
                best = next_lane_idx;
              }
            }
          }
        }
        for (auto& next_lane_idx : iter.second) {
          if (next_lane_idx == best) {
            continue;
          }
          for (auto iter_next = next_group_prev_lane[next_lane_idx].begin();
               iter_next != next_group_prev_lane[next_lane_idx].end();
               ++iter_next) {
            // 删除相关的prev
            if (*iter_next == iter.first) {
              next_group_prev_lane[next_lane_idx].erase(iter_next);
              break;
            }
          }
        }
        iter.second.clear();
        iter.second.emplace_back(best);
      }
    }
  } else {
    HLOG_DEBUG << "CUR < NEXT";
    for (int i = 0; i < next_group->lanes.size(); ++i) {
      auto& lane_in_next = next_group->lanes[i];
      if (lane_in_next->prev_lane_str_id_with_group.empty()) {
        int trackid_same_find = 0;
        for (int j = 0; j < curr_group->lanes.size(); ++j) {
          auto& lane_in_curr = curr_group->lanes[j];
          if (lane_in_next->left_boundary->id ==
                  lane_in_curr->left_boundary->id ||
              lane_in_next->right_boundary->id ==
                  lane_in_curr->right_boundary->id) {
            curr_group_next_lane[j].emplace_back(i);
            next_group_prev_lane[i].emplace_back(j);
            trackid_same_find = 1;
            break;
          }
        }
        if (trackid_same_find == 0) {
          bool has_prev_lane = false;
          for (int j = 0; j < curr_group->lanes.size(); ++j) {
            has_prev_lane =
                AreLaneConnect(curr_group, next_group, j, i,
                               &curr_group_next_lane, &next_group_prev_lane) ||
                has_prev_lane;
          }
          double next_len = CalcLaneLength(lane_in_next);
          bool next_long_enough = next_len > kSplitLengthThreshold;
          if (!has_prev_lane && next_long_enough &&
              lane_in_next->center_line_pts.size() >= 2) {
            const float kSplitDistThreshold = 3.5;
            Eigen::Vector2f next_pt0(
                lane_in_next->center_line_pts.at(0).pt.x(),
                lane_in_next->center_line_pts.at(0).pt.y());
            Eigen::Vector2f next_pt1(
                lane_in_next->center_line_pts.at(1).pt.x(),
                lane_in_next->center_line_pts.at(1).pt.y());
            if (i == 0 && (!curr_group->lanes.empty()) &&
                (!curr_group->lanes.front()->center_line_pts.empty())) {
              auto& lane_in_curr = curr_group->lanes.at(0);
              Eigen::Vector2f curr_pt(
                  lane_in_curr->center_line_pts.back().pt.x(),
                  lane_in_curr->center_line_pts.back().pt.y());
              float dist = PointToVectorDist(next_pt0, next_pt1, curr_pt);
              if (dist < kSplitDistThreshold &&
                  IsAccessLane(lane_in_curr, lane_in_next)) {
                next_group_prev_lane[i].emplace_back(0);
                curr_group_next_lane[0].emplace_back(i);
              }
            } else if (i == static_cast<int>(next_group->lanes.size()) - 1 &&
                       (!curr_group->lanes.empty()) &&
                       (!curr_group->lanes.back()->center_line_pts.empty())) {
              auto& lane_in_curr = curr_group->lanes.back();
              Eigen::Vector2f curr_pt(
                  lane_in_curr->center_line_pts.back().pt.x(),
                  lane_in_curr->center_line_pts.back().pt.y());
              float dist = PointToVectorDist(next_pt0, next_pt1, curr_pt);
              if (dist < kSplitDistThreshold &&
                  IsAccessLane(lane_in_curr, lane_in_next)) {
                next_group_prev_lane[i].emplace_back(
                    static_cast<int>(curr_group->lanes.size()) - 1);
                curr_group_next_lane
                    [static_cast<int>(curr_group->lanes.size()) - 1]
                        .emplace_back(i);
              }
            }
          }
        }
      }
    }
    for (auto& iter : next_group_prev_lane) {
      if (iter.second.size() > 1) {
        int best = iter.second[0];
        for (auto& prev_lane_idx : iter.second) {
          if (best == prev_lane_idx) {
            continue;
          }
          auto& lane_in_curr = curr_group->lanes[prev_lane_idx];
          auto& lane_best = curr_group->lanes[best];
          auto& lane_in_next = next_group->lanes[iter.first];
          if (curr_group_next_lane[prev_lane_idx].size() == 1 &&
              curr_group_next_lane[best].size() > 1) {
            best = prev_lane_idx;
          } else if ((curr_group_next_lane[prev_lane_idx].size() > 1 &&
                      curr_group_next_lane[best].size() > 1) ||
                     (curr_group_next_lane[prev_lane_idx].size() == 1 &&
                      curr_group_next_lane[best].size() == 1)) {
            size_t sizet1 = lane_in_curr->center_line_pts.size();
            size_t sizet2 = lane_best->center_line_pts.size();
            if (!lane_in_curr->center_line_param.empty() &&
                !lane_best->center_line_param.empty() &&
                !lane_in_next->center_line_param_front.empty()) {
              float angle1 =
                  Calculate2CenterlineAngle(lane_in_next, lane_in_curr, sizet1);
              float angle2 =
                  Calculate2CenterlineAngle(lane_in_next, lane_best, sizet2);
              float dis1 =
                  CalculatePoint2CenterLine(lane_in_next, lane_in_curr);
              float dis2 = CalculatePoint2CenterLine(lane_in_next, lane_best);
              if (angle2 > angle1 + 1 * 180 / pi_ && dis2 > dis1 + 0.2) {
                best = prev_lane_idx;
              }
            } else {
              float dis_pt =
                  CalculateDistPt(lane_in_next, lane_in_curr, sizet1);
              float dis_pt2 = CalculateDistPt(lane_in_next, lane_best, sizet2);
              if (dis_pt < dis_pt2) {
                best = prev_lane_idx;
              }
            }
          }
        }
        for (auto& prev_lane_idx : iter.second) {
          if (prev_lane_idx == best) {
            continue;
          }
          for (auto iter_prev = curr_group_next_lane[prev_lane_idx].begin();
               iter_prev != curr_group_next_lane[prev_lane_idx].end();
               ++iter_prev) {
            if (*iter_prev == iter.first) {
              curr_group_next_lane[prev_lane_idx].erase(iter_prev);
              break;
            }
          }
        }
        iter.second.clear();
        iter.second.emplace_back(best);
      }
    }
  }
  NextGroupLaneConnect(curr_group, next_group, curr_group_next_lane);
  // for (auto& lane_in_curr : curr_group->lanes) {
  //   if (lane_in_curr->next_lane_str_id_with_group.empty()) {
  //     int trackid_same_find = 0;
  //     for (auto& lane_in_next : next_group->lanes) {
  //       if (lane_in_next->left_boundary->id ==
  //               lane_in_curr->left_boundary->id ||
  //           lane_in_next->right_boundary->id ==
  //               lane_in_curr->right_boundary->id) {
  //         BuildVirtualProSucLane(lane_in_curr, lane_in_next, 5.0);
  //         // lane_in_curr->next_lane_str_id_with_group
  //         trackid_same_find = 1;
  //         break;
  //       }
  //     }
  //     if (trackid_same_find == 0) {
  //       for (auto& lane_in_next : next_group->lanes) {
  //         size_t sizet = lane_in_curr->center_line_pts.size();
  //         float dis_pt =
  //             pow(lane_in_next->center_line_pts[0].pt.y() -
  //                     lane_in_curr->center_line_pts[sizet - 1].pt.y(),
  //                 2) +
  //             pow(lane_in_next->center_line_pts[0].pt.x() -
  //                     lane_in_curr->center_line_pts[sizet - 1].pt.x(),
  //                 2);
  //         float angel_thresh = 0.0, dis_thresh = 0.0;
  //         if (!lane_in_curr->center_line_param.empty() &&
  //             !lane_in_next->center_line_pts.empty()) {
  //           if (dis_pt > 10000) {
  //             // 差的太远不计算
  //             continue;
  //           } else if (dis_pt > 400) {
  //             angel_thresh = 2;
  //             dis_thresh = 3;
  //           } else {
  //             angel_thresh = 10;
  //             dis_thresh = 1;
  //           }
  //           float dis = abs(lane_in_next->center_line_pts[0].pt.y() -
  //                           (lane_in_curr->center_line_param[0] +
  //                             lane_in_curr->center_line_param[1] *
  //                                 lane_in_next->center_line_pts[0].pt.x())) /
  //                       sqrt(pow(lane_in_curr->center_line_param[0], 2) +
  //                             pow(lane_in_curr->center_line_param[1], 2));
  //           float angle =
  //               atan((lane_in_next->center_line_pts[0].pt.y() -
  //                     lane_in_curr->center_line_pts[sizet - 1].pt.y()) /
  //                     (lane_in_next->center_line_pts[0].pt.x() -
  //                     lane_in_curr->center_line_pts[sizet - 1].pt.x())) -
  //               atan(lane_in_curr->center_line_param[1]);
  //           // HLOG_ERROR << "angle = "<<angle*180/pi_;
  //           if ((dis < dis_thresh && abs(angle) * 180 / pi_ < angel_thresh))
  //           {
  //             //! TBD:
  //             //!
  //             这里直接把后一个lane中心线的第一个点加到前一个lane中心线的末尾，
  //             //!
  //             后续需要考虑某些异常情况，比如后一个lane中心线的第一个点在前一个lane中心线最后
  //             //!
  //             一个点的后方，这样直连就导致整个中心线往后折返了；以及还要考虑横向偏移较大时不平
  //             //! 滑的问题
  //             BuildVirtualProSucLane(lane_in_curr, lane_in_next, dis_pt);
  //             break;
  //           }
  //         }
  //         if (dis_pt < 9 ||
  //             lane_in_curr->lanepos_id == lane_in_next->lanepos_id) {
  //           BuildVirtualProSucLane(lane_in_curr, lane_in_next, dis_pt);
  //           break;
  //         }
  //       }
  //     }
  //   }
  // }
}
bool GroupMap::IsZebraIn(const Eigen::Vector2f& curr_start_pl,
                         const Eigen::Vector2f& curr_start_pr,
                         const Eigen::Vector2f& next_start_pl,
                         const Eigen::Vector2f& next_start_pr) {
  for (auto zebra : zebra_) {
    Eigen::Vector2f zebra_center(0, 0);
    for (auto pt : zebra.second->polygon.points) {
      zebra_center += pt.head<2>();
    }
    zebra_center /= static_cast<float>(zebra.second->polygon.points.size());
    if (PointInVectorSide(curr_start_pr, curr_start_pl, zebra_center) >= 0 &&
        PointInVectorSide(next_start_pr, next_start_pl, zebra_center) <= 0) {
      return true;
    }
  }
  return false;
}

bool GroupMap::IsVehicleInJunction(Group::Ptr curr_group,
                                   Group::Ptr next_group) {
  if (curr_group->group_segments.empty() ||
      next_group->group_segments.empty()) {
    return false;
  }
  bool veh_in_this_junction = false;
  // 判断车是否在curr和next group范围内，如果是就认为在路口内
  auto curr_grp_start_slice = curr_group->group_segments.front()->start_slice;
  auto curr_grp_end_slice = curr_group->group_segments.back()->end_slice;
  auto next_grp_start_slice = next_group->group_segments.front()->start_slice;
  Eigen::Vector2f curr_start_pl(curr_grp_start_slice.pl.x(),
                                curr_grp_start_slice.pl.y());
  Eigen::Vector2f curr_start_pr(curr_grp_start_slice.pr.x(),
                                curr_grp_start_slice.pr.y());
  Eigen::Vector2f curr_end_pl(curr_grp_end_slice.pl.x(),
                              curr_grp_end_slice.pl.y());
  Eigen::Vector2f curr_end_pr(curr_grp_end_slice.pr.x(),
                              curr_grp_end_slice.pr.y());
  Eigen::Vector2f next_start_pl(next_grp_start_slice.pl.x(),
                                next_grp_start_slice.pl.y());
  Eigen::Vector2f next_start_pr(next_grp_start_slice.pr.x(),
                                next_grp_start_slice.pr.y());
  Eigen::Vector2f curr_pos(0, 0);
  if (PointInVectorSide(curr_start_pr, curr_start_pl, curr_pos) >= 0 &&
      PointInVectorSide(next_start_pr, next_start_pl, curr_pos) <= 0) {
    veh_in_this_junction = true;
  }
  //! 注意： 这里判断车与curr
  //! group的距离是否小于规控要求的长度，如果小于表示前方太短了，
  //! 此时也认为是在路口内，这样就可以将next group里车道删除，正常的使用curr
  //! group向前预测
  if (PointInVectorSide(curr_end_pr, curr_end_pl, curr_pos) <= 0 &&
      PointToVectorDist(curr_end_pr, curr_end_pl, curr_pos) <
          conf_.predict_farthest_dist) {
    veh_in_this_junction = true;
  }

  return veh_in_this_junction;
}
void GroupMap::GenerateTransitionLaneToPo3(Lane::Ptr lane_in_curr,
                                           Lane::Ptr lane_in_next,
                                           Lane::Ptr transition_lane) {
  transition_lane->str_id = lane_in_next->str_id;
  transition_lane->lanepos_id = lane_in_next->lanepos_id;
  size_t index = lane_in_curr->str_id_with_group.find(":");
  transition_lane->str_id_with_group =
      lane_in_curr->str_id_with_group.substr(0, index) +
      "V:" + transition_lane->str_id;
  transition_lane->center_line_param = lane_in_curr->center_line_param;
  transition_lane->center_line_param_front = lane_in_curr->center_line_param;
  transition_lane->prev_lane_str_id_with_group.emplace_back(
      lane_in_curr->str_id_with_group);
  lane_in_curr->next_lane_str_id_with_group.emplace_back(
      transition_lane->str_id_with_group);
  transition_lane->next_lane_str_id_with_group.emplace_back(
      lane_in_next->str_id_with_group);
  lane_in_next->prev_lane_str_id_with_group.emplace_back(
      transition_lane->str_id_with_group);
}
void GroupMap::GenerateLane(Lane::Ptr lane_next, Lane::Ptr transition_lane) {
  LineSegment& left_bound = *transition_lane->left_boundary;
  left_bound.id = lane_next->left_boundary->id;
  left_bound.lanepos = lane_next->left_boundary->lanepos;
  left_bound.type = em::LaneType_DASHED;
  left_bound.color = em::WHITE;
  left_bound.isego = lane_next->left_boundary->isego;
  left_bound.is_near_road_edge = lane_next->left_boundary->is_near_road_edge;
  left_bound.mean_end_heading = lane_next->left_boundary->mean_end_heading;
  left_bound.mean_end_heading_std_dev =
      lane_next->left_boundary->mean_end_heading_std_dev;
  left_bound.mean_end_interval = lane_next->left_boundary->mean_end_interval;
  for (const auto& delete_id : lane_next->left_boundary->deteled_ids) {
    left_bound.deteled_ids.emplace_back(delete_id);
  }
  Point left_pt_pred(VIRTUAL, 0.0, 0.0, 0.0);
  left_pt_pred.pt =
      lane_next->left_boundary->pts[0].pt - lane_next->center_line_pts[0].pt;

  LineSegment& right_bound = *transition_lane->right_boundary;
  right_bound.id = lane_next->right_boundary->id;
  right_bound.lanepos = lane_next->right_boundary->lanepos;
  right_bound.type =
      em::LaneType_DASHED;        // lane_in_curr->right_boundary->type;
  right_bound.color = em::WHITE;  // lane_in_curr->right_boundary->color;
  right_bound.isego = lane_next->right_boundary->isego;
  right_bound.is_near_road_edge = lane_next->right_boundary->is_near_road_edge;
  right_bound.mean_end_heading = lane_next->right_boundary->mean_end_heading;
  right_bound.pred_end_heading = lane_next->right_boundary->pred_end_heading;
  right_bound.mean_end_heading_std_dev =
      lane_next->right_boundary->mean_end_heading_std_dev;
  right_bound.mean_end_interval = lane_next->right_boundary->mean_end_interval;
  for (const auto& delete_id : lane_next->right_boundary->deteled_ids) {
    right_bound.deteled_ids.emplace_back(delete_id);
  }
  Point right_pt_pred(VIRTUAL, 0.0, 0.0, 0.0);
  right_pt_pred.pt =
      lane_next->right_boundary->pts[0].pt - lane_next->center_line_pts[0].pt;

  std::vector<Point>& ctr_pts = transition_lane->center_line_pts;

  Point center_pt_pred(VIRTUAL, 0.0, 0.0, 0.0);

  em::Point vec_left = lane_next->left_boundary->pts[0].pt - left_pt_pred.pt;
  while (vec_left.norm() > 1.0) {
    left_bound.pts.emplace_back(left_pt_pred);
    float pre_x = left_pt_pred.pt.x() + vec_left.x() / vec_left.norm();
    float pre_y = left_pt_pred.pt.y() + vec_left.y() / vec_left.norm();
    left_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    vec_left = lane_next->left_boundary->pts[0].pt - left_pt_pred.pt;
  }

  em::Point vec_right = lane_next->right_boundary->pts[0].pt - right_pt_pred.pt;
  while (vec_right.norm() > 1.0) {
    right_bound.pts.emplace_back(right_pt_pred);
    float pre_x = right_pt_pred.pt.x() + vec_right.x() / vec_right.norm();
    float pre_y = right_pt_pred.pt.y() + vec_right.y() / vec_right.norm();
    right_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    vec_right = lane_next->right_boundary->pts[0].pt - right_pt_pred.pt;
  }

  em::Point vec_center = lane_next->center_line_pts[0].pt - center_pt_pred.pt;
  while (vec_center.norm() > 1.0) {
    ctr_pts.emplace_back(center_pt_pred);
    float pre_x = center_pt_pred.pt.x() + vec_center.x() / vec_center.norm();
    float pre_y = center_pt_pred.pt.y() + vec_center.y() / vec_center.norm();
    center_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    vec_center = lane_next->center_line_pts[0].pt - center_pt_pred.pt;
  }
  ctr_pts.emplace_back(lane_next->center_line_pts.front());

  transition_lane->str_id = lane_next->str_id;
  transition_lane->lanepos_id = lane_next->lanepos_id;
  size_t index = lane_next->str_id_with_group.find(":");
  transition_lane->str_id_with_group =
      lane_next->str_id_with_group.substr(0, index) +
      "V:" + transition_lane->str_id;
  transition_lane->center_line_param = lane_next->center_line_param;
  transition_lane->center_line_param_front = lane_next->center_line_param;
  transition_lane->next_lane_str_id_with_group.emplace_back(
      lane_next->str_id_with_group);
  lane_next->prev_lane_str_id_with_group.emplace_back(
      transition_lane->str_id_with_group);
}
void GroupMap::FindSatisefyNextLane(Group::Ptr next_group,
                                    const float& dist_to_slice,
                                    std::vector<Lane::Ptr>* satisfy_next_lane) {
  Eigen::Vector2f thresh_v(std::cos(conf_.junction_heading_diff),
                           std::sin(conf_.junction_heading_diff));
  Eigen::Vector2f n(1, 0);  // 车前向量
  if (ego_curr_lane_ != nullptr &&
      ego_curr_lane_->center_line_pts.back().pt.x() > 0.0 &&
      !ego_curr_lane_->center_line_param.empty()) {
    // 对场景分类讨论。过路口前应该按照所在道的朝向来选择连接；在路口中可以按照车头方向来选择。
    n = Eigen::Vector2f(1, ego_curr_lane_->center_line_param[1]);
    n.normalize();
  }
  float thresh_len = std::abs(thresh_v.transpose() * n);
  for (auto& next_lane : next_group->lanes) {
    if (!next_lane->right_boundary->isego || !next_lane->left_boundary->isego) {
      continue;
    }
    Eigen::Vector2f p0(0, 0);
    Eigen::Vector2f p1(next_lane->center_line_pts.front().pt.x(),
                       next_lane->center_line_pts.front().pt.y());
    Eigen::Vector2f v = p1 - p0;
    v.normalize();
    float len = std::abs(v.transpose() * n);
    if (len >= thresh_len) {
      satisfy_next_lane->emplace_back(next_lane);
    }
  }

  const float kOffsetThreshold = 3.5 * 0.7;  // 0.7个3.5米车道宽度
  float check_offset_thresh = -10000;
  if (conf_.junction_heading_diff > 0.001) {
    check_offset_thresh =
        kOffsetThreshold / std::tan(conf_.junction_heading_diff);
  }
  // 如果此时距离足够近，并且通过角度未找到合适的next
  // lane，此时通过找最小的横向偏移来确定next lane
  if ((*satisfy_next_lane).empty() && dist_to_slice <= check_offset_thresh) {
    for (auto& next_lane : next_group->lanes) {
      if (!next_lane->right_boundary->isego ||
          !next_lane->left_boundary->isego) {
        continue;
      }
      float offset = std::abs(next_lane->center_line_pts.front().pt.y());
      if (offset <= kOffsetThreshold) {
        satisfy_next_lane->emplace_back(next_lane);
      }
    }
  }
  // 比较当前时刻的下一条lane信息和历史时刻的lane信息
  // 1.没有找到satisfy_next_lane
  if ((*satisfy_next_lane).empty()) {
    for (auto& his_lane : is_cross_.next_satisefy_lane_seg) {
      size_t index = his_lane.find("_");
      int32_t left = atoi(his_lane.substr(0, index).c_str());
      int32_t right = atoi(his_lane.substr(index + 1).c_str());
      for (auto& lane_ : next_group->lanes) {
        if (!lane_->right_boundary->isego || !lane_->left_boundary->isego) {
          continue;
        }
        if (LineIdConsistant(lane_->left_boundary, left) &&
            LineIdConsistant(lane_->right_boundary, right)) {
          satisfy_next_lane->emplace_back(lane_);
        }
      }
    }
    return;
  }
  // 2.第一帧连接的best_lane 存储一下左右边线并返回
  for (auto& sat_lane : *satisfy_next_lane) {
    if (!sat_lane->right_boundary->isego || !sat_lane->left_boundary->isego) {
      continue;
    }
    int exist = 0;
    for (auto& his_lane : is_cross_.next_satisefy_lane_seg) {
      size_t index = his_lane.find("_");
      int32_t left = atoi(his_lane.substr(0, index).c_str());
      int32_t right = atoi(his_lane.substr(index + 1).c_str());
      if (LineIdConsistant(sat_lane->left_boundary, left) &&
          LineIdConsistant(sat_lane->right_boundary, right)) {
        exist = 1;
        break;
      }
    }
    if (!exist) {
      is_cross_.next_satisefy_lane_seg.insert(sat_lane->str_id);
    }
  }
  // history 的lane存储到satisfy里
  for (auto& his_lane : is_cross_.next_satisefy_lane_seg) {
    int exist = 0;
    size_t index = his_lane.find("_");
    int32_t left = atoi(his_lane.substr(0, index).c_str());
    int32_t right = atoi(his_lane.substr(index + 1).c_str());
    for (auto& sat_lane : *satisfy_next_lane) {
      if (!sat_lane->right_boundary->isego || !sat_lane->left_boundary->isego) {
        continue;
      }
      if (LineIdConsistant(sat_lane->left_boundary, left) &&
          LineIdConsistant(sat_lane->right_boundary, right)) {
        exist = 1;
        break;
      }
    }
    if (!exist) {
      for (auto& lane_ : next_group->lanes) {
        if (!lane_->right_boundary->isego || !lane_->left_boundary->isego) {
          continue;
        }
        if (LineIdConsistant(lane_->left_boundary, left) &&
            LineIdConsistant(lane_->right_boundary, right)) {
          satisfy_next_lane->emplace_back(lane_);
        }
      }
    }
  }
}

void GroupMap::GenerateAllSatisfyTransitionLane(
    Lane::Ptr lane_in_curr, std::vector<Lane::Ptr>* virtual_lanes,
    std::vector<Lane::Ptr> history_satisfy_lane_,
    float dist_to_next_group_slice) {
  // HLOG_ERROR << "dist_to_next_group_slice = " << dist_to_next_group_slice;
  if (is_cross_.is_crossing_ && is_cross_.along_path_dis_.norm() > 2.0 &&
      dist_to_next_group_slice > incross_before_virtual_lane_length_ +
                                     incross_before_virtual_lane_length_) {
    // &&dist_to_next_group_slice > conf_.incross_before_virtual_lane_length
    // 自车在路口里的连接策略
    Lane::Ptr transition_lane = std::make_shared<Lane>();
    transition_lane->left_boundary = std::make_shared<LineSegment>();
    transition_lane->right_boundary = std::make_shared<LineSegment>();
    GenerateTransitionLaneToBefore(lane_in_curr, transition_lane);
    (*virtual_lanes).emplace_back(transition_lane);
    Lane::Ptr transition_lane_after1 = std::make_shared<Lane>();
    transition_lane_after1->left_boundary = std::make_shared<LineSegment>();
    transition_lane_after1->right_boundary = std::make_shared<LineSegment>();
    auto lane_in_next = history_satisfy_lane_[0];
    GenerateTransitionLaneToAfter(transition_lane, lane_in_next,
                                  transition_lane_after1);
    GenerateTransitionLaneToPo(transition_lane, lane_in_next,
                               transition_lane_after1);
    (*virtual_lanes).emplace_back(transition_lane_after1);
    if (history_satisfy_lane_.size() > 1) {
      for (int i = 1; i < history_satisfy_lane_.size(); ++i) {
        Lane::Ptr transition_lane_after2 = std::make_shared<Lane>();
        transition_lane_after2->left_boundary = std::make_shared<LineSegment>();
        transition_lane_after2->right_boundary =
            std::make_shared<LineSegment>();
        auto lane_next_history = history_satisfy_lane_[i];
        GenerateTransitionLaneToAfter(transition_lane, lane_next_history,
                                      transition_lane_after2);
        GenerateTransitionLaneToPo2(transition_lane, lane_next_history,
                                    transition_lane_after2);
        (*virtual_lanes).emplace_back(transition_lane_after2);
      }
    }
  } else {
    // 自车快进入路口里的路口连接策略
    // HLOG_DEBUG << "路口连接策略";
    for (int i = 0; i < history_satisfy_lane_.size(); ++i) {
      Lane::Ptr transition_lane_after2 = std::make_shared<Lane>();
      transition_lane_after2->left_boundary = std::make_shared<LineSegment>();
      transition_lane_after2->right_boundary = std::make_shared<LineSegment>();
      auto lane_next_history = history_satisfy_lane_[i];
      GenerateTransitionLaneGeo(lane_in_curr, lane_next_history,
                                transition_lane_after2);
      GenerateTransitionLaneToPo3(lane_in_curr, lane_next_history,
                                  transition_lane_after2);
      if (transition_lane_after2->center_line_pts.size() > 1) {
        (*virtual_lanes).emplace_back(transition_lane_after2);
      }
    }
  }
}
void GroupMap::ExtendFrontCenterLine(std::vector<Group::Ptr>* groups) {
  for (auto& group : *groups) {
    if (group->group_segments.empty()) {
      continue;
    }
    for (const auto& lane : group->lanes) {
      if (!lane->prev_lane_str_id_with_group.empty() ||
          lane->center_line_param_front.empty() ||
          lane->center_line_pts.empty()) {
        continue;
      }
      float dis_x = lane->center_line_pts[0].pt.x() -
                    group->group_segments[0]->start_slice.po.x();
      // HLOG_ERROR << "lane name = " << lane->str_id_with_group
      //            << "dis_x = " << dis_x;
      // HLOG_ERROR << "lane->center_line_pts[0].pt.x() = "
      //            << lane->center_line_pts[0].pt.x()
      //            << " group->group_segments[0]->start_slice.po.x() = "
      //            << group->group_segments[0]->start_slice.po.x();
      if (dis_x < 0) {
        continue;
      }
      float x = lane->center_line_pts[0].pt.x() - dis_x;
      float y = lane->center_line_param_front[0] +
                x * lane->center_line_param_front[1];
      Point point_front(VIRTUAL, x, y, 0.0);
      lane->center_line_pts.insert(lane->center_line_pts.begin(), point_front);
    }
  }
}
void GroupMap::AddCrossLaneNeighbor(Group::Ptr cross_group,
                                    Group::Ptr next_group) {
  if (cross_group->lanes.size() < 2 || next_group->lanes.size() < 2) {
    return;
  }
  for (const auto& lane : next_group->lanes) {
    if (lane->prev_lane_str_id_with_group.size() > 1) {
      return;
    }
  }
  for (const auto& lane : cross_group->lanes) {
    if (lane->next_lane_str_id_with_group.size() > 1) {
      return;
    }
  }
  std::unordered_map<std::string, int>
      cross_group_lanes;  // cross_group->lane->str_id_with_group,
                          // cross_group->lanes's index
  std::unordered_map<std::string, int>
      next_group_lanes;  // next_group->lane->str_id_with_group,
                         // next_group->lane's index
  std::unordered_map<std::string, int>
      cross_group_next_lane;  // cross_group->lane->str_id_with_group,
                              // cross_group->lane's next_lane in
                              // next_group->lanes's index
  std::unordered_map<std::string, int>
      next_group_prev_lane;  // next_group->lane->str_id_with_group,
                             // next_group->lane's prev lane in
                             // cross_group->lane's index
  for (int index = 0; index < next_group->lanes.size(); ++index) {
    next_group_lanes[next_group->lanes[index]->str_id_with_group] = index;
  }
  for (int index = 0; index < cross_group->lanes.size(); ++index) {
    auto& curr_lane = cross_group->lanes[index];
    cross_group_lanes[curr_lane->str_id_with_group] = index;
    if (curr_lane->next_lane_str_id_with_group.size() == 1 &&
        next_group_lanes.find(curr_lane->next_lane_str_id_with_group[0]) !=
            next_group_lanes.end()) {
      cross_group_next_lane[curr_lane->str_id_with_group] =
          next_group_lanes[curr_lane->next_lane_str_id_with_group[0]];
    }
  }
  for (const auto& lane : next_group->lanes) {
    if (lane->prev_lane_str_id_with_group.size() == 1 &&
        cross_group_lanes.find(lane->prev_lane_str_id_with_group[0]) !=
            cross_group_lanes.end()) {
      next_group_prev_lane[lane->str_id_with_group] =
          cross_group_lanes[lane->prev_lane_str_id_with_group[0]];
    }
  }
  for (const auto& lane : cross_group->lanes) {
    auto& next_lane_index = cross_group_next_lane[lane->str_id_with_group];
    for (const auto& left_lane :
         next_group->lanes[next_lane_index]->left_lane_str_id_with_group) {
      if (next_group_prev_lane.find(left_lane) != next_group_prev_lane.end()) {
        lane->left_lane_str_id_with_group.emplace_back(
            cross_group->lanes[next_group_prev_lane[left_lane]]
                ->str_id_with_group);
      }
    }
    for (const auto& right_lane :
         next_group->lanes[next_lane_index]->right_lane_str_id_with_group) {
      if (next_group_prev_lane.find(right_lane) != next_group_prev_lane.end()) {
        lane->right_lane_str_id_with_group.emplace_back(
            cross_group->lanes[next_group_prev_lane[right_lane]]
                ->str_id_with_group);
      }
    }
  }
}
void GroupMap::RelateGroups(std::vector<Group::Ptr>* groups, double stamp) {
  std::vector<Group::Ptr> group_virtual;
  int erase_grp_idx = -1;
  if (is_cross_.is_crossing_ && groups->size() > 0 &&
      groups->front()->group_segments.size() > 1 &&
      groups->front()->group_segments[0]->start_slice.po.x() > 0.0) {
    auto& next_group = groups->at(0);
    // 路口太大了 本车道没有
    Lane::Ptr best_next_lane = nullptr;
    auto next_grp_start_slice = next_group->group_segments.front()->start_slice;
    Eigen::Vector2f next_start_pl(next_grp_start_slice.pl.x(),
                                  next_grp_start_slice.pl.y());
    Eigen::Vector2f next_start_pr(next_grp_start_slice.pr.x(),
                                  next_grp_start_slice.pr.y());
    Eigen::Vector2f curr_pos(0, 0);
    auto dist_to_slice =
        PointToVectorDist(next_start_pl, next_start_pr, curr_pos);
    FindBestNextLane(next_group, dist_to_slice, &best_next_lane);
    if (best_next_lane != nullptr) {
      Lane::Ptr transition_lane = std::make_shared<Lane>();
      transition_lane->left_boundary = std::make_shared<LineSegment>();
      transition_lane->right_boundary = std::make_shared<LineSegment>();
      GenerateLane(best_next_lane, transition_lane);
      std::vector<Lane::Ptr> virtual_lanes;
      virtual_lanes.emplace_back(transition_lane);
      BuildVirtualGroup(virtual_lanes, &group_virtual, stamp);
      groups->insert(groups->begin(), group_virtual[0]);
      group_virtual.clear();
    }
  }
  for (int grp_idx = 0; grp_idx < static_cast<int>(groups->size()) - 1;
       ++grp_idx) {
    std::vector<Lane::Ptr> virtual_lanes;
    auto& curr_group = groups->at(grp_idx);
    auto& next_group = groups->at(grp_idx + 1);
    if (curr_group->str_id.find("V") < curr_group->str_id.length() ||
        next_group->str_id.find("V") < next_group->str_id.length() ||
        curr_group->group_segments.size() < 2 ||
        next_group->group_segments.size() < 2 || curr_group->lanes.size() < 1 ||
        next_group->lanes.size() < 1) {
      continue;
    }
    // 判断curr_group的最后一个groupsegment的中心点和next_group的第一个groupsegment的中心点的距离
    auto curr_grp_gs_size = curr_group->group_segments.size();
    double group_distance =
        (curr_group->group_segments[curr_grp_gs_size - 1]->end_slice.po -
         next_group->group_segments.front()->start_slice.po)
            .norm();
    //
    double currgrp_nearest_mindis_to_nextgrp = DBL_MAX;
    for (auto& cur_group_lane : curr_group->lanes) {
      double calcu_dis = (cur_group_lane->center_line_pts.back().pt -
                          next_group->group_segments.front()->start_slice.po)
                             .norm();
      if (calcu_dis < currgrp_nearest_mindis_to_nextgrp) {
        currgrp_nearest_mindis_to_nextgrp = calcu_dis;
      }
    }

    if (group_distance > 10 && currgrp_nearest_mindis_to_nextgrp > 10) {
      // > 10m路口
      bool veh_in_this_junction = IsVehicleInJunction(curr_group, next_group);

      // 生成curr_group和next_group的中线的角度值。
      auto cur_group_v =
          (curr_group->group_segments[curr_grp_gs_size - 1]->end_slice.po -
           curr_group->group_segments[curr_grp_gs_size - 2]->end_slice.po)
              .normalized();
      auto next_group_v = (next_group->group_segments[1]->end_slice.po -
                           next_group->group_segments[0]->end_slice.po)
                              .normalized();
      if (cur_group_v.dot(next_group_v) > 0.707) {
        // 直行场景
        auto next_grp_start_slice =
            next_group->group_segments.front()->start_slice;
        Eigen::Vector2f next_start_pl(next_grp_start_slice.pl.x(),
                                      next_grp_start_slice.pl.y());
        Eigen::Vector2f next_start_pr(next_grp_start_slice.pr.x(),
                                      next_grp_start_slice.pr.y());
        Eigen::Vector2f curr_pos(0, 0);
        auto dist_to_slice =
            PointToVectorDist(next_start_pl, next_start_pr, curr_pos);
        //! 当前路口策略是：30内有可连接的车道就直接连上
        //! 为了能使用curr group的车道向前预测，这里把next
        //! group的索引标记出来，后面会将next group 里车道都删掉，这样curr
        //! group就变成最后一个group了，后续就能正常使用其向前预测了.
        // HLOG_INFO << "curr_group NAME = " << curr_group->str_id
        //           << "  dist_to_slice = " << dist_to_slice
        //           << "  veh_in_this_junction = " << veh_in_this_junction
        //           << "  next_group_name = " << next_group->str_id
        //           << "grp_idx = " << grp_idx;
        if (veh_in_this_junction) {
          Lane::Ptr ego_curr_lane = nullptr;
          FindNearestLaneToHisVehiclePosition(curr_group, &ego_curr_lane);
          ego_curr_lane_ = ego_curr_lane;
          // HLOG_DEBUG << "ego_curr_lane is " <<
          // ego_curr_lane->str_id_with_group;
          std::vector<Lane::Ptr> history_satisfy_lane_;
          FindSatisefyNextLane(next_group, dist_to_slice,
                               &history_satisfy_lane_);
          if (ego_curr_lane != nullptr && !history_satisfy_lane_.empty() &&
              (dist_to_slice <= conf_.next_group_max_distance ||
               is_cross_.is_connect_)) {
            is_cross_.is_connect_ = 1;
            if (curr_group->group_segments.back()->end_slice.po.x() < -2.0) {
              is_cross_.is_crossing_ = 1;
              is_cross_.along_path_dis_ =
                  curr_group->group_segments.back()->end_slice.po;
            } else {
              is_cross_.is_crossing_ = 0;
            }
            auto dist_to_next_group_x = next_grp_start_slice.po.x();
            GenerateAllSatisfyTransitionLane(ego_curr_lane, &virtual_lanes,
                                             history_satisfy_lane_,
                                             dist_to_next_group_x);

          } else if (ego_curr_lane == nullptr) {
            HLOG_ERROR << "ego_curr_lane nullptr";
            Lane::Ptr best_next_lane = nullptr;
            FindBestNextLane(next_group, dist_to_slice, &best_next_lane);
            if (best_next_lane != nullptr) {
              Lane::Ptr transition_lane = std::make_shared<Lane>();
              transition_lane->left_boundary = std::make_shared<LineSegment>();
              transition_lane->right_boundary = std::make_shared<LineSegment>();
              GenerateLane(best_next_lane, transition_lane);
              std::vector<Lane::Ptr> virtual_lanes;
              virtual_lanes.emplace_back(transition_lane);
              BuildVirtualGroup(virtual_lanes, &group_virtual, stamp);
              groups->insert(groups->begin() + grp_idx + 1, group_virtual[0]);
              virtual_lanes.clear();
              group_virtual.clear();
              grp_idx++;
            }
          } else if (history_satisfy_lane_.empty() ||
                     dist_to_slice > conf_.next_group_max_distance) {
            erase_grp_idx = grp_idx + 1;
            HLOG_WARN << "best_next_lane nullptr";
          }
          // Lane::Ptr best_next_lane = nullptr;
          // FindBestNextLane(next_group, dist_to_slice, &best_next_lane);
          // // 条件判断，分三种情况
          // if (ego_curr_lane != nullptr && best_next_lane != nullptr &&
          //     (dist_to_slice <= conf_.next_group_max_distance ||
          //      is_cross_.is_connect_)) {
          //   HLOG_DEBUG << "best_next_lane = "
          //             << best_next_lane->str_id_with_group;
          //   is_cross_.is_connect_ = 1;
          //   if (curr_group->group_segments.back()->end_slice.po.x() < -2.0) {
          //     is_cross_.is_crossing_ = 1;
          //   }
          //   GenerateTransitionLane(ego_curr_lane, best_next_lane,
          //                          &virtual_lanes);
          // } else if (ego_curr_lane == nullptr) {
          //   HLOG_ERROR << "ego_curr_lane nullptr";
          // } else if (best_next_lane == nullptr ||
          //            dist_to_slice > conf_.next_group_max_distance) {
          //   // 30m内没找到对应的可连接车道，直接退出
          //   erase_grp_idx = grp_idx + 1;
          //   HLOG_WARN << "best_next_lane nullptr";
          // }
        } else {
          // wait update
          // 对应应该是车已经走到了next_group里面，现在是默认是不连接
          HLOG_WARN << "vehicle is not in junction";
        }
      } else {
        // 转弯场景
        HLOG_WARN << "it is curve scene";
      }
    } else {
      // 小路口 < 10m，或者不是路口
      FindGroupNextLane(curr_group, next_group);
    }
    if (virtual_lanes.size() > 0) {
      // BuildVirtualGroup(virtual_lanes, &group_virtual, stamp);
      BuildVirtualGroup2(virtual_lanes, &group_virtual, stamp);
    }
  }

  // 删除待删除的group
  // if (erase_grp_idx > 0) {
  //   groups->erase(groups->begin() + erase_grp_idx, groups->end());
  //   if (!groups->empty()) {
  //     groups->back()->is_last_after_erased = true;
  //   }
  // }
  // 需要erase的线不直接删除，先标记为删除状态，在远端预测模块中删除该线。
  if (erase_grp_idx > 0) {
    auto startIter = groups->begin() + erase_grp_idx;
    // 遍历最后n个元素
    for (auto it = startIter; it != groups->end(); ++it) {
      (*it)->group_state = Group::GroupState::DELETE;
    }
  }

  // 增加虚拟车道到groups中
  for (int i = static_cast<int>(groups->size()) - 1; i >= 0; --i) {
    if (group_virtual.empty()) {
      break;
    }
    if (groups->at(i)->str_id + "VV" == group_virtual.back()->str_id) {
      groups->insert(groups->begin() + i + 1, group_virtual.back());
      group_virtual.pop_back();
      if (groups->size() > i + 2) {
        AddCrossLaneNeighbor(groups->at(i + 1), groups->at(i + 2));
      }
    }
    if (!group_virtual.empty() &&
        groups->at(i)->str_id + 'V' == group_virtual.back()->str_id) {
      groups->insert(groups->begin() + i + 1, group_virtual.back());
      group_virtual.pop_back();
      if (groups->size() > i + 2) {
        AddCrossLaneNeighbor(groups->at(i + 1), groups->at(i + 2));
      }
    }
  }
}

void GroupMap::GenerateTransitionLaneGeo(Lane::Ptr lane_in_curr,
                                         Lane::Ptr lane_in_next,
                                         Lane::Ptr transition_lane) {
  LineSegment& left_bound = *transition_lane->left_boundary;
  left_bound.id = lane_in_curr->left_boundary->id;
  left_bound.lanepos = lane_in_curr->left_boundary->lanepos;
  left_bound.type = em::LaneType_DASHED;  // lane_in_curr->left_boundary->type;
  left_bound.color = em::WHITE;           // lane_in_curr->left_boundary->color;
  left_bound.isego = lane_in_curr->left_boundary->isego;
  left_bound.is_near_road_edge = lane_in_curr->left_boundary->is_near_road_edge;
  left_bound.mean_end_heading = lane_in_curr->left_boundary->mean_end_heading;
  left_bound.pred_end_heading = lane_in_curr->left_boundary->pred_end_heading;
  left_bound.mean_end_heading_std_dev =
      lane_in_curr->left_boundary->mean_end_heading_std_dev;
  left_bound.mean_end_interval = lane_in_curr->left_boundary->mean_end_interval;
  for (const auto& delete_id : lane_in_curr->left_boundary->deteled_ids) {
    left_bound.deteled_ids.emplace_back(delete_id);
  }
  size_t index_left = lane_in_curr->left_boundary->pts.size();
  if (index_left < 1) {
    return;
  }
  Point left_pt_pred(VIRTUAL,
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
  LineSegment& right_bound = *transition_lane->right_boundary;
  right_bound.id = lane_in_curr->right_boundary->id;
  right_bound.lanepos = lane_in_curr->right_boundary->lanepos;
  right_bound.type =
      em::LaneType_DASHED;        // lane_in_curr->right_boundary->type;
  right_bound.color = em::WHITE;  // lane_in_curr->right_boundary->color;
  right_bound.isego = lane_in_curr->right_boundary->isego;
  right_bound.is_near_road_edge =
      lane_in_curr->right_boundary->is_near_road_edge;
  right_bound.mean_end_heading = lane_in_curr->right_boundary->mean_end_heading;
  right_bound.pred_end_heading = lane_in_curr->right_boundary->pred_end_heading;
  right_bound.mean_end_heading_std_dev =
      lane_in_curr->right_boundary->mean_end_heading_std_dev;
  right_bound.mean_end_interval =
      lane_in_curr->right_boundary->mean_end_interval;
  for (const auto& delete_id : lane_in_curr->right_boundary->deteled_ids) {
    right_bound.deteled_ids.emplace_back(delete_id);
  }
  size_t index_right = lane_in_curr->right_boundary->pts.size();
  if (index_right < 1) {
    return;
  }
  Point right_pt_pred(
      VIRTUAL, lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
      lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
      lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
  std::vector<Point>& ctr_pts = transition_lane->center_line_pts;
  size_t index_center = lane_in_curr->center_line_pts.size();
  if (index_center < 1) {
    return;
  }
  Point center_pt_pred(VIRTUAL,
                       lane_in_curr->center_line_pts[index_center - 1].pt.x(),
                       lane_in_curr->center_line_pts[index_center - 1].pt.y(),
                       lane_in_curr->center_line_pts[index_center - 1].pt.z());
  // if (is_cross_.is_crossing_ && is_cross_.along_path_dis_.norm() > 2.0) {
  //   float i = 0.0;
  //   float len = is_cross_.along_path_dis_.norm();
  //   left_bound.pts.emplace_back(left_pt_pred);
  //   right_bound.pts.emplace_back(right_pt_pred);
  //   ctr_pts.emplace_back(center_pt_pred);
  //   while (i < len - 1.0) {
  //     i = i + 1.0;
  //     float minus_x = 1 / len * is_cross_.along_path_dis_.x();
  //     float minus_y = 1 / len * is_cross_.along_path_dis_.y();
  //     float pre_x = left_pt_pred.pt.x() - minus_x;
  //     float pre_y = left_pt_pred.pt.y() - minus_y;
  //     left_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  //     left_bound.pts.emplace_back(left_pt_pred);

  //     pre_x = right_pt_pred.pt.x() - minus_x;
  //     pre_y = right_pt_pred.pt.y() - minus_y;
  //     right_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  //     right_bound.pts.emplace_back(right_pt_pred);

  //     pre_x = center_pt_pred.pt.x() - minus_x;
  //     pre_y = center_pt_pred.pt.y() - minus_y;
  //     center_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  //     ctr_pts.emplace_back(center_pt_pred);
  //   }
  //   i = 0.0;
  //   em::Point param_left =
  //       lane_in_next->left_boundary->pts[0].pt - left_bound.pts.back().pt;
  //   len = param_left.norm();
  //   while (i < len - 1.0) {
  //     left_bound.pts.emplace_back(left_pt_pred);
  //     i = i + 1.0;
  //     float pre_x = left_pt_pred.pt.x() + 1 / len * param_left.x();
  //     float pre_y = left_pt_pred.pt.y() + 1 / len * param_left.y();
  //     left_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  //   }
  //   if (left_bound.pts.empty()) {
  //     return;
  //   }
  //   i = 0.0;
  //   em::Point param_right =
  //       lane_in_next->right_boundary->pts[0].pt - right_bound.pts.back().pt;
  //   len = param_right.norm();
  //   while (i < len - 1.0) {
  //     right_bound.pts.emplace_back(right_pt_pred);
  //     i = i + 1.0;
  //     float pre_x = right_pt_pred.pt.x() + 1 / len * param_right.x();
  //     float pre_y = right_pt_pred.pt.y() + 1 / len * param_right.y();
  //     right_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  //   }
  //   if (right_bound.pts.empty()) {
  //     return;
  //   }
  //   i = 0.0;
  //   em::Point param_center =
  //       lane_in_next->center_line_pts[0].pt - ctr_pts.back().pt;
  //   len = param_center.norm();
  //   while (i < len - 1.0) {
  //     i = i + 1.0;
  //     ctr_pts.emplace_back(center_pt_pred);
  //     float pre_x = center_pt_pred.pt.x() + 1 / len * param_center.x();
  //     float pre_y = center_pt_pred.pt.y() + 1 / len * param_center.y();
  //     center_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  //   }
  //   ctr_pts.emplace_back(lane_in_next->center_line_pts.front());
  //   if (ctr_pts.empty()) {
  //     return;
  //   }

  // } else {
  std::vector<Point> tmp_pts;
  tmp_pts.emplace_back(lane_in_curr->left_boundary->pts[index_left - 1]);
  tmp_pts.emplace_back(lane_in_next->left_boundary->pts[0]);
  std::vector<double> param_left = FitLaneline(tmp_pts);
  while (!param_left.empty() && left_pt_pred.pt.x() < lane_in_next->left_boundary->pts[0].pt.x()) {
    left_bound.pts.emplace_back(left_pt_pred);
    float pre_x = left_pt_pred.pt.x() + 1.0;
    float pre_y = param_left[0] + param_left[1] * pre_x;
    left_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  }
  // HLOG_DEBUG << "left_bound.pts.size() = " << left_bound.pts.size();
  // if (left_bound.pts.empty()) {
  //   return;
  // }

  tmp_pts.clear();
  tmp_pts.emplace_back(lane_in_curr->right_boundary->pts[index_right - 1]);
  tmp_pts.emplace_back(lane_in_next->right_boundary->pts[0]);
  param_left = FitLaneline(tmp_pts);
  while (!param_left.empty() && right_pt_pred.pt.x() < lane_in_next->right_boundary->pts[0].pt.x()) {
    right_bound.pts.emplace_back(right_pt_pred);
    float pre_x = right_pt_pred.pt.x() + 1.0;
    float pre_y = param_left[0] + param_left[1] * pre_x;
    right_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  }
  // HLOG_DEBUG << "right_bound.pts.size() = " << right_bound.pts.size();
  // if (right_bound.pts.empty()) {
  //   return;
  // }
  tmp_pts.clear();
  tmp_pts.emplace_back(lane_in_curr->center_line_pts[index_center - 1]);
  tmp_pts.emplace_back(lane_in_next->center_line_pts[0]);
  param_left = FitLaneline(tmp_pts);
  while (!param_left.empty() && center_pt_pred.pt.x() < lane_in_next->center_line_pts[0].pt.x()) {
    ctr_pts.emplace_back(center_pt_pred);
    float pre_x = center_pt_pred.pt.x() + 1.0;
    float pre_y = param_left[0] + param_left[1] * pre_x;
    center_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  }
  ctr_pts.emplace_back(lane_in_next->center_line_pts.front());
  // HLOG_DEBUG << "ctr_pts.pts.size() = " << ctr_pts.size();
  if (ctr_pts.empty()) {
    return;
  }
  // }
}

void GroupMap::GenerateTransitionLaneToPo(Lane::Ptr lane_in_curr,
                                          Lane::Ptr lane_in_next,
                                          Lane::Ptr transition_lane) {
  transition_lane->str_id = lane_in_curr->str_id;
  transition_lane->lanepos_id = lane_in_curr->lanepos_id;
  size_t index = lane_in_curr->str_id_with_group.find(":");
  transition_lane->str_id_with_group =
      lane_in_curr->str_id_with_group.substr(0, index) +
      "V:" + transition_lane->str_id;
  transition_lane->center_line_param = lane_in_curr->center_line_param;
  transition_lane->center_line_param_front = lane_in_curr->center_line_param;
  transition_lane->prev_lane_str_id_with_group.emplace_back(
      lane_in_curr->str_id_with_group);
  lane_in_curr->next_lane_str_id_with_group.emplace_back(
      transition_lane->str_id_with_group);
  transition_lane->next_lane_str_id_with_group.emplace_back(
      lane_in_next->str_id_with_group);
  lane_in_next->prev_lane_str_id_with_group.emplace_back(
      transition_lane->str_id_with_group);
}

void GroupMap::GenerateTransitionLaneToPo2(Lane::Ptr lane_in_curr,
                                           Lane::Ptr lane_in_next,
                                           Lane::Ptr transition_lane) {
  transition_lane->str_id = lane_in_next->str_id;
  transition_lane->lanepos_id = lane_in_next->lanepos_id;
  size_t index = lane_in_curr->str_id_with_group.find("V");
  transition_lane->str_id_with_group =
      lane_in_curr->str_id_with_group.substr(0, index) +
      "2VV:" + transition_lane->str_id;
  transition_lane->center_line_param = lane_in_curr->center_line_param;
  transition_lane->center_line_param_front = lane_in_curr->center_line_param;
  transition_lane->prev_lane_str_id_with_group.emplace_back(
      lane_in_curr->str_id_with_group);
  lane_in_curr->next_lane_str_id_with_group.emplace_back(
      transition_lane->str_id_with_group);
  transition_lane->next_lane_str_id_with_group.emplace_back(
      lane_in_next->str_id_with_group);
  lane_in_next->prev_lane_str_id_with_group.emplace_back(
      transition_lane->str_id_with_group);
}

void GroupMap::GenerateTransitionLaneToBefore(Lane::Ptr lane_in_curr,
                                              Lane::Ptr transition_lane) {
  LineSegment& left_bound = *transition_lane->left_boundary;
  left_bound.id = lane_in_curr->left_boundary->id;
  left_bound.lanepos = lane_in_curr->left_boundary->lanepos;
  left_bound.type = em::LaneType_DASHED;  // lane_in_curr->left_boundary->type;
  left_bound.color = em::WHITE;           // lane_in_curr->left_boundary->color;
  left_bound.isego = lane_in_curr->left_boundary->isego;
  left_bound.is_near_road_edge = lane_in_curr->left_boundary->is_near_road_edge;
  left_bound.mean_end_heading = lane_in_curr->left_boundary->mean_end_heading;
  left_bound.mean_end_heading_std_dev =
      lane_in_curr->left_boundary->mean_end_heading_std_dev;
  left_bound.mean_end_interval = lane_in_curr->left_boundary->mean_end_interval;
  for (const auto& delete_id : lane_in_curr->left_boundary->deteled_ids) {
    left_bound.deteled_ids.emplace_back(delete_id);
  }
  size_t index_left = lane_in_curr->left_boundary->pts.size();
  Point left_pt_pred(VIRTUAL,
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
  LineSegment& right_bound = *transition_lane->right_boundary;
  right_bound.id = lane_in_curr->right_boundary->id;
  right_bound.lanepos = lane_in_curr->right_boundary->lanepos;
  right_bound.type =
      em::LaneType_DASHED;        // lane_in_curr->right_boundary->type;
  right_bound.color = em::WHITE;  // lane_in_curr->right_boundary->color;
  right_bound.isego = lane_in_curr->right_boundary->isego;
  right_bound.is_near_road_edge =
      lane_in_curr->right_boundary->is_near_road_edge;
  right_bound.mean_end_heading = lane_in_curr->right_boundary->mean_end_heading;
  right_bound.mean_end_heading_std_dev =
      lane_in_curr->right_boundary->mean_end_heading_std_dev;
  right_bound.mean_end_interval =
      lane_in_curr->right_boundary->mean_end_interval;
  for (const auto& delete_id : lane_in_curr->right_boundary->deteled_ids) {
    right_bound.deteled_ids.emplace_back(delete_id);
  }
  size_t index_right = lane_in_curr->right_boundary->pts.size();
  Point right_pt_pred(
      VIRTUAL, lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
      lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
      lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
  std::vector<Point>& ctr_pts = transition_lane->center_line_pts;
  size_t index_center = lane_in_curr->center_line_pts.size();
  Point center_pt_pred(VIRTUAL,
                       lane_in_curr->center_line_pts[index_center - 1].pt.x(),
                       lane_in_curr->center_line_pts[index_center - 1].pt.y(),
                       lane_in_curr->center_line_pts[index_center - 1].pt.z());
  float i = 0.0;
  is_cross_.along_path_dis_.x() =
      is_cross_.along_path_dis_.x() -
      incross_before_virtual_lane_length_;  // conf_.incross_before_virtual_lane_length
  float len = is_cross_.along_path_dis_.norm();
  left_bound.pts.emplace_back(left_pt_pred);
  right_bound.pts.emplace_back(right_pt_pred);
  ctr_pts.emplace_back(center_pt_pred);
  while (i < len - 1.0) {
    i = i + 1.0;
    float minus_x = 1 / len * is_cross_.along_path_dis_.x();
    float minus_y = 1 / len * is_cross_.along_path_dis_.y();
    float pre_x = left_pt_pred.pt.x() - minus_x;
    float pre_y = left_pt_pred.pt.y() - minus_y;
    left_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    left_bound.pts.emplace_back(left_pt_pred);

    pre_x = right_pt_pred.pt.x() - minus_x;
    pre_y = right_pt_pred.pt.y() - minus_y;
    right_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    right_bound.pts.emplace_back(right_pt_pred);

    pre_x = center_pt_pred.pt.x() - minus_x;
    pre_y = center_pt_pred.pt.y() - minus_y;
    center_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    ctr_pts.emplace_back(center_pt_pred);
  }
  if (left_bound.pts.empty() || right_bound.pts.empty() || ctr_pts.empty()) {
    return;
  }
  transition_lane->str_id = lane_in_curr->str_id;
  transition_lane->lanepos_id = lane_in_curr->lanepos_id;
  size_t index = lane_in_curr->str_id_with_group.find(":");
  transition_lane->str_id_with_group =
      lane_in_curr->str_id_with_group.substr(0, index) +
      "V:" + transition_lane->str_id;
  transition_lane->center_line_param = lane_in_curr->center_line_param;
  transition_lane->center_line_param_front = lane_in_curr->center_line_param;
  transition_lane->prev_lane_str_id_with_group.emplace_back(
      lane_in_curr->str_id_with_group);
  lane_in_curr->next_lane_str_id_with_group.emplace_back(
      transition_lane->str_id_with_group);
}
void GroupMap::GenerateTransitionLaneToAfter(Lane::Ptr lane_in_curr,
                                             Lane::Ptr lane_in_next,
                                             Lane::Ptr transition_lane) {
  LineSegment& left_bound = *transition_lane->left_boundary;
  left_bound.id = lane_in_curr->left_boundary->id;
  left_bound.lanepos = lane_in_curr->left_boundary->lanepos;
  left_bound.type = em::LaneType_DASHED;  // lane_in_curr->left_boundary->type;
  left_bound.color = em::WHITE;           // lane_in_curr->left_boundary->color;
  left_bound.isego = lane_in_curr->left_boundary->isego;
  left_bound.is_near_road_edge = lane_in_curr->left_boundary->is_near_road_edge;
  left_bound.mean_end_heading = lane_in_curr->left_boundary->mean_end_heading;
  left_bound.mean_end_heading_std_dev =
      lane_in_curr->left_boundary->mean_end_heading_std_dev;
  left_bound.mean_end_interval = lane_in_curr->left_boundary->mean_end_interval;
  for (const auto& delete_id : lane_in_curr->left_boundary->deteled_ids) {
    left_bound.deteled_ids.emplace_back(delete_id);
  }
  size_t index_left = lane_in_curr->left_boundary->pts.size();
  Point left_pt_pred(VIRTUAL,
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
  LineSegment& right_bound = *transition_lane->right_boundary;
  right_bound.id = lane_in_curr->right_boundary->id;
  right_bound.lanepos = lane_in_curr->right_boundary->lanepos;
  right_bound.type =
      em::LaneType_DASHED;        // lane_in_curr->right_boundary->type;
  right_bound.color = em::WHITE;  // lane_in_curr->right_boundary->color;
  right_bound.isego = lane_in_curr->right_boundary->isego;
  right_bound.is_near_road_edge =
      lane_in_curr->right_boundary->is_near_road_edge;
  right_bound.mean_end_heading = lane_in_curr->right_boundary->mean_end_heading;
  right_bound.mean_end_heading_std_dev =
      lane_in_curr->right_boundary->mean_end_heading_std_dev;
  right_bound.mean_end_interval =
      lane_in_curr->right_boundary->mean_end_interval;
  for (const auto& delete_id : lane_in_curr->right_boundary->deteled_ids) {
    right_bound.deteled_ids.emplace_back(delete_id);
  }
  size_t index_right = lane_in_curr->right_boundary->pts.size();
  Point right_pt_pred(
      VIRTUAL, lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
      lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
      lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
  std::vector<Point>& ctr_pts = transition_lane->center_line_pts;
  size_t index_center = lane_in_curr->center_line_pts.size();
  Point center_pt_pred(VIRTUAL,
                       lane_in_curr->center_line_pts[index_center - 1].pt.x(),
                       lane_in_curr->center_line_pts[index_center - 1].pt.y(),
                       lane_in_curr->center_line_pts[index_center - 1].pt.z());
  left_bound.pts.emplace_back(left_pt_pred);
  float i = 0.0;
  em::Point param_left =
      lane_in_next->left_boundary->pts[0].pt - left_bound.pts.back().pt;
  float len = param_left.norm();
  while (i < len - 1.0) {
    i = i + 1.0;
    float pre_x = left_pt_pred.pt.x() + 1 / len * param_left.x();
    float pre_y = left_pt_pred.pt.y() + 1 / len * param_left.y();
    left_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    left_bound.pts.emplace_back(left_pt_pred);
  }
  if (left_bound.pts.empty()) {
    return;
  }
  right_bound.pts.emplace_back(right_pt_pred);
  i = 0.0;
  em::Point param_right =
      lane_in_next->right_boundary->pts[0].pt - right_bound.pts.back().pt;
  len = param_right.norm();
  while (i < len - 1.0) {
    i = i + 1.0;
    float pre_x = right_pt_pred.pt.x() + 1 / len * param_right.x();
    float pre_y = right_pt_pred.pt.y() + 1 / len * param_right.y();
    right_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    right_bound.pts.emplace_back(right_pt_pred);
  }
  if (right_bound.pts.empty()) {
    return;
  }
  ctr_pts.emplace_back(center_pt_pred);
  i = 0.0;
  em::Point param_center =
      lane_in_next->center_line_pts[0].pt - ctr_pts.back().pt;
  len = param_center.norm();
  while (i < len - 2.0) {
    i = i + 1.0;
    float pre_x = center_pt_pred.pt.x() + 1 / len * param_center.x();
    float pre_y = center_pt_pred.pt.y() + 1 / len * param_center.y();
    center_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    ctr_pts.emplace_back(center_pt_pred);
  }
  ctr_pts.emplace_back(lane_in_next->center_line_pts.front());
  if (ctr_pts.empty()) {
    return;
  }
}
void GroupMap::GenerateTransitionLane(Lane::Ptr lane_in_curr,
                                      Lane::Ptr lane_in_next,
                                      std::vector<Lane::Ptr>* virtual_lanes) {
  Lane::Ptr transition_lane = std::make_shared<Lane>();
  transition_lane->left_boundary = std::make_shared<LineSegment>();
  transition_lane->right_boundary = std::make_shared<LineSegment>();
  // HLOG_DEBUG << "is_cross_.is_crossing_ = " << is_cross_.is_crossing_
  //            << "   is_cross_.along_path_dis_.norm() = "
  //            << is_cross_.along_path_dis_.norm();
  if (is_cross_.is_crossing_ && is_cross_.along_path_dis_.norm() > 2.0) {
    // 自车在路口里的连接策略
    GenerateTransitionLaneToBefore(lane_in_curr, transition_lane);
    (*virtual_lanes).emplace_back(transition_lane);
    Lane::Ptr transition_lane_after1 = std::make_shared<Lane>();
    transition_lane_after1->left_boundary = std::make_shared<LineSegment>();
    transition_lane_after1->right_boundary = std::make_shared<LineSegment>();
    GenerateTransitionLaneToAfter(transition_lane, lane_in_next,
                                  transition_lane_after1);
    GenerateTransitionLaneToPo(transition_lane, lane_in_next,
                               transition_lane_after1);
    (*virtual_lanes).emplace_back(transition_lane_after1);
    if (history_best_lane_ != nullptr) {
      Lane::Ptr transition_lane_after2 = std::make_shared<Lane>();
      transition_lane_after2->left_boundary = std::make_shared<LineSegment>();
      transition_lane_after2->right_boundary = std::make_shared<LineSegment>();
      GenerateTransitionLaneToAfter(transition_lane, history_best_lane_,
                                    transition_lane_after2);
      GenerateTransitionLaneToPo2(transition_lane, history_best_lane_,
                                  transition_lane_after2);
      (*virtual_lanes).emplace_back(transition_lane_after2);
    }
  } else {
    // 自车快进入路口里的路口连接策略
    GenerateTransitionLaneGeo(lane_in_curr, lane_in_next, transition_lane);
    GenerateTransitionLaneToPo(lane_in_curr, lane_in_next, transition_lane);

    if (transition_lane->center_line_pts.size() > 1) {
      (*virtual_lanes).emplace_back(transition_lane);
    }
  }
}

bool GroupMap::MatchLanePtAndStopLine(const em::Point& left_pt,
                                      const em::Point& right_pt,
                                      const Stpl& stop_line) {
  if (stop_line.points.size() < 2) {
    return false;
  }

  const double kLaneStopLineDistThresh = 5.0;
  Eigen::Vector2f p0(stop_line.points.front().x(),
                     stop_line.points.front().y());
  Eigen::Vector2f p1(stop_line.points.back().x(), stop_line.points.back().y());
  Eigen::Vector2f left(left_pt.x(), left_pt.y());
  Eigen::Vector2f right(right_pt.x(), right_pt.y());
  bool left_in_range =
      (ProjectedInSegment(p0, p1, left) &&
       PointToVectorDist(p0, p1, left) < kLaneStopLineDistThresh);
  bool right_in_range =
      (ProjectedInSegment(p0, p1, right) &&
       PointToVectorDist(p0, p1, right) < kLaneStopLineDistThresh);
  // 左右点都在停止线范围内，认为关联
  if (left_in_range && right_in_range) {
    return true;
  }

  Eigen::Vector2f center = (left + right) * 0.5;
  bool center_in_range =
      (ProjectedInSegment(p0, p1, center) &&
       PointToVectorDist(p0, p1, center) < kLaneStopLineDistThresh);
  // 左边点在范围内且中心点也在范围内，认为关联
  if (left_in_range && center_in_range) {
    return true;
  }

  // 右边点在范围内且中心点也在范围内，认为关联
  if (right_in_range && center_in_range) {
    return true;
  }

  return false;
}

bool GroupMap::MatchLanePtAndZebraLine(const em::Point& left_pt,
                                       const em::Point& right_pt,
                                       const Zebra& zebra_line) {
  if (zebra_line.polygon.points.size() < 4) {
    return false;
  }

  const double kLaneZebraLineDistThresh = 10.;
  Eigen::Vector2f p0(zebra_line.polygon.points[1].x(),
                     zebra_line.polygon.points[1].y());
  Eigen::Vector2f p1(zebra_line.polygon.points[2].x(),
                     zebra_line.polygon.points[2].y());
  Eigen::Vector2f left(left_pt.x(), left_pt.y());
  Eigen::Vector2f right(right_pt.x(), right_pt.y());
  bool left_in_range =
      (ProjectedInSegment(p0, p1, left) &&
       PointToVectorDist(p0, p1, left) < kLaneZebraLineDistThresh);
  bool right_in_range =
      (ProjectedInSegment(p0, p1, right) &&
       PointToVectorDist(p0, p1, right) < kLaneZebraLineDistThresh);
  // 左右点都在停止线范围内，认为关联
  if (left_in_range && right_in_range) {
    return true;
  }

  Eigen::Vector2f center = (left + right) * 0.5;
  bool center_in_range =
      (ProjectedInSegment(p0, p1, center) &&
       PointToVectorDist(p0, p1, center) < kLaneZebraLineDistThresh);
  // 左边点在范围内且中心点也在范围内，认为关联
  if (left_in_range && center_in_range) {
    return true;
  }

  // 右边点在范围内且中心点也在范围内，认为关联
  if (right_in_range && center_in_range) {
    return true;
  }

  return false;
}

bool GroupMap::MatchLaneAndStopLine(const Lane::Ptr& lane,
                                    const Stpl::Ptr& stop_line) {
  if (lane == nullptr || stop_line == nullptr) {
    return false;
  }

  if (lane->left_boundary->pts.empty() || lane->right_boundary->pts.empty() ||
      stop_line->points.size() < 2) {
    return false;
  }

  auto start_left = lane->left_boundary->pts.front().pt;
  auto start_right = lane->right_boundary->pts.front().pt;
  auto end_left = lane->left_boundary->pts.back().pt;
  auto end_right = lane->right_boundary->pts.back().pt;
  Eigen::Vector2f stop_line_center =
      (Eigen::Vector2f(stop_line->points.back().x(),
                       stop_line->points.back().y()) +
       Eigen::Vector2f(stop_line->points.front().x(),
                       stop_line->points.front().y())) /
      2;
  bool is_right_side =
      PointInVectorSide(Eigen::Vector2f(end_left.x(), end_left.y()),
                        Eigen::Vector2f(end_right.x(), end_right.y()),
                        stop_line_center) >= 0;
  bool is_start_match =
      MatchLanePtAndStopLine(start_left, start_right, *stop_line);
  bool is_end_match = MatchLanePtAndStopLine(end_left, end_right, *stop_line);
  // HLOG_ERROR << "lane id: " << lane->str_id_with_group
  //            << "lane->left_boundary->lanepos: " <<
  //            lane->left_boundary->lanepos
  //            << " ,lane->right_boundary->lanepos: "
  //            << lane->right_boundary->lanepos;
  // HLOG_ERROR << " start_left x: " << start_left.x()
  //            << " ,y : " << start_left.y()
  //            << " ,start_right x: " << start_right.x()
  //            << " , y: " << start_right.y();
  // HLOG_ERROR << " end_left x: " << end_left.x() << " ,y : " << end_left.y()
  //            << " ,end_right x: " << end_right.x() << " , y: " <<
  //            end_right.y();
  // if (stop_line->points.size() > 1) {
  //   HLOG_ERROR << " stop_line front x: " << stop_line->points.front().x()
  //              << " ,y : " << stop_line->points.front().y()
  //              << " ,stop_line back x: " << stop_line->points.back().x()
  //              << " , y: " << stop_line->points.back().y();
  // }

  // if (lane->next_lane_str_id_with_group.size() > 0) {
  //   HLOG_ERROR << "next_lane_str_id_with_group:"
  //              << lane->next_lane_str_id_with_group[0];
  // }
  /* 三种情况: 1. 停止线在lane的开头附近，且位于lane结尾的右侧
  **          2. 停止线在lane的结尾附近， 且位于lane结尾的右侧
  **          3. 停止线在lane的结尾附近，但是位于lane结尾的左侧，需要lane无后继
  */
  return (is_start_match && is_right_side) || (is_end_match && is_right_side) ||
         (is_end_match && !is_right_side &&
          lane->next_lane_str_id_with_group.size() == 0);
}

bool GroupMap::MatchLaneAndZebraLine(const Lane::Ptr& lane,
                                     const Zebra::Ptr& zebra_line) {
  if (lane == nullptr || zebra_line == nullptr) {
    return false;
  }

  if (lane->left_boundary->pts.empty() || lane->right_boundary->pts.empty() ||
      zebra_line->polygon.points.size() < 4) {
    return false;
  }

  auto start_left = lane->left_boundary->pts.front().pt;
  auto start_right = lane->right_boundary->pts.front().pt;
  auto end_left = lane->left_boundary->pts.back().pt;
  auto end_right = lane->right_boundary->pts.back().pt;

  return MatchLanePtAndZebraLine(start_left, start_right, *zebra_line) ||
         MatchLanePtAndZebraLine(end_left, end_right, *zebra_line);
}

Stpl::Ptr GroupMap::MatchedStopLine(const std::string& lane_id) {
  for (const auto& stop : stopline_) {
    if (std::find(stop.second->lane_id.begin(), stop.second->lane_id.end(),
                  lane_id) != stop.second->lane_id.end()) {
      return stop.second;
    }
  }
  return nullptr;
}
void GroupMap::FitCenterLine(Lane::Ptr lane) {
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
    Point cpt(gm::RAW, static_cast<float>(pt.x()), static_cast<float>(pt.y()),
              0);
    lane->center_line_pts.emplace_back(cpt);
  }

  if (lane->center_line_pts.size() > 3) {
    lane->center_line_param = FitLaneline(lane->center_line_pts);
    lane->center_line_param_front = FitLanelinefront(lane->center_line_pts);
  }
}

void GroupMap::ComputeCenterPoints(Lane::Ptr lane) {
  auto main_ptr = lane->left_boundary;
  auto less_ptr = lane->right_boundary;
  if (lane->left_boundary->pts.size() <= lane->right_boundary->pts.size()) {
    main_ptr = lane->right_boundary;
    less_ptr = lane->left_boundary;
  }
  const auto& main_pts = main_ptr->pts;
  const auto& less_pts = less_ptr->pts;
  size_t less_pts_start_idx = 0;
  for (size_t i = 0; i < main_pts.size(); i++) {
    const auto& main_pt = main_pts[i];

    size_t nearest_idx = 0;
    float nearest_dis = FLT_MAX;
    for (size_t j = less_pts_start_idx; j < less_pts.size(); j++) {
      float dis = (main_pt.pt - less_pts[j].pt).norm();
      if (dis < nearest_dis) {
        nearest_dis = dis;
        nearest_idx = j;
      }
    }
    less_pts_start_idx = nearest_idx;
    const auto& less_pt = less_pts[nearest_idx];
    auto center_pt = (main_pt.pt + less_pt.pt) / 2;
    Point ct_pt(gm::RAW, static_cast<float>(center_pt.x()),
                static_cast<float>(center_pt.y()),
                static_cast<float>(center_pt.z()));
    lane->center_line_pts.emplace_back(ct_pt);
  }
  if (lane->center_line_pts.size() > 3) {
    lane->center_line_param = FitLaneline(lane->center_line_pts);
    lane->center_line_param_front = FitLanelinefront(lane->center_line_pts);
  }
}

void GroupMap::UpdateLaneBoundaryId(Group::Ptr curr_group) {
  std::unordered_map<em::Id, em::Id> line_next;
  for (auto lane : curr_group->lanes) {
    if (lane->left_boundary->id_next != -1000) {
      line_next[lane->left_boundary->id] = lane->left_boundary->id_next;
    }
    if (lane->right_boundary->id_next != -1000) {
      line_next[lane->right_boundary->id] = lane->right_boundary->id_next;
    }
  }
  for (auto lane : curr_group->lanes) {
    if (lane->left_boundary->id_next == -1000 &&
        line_next.find(lane->left_boundary->id) != line_next.end()) {
      lane->left_boundary->id_next = line_next[lane->left_boundary->id];
    }
    if (lane->right_boundary->id_next == -1000 &&
        line_next.find(lane->right_boundary->id) != line_next.end()) {
      lane->right_boundary->id_next = line_next[lane->right_boundary->id];
    }
  }
}

void GroupMap::DelLaneNextStrIdInGroup(Group::Ptr curr_group) {
  for (auto& lane_in_curr : curr_group->lanes) {
    lane_in_curr->next_lane_str_id_with_group.clear();
  }
}

void GroupMap::DelLanePrevStrIdInGroup(Group::Ptr curr_group) {
  for (auto& lane_in_curr : curr_group->lanes) {
    lane_in_curr->prev_lane_str_id_with_group.clear();
  }
}

void GroupMap::FindNearestLaneToHisVehiclePosition(Group::Ptr curr_group,
                                                   Lane::Ptr* ego_curr_lane) {
  if (curr_group->group_segments.empty()) {
    return;
  }

  auto curr_grp_start_slice = curr_group->group_segments.front()->start_slice;
  auto curr_grp_end_slice = curr_group->group_segments.back()->end_slice;
  Eigen::Vector2f curr_start_pl(curr_grp_start_slice.pl.x(),
                                curr_grp_start_slice.pl.y());
  Eigen::Vector2f curr_start_pr(curr_grp_start_slice.pr.x(),
                                curr_grp_start_slice.pr.y());
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
  min_dist = conf_.min_lane_width;  // min_dist = FLT_MAX;
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
      if (dist < min_dist) {
        min_dist = dist;
        *ego_curr_lane = curr_lane;
      }
    }
  }
}

bool GroupMap::LineIdConsistant(LineSegment::Ptr line, em::Id id) {
  if (line->id == id) {
    return true;
  }
  for (const auto& detele_id : line->deteled_ids) {
    if (detele_id == id) {
      return true;
    }
  }
  return false;
}

void GroupMap::FindBestNextLane(Group::Ptr next_group,
                                const float& dist_to_slice,
                                Lane::Ptr* best_next_lane) {
  Eigen::Vector2f thresh_v(std::cos(conf_.junction_heading_diff),
                           std::sin(conf_.junction_heading_diff));
  Eigen::Vector2f n(1, 0);  // 车前向量
  float thresh_len = std::abs(thresh_v.transpose() * n);

  float max_len = 0;
  for (auto& next_lane : next_group->lanes) {
    if (!next_lane->right_boundary->isego || !next_lane->left_boundary->isego) {
      continue;
    }
    Eigen::Vector2f p0(0, 0);
    Eigen::Vector2f p1(next_lane->center_line_pts.front().pt.x(),
                       next_lane->center_line_pts.front().pt.y());
    Eigen::Vector2f v = p1 - p0;
    v.normalize();
    float len = std::abs(v.transpose() * n);
    if (len > max_len && len >= thresh_len) {
      max_len = len;
      *best_next_lane = next_lane;
    }
    // HLOG_DEBUG << "len = " << acos(len) * 180 / pi_;
  }

  float min_offset = FLT_MAX;
  const float kOffsetThreshold = 3.5 * 0.7;  // 0.7个3.5米车道宽度
  float check_offset_thresh = -10000;
  if (conf_.junction_heading_diff > 0.001) {
    check_offset_thresh =
        kOffsetThreshold / std::tan(conf_.junction_heading_diff);
  }
  // 如果此时距离足够近，并且通过角度未找到合适的next
  // lane，此时通过找最小的横向偏移来确定next lane
  if (*best_next_lane == nullptr && dist_to_slice <= check_offset_thresh) {
    for (auto& next_lane : next_group->lanes) {
      if (!next_lane->right_boundary->isego ||
          !next_lane->left_boundary->isego) {
        continue;
      }
      float offset = std::abs(next_lane->center_line_pts.front().pt.y());
      if (offset < min_offset && offset <= kOffsetThreshold) {
        min_offset = offset;
        *best_next_lane = next_lane;
      }
    }
  }
  // 比较当前时刻的下一条lane信息和历史时刻的lane信息
  // 1.没有找到best_next_lane
  if (*best_next_lane == nullptr) {
    if (is_cross_.next_lane_left != -1000 ||
        is_cross_.next_lane_right != -1000) {
      // 如果上一帧存在best_lane
      for (auto& lane_ : next_group->lanes) {
        if (!lane_->right_boundary->isego || !lane_->left_boundary->isego) {
          continue;
        }
        if (LineIdConsistant(lane_->left_boundary, is_cross_.next_lane_left) ||
            LineIdConsistant(lane_->right_boundary,
                             is_cross_.next_lane_right)) {
          *best_next_lane = lane_;
          return;
        }
      }
    }
    return;
  }
  // 2.第一帧连接的best_lane 存储一下左右边线并返回
  if (is_cross_.next_lane_left == -1000 || is_cross_.next_lane_right == -1000) {
    is_cross_.next_lane_left = (*best_next_lane)->left_boundary->id;
    is_cross_.next_lane_right = (*best_next_lane)->right_boundary->id;
    return;
  }
  // HLOG_ERROR << " best is equal to exist "
  //            << LineIdConsistant((*best_next_lane)->left_boundary,
  //                                is_cross_.next_lane_left)
  //            << "  "
  //            << LineIdConsistant((*best_next_lane)->right_boundary,
  //                                is_cross_.next_lane_right);
  // 3.跟上一帧连接的车道一致，直接返回
  if (LineIdConsistant((*best_next_lane)->left_boundary,
                       is_cross_.next_lane_left) ||
      LineIdConsistant((*best_next_lane)->right_boundary,
                       is_cross_.next_lane_right)) {
    return;
  }
  // 4.跟上一帧连接的车道不一致，看看是否需要更新best_lane
  int exist = -1;
  for (int i = 0; i < next_group->lanes.size(); ++i) {
    auto& lane_ = next_group->lanes[i];
    if (!lane_->right_boundary->isego || !lane_->left_boundary->isego) {
      continue;
    }
    if (LineIdConsistant(lane_->left_boundary, is_cross_.next_lane_left) &&
        LineIdConsistant(lane_->right_boundary, is_cross_.next_lane_right)) {
      exist = i;
      history_best_lane_ = lane_;
      break;
    }
  }
  if (exist == -1) {
    is_cross_.next_lane_left = (*best_next_lane)->left_boundary->id;
    is_cross_.next_lane_right = (*best_next_lane)->right_boundary->id;
    return;
  }

// 之前是会筛选是否要更新历史的连接，现在交给规控选择，因此这段注释掉
#if 0
  if (exist > -1 && exist < next_group->lanes.size()) {
    auto& exist_pt = next_group->lanes[exist]->center_line_pts.front().pt;
    float atan_exist = abs(atan(exist_pt.y() / exist_pt.x())) * 180 / pi_;
    auto best_pt = (*best_next_lane)->center_line_pts.front().pt;
    float atan_best = abs(atan(best_pt.y() / best_pt.x())) * 180 / pi_;
    float offset_exist =
        std::abs(next_group->lanes[exist]->center_line_pts.front().pt.y());
    float offset_best =
        std::abs((*best_next_lane)->center_line_pts.front().pt.y());
    if (atan_exist < atan_best + 1.0 || offset_exist < offset_best + 0.5) {
      // && offset_exist < offset_best + 1.0
      // 1度以内不变化或者横向偏差不超过1m
      *best_next_lane = next_group->lanes[exist];
    } else {
      is_cross_.next_lane_left = (*best_next_lane)->left_boundary->id;
      is_cross_.next_lane_right = (*best_next_lane)->right_boundary->id;
    }
  }
#endif
}

std::vector<Point> GroupMap::SigmoidFunc(std::vector<Point> centerline,
                                         float sigma) {
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
    Point ctp(VIRTUAL, 0.0, 0.0, 0.0);
    if (center.back().pt.x() < centerline[i + 1].pt.x()) {
      ctp.pt.x() = centerline[i + 1].pt.x();
      float deta_x = centerline[i + 1].pt.x() - center[0].pt.x();
      // 1/(1+e^(-x)) sigmoid函数
      // 把x的范围圈定在[-sigma,sigma]范围内，deta_x的范围是(0,dis_x]
      // y的范围是(centerline[0].pt.y(),centerline[0].y()+dis_y)
      ctp.pt.y() = centerline[0].pt.y() +
                   dis_y / (1 + exp(-(2 * deta_x - dis_x) / dis_x * sigma));
    } else {
      float x_to_end = centerline.back().pt.x() - center.back().pt.x();
      float interval = x_to_end / static_cast<float>(centerline.size() - i - 1);
      ctp.pt.x() = center.back().pt.x() + interval;
      float deta_x = ctp.pt.x() - center[0].pt.x();
      ctp.pt.y() = centerline[0].pt.y() +
                   dis_y / (1 + exp(-(2 * deta_x - dis_x) / dis_x * sigma));
    }
    center.emplace_back(ctp);
  }
  center.emplace_back(centerline.back());
  return center;
}
std::vector<Point> GroupMap::SlidingWindow(std::vector<Point> centerline,
                                           int w) {
  int size_ct = centerline.size();
  if (size_ct < w) {
    return centerline;
  }
  std::vector<Point> center;
  Point ptc(VIRTUAL, 0.0, 0.0, 0.0);
  center.emplace_back(ptc);
  for (int i = 0; i < centerline.size(); ++i) {
    if (i < w) {
      center[0].pt += centerline[i].pt;
    } else {
      Point pt;
      pt.type = VIRTUAL;
      pt.pt = center[i - w].pt - centerline[i - w].pt + centerline[i].pt;
      center.emplace_back(pt);
      center[i - w].pt /= w;
    }
  }
  center.back().pt /= w;
  return center;
}

void GroupMap::ComputeLineCurvatureV2(std::vector<Point>* guide_points,
                                      std::vector<Group::Ptr>* groups,
                                      double stamp) {
  // 1. 删除待删除状态的group
  groups->erase(
      std::remove_if(groups->begin(), groups->end(),
                     [&](Group::Ptr& group) {
                       return (group->group_state == Group::GroupState::DELETE);
                     }),
      groups->end());
  // 如果能够生成引导线的话，
  // 需要加上is_last_after_erased标记，否则就不需要了。
  if (groups->empty()) {
    return;
  }
  if (guide_points->empty()) {
    groups->back()->is_last_after_erased = true;
    return;
  }

  // 找到最后的group中的车道
  Group::Ptr last_grp = nullptr;
  // 找到最后一个非空的group，只预测最后一个group里的lane
  for (auto it = groups->rbegin(); it != groups->rend(); ++it) {
    if ((*it) == nullptr || (*it)->lanes.empty()) {
      continue;
    }
    last_grp = *it;
    break;
  }

  // 找到历史最靠近的车道
  Lane::Ptr last_ego_lane = nullptr;
  if (last_grp != nullptr) {
    FindNearestLaneToHisVehiclePosition(last_grp, &last_ego_lane);
  }
  if (last_ego_lane == nullptr) {
    return;
  }

  // 2. 生成拟合点
  static int idx = 0;
  HLOG_DEBUG << "[predict] idx: " << ++idx;
  std::vector<Eigen::Vector3f> pts;
  Eigen::Vector3f first_pt(-10.0, 0.0, 0.0);
  pts.emplace_back(first_pt);
  for (const auto& it : *guide_points) {
    Eigen::Vector3f pt(static_cast<float>(it.pt.x()),
                       static_cast<float>(it.pt.y()),
                       static_cast<float>(it.pt.z()));
    pts.emplace_back(pt);
  }
  for (auto point : pts) {
    HLOG_DEBUG << "[predict] keypoint, x:" << point.x() << " y:" << point.y()
               << " z:" << point.z();
  }

  std::vector<Vec2d> all_points;
  Vec2d first_point(pts[0].x(), pts[0].y());
  Vec2d second_point(pts[1].x(), pts[1].y());
  Vec2d third_point(pts[2].x(), pts[2].y());
  Vec2d fourth_point(pts[3].x(), pts[3].y());
  all_points.emplace_back(first_point);
  for (const auto& point : math::LinearInterp(first_point, second_point, 5.0)) {
    all_points.emplace_back(point);
  }
  all_points.emplace_back(second_point);
  for (const auto& point : math::LinearInterp(second_point, third_point, 5.0)) {
    all_points.emplace_back(point);
  }
  all_points.emplace_back(third_point);
  for (const auto& point : math::LinearInterp(third_point, fourth_point, 5.0)) {
    all_points.emplace_back(point);
  }
  all_points.emplace_back(fourth_point);

  for (auto point : all_points) {
    HLOG_DEBUG << "[predict] after CatmullRom point, x:" << point.x()
               << " y:" << point.y();
  }

  std::vector<double> fit_result;
  math::FitLaneLinePoint(all_points, &fit_result);

  std::vector<Vec2d> fit_points;
  for (float x = second_point.x(); x <= fourth_point.x(); x = x + 1.0) {
    float y = math::CalCubicCurveY(fit_result, x);
    fit_points.emplace_back(Vec2d(x, y));
  }

  for (auto point : fit_points) {
    HLOG_DEBUG << "[predict] after Fit point, x:" << point.x()
               << " y:" << point.y();
  }

  // std::vector<Vec2d> key_guide_points;
  // for (const auto& it : path_in_curr_pose_) {
  //   if (it.pos.x() > last_ego_lane->center_line_pts.back().x() &&
  //       it.pos.x() < 0.0) {
  //     Vec2d pt(it.pos.x(), it.pos.y());
  //     key_guide_points.emplace_back(pt);
  //   }
  // }

  for (auto point : fit_points) {
    HLOG_DEBUG << "[predict] after Fit point, x:" << point.x()
               << " y:" << point.y();
  }

  // if(fit_points )
  if (fit_points.empty()) {
    guide_points->clear();
    groups->back()->is_last_after_erased = true;
    return;
  }

  // 3. 生成引导线几何
  Lane::Ptr guide_lane = std::make_shared<Lane>();
  guide_lane->left_boundary = std::make_shared<LineSegment>();
  guide_lane->right_boundary = std::make_shared<LineSegment>();
  GenetateGuideLaneGeo(&fit_points, &guide_lane, last_ego_lane);

  // 4. 生成引导线拓扑
  auto left_line = last_ego_lane->left_boundary->pts;
  auto right_line = last_ego_lane->right_boundary->pts;
  auto last_x = std::max(left_line.back().pt.x(), right_line.back().pt.x());

  std::vector<Lane::Ptr> virtual_lanes;
  // Lane::Ptr transition_lane = nullptr;
  if (last_x < 0) {
    // 路口内
    Lane::Ptr transition_lane = std::make_shared<Lane>();
    transition_lane->left_boundary = std::make_shared<LineSegment>();
    transition_lane->right_boundary = std::make_shared<LineSegment>();
    GenerateTransitionLaneGeo(last_ego_lane, guide_lane, transition_lane);
    GenerateTransitionLaneToPo(last_ego_lane, guide_lane, transition_lane);
    // HLOG_DEBUG << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxguide_lane left: " <<
    // guide_lane->left_boundary->pts.size(); HLOG_DEBUG <<
    // "xxxxxxxxxxxxxxxxxxxxxxxxxxxxguide_lane right: " <<
    // guide_lane->right_boundary->pts.size(); for (const auto& it :
    // guide_lane->left_boundary->pts) {
    //   HLOG_DEBUG << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxpt: " << it.pt.x() << ", "
    //   << it.pt.y();
    // }
    // HLOG_DEBUG<<"transition_lane->left_boundary->pts.size() =
    // "<<transition_lane->left_boundary->pts.size();
    // HLOG_DEBUG<<"transition_lane->right_boundary->pts.size() =
    // "<<transition_lane->right_boundary->pts.size();
    // HLOG_DEBUG<<"transition_lane->center_line_pts.size() =
    // "<<transition_lane->center_line_pts.size(); for (auto point :
    // transition_lane->center_line_pts) {
    //   HLOG_DEBUG << "[debug] after Fit point, x:" << point.pt.x()
    //             << " y:" << point.pt.y();
    // }
    // HLOG_DEBUG << "last_ego_lane id: " << last_ego_lane->str_id_with_group;
    // HLOG_DEBUG << "transition_lane id: " <<
    // transition_lane->str_id_with_group; HLOG_DEBUG << "guide_lane id: " <<
    // guide_lane->str_id_with_group; for (const auto& it :
    // transition_lane->next_lane_str_id_with_group) {
    //   HLOG_DEBUG << "transition_lane next: " << it;
    // }
    // for (const auto& it : transition_lane->prev_lane_str_id_with_group) {
    //   HLOG_DEBUG << "transition_lane pre: " << it;
    // }
    // for (const auto& it : guide_lane->prev_lane_str_id_with_group) {
    //   HLOG_DEBUG << "guide_lane pre: " << it;
    // }
    // for (const auto& it : last_ego_lane->next_lane_str_id_with_group) {
    //   HLOG_DEBUG << "last_ego_lane next: " << it;
    // }

    if (transition_lane->left_boundary->pts.size() > 1 &&
        transition_lane->right_boundary->pts.size() > 1 &&
        transition_lane->center_line_pts.size() > 1) {
      (virtual_lanes).emplace_back(transition_lane);
    }
  } else {
    // 路口外
    GenerateGuideLaneToPo(last_ego_lane, guide_lane);
  }
  (virtual_lanes).emplace_back(guide_lane);
  std::vector<Group::Ptr> group_virtual;
  BuildVirtualGroup(virtual_lanes, &group_virtual, stamp);
  // 增加虚拟车道到groups中
  for (int i = static_cast<int>(groups->size()) - 1; i >= 0; --i) {
    if (group_virtual.empty()) {
      break;
    }
    if (groups->at(i)->str_id + "VV" == group_virtual.back()->str_id) {
      groups->insert(groups->begin() + i + 1, group_virtual.back());
      group_virtual.pop_back();
    }
    if (!group_virtual.empty() &&
        groups->at(i)->str_id + 'V' == group_virtual.back()->str_id) {
      groups->insert(groups->begin() + i + 1, group_virtual.back());
      group_virtual.pop_back();
    }
  }
}

void GroupMap::GenerateGuideLaneToPo(Lane::Ptr lane_in_curr,
                                     Lane::Ptr guide_lane) {
  lane_in_curr->next_lane_str_id_with_group.emplace_back(
      guide_lane->str_id_with_group);
  guide_lane->prev_lane_str_id_with_group.emplace_back(
      lane_in_curr->str_id_with_group);
}

void GroupMap::GenetateGuideLaneGeo(std::vector<Vec2d>* fit_points,
                                    Lane::Ptr* guide_lane,
                                    const Lane::Ptr& last_ego_lane) {
  // 根据拟合点生成lane
  if (fit_points->empty()) {
    HLOG_ERROR << "fit_points is empty!";
    return;
  }
  // 计算last_ego_lane宽度(暂时用左右边线最后两个点的距离代替)
  auto left_last_pt = last_ego_lane->left_boundary->pts.back().pt;
  auto right_last_pt = last_ego_lane->right_boundary->pts.back().pt;
  double width = (left_last_pt - right_last_pt).norm();
  // 判断自车与last_ego_lane末端点的位置
  auto last_x = std::max(left_last_pt.x(), right_last_pt.x());
  if (last_x < 0) {
    // 删除fit_points中x小于0的点
    fit_points->erase(std::remove_if(fit_points->begin(), fit_points->end(),
                                     [](const Vec2d& p) { return p.x() < 0; }),
                      fit_points->end());
  } else {
    // 删除fit_points中x小于last_ego_lane末端点x的点
    fit_points->erase(
        std::remove_if(fit_points->begin(), fit_points->end(),
                       [&last_x](const Vec2d& p) { return p.x() < last_x; }),
        fit_points->end());
  }
  if (fit_points->empty()) {
    HLOG_ERROR << "fit_points is empty!";
    return;
  }
  // 根据宽度和fit_points计算虚拟车道
  std::vector<Vec2d> left_pts;
  std::vector<Vec2d> right_pts;
  for (int i = 0; i < static_cast<int>(fit_points->size()) - 1; i++) {
    auto dir_vec = fit_points->at(i + 1) - fit_points->at(i);
    dir_vec.Normalize();
    auto normal_vec = Vec2d(-dir_vec.y(), dir_vec.x());
    auto left_pt = fit_points->at(i) + normal_vec * (width / 2.0);
    auto right_pt = fit_points->at(i) - normal_vec * (width / 2.0);
    left_pts.emplace_back(left_pt);
    right_pts.emplace_back(right_pt);
  }
  if (left_pts.empty() || right_pts.empty()) {
    HLOG_ERROR << "left_pts or right_pts is empty";
    return;
  }
  // 封装接口计算lane左右线的heading
  auto left_line = std::make_shared<Line>();
  for (const auto& it : left_pts) {
    Point pt;
    pt.pt.x() = static_cast<float>(it.x());
    pt.pt.y() = static_cast<float>(it.y());
    pt.pt.z() = 0.0;
    left_line->pts.emplace_back(pt);
  }
  ComputeLineHeading(left_line);
  auto right_line = std::make_shared<Line>();
  for (const auto& it : right_pts) {
    Point pt;
    pt.pt.x() = static_cast<float>(it.x());
    pt.pt.y() = static_cast<float>(it.y());
    pt.pt.z() = 0.0;
    right_line->pts.emplace_back(pt);
  }
  ComputeLineHeading(right_line);
  // 将构造出的几何点塞到guide_lane中
  for (const auto& it : *fit_points) {
    Point pt;
    pt.pt.x() = static_cast<float>(it.x());
    pt.pt.y() = static_cast<float>(it.y());
    pt.pt.z() = 0.0;
    (*guide_lane)->center_line_pts.emplace_back(pt);
  }
  (*guide_lane)->str_id = last_ego_lane->str_id;
  (*guide_lane)->lanepos_id = last_ego_lane->lanepos_id;
  size_t index = last_ego_lane->str_id_with_group.find(":");
  (*guide_lane)->str_id_with_group =
      last_ego_lane->str_id_with_group.substr(0, index) +
      "VV:" + (*guide_lane)->str_id;
  (*guide_lane)->center_line_param = last_ego_lane->center_line_param;
  (*guide_lane)->center_line_param_front = last_ego_lane->center_line_param;

  (*guide_lane)->left_boundary->id = last_ego_lane->left_boundary->id;
  (*guide_lane)->left_boundary->type = em::LaneType_DASHED;
  (*guide_lane)->left_boundary->color = em::WHITE;
  (*guide_lane)->left_boundary->isego = em::Ego_Road;
  (*guide_lane)->left_boundary->is_near_road_edge =
      left_line->is_near_road_edge;
  (*guide_lane)->left_boundary->pts = left_line->pts;
  (*guide_lane)->left_boundary->mean_end_heading = left_line->mean_end_heading;
  (*guide_lane)->left_boundary->pred_end_heading = left_line->pred_end_heading;
  (*guide_lane)->left_boundary->mean_end_interval =
      left_line->mean_end_interval;
  (*guide_lane)->left_boundary->mean_end_heading_std_dev =
      left_line->mean_end_heading_std_dev;

  (*guide_lane)->right_boundary->id = last_ego_lane->right_boundary->id;
  (*guide_lane)->right_boundary->type = em::LaneType_DASHED;
  (*guide_lane)->right_boundary->color = em::WHITE;
  (*guide_lane)->right_boundary->isego = em::Ego_Road;
  (*guide_lane)->right_boundary->is_near_road_edge =
      right_line->is_near_road_edge;
  (*guide_lane)->right_boundary->pts = right_line->pts;
  (*guide_lane)->right_boundary->mean_end_heading =
      right_line->mean_end_heading;
  (*guide_lane)->right_boundary->pred_end_heading =
      right_line->pred_end_heading;
  (*guide_lane)->right_boundary->mean_end_interval =
      right_line->mean_end_interval;
  (*guide_lane)->right_boundary->mean_end_heading_std_dev =
      right_line->mean_end_heading_std_dev;
}

void GroupMap::ComputeLineHeading(const Line::Ptr& line) {
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
    line->mean_end_interval = 0;
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

void GroupMap::SmoothCenterline(std::vector<Group::Ptr>* groups) {
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
void GroupMap::ErasePrevRelation(Group::Ptr curr_group, int curr_erase_index,
                                 Group::Ptr next_group, int next_index) {
  if (next_index < 0 || next_index >= next_group->lanes.size() ||
      curr_erase_index < 0 || curr_erase_index >= curr_group->lanes.size()) {
    return;
  }
  for (int next_before_index = 0;
       next_before_index <
       next_group->lanes[next_index]->prev_lane_str_id_with_group.size();
       next_before_index++) {
    if (next_group->lanes[next_index]
            ->prev_lane_str_id_with_group[next_before_index] ==
        curr_group->lanes[curr_erase_index]->str_id_with_group) {
      next_group->lanes[next_index]->prev_lane_str_id_with_group.erase(
          next_group->lanes[next_index]->prev_lane_str_id_with_group.begin() +
          next_before_index);
      return;
    }
  }
}

void GroupMap::EraseSucessorRelation(Group::Ptr curr_group,
                                     int curr_erase_index,
                                     Group::Ptr next_group, int next_index) {
  if (next_index < 0 || next_index >= next_group->lanes.size() ||
      curr_erase_index < 0 || curr_erase_index >= curr_group->lanes.size()) {
    return;
  }
  for (int curr_next_index = 0;
       curr_next_index <
       curr_group->lanes[curr_erase_index]->next_lane_str_id_with_group.size();
       ++curr_next_index) {
    if (curr_group->lanes[curr_erase_index]
            ->next_lane_str_id_with_group[curr_next_index] ==
        next_group->lanes[next_index]->str_id_with_group) {
      curr_group->lanes[curr_erase_index]->next_lane_str_id_with_group.erase(
          curr_group->lanes[curr_erase_index]
              ->next_lane_str_id_with_group.begin() +
          curr_next_index);
      return;
    }
  }
}
void GroupMap::AvoidSplitMergeLane(std::vector<Group::Ptr>* groups) {
  //  青鸾号:1209412
  for (int grp_idx = 0; grp_idx < static_cast<int>(groups->size()) - 2;
       ++grp_idx) {
    auto& before_group = groups->at(grp_idx);
    auto& curr_group = groups->at(grp_idx + 1);
    auto& next_group = groups->at(grp_idx + 2);
    std::unordered_map<std::string, int> curr_group_lane_index;
    std::unordered_map<std::string, int> next_group_lane_index;
    for (int index_curr_goup_lane = 0;
         index_curr_goup_lane < curr_group->lanes.size();
         ++index_curr_goup_lane) {
      curr_group_lane_index.insert_or_assign(
          curr_group->lanes[index_curr_goup_lane]->str_id_with_group,
          index_curr_goup_lane);
    }
    for (int index_next_goup_lane = 0;
         index_next_goup_lane < next_group->lanes.size();
         ++index_next_goup_lane) {
      next_group_lane_index.insert_or_assign(
          next_group->lanes[index_next_goup_lane]->str_id_with_group,
          index_next_goup_lane);
    }
    for (int index_before_group_lane = 0;
         index_before_group_lane < before_group->lanes.size();
         ++index_before_group_lane) {
      if (before_group->lanes[index_before_group_lane]
              ->next_lane_str_id_with_group.size() > 1) {
        std::unordered_map<std::string, int> successor_id_and_curr_index;
        std::unordered_set<int>
            erase_relate_curr_index;  // 需要被before_group->lanes[index_before_group_lane]删除后继关联的cur_group
                                      // lane index
        for (auto& lane_id : before_group->lanes[index_before_group_lane]
                                 ->next_lane_str_id_with_group) {
          if (curr_group_lane_index.find(lane_id) !=
              curr_group_lane_index.end()) {
            int curr_index = curr_group_lane_index[lane_id];
            for (auto& lane_next_id :
                 curr_group->lanes[curr_index]->next_lane_str_id_with_group) {
              if (next_group_lane_index.find(lane_next_id) !=
                  next_group_lane_index.end()) {
                int next_index = next_group_lane_index[lane_next_id];
                if (successor_id_and_curr_index.find(lane_next_id) !=
                    successor_id_and_curr_index.end()) {
                  // 有重合的后继 根据trackid来筛选保留的前后继
                  if (curr_group->lanes[curr_index]->left_boundary->id ==
                          before_group->lanes[index_before_group_lane]
                              ->left_boundary->id ||
                      curr_group->lanes[curr_index]->right_boundary->id ==
                          before_group->lanes[index_before_group_lane]
                              ->right_boundary->id) {
                    erase_relate_curr_index.insert(
                        successor_id_and_curr_index[lane_next_id]);
                    int curr_erase_index =
                        successor_id_and_curr_index[lane_next_id];
                    successor_id_and_curr_index[lane_next_id] = curr_index;
                    // ErasePrevRelation(curr_group, curr_erase_index,
                    // next_group,
                    //                   next_index);
                    // EraseSucessorRelation(curr_group, curr_erase_index,
                    //                       next_group, next_index);
                    ErasePrevRelation(before_group, index_before_group_lane,
                                      curr_group, curr_erase_index);
                  } else {
                    erase_relate_curr_index.insert(curr_index);
                    // ErasePrevRelation(curr_group, curr_index, next_group,
                    //                   next_index);
                    // EraseSucessorRelation(curr_group, curr_index, next_group,
                    //                       next_index);
                    ErasePrevRelation(before_group, index_before_group_lane,
                                      curr_group, curr_index);
                  }
                } else {
                  successor_id_and_curr_index[lane_next_id] = curr_index;
                }
              }
            }
          }
        }
        if (!erase_relate_curr_index.empty()) {
          std::unordered_set<std::string>
              erase_relate_curr_id;  // 需要被删除前后继关联的idname
          for (int i = 0; i < curr_group->lanes.size(); ++i) {
            if (erase_relate_curr_index.find(i) !=
                erase_relate_curr_index.end()) {
              erase_relate_curr_id.insert(
                  curr_group->lanes[i]->str_id_with_group);
            }
          }
          for (int before_next = 0;
               before_next < before_group->lanes[index_before_group_lane]
                                 ->next_lane_str_id_with_group.size();
               ++before_next) {
            std::string before_next_name =
                before_group->lanes[index_before_group_lane]
                    ->next_lane_str_id_with_group[before_next];
            if (erase_relate_curr_id.find(before_next_name) !=
                erase_relate_curr_id.end()) {
              before_group->lanes[index_before_group_lane]
                  ->next_lane_str_id_with_group.erase(
                      before_group->lanes[index_before_group_lane]
                          ->next_lane_str_id_with_group.begin() +
                      before_next);
            }
          }
        }
      }
    }
  }
}
int GroupMap::FindEgoGroup(const std::vector<Group::Ptr>* groups) {
  int index = -1;
  for (auto& grp : *groups) {
    index++;
    if (grp->group_segments.size() < 1) {
      continue;
    }
    if (grp->group_segments.front()->start_slice.po.x() > 0.0) {
      // 前一个group是当前group或者没有找到当前group，需要再进行判断
      return -1;
    }
    if (grp->group_segments.back()->end_slice.po.x() > 0.0) {
      // 当前group是
      return index;
    }
  }
  return -1;
}
void GroupMap::EraseEgoGroupWithNoEgoLane(std::vector<Group::Ptr>* groups) {
  int index = FindEgoGroup(groups);
  if (index == -1 || groups->size() <= index) {
    // 没找到自车所在group
    return;
    if (groups->empty()) {
      return;
    }
    auto curr_group = groups->back();
    if (curr_group->group_segments.empty()) {
      return;
    }

    auto curr_grp_start_slice = curr_group->group_segments.front()->start_slice;
    auto curr_grp_end_slice = curr_group->group_segments.back()->end_slice;

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
  if (groups->size() > index + 1 && ego_group->group_segments.size() > 0 &&
      ego_group->group_segments.back()->end_slice.po.x() < 0.0 &&
      groups->at(index + 1)->group_segments.size() > 0 &&
      groups->at(index + 1)->group_segments.front()->start_slice.po.x() > 0.0) {
    // 没找到自车所在group
    return;
  }
  if (groups->size() == index + 1 &&
      ego_group->group_segments.back()->end_slice.po.x() < 0.0) {
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
int GroupMap::BesideGroup(Group::Ptr group) {
  if (group->lanes.empty()) {
    return 0;
  }
  int is_left = 0;
  for (const auto& lane : group->lanes) {
    Eigen::Vector2f start_point = lane->center_line_pts[0].pt.head<2>();
    Eigen::Vector2f end_point = lane->center_line_pts.back().pt.head<2>();
    Eigen::Vector2f ego_pos(0.0, 0.0);
    if (PointInVectorSide(start_point, end_point, ego_pos) <= 0) {
      is_left++;
    }
  }
  if (is_left == 0) {
    return 2;
  } else if (is_left == group->lanes.size()) {
    return 1;
  }
  return 0;
}

void GroupMap::AddVirtualLine(std::vector<Group::Ptr>* groups) {
  int index = FindEgoGroup(groups);
  if (index == -1 || groups->size() <= index || groups->empty()) {
    // 没找到自车所在group
    return;
  }
  auto& ego_group = groups->at(index);
  if (ego_group->lanes.empty()) {
    return;
  }
  // 判断是否有自车道
  float min_dis = FLT_MAX;
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
    min_dis = std::min(min_dis, best_dis);
    if (index == -1) {
      continue;
    }
    if (lane->center_line_pts[index].pt.norm() < 1.5) {
      return;
    }
  }
  int status = BesideGroup(ego_group);
  if (min_dis > 3.5 || !status) {
    return;
  }
  if (status == 1) {
    BuildVirtualLaneLeft(ego_group);
  } else {
    BuildVirtualLaneRight(ego_group);
  }
}
bool GroupMap::IsNearLine(LineSegment::Ptr line1, LineSegment::Ptr line2) {
  // 第一个点的距离和最后一个点的距离都得算
  if (line1->id == line2->id) {
    if ((line1->pts.empty() || line1->pts[0].type == RAW) &&
        (line2->pts.empty() || line2->pts[0].type == RAW)) {
      return true;
    }
  }
  if (line1->pts.size() < 2 || line2->pts.size() < 2) {
    return true;
  }
  const double line_dis_thresh = 0.5;
  std::vector<Point> line1_pts(
      std::vector<Point>(line1->pts.begin(), line1->pts.end()));

  std::vector<Point> line2_pts(
      std::vector<Point>(line2->pts.begin(), line2->pts.end()));
  std::vector<double> line1_param_front;
  std::vector<double> line2_param_front;
  std::vector<double> line1_param;
  std::vector<double> line2_param;
  line1_param_front = FitLanelinefront(line1_pts);
  line2_param_front = FitLanelinefront(line2_pts);
  line1_param = FitLaneline(line1_pts);
  line2_param = FitLaneline(line2_pts);
  if (line1_param_front.empty() && line2_param_front.empty()) {
    if ((line1->pts[0].pt, line2->pts[0].pt).norm() > line_dis_thresh) {
      return false;
    }
  } else if (line1_param_front.empty()) {
    auto x = line1->pts[0].pt.x();
    auto y = line1->pts[0].pt.y();
    double dis = abs(line2_param_front[0] + line2_param_front[1] * x - y) /
                 sqrt(1 + pow(line2_param_front[1], 2));
    if (dis > line_dis_thresh) {
      return false;
    }
  } else {
    auto x = line2->pts[0].pt.x();
    auto y = line2->pts[0].pt.y();
    double dis = abs(line1_param_front[0] + line1_param_front[1] * x - y) /
                 sqrt(1 + pow(line1_param_front[1], 2));
    if (dis > line_dis_thresh) {
      return false;
    }
  }

  if (line1_param.empty() && line2_param.empty()) {
    if ((line1->pts.back().pt, line2->pts.back().pt).norm() > line_dis_thresh) {
      return false;
    }
  } else if (line1_param.empty()) {
    auto x = line1->pts.back().pt.x();
    auto y = line1->pts.back().pt.y();
    double dis = abs(line2_param[0] + line2_param[1] * x - y) /
                 sqrt(1 + pow(line2_param[1], 2));
    if (dis > line_dis_thresh) {
      return false;
    }
  } else {
    auto x = line2->pts.back().pt.x();
    auto y = line2->pts.back().pt.y();
    double dis = abs(line1_param[0] + line1_param[1] * x - y) /
                 sqrt(1 + pow(line1_param[1], 2));
    if (dis > line_dis_thresh) {
      return false;
    }
  }
  return true;
}
void GroupMap::NeighborLane(std::vector<Group::Ptr>* groups) {
  for (auto& grp : *groups) {
    std::sort(grp->lanes.begin(), grp->lanes.end(),
              [](const Lane::Ptr& a, const Lane::Ptr& b) {
                return a->right_lane_str_id_with_group.size() >
                       b->right_lane_str_id_with_group.size();
              });
    for (const auto& lane : grp->lanes) {
      lane->left_lane_str_id_with_group.clear();
      lane->right_lane_str_id_with_group.clear();
    }
    for (int i = 0; i < grp->lanes.size() - 1; ++i) {
      // 车道线是否逆向，类型不一致不能是左右邻
      if (grp->lanes[i]->left_boundary->isego !=
          grp->lanes[i + 1]->right_boundary->isego) {
        continue;
      }
      // 边线差距不能错车道
      if (!IsNearLine(grp->lanes[i]->right_boundary,
                      grp->lanes[i + 1]->left_boundary)) {
        continue;
      }
      grp->lanes[i]->right_lane_str_id_with_group.emplace_back(
          grp->lanes[i + 1]->str_id_with_group);
      grp->lanes[i + 1]->left_lane_str_id_with_group.emplace_back(
          grp->lanes[i]->str_id_with_group);
    }
  }
}
// 把Group里一个个GroupSegment中包含的小的LaneSegment，纵向上聚合成一个个大的Lane，
// 并且生成出：左右关联、前后关联、远端预测线、中心线
void GroupMap::GenLanesInGroups(std::vector<Group::Ptr>* groups,
                                std::map<em::Id, em::OccRoad::Ptr> occ_roads,
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
    // 区分出同向、反向车道
    //! 注意：当前同向、反向车道仅用于路口处排除关联到反向车道，普通路段不考虑同向、反向.
    // 同向、反向可能有多种隔离场景：单/双黄线、隔离带（路沿）、隔离栅栏等；
    // 当前仅考虑使用黄线隔离的场景，并且假设只有一根隔离（即仅考虑一根黄线）；当前不考虑双向车道.
    // 找到隔离线（当前只用黄线），判断每个车道的中心线在隔离线的左侧还是右侧，
    // 右侧为同向车道，左侧为反向车道.
    //! TBD：综合考虑黄线、路沿、栅栏等作为隔离线.
    SetGroupLaneOrient(grp);
    // 关联左右
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
  // 关联前后，并且把前后lane的中心线连接上
  // 关联同一个lanepos但不同trackid的前后继
  //! TBD:后续要加上斑马线、停止线等有标识的车道线前后继关系，以及前后关系计算
  // 删除脑部多的ego_group，青鸾号:1273597
  EraseEgoGroupWithNoEgoLane(groups);
  // 车道线补齐逻辑
  InferenceLaneLength(groups);
  AddVirtualLine(groups);
  // 设置lane属性(is_ego、is_tran)
  SetLaneStatus(groups);
  // 左车道断开或者右车道断开导致没形成道

  // 删除空的group数据
  int before_remove_grp_nums = groups->size();
  RemoveNullGroup(groups);
  int after_remove_grp_nums = groups->size();
  if (after_remove_grp_nums != before_remove_grp_nums) {
    HLOG_WARN << "*********[CrossWalk]*******delete null group nums: "
              << after_remove_grp_nums - before_remove_grp_nums;
  }

  // 停止线斑马线路面箭头与车道绑定
  for (auto& grp : *groups) {
    // 关联停止线
    MatchStopLineWithGroup(grp);
    // 关联斑马线
    MatchZebraWithGroup(grp);
  }

  if (groups->size() > 1) {
    // 仅保留前向只有一个路口存在
    RemainOnlyOneForwardCrossWalk(groups);
  }
  SetCurrentRoadScene(groups);
  RelateGroups(groups, stamp);
  NeighborLane(groups);
  // 1351477
  ExtendFrontCenterLine(groups);
  AvoidSplitMergeLane(groups);

  SmoothCenterline(groups);

#if 0
  OptiNextLane(groups);
#endif

  LaneForwardPredict(groups, stamp);
}

void GroupMap::CollectGroupPossibleLanes(
    Group::Ptr grp, std::vector<Lane::Ptr>* possible_lanes) {
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

bool GroupMap::FilterGroupBadLane(const std::vector<Lane::Ptr>& possible_lanes,
                                  Group::Ptr grp) {
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

bool GroupMap::MatchLRLane(Group::Ptr grp) {
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

bool GroupMap::MatchStopLineWithGroup(Group::Ptr grp) {
  // 将车道关联到停止线
  // HLOG_ERROR << "*****grp id: " << grp->seg_str_id;
  for (auto& stop_line : stopline_) {
    // grp已经关联过停止线后后面的grp不再关联,不然会关联到前后两个lane上
    // TODO(fml)
    // 关联到lane的相应位置而不是lane的头或尾，没有lane就关联到最近的lane尾部
    // HLOG_ERROR << "stop_line id: " << stop_line.first;
    if (stop_line.second->lane_id.size() > 0) {
      continue;
    }
    for (const auto& lane : grp->lanes) {
      // 过滤对向车道
      if (lane->left_boundary->isego == em::Other_Road) {
        continue;
      }
      if (MatchLaneAndStopLine(lane, stop_line.second)) {
        stop_line.second->lane_id.emplace_back(lane->str_id_with_group);
      }
    }
    // 如果当前grop中有一个lane能关联上，其它同向的都关联上
    if (stop_line.second->lane_id.size() > 0) {
      stop_line.second->lane_id.clear();
      for (const auto& lane : grp->lanes) {
        // 过滤对向车道
        if (lane->left_boundary->isego == em::Other_Road) {
          continue;
        }
        stop_line.second->lane_id.emplace_back(lane->str_id_with_group);
      }
    }
  }
  return true;
}

bool GroupMap::CloseToLaneEnd(const std::vector<Group::Ptr>& groups,
                              std::string str_id,
                              const Eigen::Vector3f& target_point) {
  for (const auto& grp : groups) {
    for (const auto& lane : grp->lanes) {
      if (lane->str_id_with_group == str_id) {
        if (lane->center_line_pts.size() >= 2) {
          Eigen::Vector3f front_pt = lane->center_line_pts.front().pt;
          Eigen::Vector3f end_pt = lane->center_line_pts.back().pt;
          auto length_1 = Dist(target_point, front_pt);
          auto length_2 = Dist(target_point, end_pt);
          if (length_2 < length_1) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

float GroupMap::LengthToLaneStart(const std::vector<Group::Ptr>& groups,
                                  std::string str_id,
                                  const Eigen::Vector3f& target_point) {
  float length2start{0.0};
  for (const auto& grp : groups) {
    for (const auto& lane : grp->lanes) {
      if (lane->str_id_with_group == str_id) {
        if (lane->center_line_pts.size() >= 2 &&
            lane->left_boundary->pts.size() >= 2 &&
            lane->right_boundary->pts.size() >= 2) {
          float length{0.0};
          em::Point prev_pt = lane->center_line_pts[0].pt;
          for (size_t i = 1; i < lane->center_line_pts.size(); ++i) {
            const auto& pt = lane->center_line_pts[i].pt;
            length += Dist(pt, prev_pt);
            prev_pt = pt;
          }
          auto start_left = lane->left_boundary->pts.front().pt;
          auto start_right = lane->right_boundary->pts.front().pt;
          auto end_left = lane->left_boundary->pts.back().pt;
          auto end_right = lane->right_boundary->pts.back().pt;
          Eigen::Vector2f stop_point(target_point.x(), target_point.y());
          if (PointInVectorSide(
                  Eigen::Vector2f(start_left.x(), start_left.y()),
                  Eigen::Vector2f(start_right.x(), start_right.y()),
                  stop_point) > 0) {
            length2start = 0.0;
          } else if (PointInVectorSide(
                         Eigen::Vector2f(end_left.x(), end_left.y()),
                         Eigen::Vector2f(end_right.x(), end_right.y()),
                         stop_point) < 0) {
            length2start = length - 0.1;
          } else {
            Eigen::Vector3f front_pt = lane->center_line_pts.front().pt;
            Eigen::Vector3f end_pt = lane->center_line_pts.back().pt;
            auto length_to_front = Dist(target_point, front_pt);
            auto length_to_end = Dist(target_point, end_pt);
            length2start = length_to_end < length_to_front
                               ? length - length_to_end - 0.1
                               : length_to_front - 0.1;
          }
        }
        return length2start;
      }
    }
  }
  return length2start;
}

bool GroupMap::MatchZebraWithGroup(Group::Ptr grp) {
  // 将车道关联到斑马线
  for (const auto& lane : grp->lanes) {
    for (auto& zebra_line : zebra_) {
      if (MatchLaneAndZebraLine(lane, zebra_line.second)) {
        zebra_line.second->lane_id.emplace_back(lane->str_id_with_group);
      }
    }
  }
  return true;
}

bool GroupMap::SetGroupLaneOrient(Group::Ptr grp) {
  std::vector<em::Point> separate;
  for (auto it = grp->lanes.rbegin(); it != grp->lanes.rend(); ++it) {
    LineSegment::Ptr yellow_bound = nullptr;
    if ((*it)->right_boundary->color == em::YELLOW &&
        (*it)->right_boundary->pts.size() > 1) {
      yellow_bound = (*it)->right_boundary;
    } else if ((*it)->left_boundary->color == em::YELLOW &&
               (*it)->left_boundary->pts.size() > 1) {
      yellow_bound = (*it)->left_boundary;
    }
    if (yellow_bound != nullptr) {
      auto start = yellow_bound->pts.front().pt;
      auto end = yellow_bound->pts.back().pt;
      separate.emplace_back(start);
      separate.emplace_back(end);
      break;
    }
  }
  // 没有隔离线，就默认所有车道都是同向
  if (separate.size() < 2) {
    for (auto& lane : grp->lanes) {
      lane->direction_type = em::DIRECTION_FORWARD;
    }
  } else {
    for (auto& lane : grp->lanes) {
      auto start_center = (lane->left_boundary->pts.front().pt +
                           lane->right_boundary->pts.front().pt) *
                          0.5;
      auto end_center = (lane->left_boundary->pts.back().pt +
                         lane->right_boundary->pts.back().pt) *
                        0.5;
      auto center = (start_center + end_center) * 0.5;
      Eigen::Vector2f p0(separate.front().x(), separate.front().y());
      Eigen::Vector2f p1(separate.back().x(), separate.back().y());
      Eigen::Vector2f pt(center.x(), center.y());
      if (PointInVectorSide(p0, p1, pt) > 0) {
        lane->direction_type = em::DIRECTION_FORWARD;
      } else {
        lane->direction_type = em::DIRECTION_BACKWARD;
      }
    }
  }
  return true;
}

bool GroupMap::GenLaneCenterLine(std::vector<Group::Ptr>* groups) {
  for (auto& grp : *groups) {
    for (auto& lane : grp->lanes) {
      // FitCenterLine(lane);
      ComputeCenterPoints(lane);
    }
  }

  return true;
}

bool GroupMap::OptiPreNextLaneBoundaryPoint(std::vector<Group::Ptr>* groups) {
  for (int i = 0; i < static_cast<int>(groups->size()) - 1; ++i) {
    auto& curr_grp = groups->at(i);
    auto& next_grp = groups->at(i + 1);
    std::map<em::Id, LineSegment::Ptr> curr_lines;
    std::map<em::Id, LineSegment::Ptr> next_lines;
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
      em::Id line_id = curr_line.first;
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
void GroupMap::VirtualLaneNeighborBefore(Group::Ptr curr_group,
                                         Group::Ptr next_group) {
  bool is_all_lane_has_ctlpf = true;
  for (auto& lane : curr_group->lanes) {
    lane->left_lane_str_id_with_group.clear();
    lane->right_lane_str_id_with_group.clear();
    if (lane->center_line_param.empty() || lane->center_line_pts.empty()) {
      is_all_lane_has_ctlpf = false;
      break;
    }
  }
  if (!curr_group->group_segments.empty() && is_all_lane_has_ctlpf &&
      !curr_group->lanes.empty()) {
    const auto& start_slice = curr_group->group_segments.back()->start_slice;
    Eigen::Vector2f start_po = start_slice.po.head<2>();
    const auto& end_slice = curr_group->group_segments.back()->end_slice;
    Eigen::Vector2f end_po = end_slice.po.head<2>();
    float po_y = curr_group->group_segments.back()->end_slice.po.y();
    std::sort(
        curr_group->lanes.begin(), curr_group->lanes.end(),
        [&start_po, &end_po](const Lane::Ptr& a, const Lane::Ptr& b) {
          Eigen::Vector2f center_a = a->center_line_pts.back().pt.head<2>();
          auto dist_a = PointToVectorDist(start_po, end_po, center_a);
          // 在path右边，距离设为负值
          if (PointInVectorSide(start_po, end_po, center_a) > 0) {
            dist_a = -1 * dist_a;
          }
          Eigen::Vector2f center_b = b->center_line_pts.back().pt.head<2>();
          auto dist_b = PointToVectorDist(start_po, end_po, center_b);
          // 在path右边，距离设为负值
          if (PointInVectorSide(start_po, end_po, center_b) > 0) {
            dist_b = -1 * dist_b;
          }
          return dist_a > dist_b;
        });
    MatchLRLane(curr_group);
  } else {
    std::unordered_map<std::string, int>
        curr_lane_index;  // curr_group的lane对应的index
    for (int i = 0; i < curr_group->lanes.size(); ++i) {
      curr_lane_index[curr_group->lanes[i]->str_id_with_group] = i;
      // 按照curr_group的左右邻来更新
      curr_group->lanes[i]->left_lane_str_id_with_group.clear();
      curr_group->lanes[i]->right_lane_str_id_with_group.clear();
    }
    std::unordered_map<std::string, int>
        next_lane_index;  // curr_group的lane对应的index
    for (int i = 0; i < next_group->lanes.size(); ++i) {
      next_lane_index[next_group->lanes[i]->str_id_with_group] = i;
    }
    for (int curr_index = 0; curr_index < next_group->lanes.size();
         ++curr_index) {
      if (next_group->lanes[curr_index]->prev_lane_str_id_with_group.empty()) {
        continue;
      }
      int index = curr_lane_index[next_group->lanes[curr_index]
                                      ->prev_lane_str_id_with_group[0]];
      auto& lane_prev =
          curr_group->lanes[index];  // curr_lane对应next_group里的后继lane
      for (auto& left_lane :
           next_group->lanes[curr_index]->left_lane_str_id_with_group) {
        // curr_lane所有的左边线
        int left_lane_index =
            next_lane_index[left_lane];  // 左边线对应curr_group的index
        if (next_group->lanes[left_lane_index]
                ->prev_lane_str_id_with_group.empty()) {
          continue;
        }
        std::string left_lane_next_id =
            next_group->lanes[left_lane_index]->prev_lane_str_id_with_group
                [0];  // 左边线对应next_group里的后继lane_id
        lane_prev->left_lane_str_id_with_group.emplace_back(
            left_lane_next_id);  // 将左边线lane_id添加到lane_next的左边线中
      }
      for (auto& right_lane :
           next_group->lanes[curr_index]->right_lane_str_id_with_group) {
        int right_lane_index = curr_lane_index[right_lane];
        if (next_group->lanes[right_lane_index]
                ->prev_lane_str_id_with_group.empty()) {
          continue;
        }
        std::string right_lane_next_id =
            next_group->lanes[right_lane_index]->prev_lane_str_id_with_group[0];
        lane_prev->right_lane_str_id_with_group.emplace_back(
            right_lane_next_id);
      }
    }

    // 存在少变多场景，导致误删左右邻。青鸾问题:1232877
    if (curr_group->lanes.size() > 1) {
      std::vector<int> curr_group_lanes;  // lane对应group的下标
      for (auto& lane : curr_group->lanes) {
        // HLOG_DEBUG << "lane->str_id is " << lane->str_id
        //           << "  next_group->str_id is " << next_group->str_id;
        if (curr_group->str_id.find(lane->str_id) <
            curr_group->str_id.length()) {
          // int index = next_group->str_id.find(lane->str_id);
          // HLOG_DEBUG << "lane->str_id is " << lane->str_id
          //           << "  next_group->str_id is " << next_group->str_id
          //           << "  index = " << index;
          curr_group_lanes.emplace_back(
              curr_lane_index[lane->str_id_with_group]);
        }
      }
      if (curr_group_lanes.empty()) {
        return;
      }
      for (int i = 0; i < curr_group_lanes.size(); i++) {
        int cur_lane_index = curr_group_lanes[i];
        if (cur_lane_index >= curr_group->lanes.size()) {
          continue;
        }
        for (int j = i + 1; j < curr_group_lanes.size(); ++j) {
          int right_lane_inex = curr_group_lanes[j];
          if (right_lane_inex >= curr_group->lanes.size()) {
            continue;
          }
          bool exist = IsRightLane(curr_group, cur_lane_index, right_lane_inex);

          if (!exist) {
            curr_group->lanes[cur_lane_index]
                ->right_lane_str_id_with_group.emplace_back(
                    curr_group->lanes[right_lane_inex]->str_id_with_group);
            curr_group->lanes[right_lane_inex]
                ->left_lane_str_id_with_group.emplace_back(
                    curr_group->lanes[cur_lane_index]->str_id_with_group);
          }
        }
      }
      // 由于补全的车道都添加在group->lanes的后面，所以直接从后面找
      for (int i = curr_group_lanes.back() + 1; i < curr_group->lanes.size();
           ++i) {
        int index =
            curr_group_lanes
                .size();  // 补全的lane相对与实际lane所在的index，物理意义：实际从下标index开始的lane的左边
        for (int j = 0; j < curr_group_lanes.size(); ++j) {
          int cur_lane_index = curr_group_lanes[j];
          bool exist_left = IsLeftLane(curr_group, cur_lane_index, i);

          if (exist_left) {
            index = j;
            break;
          }
        }
        // 将这根虚拟道next_group->lanes[i]，添加到其左边的右邻
        for (int j = 0; j < index; ++j) {
          int cur_lane_index = curr_group_lanes[j];
          bool exist_right =
              IsRightLane(curr_group, cur_lane_index, i);  // 是否已经存在右邻
          if (!exist_right) {
            curr_group->lanes[cur_lane_index]
                ->right_lane_str_id_with_group.emplace_back(
                    curr_group->lanes[i]->str_id_with_group);
            curr_group->lanes[i]->left_lane_str_id_with_group.emplace_back(
                curr_group->lanes[cur_lane_index]->str_id_with_group);
          }
        }
        // 将这根虚拟道next_group->lanes[i]，添加到其右边的左邻
        for (int j = index; j < curr_group_lanes.size(); ++j) {
          int cur_lane_index = curr_group_lanes[j];
          bool exist_left =
              IsLeftLane(curr_group, cur_lane_index, i);  // 是否已经存在左邻
          if (!exist_left) {
            curr_group->lanes[cur_lane_index]
                ->left_lane_str_id_with_group.emplace_back(
                    curr_group->lanes[i]->str_id_with_group);
            curr_group->lanes[i]->right_lane_str_id_with_group.emplace_back(
                curr_group->lanes[cur_lane_index]->str_id_with_group);
          }
        }
      }
    }
  }
}
void GroupMap::VirtualLaneLeftRight(Group::Ptr curr_group,
                                    Group::Ptr next_group) {
  bool is_all_lane_has_ctlpf = true;  // center_line_param_front
  for (auto& lane : next_group->lanes) {
    lane->left_lane_str_id_with_group.clear();
    lane->right_lane_str_id_with_group.clear();
    if (lane->center_line_param_front.empty() ||
        lane->center_line_pts.empty()) {
      is_all_lane_has_ctlpf = false;
      break;
    }
  }
  if (!next_group->group_segments.empty() && is_all_lane_has_ctlpf &&
      !next_group->lanes.empty()) {
    // 每条lane的centerline的第一个点到start_po end_po向量的距离
    const auto& start_slice = next_group->group_segments.front()->start_slice;
    Eigen::Vector2f start_po = start_slice.po.head<2>();
    const auto& end_slice = next_group->group_segments.front()->end_slice;
    Eigen::Vector2f end_po = end_slice.po.head<2>();
    float po_y = next_group->group_segments.front()->start_slice.po.y();
    std::sort(next_group->lanes.begin(), next_group->lanes.end(),
              [&start_po, &end_po](const Lane::Ptr& a, const Lane::Ptr& b) {
                Eigen::Vector2f center_a = a->center_line_pts[0].pt.head<2>();
                auto dist_a = PointToVectorDist(start_po, end_po, center_a);
                // 在path右边，距离设为负值
                if (PointInVectorSide(start_po, end_po, center_a) > 0) {
                  dist_a = -1 * dist_a;
                }
                Eigen::Vector2f center_b = b->center_line_pts[0].pt.head<2>();
                auto dist_b = PointToVectorDist(start_po, end_po, center_b);
                // 在path右边，距离设为负值
                if (PointInVectorSide(start_po, end_po, center_b) > 0) {
                  dist_b = -1 * dist_b;
                }
                return dist_a > dist_b;
              });
    MatchLRLane(next_group);
  } else {
    // 默认补齐的group的lanenumber一致，且一条lane有且仅有一个后继或前驱
    // is_after对应的从前往后或者从后往前
    std::unordered_map<std::string, int>
        next_lane_index;  // next_group的lane对应的index
    for (int i = 0; i < next_group->lanes.size(); ++i) {
      next_lane_index[next_group->lanes[i]->str_id_with_group] = i;
      // 按照curr_group的左右邻来更新
      next_group->lanes[i]->left_lane_str_id_with_group.clear();
      next_group->lanes[i]->right_lane_str_id_with_group.clear();
    }
    std::unordered_map<std::string, int>
        curr_lane_index;  // curr_group的lane对应的index
    for (int i = 0; i < curr_group->lanes.size(); ++i) {
      curr_lane_index[curr_group->lanes[i]->str_id_with_group] = i;
    }
    for (int curr_index = 0; curr_index < curr_group->lanes.size();
         ++curr_index) {
      if (curr_group->lanes[curr_index]->next_lane_str_id_with_group.empty()) {
        continue;
      }
      int index = next_lane_index[curr_group->lanes[curr_index]
                                      ->next_lane_str_id_with_group[0]];
      auto& lane_next =
          next_group->lanes[index];  // curr_lane对应next_group里的后继lane
      for (auto& left_lane :
           curr_group->lanes[curr_index]->left_lane_str_id_with_group) {
        // curr_lane所有的左边线
        int left_lane_index =
            curr_lane_index[left_lane];  // 左边线对应curr_group的index
        if (curr_group->lanes[left_lane_index]
                ->next_lane_str_id_with_group.empty()) {
          continue;
        }
        std::string left_lane_next_id =
            curr_group->lanes[left_lane_index]->next_lane_str_id_with_group
                [0];  // 左边线对应next_group里的后继lane_id
        lane_next->left_lane_str_id_with_group.emplace_back(
            left_lane_next_id);  // 将左边线lane_id添加到lane_next的左边线中
      }
      for (auto& right_lane :
           curr_group->lanes[curr_index]->right_lane_str_id_with_group) {
        int right_lane_index = curr_lane_index[right_lane];
        if (curr_group->lanes[right_lane_index]
                ->next_lane_str_id_with_group.empty()) {
          continue;
        }
        std::string right_lane_next_id =
            curr_group->lanes[right_lane_index]->next_lane_str_id_with_group[0];
        lane_next->right_lane_str_id_with_group.emplace_back(
            right_lane_next_id);
      }
    }

    // 存在少变多场景，导致误删左右邻。青鸾问题:1232877
    if (next_group->lanes.size() > 1) {
      std::vector<int> next_group_lanes;  // lane对应group的下标
      for (auto& lane : next_group->lanes) {
        // HLOG_DEBUG << "lane->str_id is " << lane->str_id
        //           << "  next_group->str_id is " << next_group->str_id;
        if (next_group->str_id.find(lane->str_id) <
            next_group->str_id.length()) {
          // int index = next_group->str_id.find(lane->str_id);
          // HLOG_DEBUG << "lane->str_id is " << lane->str_id
          //           << "  next_group->str_id is " << next_group->str_id
          //           << "  index = " << index;
          next_group_lanes.emplace_back(
              next_lane_index[lane->str_id_with_group]);
        }
      }
      if (next_group_lanes.empty()) {
        return;
      }
      for (int i = 0; i < next_group_lanes.size(); i++) {
        int cur_lane_index = next_group_lanes[i];
        if (cur_lane_index >= next_group->lanes.size()) {
          continue;
        }
        for (int j = i + 1; j < next_group_lanes.size(); ++j) {
          int right_lane_inex = next_group_lanes[j];
          if (right_lane_inex >= next_group->lanes.size()) {
            continue;
          }
          bool exist = IsRightLane(next_group, cur_lane_index, right_lane_inex);

          if (!exist) {
            next_group->lanes[cur_lane_index]
                ->right_lane_str_id_with_group.emplace_back(
                    next_group->lanes[right_lane_inex]->str_id_with_group);
            next_group->lanes[right_lane_inex]
                ->left_lane_str_id_with_group.emplace_back(
                    next_group->lanes[cur_lane_index]->str_id_with_group);
          }
        }
      }
      // 由于补全的车道都添加在group->lanes的后面，所以直接从后面找
      for (int i = next_group_lanes.back() + 1; i < next_group->lanes.size();
           ++i) {
        int index = next_group_lanes.size();  //
        // 补全的lane相对与实际lane所在的index，物理意义：实际从下标index开始的lane的左边
        for (int j = 0; j < next_group_lanes.size(); ++j) {
          int cur_lane_index = next_group_lanes[j];
          bool exist_left = IsLeftLane(next_group, cur_lane_index, i);

          if (exist_left) {
            index = j;
            break;
          }
        }
        // 将这根虚拟道next_group->lanes[i]，添加到其左边的右邻
        for (int j = 0; j < index; ++j) {
          int cur_lane_index = next_group_lanes[j];
          bool exist_right =
              IsRightLane(next_group, cur_lane_index, i);  // 是否已经存在右邻
          if (!exist_right) {
            next_group->lanes[cur_lane_index]
                ->right_lane_str_id_with_group.emplace_back(
                    next_group->lanes[i]->str_id_with_group);
            next_group->lanes[i]->left_lane_str_id_with_group.emplace_back(
                next_group->lanes[cur_lane_index]->str_id_with_group);
          }
        }
        // 将这根虚拟道next_group->lanes[i]，添加到其右边的左邻
        for (int j = index; j < next_group_lanes.size(); ++j) {
          int cur_lane_index = next_group_lanes[j];
          bool exist_left =
              IsLeftLane(next_group, cur_lane_index, i);  // 是否已经存在左邻
          if (!exist_left) {
            next_group->lanes[cur_lane_index]
                ->left_lane_str_id_with_group.emplace_back(
                    next_group->lanes[i]->str_id_with_group);
            next_group->lanes[i]->right_lane_str_id_with_group.emplace_back(
                next_group->lanes[cur_lane_index]->str_id_with_group);
          }
        }
      }
    }
  }
}
bool GroupMap::IsRightLane(Group::Ptr next_group, int cur_lane_index,
                           int right_lane_inex) {
  for (auto right_lane_name :
       next_group->lanes[cur_lane_index]->right_lane_str_id_with_group) {
    if (right_lane_name ==
        next_group->lanes[right_lane_inex]->str_id_with_group) {
      return true;
      break;
    }
  }
  return false;
}

bool GroupMap::IsLeftLane(Group::Ptr next_group, int cur_lane_index,
                          int left_lane_index) {
  for (auto left_lane_name :
       next_group->lanes[cur_lane_index]->left_lane_str_id_with_group) {
    if (left_lane_name ==
        next_group->lanes[left_lane_index]->str_id_with_group) {
      return true;
    }
  }
  return false;
}
bool GroupMap::ContainEgoLane(std::vector<Group::Ptr>* groups,
                              int next_grp_index) {
  // 青鸾号:1250597
  for (int grp_idx = next_grp_index; grp_idx < static_cast<int>(groups->size());
       ++grp_idx) {
    auto& next_groups = groups->at(grp_idx);
    for (auto next_grp_lane : next_groups->lanes) {
      if (next_grp_lane->lanepos_id == "-1_1") {
        return true;
      }
    }
  }
  return false;
}
bool GroupMap::InferenceLaneLength(std::vector<Group::Ptr>* groups) {
  // 从前往后
  if (groups->size() < 2) {
    return true;
  }
  for (int i = 0; i < static_cast<int>(groups->size()) - 1; ++i) {
    auto& curr_group = groups->at(i);
    auto& next_group = groups->at(i + 1);
    if (curr_group->group_segments.empty() ||
        next_group->group_segments.empty()) {
      continue;
    }
    bool is_any_next_lane_exit =
        false;  // 前后group是否存在某个车道根据trackid关联
    bool is_all_next_lane_exit =
        true;  // currgroup的车道是否全部都有nextgroup关联
    double group_distance = (curr_group->group_segments.back()->end_slice.po -
                             next_group->group_segments.front()->start_slice.po)
                                .norm();
    double currgrp_nearest_mindis_to_nextgrp = DBL_MAX;
    for (auto& cur_group_lane : curr_group->lanes) {
      double calcu_dis = (cur_group_lane->center_line_pts.back().pt -
                          next_group->group_segments.front()->start_slice.po)
                             .norm();
      if (calcu_dis < currgrp_nearest_mindis_to_nextgrp) {
        currgrp_nearest_mindis_to_nextgrp = calcu_dis;
      }
    }

    if (group_distance > 10 && currgrp_nearest_mindis_to_nextgrp > 10) {
      // 路口场景不补全
      continue;
    }
    //! TBD:
    //! 把分合流场景判断进来，使得分合流的场景不向前延伸。FindGroupNextLane不用再判断一遍前后继了直接沿用
    for (auto& lane_in_curr : curr_group->lanes) {
      bool next_lane_exit = false;  // currlane是否有后继
      // HLOG_ERROR << "curr+lane = " << lane_in_curr->str_id_with_group;
      for (auto& lane_in_next : next_group->lanes) {
        if (lane_in_curr->str_id == lane_in_next->str_id ||
            lane_in_curr->left_boundary->id ==
                lane_in_next->left_boundary->id ||
            lane_in_curr->right_boundary->id ==
                lane_in_next->right_boundary->id) {
          // 目前这个是否有next是为了记录是否需要补道，如果有后继就不用补道
          lane_in_curr->next_lane_str_id_with_group.emplace_back(
              lane_in_next->str_id_with_group);
          // HLOG_ERROR << "the next lane is " <<
          // lane_in_next->str_id_with_group;
          lane_in_curr->left_boundary->id_next =
              lane_in_next->left_boundary->id;
          lane_in_curr->right_boundary->id_next =
              lane_in_next->right_boundary->id;
          lane_in_next->prev_lane_str_id_with_group.emplace_back(
              lane_in_curr->str_id_with_group);

          // ! TBD:
          // ！这里直接把后一个lane中心线的第一个点加到前一个lane中心线的末尾，
          // !后续需要考虑某些异常情况，比如后一个lane中心线的第一个点在前一个lane中心线最后
          // !一个点的后方，这样直连就导致整个中心线往后折返了；以及还要考虑横向偏移较大时不平
          // ! 滑的问题
          // if (!lane_in_next->center_line_pts.empty()) {
          // lane_in_curr->center_line_pts.emplace_back(
          //     lane_in_next->center_line_pts.front());
          // }
          is_any_next_lane_exit = true;
          next_lane_exit = true;
          if (lane_in_next->center_line_param.empty()) {
            lane_in_next->center_line_param = lane_in_curr->center_line_param;
          }
          if (lane_in_next->center_line_param_front.empty()) {
            lane_in_next->center_line_param_front =
                lane_in_next->center_line_param;
          }
          break;
        }
        if (AreLaneConnect(lane_in_curr, lane_in_next)) {
          lane_in_curr->next_lane_str_id_with_group.emplace_back(
              lane_in_next->str_id_with_group);
          lane_in_curr->left_boundary->id_next =
              lane_in_next->left_boundary->id;
          lane_in_curr->right_boundary->id_next =
              lane_in_next->right_boundary->id;
          // HLOG_ERROR << "the next lane is " <<
          // lane_in_next->str_id_with_group;
          is_any_next_lane_exit = true;
          next_lane_exit = true;
          // if (lane_in_next->center_line_param.empty()) {
          //   lane_in_next->center_line_param =
          //   lane_in_curr->center_line_param;
          // }
          // if (lane_in_next->center_line_param_front.empty()) {
          //   lane_in_next->center_line_param_front =
          //       lane_in_next->center_line_param;
          // }
          break;
        }
      }
      if (!next_lane_exit) {
        double curr_len = CalcLaneLength(lane_in_curr);
        bool shrink = IsLaneShrink(lane_in_curr);
        const float dis_thresh = 4.5;
        if (lane_in_curr->str_id_with_group ==
                curr_group->lanes[0]->str_id_with_group &&
            curr_len > kMergeLengthThreshold && shrink &&
            IsAccessLane(lane_in_curr, next_group->lanes[0]) &&
            LaneDist(lane_in_curr, next_group->lanes[0]) < dis_thresh &&
            CalcLaneLength(next_group->lanes[0]) > kMergeLengthThreshold) {
          lane_in_curr->next_lane_str_id_with_group.emplace_back(
              next_group->lanes[0]->str_id_with_group);
          next_lane_exit = true;
          // HLOG_ERROR << "the next lane is "
          //            << next_group->lanes[0]->str_id_with_group;
        } else if (lane_in_curr->str_id_with_group ==
                       curr_group->lanes.back()->str_id_with_group &&
                   curr_len > kMergeLengthThreshold && shrink &&
                   IsAccessLane(lane_in_curr, next_group->lanes.back()) &&
                   LaneDist(lane_in_curr, next_group->lanes.back()) <
                       dis_thresh &&
                   CalcLaneLength(next_group->lanes.back()) >
                       kMergeLengthThreshold) {
          lane_in_curr->next_lane_str_id_with_group.emplace_back(
              next_group->lanes.back()->str_id_with_group);
          next_lane_exit = true;
          // HLOG_ERROR << "the next lane is "
          //            << next_group->lanes.back()->str_id_with_group;
        }
      }
      if (!next_lane_exit) {
        is_all_next_lane_exit = false;
      }
    }
    if (is_any_next_lane_exit) {
      // 查看trackid一致但是groupsize不一致的情况
      // 如果是线段长短不一致
      // float max_length_lane = 0.0;
      // for (auto& lane : next_group->lanes) {
      //   size_t size_lane = lane->center_line_pts.size();
      //   float len = (lane->center_line_pts[size_lane - 1].pt -
      //                lane->center_line_pts[0].pt)
      //                   .norm();
      //   if (len > max_length_lane) {
      //     max_length_lane = len;
      //   }
      // }

      // if (max_length_lane <= 10.0 && is_all_next_lane_exit == 0) {
      //   BuildVirtualLaneAfter(curr_group, next_group);
      //   // groups->erase(groups->begin() + i + 1);
      //   // i--;
      // }
      double group_distance =
          (curr_group->group_segments.back()->end_slice.po -
           next_group->group_segments.front()->start_slice.po)
              .norm();
      if (!is_all_next_lane_exit && group_distance < 10.0) {
        UpdateLaneBoundaryId(curr_group);
        // 后侧车道线补齐
        BuildVirtualLaneAfter(curr_group, next_group);
        EraseIntersectLane(curr_group, next_group);
        VirtualLaneLeftRight(curr_group, next_group);
      }
      DelLaneNextStrIdInGroup(curr_group);
    } else {
      if (!ContainEgoLane(groups, i + 1)) {
        // &&
        // curr_group->group_segments.back()->end_slice.po.x() > 2.0
        groups->erase(groups->begin() + i + 1);
        i--;
      }
    }
  }

  // 从后往前
  for (int i = static_cast<int>(groups->size()) - 1; i > 0; --i) {
    auto& curr_group = groups->at(i - 1);
    auto& next_group = groups->at(i);
    if (curr_group->group_segments.empty() ||
        next_group->group_segments.empty()) {
      continue;
    }
    bool is_any_next_lane_exit =
        false;  // 前后group是否存在某个车道根据trackid关联
    bool is_all_next_lane_exit =
        true;  // currgroup的车道是否全部都有nextgroup关联
    for (auto& lane_in_next : next_group->lanes) {
      bool next_lane_exit = false;  // next_group是否有前驱（0:没有,1:有）
      // HLOG_ERROR << "next+lane = " << lane_in_next->str_id_with_group;
      for (auto& lane_in_curr : curr_group->lanes) {
        if (lane_in_curr->str_id == lane_in_next->str_id ||
            lane_in_curr->left_boundary->id ==
                lane_in_next->left_boundary->id ||
            lane_in_curr->right_boundary->id ==
                lane_in_next->right_boundary->id) {
          is_any_next_lane_exit = true;
          next_lane_exit = true;
          // HLOG_ERROR << "the prev lane is " <<
          // lane_in_curr->str_id_with_group;
          lane_in_next->prev_lane_str_id_with_group.emplace_back(
              lane_in_curr->str_id_with_group);
          if (lane_in_curr->center_line_param.empty()) {
            lane_in_curr->center_line_param =
                lane_in_next->center_line_param_front;
          }
          if (lane_in_curr->center_line_param_front.empty()) {
            lane_in_curr->center_line_param_front =
                lane_in_curr->center_line_param;
          }
          break;
        }
        if (AreLaneConnect(lane_in_curr, lane_in_next)) {
          is_any_next_lane_exit = true;
          next_lane_exit = true;
          lane_in_next->prev_lane_str_id_with_group.emplace_back(
              lane_in_curr->str_id_with_group);
          // HLOG_ERROR << "the prev lane is " <<
          // lane_in_curr->str_id_with_group;
          // if(lane_in_curr->center_line_param.empty()) {
          //   lane_in_curr->center_line_param =
          //       lane_in_next->center_line_param_front;
          // }
          // if (lane_in_curr->center_line_param_front.empty()) {
          //   lane_in_curr->center_line_param_front =
          //       lane_in_curr->center_line_param;
          // }
          break;
        }
      }
      if (!next_lane_exit) {
        is_all_next_lane_exit = false;
      }
    }
    if (is_any_next_lane_exit || ContainEgoLane(groups, i)) {
      // 查看trackid一致但是groupsize不一致的情况
      // 如果是线段长短不一致
      float max_length_lane = 0.0;
      for (auto& lane : curr_group->lanes) {
        size_t size_lane = lane->center_line_pts.size();
        float len = (lane->center_line_pts[size_lane - 1].pt -
                     lane->center_line_pts[0].pt)
                        .norm();
        if (len > max_length_lane) {
          max_length_lane = len;
        }
      }
      double group_distance =
          (curr_group->group_segments.back()->end_slice.po -
           next_group->group_segments.front()->start_slice.po)
              .norm();
      // HLOG_DEBUG << "max_length_lane = " << max_length_lane;
      if (((IsGroupsNoEgoLane(groups, i - 1) && max_length_lane <= 50.0) ||
           max_length_lane <= 15.0) &&
          !is_all_next_lane_exit && group_distance < 10.0) {
        // HLOG_DEBUG << " is connect";
        BuildVirtualLaneBefore(curr_group, next_group);
        VirtualLaneNeighborBefore(curr_group, next_group);
        // groups->erase(groups->begin() + i - 1);
      }
      DelLanePrevStrIdInGroup(next_group);
    }
  }

  return true;
}
bool GroupMap::IsGroupNoEgoLane(Group::Ptr group) {
  for (auto& lane : group->lanes) {
    if (lane->left_boundary->pts.empty() || lane->right_boundary->pts.empty()) {
      continue;
    }
    auto left_front_point = lane->left_boundary->pts[0].pt;
    auto left_back_point = lane->left_boundary->pts.back().pt;
    auto right_front_point = lane->right_boundary->pts[0].pt;
    auto right_back_point = lane->right_boundary->pts.back().pt;
    float is_leftline_right =
        left_front_point.y() * left_back_point.x() -
        left_front_point.x() * left_back_point.y();  // >0在右侧 <0 在左侧
    float is_rightline_right = right_front_point.y() * right_back_point.x() -
                               right_front_point.x() * right_back_point.y();
    if (is_leftline_right * is_rightline_right <= 0.0) {
      return false;
    }
  }
  return true;
}

bool GroupMap::IsGroupsNoEgoLane(std::vector<Group::Ptr>* groups,
                                 int curr_group_index) {
  if (curr_group_index >= groups->size()) {
    return false;
  }
  if (groups->at(curr_group_index)->group_segments.empty()) {
    return false;
  }
  if (groups->at(curr_group_index)->group_segments.back()->end_slice.po.x() <
      2.0) {
    // 不关心自车后方group
    return false;
  }
  for (int i = curr_group_index; i >= 0; i--) {
    auto curr_group = groups->at(curr_group_index);
    if (curr_group->group_segments.empty()) {
      continue;
    }
    if (curr_group->group_segments.back()->end_slice.po.x() < -2.0) {
      break;
    }
    if (!IsGroupNoEgoLane(curr_group)) {
      return false;
    }
    if (curr_group->group_segments.front()->start_slice.po.x() > 0.0) {
      // 到自车group为止都没有egolane
      return true;
    }
  }

  return true;
}

bool GroupMap::IsInGroupAndNoLane(Group::Ptr group) {
  if (group->group_segments.size() < 2.0) {
    return false;
  }
  if (group->group_segments.front()->start_slice.po.x() > 2.0 ||
      group->group_segments.back()->end_slice.po.x() < -2.0) {
    return false;
  }

  return IsGroupNoEgoLane(group);
}

bool GroupMap::SetLaneStatus(std::vector<Group::Ptr>* groups) {
  // 添加主路和是否当前朝向属性
  for (auto& group : *groups) {
    int flag = 0;
    for (auto& lane : group->lanes) {
      if (lane->left_boundary->isego == em::Ego_Road) {
        lane->is_ego = 1;
      }
      if (flag == 0) {
        if (lane->left_boundary->color == em::YELLOW) {
          flag = 1;
          lane->is_trans = 1;
        }
      } else {
        lane->is_trans = 1;
      }
    }
    if (flag == 0) {
      for (auto& lane : group->lanes) {
        if (lane->is_ego) {
          lane->is_trans = 1;
        }
      }
    }
  }
  return true;
}

bool GroupMap::OptiNextLane(std::vector<Group::Ptr>* groups) {
  //! TBD: 临时修改，解除多个后继，仅保留角度变化最小的一个后继
  HLOG_DEBUG << "get success of smallest angle";
  std::map<std::string, LaneWithNextLanes> lanes_has_next;
  if (groups->size() > 1) {
    for (int i = 0; i < static_cast<int>(groups->size()) - 1; ++i) {
      auto& curr_grp = groups->at(i);
      auto& next_grp = groups->at(i + 1);
      for (auto& curr_lane : curr_grp->lanes) {
        LaneWithNextLanes lanes_next;
        lanes_next.lane = curr_lane;
        for (const auto& id : curr_lane->next_lane_str_id_with_group) {
          for (auto& next_lane : next_grp->lanes) {
            if (id == next_lane->str_id_with_group) {
              lanes_next.next_lanes.emplace_back(next_lane);
            }
          }
        }
        if (lanes_next.next_lanes.size() > 1) {
          lanes_has_next.insert_or_assign(curr_lane->str_id_with_group,
                                          lanes_next);
        }
      }
    }
  }

  for (auto it : lanes_has_next) {
    auto curr_lane = it.second.lane;
    auto curr_pts_size = curr_lane->center_line_pts.size();
    if (curr_pts_size < 2) {
      continue;
    }
    Eigen::Vector2f c0(curr_lane->center_line_pts.at(curr_pts_size - 2).pt.x(),
                       curr_lane->center_line_pts.at(curr_pts_size - 2).pt.y());
    Eigen::Vector2f c1(curr_lane->center_line_pts.at(curr_pts_size - 1).pt.x(),
                       curr_lane->center_line_pts.at(curr_pts_size - 1).pt.y());
    Eigen::Vector2f c = c1 - c0;
    c.normalize();
    Lane::Ptr best_next = nullptr;
    float max_len = 0;
    for (auto next_lane : it.second.next_lanes) {
      auto next_pts_size = next_lane->center_line_pts.size();
      if (next_pts_size < 3) {
        continue;
      }
      Eigen::Vector2f n0(next_lane->center_line_pts.at(0).pt.x(),
                         next_lane->center_line_pts.at(0).pt.y());
      Eigen::Vector2f n1(next_lane->center_line_pts.at(1).pt.x(),
                         next_lane->center_line_pts.at(1).pt.y());
      Eigen::Vector2f n = n1 - n0;
      if (n.norm() < 0.01) {
        n0 = n1;
        n1 << next_lane->center_line_pts.at(2).pt.x(),
            next_lane->center_line_pts.at(2).pt.y();
        n = n1 - n0;
      }
      n.normalize();
      float len = std::abs(n.transpose() * c);
      if (len > max_len) {
        max_len = len;
        best_next = next_lane;
      }
    }
    if (best_next != nullptr) {
      curr_lane->next_lane_str_id_with_group.clear();
      curr_lane->next_lane_str_id_with_group.emplace_back(
          best_next->str_id_with_group);
      best_next->prev_lane_str_id_with_group.clear();
      best_next->prev_lane_str_id_with_group.emplace_back(
          curr_lane->str_id_with_group);
    }
  }

  return true;
}

void GroupMap::ComputeLineHeadingPredict(
    std::vector<Group::Ptr>* groups,
    std::vector<LineSegment::Ptr>* lines_need_pred) {
  for (auto& line : *lines_need_pred) {
    std::vector<Vec2d> line_points;
    // 从group中往前递推寻找同一个track id的车道线并把点填充到line points当中
    for (auto it = groups->rbegin(); it != groups->rend(); ++it) {
      if ((*it) == nullptr || (*it)->lanes.empty()) {
        continue;
      }
      std::vector<Vec2d> line_points_temp;
      for (const auto& lane : (*it)->lanes) {
        if (lane->left_boundary->id == line->id ||
            lane->right_boundary->id == line->id) {
          std::vector<Vec2d> lane_boundary_points;
          if (lane->left_boundary->id == line->id) {
            for (const auto& point : lane->left_boundary->pts) {
              lane_boundary_points.emplace_back(point.pt.x(), point.pt.y());
            }
          } else {
            for (const auto& point : lane->right_boundary->pts) {
              lane_boundary_points.emplace_back(point.pt.x(), point.pt.y());
            }
          }
          Vec2d dist_point_temp;
          for (const auto& point : lane_boundary_points) {
            Vec2d temp_point;
            temp_point.set_x(dist_point_temp.x() - point.x());
            temp_point.set_y(dist_point_temp.y() - point.y());
            if (temp_point.Length() >= 0.4) {
              line_points_temp.emplace_back(point.x(), point.y());
            }
            dist_point_temp.set_x(point.x());
            dist_point_temp.set_y(point.y());
          }
          break;
        }
      }

      line_points_temp.insert(line_points_temp.end(), line_points.begin(),
                              line_points.end());
      line_points.clear();
      line_points = line_points_temp;
      if (line_points.size() >= 10) {
        break;
      }
    }

    double heading = 0;
    std::vector<double> thetas;
    for (int i = static_cast<int>(line_points.size()) - 1; i > 0; i--) {
      int j = i - 1;
      for (; j >= 0; j--) {
        const Eigen::Vector2f pb(line_points[i].x(), line_points[i].y());
        const Eigen::Vector2f pa(line_points[j].x(), line_points[j].y());
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
    heading =
        thetas.empty()
            ? 0.0
            : std::accumulate(thetas.begin(), thetas.end(), 0.) / thetas.size();

    std::vector<double> fit_result;
    std::vector<Vec2d> fit_points;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    double kappa = 0.;
    double dkappa = 0.;
    if (line_points.size() >= 10) {
      int fit_num = std::min(50, static_cast<int>(line_points.size()));
      std::copy(line_points.rbegin(), line_points.rbegin() + fit_num,
                std::back_inserter(fit_points));
      math::FitLaneLinePoint(fit_points, &fit_result);
      std::vector<Vec2d> cmp_points;
      std::copy(line_points.rbegin(), line_points.rbegin() + 10,
                std::back_inserter(cmp_points));
      math::ComputeDiscretePoints(cmp_points, fit_result, &kappas, &dkappas);
      // 整体平均
      kappa = kappas.empty() ? 0.0
                             : std::accumulate(kappas.begin() + 5,
                                               kappas.begin() + 10, 0.) /
                                   5.;
      dkappa = dkappas.empty() ? 0.0
                               : std::accumulate(dkappas.begin() + 5,
                                                 dkappas.begin() + 10, 0.) /
                                     5.;
      kappa = kappa / 8.;
      dkappa = dkappa / 8.;
    }

    line->pred_end_heading = std::make_tuple(heading, kappa, dkappa);
  }
}

bool GroupMap::LaneForwardPredict(std::vector<Group::Ptr>* groups,
                                  const double& stamp) {
  // 对远处车道线进行预测，仅对无后继的lane尝试预测
  HLOG_DEBUG << "predict success lane";
  if (conf_.predict_farthest_dist > conf_.robust_percep_dist) {
    // 1. 删除待删除状态的group
    int before_delete_group_nums = groups->size();
    groups->erase(std::remove_if(groups->begin(), groups->end(),
                                 [&](Group::Ptr& group) {
                                   return (group->group_state ==
                                           Group::GroupState::DELETE);
                                 }),
                  groups->end());
    int after_delete_group_nums = groups->size();
    if ((before_delete_group_nums > after_delete_group_nums) &&
        !groups->empty()) {
      groups->back()->is_last_after_erased = true;
    }

    Group::Ptr last_grp = nullptr;
    // 找到最后一个非空的group，只预测最后一个group里的lane
    for (auto it = groups->rbegin(); it != groups->rend(); ++it) {
      if ((*it) == nullptr || (*it)->lanes.empty()) {
        continue;
      }
      last_grp = *it;
      break;
    }
    bool need_pred_kappa = true;
    if (last_grp != nullptr) {
      // 判断车辆距离group最后一个segment的距离，如果太近，就认为是路口，就将kappa、dkappa置为0
      if (!last_grp->group_segments.empty()) {
        auto last_grp_end_slice = last_grp->group_segments.back()->end_slice;
        Eigen::Vector2f end_pl(last_grp_end_slice.pl.x(),
                               last_grp_end_slice.pl.y());
        Eigen::Vector2f end_pr(last_grp_end_slice.pr.x(),
                               last_grp_end_slice.pr.y());
        Eigen::Vector2f curr_pos(0, 0);
        auto dist_to_slice = PointToVectorDist(end_pl, end_pr, curr_pos);
        if (dist_to_slice <= 30) {
          need_pred_kappa = false;
        }
      }

      bool check_back = true;
      if (last_grp->is_last_after_erased) {
        check_back = false;
      }
      std::vector<Lane::Ptr> lanes_wo_next;  // 末端lane，即无后继的lane

      for (const auto& lane : last_grp->lanes) {
        if (lane == nullptr) {
          HLOG_ERROR << "found nullptr lane";
          continue;
        }
        if (lane->next_lane_str_id_with_group.empty()) {
          lanes_wo_next.emplace_back(lane);
        }
      }
      std::vector<LineSegment::Ptr> lines_need_pred;
      std::vector<Lane::Ptr> lanes_need_pred;
      for (auto& lane : lanes_wo_next) {
        int lane_center_need_pre = 0;
        if (lane->left_boundary != nullptr &&
            LaneLineNeedToPredict(*lane->left_boundary, check_back)) {
          lane_center_need_pre++;
        }
        if (lane->right_boundary != nullptr &&
            LaneLineNeedToPredict(*lane->right_boundary, check_back)) {
          lane_center_need_pre++;
        }
        // 左右边线都满足预测要求才会往前预测
        if (lane_center_need_pre == 2) {
          lines_need_pred.emplace_back(lane->left_boundary);
          lines_need_pred.emplace_back(lane->right_boundary);
          lanes_need_pred.emplace_back(lane);
        }
      }
      // 计算切割并构建group之后的车道线heading避免local map尾端不准带来误差
      ComputeLineHeadingPredict(groups, &lines_need_pred);
      // 使用平均heading，这样可以使得预测的线都是平行的，不交叉
      // 由于部分弯道heading偏差较大，导致整体平均heading发生偏差，
      // 现增加pred_end_heading字段用于预测，使用PCL欧式聚类对heading进行聚类
      // util::TicToc heading_cluster_tic;
      // util::common tic_common;
      if (!lines_need_pred.empty()) {
        // PCL欧式聚类 阈值为10度
        const double heading_threshold = 10. / 180. * M_PI;
        // heading_cluster_tic.Tic();
        HeadingCluster(lanes_need_pred, &lines_need_pred, heading_threshold,
                       need_pred_kappa);
        // auto time_cost = tic_common.Precusion(heading_cluster_tic.Toc(), 2);
        // HLOG_ERROR << "HeadingCluster cost:" << time_cost;
        // 重新构建group 线的类型为虚线
        std::vector<Lane::Ptr> center_line_pred;
        for (auto& lane : lanes_need_pred) {
          PredictLaneLine(&center_line_pred, lane);
        }
        Group grp;
        grp.stamp = stamp;
        grp.str_id = last_grp->str_id + "P";
        grp.lanes = center_line_pred;
        (*groups).emplace_back(std::make_shared<Group>(grp));
      }
    }
  }

  return true;
}

void GroupMap::SetAllOverlaps(std::map<em::Id, Zebra::Ptr>* zebra,
                              std::map<em::Id, Arrow::Ptr>* arrow,
                              std::map<em::Id, Stpl::Ptr>* stopline,
                              std::map<em::Id, Overlaps::Ptr>* overlap,
                              Lane::Ptr lane) {
  // for (auto& zeb : *zebra) {
  //   auto pts = zeb.second->polygon.points;
  //   if ((pts[0] - pts[1]).norm() < (pts[1] - pts[2]).norm()) {
  //   }
  // }

  // for (auto& arw : *arrow) {
  //   auto center =
  //       (arw.second->polygon.points[0] + arw.second->polygon.points[1] +
  //        arw.second->polygon.points[2] + arw.second->polygon.points[3]) /
  //       4;
  //   for(size_t i = 0; i < lane->center_line_pts.size() - 1; ++i){
  //     if(center.x() >= lane->center_line_pts[i].pt.x() && center.x() <=
  //     lane->center_line_pts[i].pt.x()){
  //       if()
  //       break;
  //     }
  //   }
  // }
}

void GroupMap::BuildVirtualGroup(std::vector<Lane::Ptr> virtual_lanes,
                                 std::vector<Group::Ptr>* group_virtual,
                                 double stamp) {
  Group grp;
  grp.stamp = stamp;
  grp.lanes.emplace_back(virtual_lanes[0]);
  grp.group_state = Group::GroupState::VIRTUAL;
  grp.str_id = virtual_lanes[0]->str_id_with_group.substr(
      0, virtual_lanes[0]->str_id_with_group.find(":"));
  (*group_virtual).emplace_back(std::make_shared<Group>(grp));
  if (virtual_lanes.size() > 1) {
    Group grp;
    grp.stamp = stamp;
    grp.group_state = Group::GroupState::VIRTUAL;
    for (int i = 1; i < virtual_lanes.size(); ++i) {
      grp.lanes.emplace_back(virtual_lanes[i]);
    }
    grp.str_id = virtual_lanes[1]->str_id_with_group.substr(
        0, virtual_lanes[1]->str_id_with_group.find(":"));
    (*group_virtual).emplace_back(std::make_shared<Group>(grp));
  }
}

void GroupMap::BuildVirtualGroup2(std::vector<Lane::Ptr> virtual_lanes,
                                  std::vector<Group::Ptr>* group_virtual,
                                  double stamp) {
  if (virtual_lanes.size() > 1 &&
      virtual_lanes[1]->str_id_with_group.find("VV") <
          virtual_lanes[1]->str_id_with_group.length()) {
    Group grp;
    grp.stamp = stamp;
    grp.group_state = Group::GroupState::VIRTUAL;
    grp.lanes.emplace_back(virtual_lanes[0]);
    grp.str_id = virtual_lanes[0]->str_id_with_group.substr(
        0, virtual_lanes[0]->str_id_with_group.find(":"));
    (*group_virtual).emplace_back(std::make_shared<Group>(grp));
    if (virtual_lanes.size() > 1) {
      Group grp2;
      grp2.stamp = stamp;
      grp2.group_state = Group::GroupState::VIRTUAL;
      for (int i = 1; i < virtual_lanes.size(); ++i) {
        grp2.lanes.emplace_back(virtual_lanes[i]);
      }
      grp2.str_id = virtual_lanes[1]->str_id_with_group.substr(
          0, virtual_lanes[1]->str_id_with_group.find(":"));
      (*group_virtual).emplace_back(std::make_shared<Group>(grp2));
    }
  } else {
    Group grp;
    grp.stamp = stamp;
    grp.group_state = Group::GroupState::VIRTUAL;
    for (auto& lane : virtual_lanes) {
      grp.lanes.emplace_back(lane);
    }
    grp.str_id = virtual_lanes[0]->str_id_with_group.substr(
        0, virtual_lanes[0]->str_id_with_group.find(":"));
    (*group_virtual).emplace_back(std::make_shared<Group>(grp));
  }
}

void GroupMap::CatmullRom(const std::vector<Eigen::Vector3f>& pts,
                          std::vector<Eigen::Vector3f>* fit_points, int num) {
  auto func = [](double p0, double p1, double p2, double p3, double t) {
    double s = 0.5;
    double a = -s * p0 + (2 - s) * p1 + (s - 2) * p2 + s * p3;
    double b = 2 * s * p0 + (s - 3) * p1 + (3 - 2 * s) * p2 - s * p3;
    double c = -s * p0 + s * p2;
    double d = p1;
    double t2 = t * t;
    double t3 = t2 * t;
    return (a * t3 + b * t2 + c * t + d);
  };
  for (size_t i = 0; i < pts.size() - 3; ++i) {
    double t = 0;
    while (t < 1) {
      double px =
          func(pts[i].x(), pts[i + 1].x(), pts[i + 2].x(), pts[i + 3].x(), t);
      double py =
          func(pts[i].y(), pts[i + 1].y(), pts[i + 2].y(), pts[i + 3].y(), t);
      Eigen::Vector3f point = {px, py, 0.0};
      fit_points->emplace_back(point);
      t += 1.0 / num;
    }
  }
}

float GroupMap::PointToLaneDis(const Lane::Ptr& lane_ptr,
                               Eigen::Vector3f point) {
  if (lane_ptr == nullptr || lane_ptr->left_boundary == nullptr ||
      lane_ptr->right_boundary == nullptr) {
    return std::numeric_limits<float>::max();
  }
  if (lane_ptr->left_boundary->pts.size() < 2 ||
      lane_ptr->right_boundary->pts.size() < 2) {
    return std::numeric_limits<float>::max();
  }
  float left_dist = 0;
  if (lane_ptr->left_boundary->pts.back().pt.x() < point.x()) {
    int size = lane_ptr->left_boundary->pts.size();
    left_dist =
        PointToVectorDist(lane_ptr->left_boundary->pts[size - 2].pt,
                          lane_ptr->left_boundary->pts[size - 1].pt, point);
  } else {
    left_dist = PointToVectorDist(lane_ptr->left_boundary->pts[0].pt,
                                  lane_ptr->left_boundary->pts[1].pt, point);
  }

  float right_dist = 0;
  if (lane_ptr->right_boundary->pts.back().pt.x() < point.x()) {
    int size = lane_ptr->right_boundary->pts.size();
    right_dist =
        PointToVectorDist(lane_ptr->right_boundary->pts[size - 2].pt,
                          lane_ptr->right_boundary->pts[size - 1].pt, point);
  } else {
    right_dist = PointToVectorDist(lane_ptr->right_boundary->pts[0].pt,
                                   lane_ptr->right_boundary->pts[1].pt, point);
  }
  HLOG_DEBUG << "left_dist:" << left_dist << ",right_dist:" << right_dist << ","
             << left_dist * right_dist;
  return left_dist * right_dist;
}

void GroupMap::HeadingCluster(const std::vector<Lane::Ptr>& lanes_need_pred,
                              std::vector<LineSegment::Ptr>* lines_need_pred,
                              double threshold, bool need_pred_kappa) {
  static double last_predict_angle = 0.0;
  static double last_predict_kappa = 0.0;
  static double last_predict_dkappa = 0.0;
  last_predict_angle -= delta_pose_heading_;

  std::tuple<double, double, double> ego_left_pred_data{0, 0, 0};
  std::tuple<double, double, double> ego_right_pred_data{0, 0, 0};
  // pcl::PointCloud<pcl::PointXYZ>::Ptr heading_data(
  //     new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& line : *lines_need_pred) {
    pcl::PointXYZ point;
    point.x = static_cast<float>(
        std::get<0>(line->pred_end_heading));  // 将一维heading添加到x轴上
    point.y = 0.0;
    point.z = 0.0;
    // heading_data->emplace_back(point);
    HLOG_DEBUG << "----mean_heading:" << line->id << ","
               << std::get<0>(line->pred_end_heading) * 180 / M_PI;
    HLOG_DEBUG << "----mean_heading:" << line->id << ","
               << std::get<0>(line->pred_end_heading) * 180 / M_PI;

    if (line->id == ego_line_id_.left_id) {
      ego_left_pred_data = line->pred_end_heading;
    }
    if (line->id == ego_line_id_.right_id) {
      ego_right_pred_data = line->pred_end_heading;
    }
  }

  double heading = 0.0;
  double kappa = 0.;
  double dkappa = 0.;
  bool ego_line_flag = false;

  if (std::get<1>(ego_left_pred_data) != 0. &&
      std::get<1>(ego_right_pred_data) != 0.) {
    heading =
        (std::get<0>(ego_left_pred_data) + std::get<0>(ego_right_pred_data)) /
        2;
    kappa =
        (std::get<1>(ego_left_pred_data) + std::get<1>(ego_right_pred_data)) /
        2;
    dkappa =
        (std::get<2>(ego_left_pred_data) + std::get<2>(ego_right_pred_data)) /
        2;
    ego_line_flag = true;
  } else if (std::get<1>(ego_left_pred_data) != 0.) {
    heading = std::get<0>(ego_left_pred_data);
    kappa = std::get<1>(ego_left_pred_data);
    dkappa = std::get<2>(ego_left_pred_data);
    ego_line_flag = true;
  } else if (std::get<1>(ego_right_pred_data) != 0.) {
    heading = std::get<0>(ego_right_pred_data);
    kappa = std::get<1>(ego_right_pred_data);
    dkappa = std::get<2>(ego_right_pred_data);
    ego_line_flag = true;
  }

  if (math::DoubleHasSameSign(kappa, dkappa) ||
      Compare(std::fabs(dkappa), std::fabs(kappa)) > 0) {
    dkappa = 0.0;
  }

  if (!need_pred_kappa) {
    kappa = 0.;
    dkappa = 0.;
  }

  // pcl::search::KdTree<pcl::PointXYZ>::Ptr heading_data_tree(
  //     new pcl::search::KdTree<pcl::PointXYZ>);
  // heading_data_tree->setInputCloud(heading_data);

  // // 对heading进行欧式聚类
  // pcl::EuclideanClusterExtraction<pcl::PointXYZ> heading_cluster;
  // heading_cluster.setClusterTolerance(threshold);
  // heading_cluster.setMinClusterSize(1);
  // heading_cluster.setMaxClusterSize(heading_data->size());
  // heading_cluster.setSearchMethod(heading_data_tree);
  // heading_cluster.setInputCloud(heading_data);

  // std::vector<pcl::PointIndices> heading_cluster_indices;
  // heading_cluster.extract(heading_cluster_indices);
  // HLOG_DEBUG << "cluser size:" << heading_cluster_indices.size();
  double predict_heading = 0.0;
  if (ego_line_flag) {
    predict_heading = heading;
    predict_heading = 0.8 * last_predict_angle + 0.2 * predict_heading;
    HLOG_DEBUG << "heading mode 1:" << predict_heading;
  } else {
    // if (heading_cluster_indices.size() == 1) {
    //   double heading_sum = 0.;
    //   int heading_data_size = 0;
    //   for (const auto& idx : heading_cluster_indices[0].indices) {
    //     auto heading_data_element = (*heading_data)[idx].x;
    //     heading_sum = heading_sum + heading_data_element;
    //     heading_data_size = heading_data_size + 1;
    //   }
    //   predict_heading =
    //       heading_data_size != 0 ? heading_sum / heading_data_size : 0;
    //   HLOG_DEBUG << "heading mode 2:" << predict_heading << ","
    //              << heading_cluster_indices.size();
    //   predict_heading = 0.8 * last_predict_angle + 0.2 * predict_heading;
    // } else {
    int nearest_lane_index = -1;
    double nearest_lane_dist = std::numeric_limits<float>::max();
    // 找到自车所在的lane
    Eigen::Vector3f cur_pose = {0, 0, 0};
    for (int i = 0; i < lanes_need_pred.size(); i++) {
      auto lane_ptr = lanes_need_pred[i];
      float cur_dist = PointToLaneDis(lane_ptr, cur_pose);
      if (cur_dist < nearest_lane_dist) {
        nearest_lane_dist = cur_dist;
        nearest_lane_index = i;
      }
    }
    if (nearest_lane_index != -1) {
      auto lane_ptr = lanes_need_pred[nearest_lane_index];
      predict_heading =
          (std::get<0>(lane_ptr->left_boundary->pred_end_heading) +
           std::get<0>(lane_ptr->right_boundary->pred_end_heading)) /
          2.0;
      HLOG_DEBUG << "heading mode 3:" << predict_heading << ","
                 << nearest_lane_dist;
      predict_heading = 0.8 * last_predict_angle + 0.2 * predict_heading;
    } else {
      // 求平均值
      double sum = 0;
      for (const auto& line : *lines_need_pred) {
        sum += std::get<0>(line->pred_end_heading);
      }
      predict_heading =
          lines_need_pred->empty() ? 0.0 : (sum / lines_need_pred->size());
      HLOG_DEBUG << "heading mode 4:" << predict_heading;
      predict_heading = 0.8 * last_predict_angle + 0.2 * predict_heading;
    }
    //}
  }

  kappa = 0.4 * last_predict_kappa + 0.6 * kappa;
  dkappa = 0.4 * last_predict_dkappa + 0.6 * dkappa;
  // 对曲率kappa做一个阈值限制 曲率半径200m
  kappa = std::min(kappa, static_cast<double>(1.0 / 200.0));

  last_predict_angle = predict_heading;
  last_predict_kappa = kappa;
  last_predict_dkappa = dkappa;
  for (auto& line : *lines_need_pred) {
    line->pred_end_heading = std::make_tuple(predict_heading, kappa, dkappa);
  }
}

void GroupMap::BuildVirtualProSucLane(Lane::Ptr lane_in_curr,
                                      Lane::Ptr lane_in_next, float dis_pt) {
  // ! TBD: 要写成是否路口判断条件，如是否有斑马线等
  if (dis_pt < 100.0) {
    lane_in_curr->next_lane_str_id_with_group.emplace_back(
        lane_in_next->str_id_with_group);
    lane_in_next->prev_lane_str_id_with_group.emplace_back(
        lane_in_curr->str_id_with_group);
    if (lane_in_next->center_line_pts.size() > 2 &&
        lane_in_curr->center_line_pts.size() > 4) {
      size_t cur_num = lane_in_curr->center_line_pts.size();
      std::vector<Eigen::Vector3f> tmp_vec;
      tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num - 5].pt);
      tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num - 4].pt);
      tmp_vec.emplace_back(lane_in_next->center_line_pts[0].pt);
      tmp_vec.emplace_back(lane_in_next->center_line_pts[1].pt);
      std::vector<Eigen::Vector3f> center_fit;
      CatmullRom(tmp_vec, &center_fit, 4);
      Point pt_center;
      pt_center.type = gm::VIRTUAL;
      pt_center.pt = center_fit[0];
      lane_in_curr->center_line_pts[cur_num - 3] = pt_center;
      pt_center.pt = center_fit[1];
      lane_in_curr->center_line_pts[cur_num - 2] = pt_center;
      pt_center.pt = center_fit[2];
      lane_in_curr->center_line_pts[cur_num - 1] = pt_center;
      lane_in_curr->center_line_pts.emplace_back(
          lane_in_next->center_line_pts.front());
    } else if (!lane_in_next->center_line_pts.empty()) {
      lane_in_curr->center_line_pts.emplace_back(
          lane_in_next->center_line_pts.front());
    }
  }
  // else {
  //   if (lane_in_curr->center_line_param.empty()) {
  //     std::vector<Point> tmp;
  //     size_t center_size = lane_in_curr->center_line_pts.size();
  //     tmp.emplace_back(lane_in_curr->center_line_pts[center_size - 1]);
  //     tmp.emplace_back(lane_in_next->center_line_pts[0]);
  //     lane_in_curr->center_line_param = FitLaneline(tmp);
  //   }
  //   size_t index_left = lane_in_curr->left_boundary->pts.size();
  //   Point left_pt_pred(VIRTUAL,
  //                     lane_in_curr->left_boundary->pts[index_left -
  //                     1].pt.x(),
  //                     lane_in_curr->left_boundary->pts[index_left
  //                     - 1].pt.y(),
  //                     lane_in_curr->left_boundary->pts[index_left -
  //                     1].pt.z());
  //   double b_left = left_pt_pred.pt.y() -
  //                   lane_in_curr->center_line_param[1] *
  //                   left_pt_pred.pt.x();
  //   while (left_pt_pred.pt.x() <
  //          lane_in_next->left_boundary->pts[0].pt.x() - 1.0) {
  //     lane_in_curr->left_boundary->pts.emplace_back(left_pt_pred);
  //     float pre_x = left_pt_pred.pt.x() + 1.0;
  //     float pre_y = b_left + lane_in_curr->center_line_param[1] * pre_x;
  //     left_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  //   }
  //   size_t index_right = lane_in_curr->right_boundary->pts.size();
  //   Point right_pt_pred(
  //       VIRTUAL, lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
  //       lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
  //       lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
  //   double b_right = right_pt_pred.pt.y() -
  //                    lane_in_curr->center_line_param[1] *
  //                    right_pt_pred.pt.x();
  //   while (right_pt_pred.pt.x() <
  //          lane_in_next->right_boundary->pts[0].pt.x() - 1.0) {
  //     lane_in_curr->right_boundary->pts.emplace_back(right_pt_pred);
  //     float pre_x = right_pt_pred.pt.x() + 1.0;
  //     float pre_y = b_right + lane_in_curr->center_line_param[1] * pre_x;
  //     right_pt_pred = Point(VIRTUAL, pre_x, pre_y,
  //     static_cast<float>(0.0));
  //   }
  //   size_t index_center = lane_in_curr->center_line_pts.size();
  //   Point center_pt_pred(VIRTUAL,
  //                       lane_in_curr->center_line_pts[index_center -
  //                       1].pt.x(),
  //                       lane_in_curr->center_line_pts[index_center
  //                       - 1].pt.y(),
  //                       lane_in_curr->center_line_pts[index_center -
  //                       1].pt.z());
  //   while (center_pt_pred.pt.x() <
  //          lane_in_next->center_line_pts[0].pt.x() - 1.0) {
  //     lane_in_curr->center_line_pts.emplace_back(center_pt_pred);
  //     float pre_x = center_pt_pred.pt.x() + 1.0;
  //     float pre_y = lane_in_curr->center_line_param[0] +
  //                   lane_in_curr->center_line_param[1] * pre_x;
  //     center_pt_pred = Point(VIRTUAL, pre_x, pre_y,
  //     static_cast<float>(0.0));
  //   }
  //   lane_in_curr->center_line_pts.emplace_back(
  //       lane_in_next->center_line_pts.front());
  // }
}
std::vector<double> GroupMap::FitLaneline(
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

std::vector<double> GroupMap::FitLanelinefront(
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
void GroupMap::BuildConnectLane(Lane::Ptr lane_in_curr, Group::Ptr next_group,
                                Lane::Ptr lane_in_next_next) {
  if (lane_in_curr->next_lane_str_id_with_group.empty()) {
    Lane lane_pre;
    LineSegment left_bound;
    left_bound.id = lane_in_curr->left_boundary->id;
    left_bound.lanepos = lane_in_curr->left_boundary->lanepos;
    left_bound.type = lane_in_curr->left_boundary->type;
    left_bound.color = lane_in_curr->left_boundary->color;
    left_bound.mean_end_heading = lane_in_curr->left_boundary->mean_end_heading;
    left_bound.pred_end_heading = lane_in_curr->left_boundary->pred_end_heading;
    left_bound.mean_end_heading_std_dev =
        lane_in_curr->left_boundary->mean_end_heading_std_dev;
    left_bound.mean_end_interval =
        lane_in_curr->left_boundary->mean_end_interval;
    size_t index_left = lane_in_curr->left_boundary->pts.size();
    Point left_pt_pred(PREDICTED,
                       lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                       lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                       lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
    Eigen::Vector3f left_vec_norm;
    left_vec_norm = (lane_in_next_next->left_boundary->pts[0].pt -
                     lane_in_curr->left_boundary->pts[index_left - 1].pt)
                        .normalized();
    while (left_pt_pred.pt.x() <
           lane_in_next_next->left_boundary->pts[0].pt.x() - 1.0) {
      left_bound.pts.emplace_back(left_pt_pred);
      float pre_x = left_pt_pred.pt.x() + left_vec_norm.x();
      float pre_y = left_pt_pred.pt.y() + left_vec_norm.y();
      left_pt_pred = Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
    }
    if (left_bound.pts.empty()) {
      return;
    }
    LineSegment right_bound;
    right_bound.id = lane_in_curr->right_boundary->id;
    right_bound.lanepos = lane_in_curr->right_boundary->lanepos;
    right_bound.type = lane_in_curr->right_boundary->type;
    right_bound.color = lane_in_curr->right_boundary->color;
    right_bound.mean_end_heading =
        lane_in_curr->right_boundary->mean_end_heading;
    right_bound.pred_end_heading =
        lane_in_curr->right_boundary->pred_end_heading;
    right_bound.mean_end_heading_std_dev =
        lane_in_curr->right_boundary->mean_end_heading_std_dev;
    right_bound.mean_end_interval =
        lane_in_curr->right_boundary->mean_end_interval;
    size_t index_right = lane_in_curr->right_boundary->pts.size();
    Point right_pt_pred(
        PREDICTED, lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
        lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
        lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
    Eigen::Vector3f right_vec_norm;
    right_vec_norm = (lane_in_next_next->right_boundary->pts[0].pt -
                      lane_in_curr->right_boundary->pts[index_right - 1].pt)
                         .normalized();
    while (right_pt_pred.pt.x() <
           lane_in_next_next->right_boundary->pts[0].pt.x() - 1.0) {
      right_bound.pts.emplace_back(right_pt_pred);
      float pre_x = right_pt_pred.pt.x() + right_vec_norm.x();
      float pre_y = right_pt_pred.pt.y() + right_vec_norm.y();
      right_pt_pred = Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
    }
    if (right_bound.pts.empty()) {
      return;
    }
    std::vector<Point> ctr_pts;
    size_t index_center = lane_in_curr->center_line_pts.size();
    Point center_pt_pred(
        PREDICTED, lane_in_curr->center_line_pts[index_center - 1].pt.x(),
        lane_in_curr->center_line_pts[index_center - 1].pt.y(),
        lane_in_curr->center_line_pts[index_center - 1].pt.z());
    Eigen::Vector3f cent_vec_norm;
    cent_vec_norm = (lane_in_next_next->center_line_pts[0].pt -
                     lane_in_curr->center_line_pts[index_center - 1].pt)
                        .normalized();
    while (center_pt_pred.pt.x() <
           lane_in_next_next->center_line_pts[0].pt.x() - 1.0) {
      ctr_pts.emplace_back(center_pt_pred);
      float pre_x = center_pt_pred.pt.x() + cent_vec_norm.x();
      float pre_y = center_pt_pred.pt.y() + cent_vec_norm.y();
      center_pt_pred = Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
    }
    if (ctr_pts.empty()) {
      return;
    }
    lane_pre.str_id = lane_in_curr->str_id;
    lane_pre.lanepos_id = lane_in_curr->lanepos_id;
    lane_pre.str_id_with_group = next_group->str_id + ":" + lane_pre.str_id;
    lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
    lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
    lane_pre.center_line_param = lane_in_curr->center_line_param;
    lane_pre.center_line_param_front = lane_in_curr->center_line_param;
    lane_pre.center_line_pts = ctr_pts;
    if (lane_pre.center_line_pts.size() > 1) {
      next_group->lanes.emplace_back(std::make_shared<Lane>(lane_pre));
    }
  }
}

void GroupMap::FillLineSegment(LineSegment::Ptr line, LineSegment* line_set) {
  for (auto& line_pt : line->pts) {
    line_set->pts.emplace_back(line_pt);
  }
}

bool GroupMap::IsIntersect(Lane::Ptr line1, Lane::Ptr line2) {
  if (line1->center_line_pts.size() < 2 || line2->center_line_pts.size() < 2) {
    return false;
  }
  Eigen::Vector3d l1_s(line1->center_line_pts[0].pt.x(),
                       line1->center_line_pts[0].pt.y(), 0.0);
  Eigen::Vector3d l1_e(line1->center_line_pts.back().pt.x(),
                       line1->center_line_pts.back().pt.y(), 0.0);
  Eigen::Vector3d l2_s(line2->center_line_pts[0].pt.x(),
                       line2->center_line_pts[0].pt.y(), 0.0);
  Eigen::Vector3d l2_e(line2->center_line_pts.back().pt.x(),
                       line2->center_line_pts.back().pt.y(), 0.0);
  if (!line1->center_line_param.empty() && !line2->center_line_param.empty()) {
    // 往前延伸10米
    Eigen::Vector3d l1_forward_norm(1.0, line1->center_line_param[1], 0.0);
    Eigen::Vector3d l2_forward_norm(1.0, line2->center_line_param[1], 0.0);
    l1_forward_norm.normalized();
    l2_forward_norm.normalized();
    // HLOG_ERROR << "l1_forward_norm.Y = " << l1_forward_norm.y()
    //            << "  l2_forward_norm.Y = " << l2_forward_norm.y();
    l1_e.x() = l1_e.x() + l1_forward_norm.x() * 10;
    l1_e.y() = l1_e.y() + l1_forward_norm.y() * 10;
    l2_e.x() = l2_e.x() + l2_forward_norm.x() * 10;
    l2_e.y() = l2_e.y() + l2_forward_norm.y() * 10;
  }
  // HLOG_ERROR << " l1_s = " << l1_s.x() << "  " << l1_s.y()
  //            << "  l1_e = " << l1_e.x() << "  " << l1_e.y();
  // HLOG_ERROR << "l2_s = " << l2_s.x() << "  " << l2_s.y()
  //            << "   l2_e = " << l2_e.x() << "  " << l2_e.y();
  // 快速排除不可能相交的线
  if ((l1_s.x() > l1_e.x() ? l1_s.x() : l1_e.x()) <
          (l2_s.x() < l2_e.x() ? l2_s.x() : l2_e.x()) ||
      (l2_s.x() > l2_e.x() ? l2_s.x() : l2_e.x()) <
          (l1_s.x() < l1_e.x() ? l1_s.x() : l1_e.x()) ||
      (l1_s.y() > l1_e.y() ? l1_s.y() : l1_e.y()) <
          (l2_s.y() < l2_e.y() ? l2_s.y() : l2_e.y()) ||
      (l2_s.y() > l2_e.y() ? l2_s.y() : l2_e.y()) <
          (l1_s.y() < l1_e.y() ? l1_s.y() : l1_e.y())) {
    return false;
  }
  // 叉乘判断是否相交, 叉乘的正负代表逆时针顺时针即可判断方向 AB.cross(AP)
  if (((l1_e - l2_s).cross(l2_e - l2_s)).dot((l1_s - l2_s).cross(l2_e - l2_s)) >
          0 ||
      ((l2_e - l1_s).cross(l1_e - l1_s)).dot((l2_s - l1_s).cross(l1_e - l1_s)) >
          0) {
    return false;
  }
  return true;
}

void GroupMap::EraseIntersectLane(Group::Ptr curr_group,
                                  Group::Ptr next_group) {
  std::unordered_set<std::string> erase_next_lane_str_id;
  double real_lane_mean_end_heading = 0.0;
  for (auto& lane : next_group->lanes) {
    if (!lane->right_boundary->pts.empty() &&
        lane->right_boundary->pts[0].type == gm::RAW) {
      real_lane_mean_end_heading = lane->right_boundary->mean_end_heading;
      break;
    }
  }

  for (int i = 0; i < next_group->lanes.size(); ++i) {
    for (int j = i + 1; j < next_group->lanes.size(); ++j) {
      bool is_intersect =
          IsIntersect(next_group->lanes[i], next_group->lanes[j]);
      // HLOG_ERROR << "lane_i = " << next_group->lanes[i]->str_id_with_group
      //            << "  lane_j = " <<
      //            next_group->lanes[j]->str_id_with_group
      //            << "  is_intersect = " << is_intersect;
      if (is_intersect) {
        auto& lane_i = next_group->lanes[i];
        auto& lane_j = next_group->lanes[j];
        if ((lane_i->left_boundary->pts[0].type == gm::RAW ||
             lane_i->right_boundary->pts[0].type == gm::RAW) &&
            (lane_j->left_boundary->pts[0].type == gm::RAW ||
             lane_j->right_boundary->pts[0].type == gm::RAW)) {
          continue;
        } else if (lane_i->left_boundary->id == ego_line_id_.left_id &&
                   lane_i->right_boundary->id == ego_line_id_.right_id) {
          erase_next_lane_str_id.insert(lane_j->str_id_with_group);
        } else if (lane_j->left_boundary->id == ego_line_id_.left_id &&
                   lane_j->right_boundary->id == ego_line_id_.right_id) {
          erase_next_lane_str_id.insert(lane_i->str_id_with_group);
        } else if ((lane_i->left_boundary->pts[0].type != gm::RAW) &&
                   (lane_i->right_boundary->pts[0].type != gm::RAW) &&
                   (lane_j->left_boundary->pts[0].type == gm::RAW ||
                    lane_j->right_boundary->pts[0].type == gm::RAW)) {
          erase_next_lane_str_id.insert(lane_i->str_id_with_group);
        } else if ((lane_j->left_boundary->pts[0].type != gm::RAW) &&
                   (lane_j->right_boundary->pts[0].type != gm::RAW) &&
                   (lane_i->left_boundary->pts[0].type == gm::RAW ||
                    lane_i->right_boundary->pts[0].type == gm::RAW)) {
          erase_next_lane_str_id.insert(lane_j->str_id_with_group);
        } else {
          if (abs(lane_i->right_boundary->mean_end_heading -
                  real_lane_mean_end_heading) >
              abs(lane_j->right_boundary->mean_end_heading -
                  real_lane_mean_end_heading)) {
            erase_next_lane_str_id.insert(lane_i->str_id_with_group);
          } else {
            erase_next_lane_str_id.insert(lane_j->str_id_with_group);
          }
        }
      }
    }
  }

  for (auto& lane : curr_group->lanes) {
    if (!lane->next_lane_str_id_with_group.empty() &&
        erase_next_lane_str_id.find(lane->next_lane_str_id_with_group[0]) !=
            erase_next_lane_str_id.end()) {
      lane->next_lane_str_id_with_group.clear();
    }
  }
  for (int i = static_cast<int>(next_group->lanes.size()) - 1; i >= 0; i--) {
    if (erase_next_lane_str_id.find(next_group->lanes[i]->str_id_with_group) !=
        erase_next_lane_str_id.end()) {
      next_group->lanes.erase(next_group->lanes.begin() + i);
    }
  }
}

bool GroupMap::BoundaryIsValid(const LineSegment& line) {
  if (line.pts.size() < 3) {
    return true;
  }
  em::Point x1, x2, x3;
  x1 = line.pts[0].pt;
  x2 = line.pts[1].pt;
  int index2 = 1;
  // HLOG_ERROR << "X1=" << x1.x() << "  " << x1.y();
  while ((x2 - x1).norm() < 0.2 &&
         index2 < static_cast<int>(line.pts.size()) - 1) {
    index2++;
    x2 = line.pts[index2].pt;
  }
  // HLOG_ERROR << "x2=" << x2.x() << "  " << x2.y();
  if (index2 > line.pts.size() - 2) {
    return true;
  }
  int index3 = index2 + 1;
  x3 = line.pts[index3].pt;
  while ((x2 - x3).norm() < 0.2 &&
         index3 < static_cast<int>(line.pts.size()) - 1) {
    index3++;
    x3 = line.pts[index3].pt;
  }
  // HLOG_ERROR << "x3=" << x3.x() << "  " << x3.y();
  if (index3 > static_cast<int>(line.pts.size()) - 1) {
    return true;
  }
  em::Point norm_x12 = (x1 - x2).normalized();
  em::Point norm_x32 = (x3 - x2).normalized();
  if (abs(norm_x12.dot(norm_x32)) < 0.5) {
    return false;
  }
  return true;
}
bool GroupMap::Distanceline(const LineSegment& left_line, float line_front_x,
                            float line_front_y) {
  int index_left = 0;
  while (index_left < static_cast<int>(left_line.pts.size()) - 1 &&
         left_line.pts[index_left + 1].pt.x() < line_front_x) {
    index_left++;
  }
  if (index_left < static_cast<int>(left_line.pts.size()) - 1) {
    int index_right = index_left + 1;
    while (index_right < static_cast<int>(left_line.pts.size()) &&
           left_line.pts[index_right].pt.x() -
                   left_line.pts[index_left].pt.x() <
               0.1) {
      index_right++;
    }
    if (index_right < static_cast<int>(left_line.pts.size())) {
      const auto& p_left = left_line.pts[index_left].pt;
      const auto& p_right = left_line.pts[index_right].pt;
      float k = (p_right.y() - p_left.y()) / (p_right.x() - p_left.x());
      float b = p_right.y() - k * p_right.x();
      float dis = abs(line_front_x * k + b - line_front_y) / sqrt(1 + k * k);
      if (dis < conf_.max_lane_width && dis > conf_.min_lane_width) {
        return true;
      } else {
        return false;
      }
    }
  }
  return false;
}
bool GroupMap::DistanceInferenceLane(const LineSegment& left_line,
                                     const LineSegment& right_line) {
  if (left_line.pts.empty() || right_line.pts.empty()) {
    return false;
  }
  if (left_line.pts.front().pt.x() < right_line.pts.front().pt.x()) {
    float right_line_front_x = right_line.pts.front().pt.x();
    float right_line_front_y = right_line.pts.front().pt.y();
    return Distanceline(left_line, right_line_front_x, right_line_front_y);
  } else {
    float left_line_front_x = left_line.pts.front().pt.x();
    float left_line_front_y = left_line.pts.front().pt.y();
    return Distanceline(right_line, left_line_front_x, left_line_front_y);
  }
  return false;
}
float GroupMap::PointToLineDis(const LineSegment& line, float line_front_x,
                               float line_front_y) {
  if (line.pts.empty()) {
    return 0.0;
  }
  if (line.pts.size() == 1) {
    return std::hypot(line.pts[0].pt.x() - line_front_x,
                      line.pts[0].pt.y() - line_front_y);
  }
  int index_left = 0;
  if (index_left < static_cast<int>(line.pts.size()) - 1) {
    int index_right = index_left + 1;
    while (index_right < static_cast<int>(line.pts.size()) &&
           line.pts[index_right].pt.x() - line.pts[index_left].pt.x() < 0.1) {
      index_right++;
    }
    if (index_right < static_cast<int>(line.pts.size())) {
      const auto& p_left = line.pts[index_left].pt;
      const auto& p_right = line.pts[index_right].pt;
      float k = (p_right.y() - p_left.y()) / (p_right.x() - p_left.x());
      float b = p_right.y() - k * p_right.x();
      return abs(line_front_x * k + b - line_front_y) / sqrt(1 + k * k);
    }
  }
  return 0.0;
}
void GroupMap::BuildVirtualLaneLeft(Group::Ptr group) {
  if (group->lanes.empty()) {
    return;
  }
  // HLOG_DEBUG << "lane_in_curr name is " << lane_in_curr->str_id_with_group;
  auto lane_in_curr = group->lanes[0];
  if (lane_in_curr->center_line_param_front.empty()) {
    return;
  }
  Lane lane_pre;
  LineSegment left_bound;
  left_bound.id = lane_in_curr->left_boundary->id + 4000;
  left_bound.lanepos = em::LanePos::LanePositionType_OTHER;
  left_bound.type = lane_in_curr->left_boundary->type;
  left_bound.color = lane_in_curr->left_boundary->color;
  left_bound.isego = lane_in_curr->left_boundary->isego;
  left_bound.mean_end_heading = lane_in_curr->left_boundary->mean_end_heading;
  left_bound.pred_end_heading = lane_in_curr->left_boundary->pred_end_heading;
  left_bound.mean_end_heading_std_dev =
      lane_in_curr->left_boundary->mean_end_heading_std_dev;
  left_bound.mean_end_interval = lane_in_curr->left_boundary->mean_end_interval;
  float norm_line = std::hypot(lane_in_curr->center_line_param_front[1], 1);

  float detax =
      -lane_in_curr->center_line_param_front[1] / norm_line * kLaneWidth;
  float detay = 1 / norm_line * kLaneWidth;
  // 判断现有的line是否已经存在
  size_t index_left = lane_in_curr->left_boundary->pts.size();
  if (index_left < 1) {
    return;
  }
  for (const auto& right_pt : lane_in_curr->left_boundary->pts) {
    Point left_pt_pred(PREDICTED, right_pt.pt.x() + detax,
                       right_pt.pt.y() + detay, right_pt.pt.z());
    left_bound.pts.emplace_back(left_pt_pred);
  }

  if (left_bound.pts.empty()) {
    HLOG_WARN << "left bound empty!";
    return;
  }

  lane_pre.str_id = std::to_string(left_bound.id) + "_" +
                    std::to_string(lane_in_curr->left_boundary->id);
  lane_pre.lanepos_id = std::to_string(left_bound.lanepos) + "_" +
                        std::to_string(lane_in_curr->left_boundary->lanepos);
  lane_pre.str_id_with_group = group->str_id + ":" + lane_pre.str_id;
  lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
  lane_pre.right_boundary = lane_in_curr->left_boundary;
  Lane::Ptr lane_ptr = std::make_shared<Lane>(lane_pre);
  // FitCenterLine(lane_ptr);
  ComputeCenterPoints(lane_ptr);
  if (lane_ptr->center_line_param.empty()) {
    lane_ptr->center_line_param = lane_in_curr->center_line_param;
  }
  if (lane_ptr->center_line_param_front.empty()) {
    lane_ptr->center_line_param_front = lane_in_curr->center_line_param;
  }

  if (left_bound.pts.size() < 2 ||
      lane_in_curr->left_boundary->pts.size() < 2) {
    lane_ptr->center_line_pts.emplace_back(
        lane_in_curr->center_line_pts.back());
  }
  if (lane_ptr->center_line_pts.empty()) {
    return;
  }
  for (const auto& lane : group->lanes) {
    lane->left_lane_str_id_with_group.emplace_back(lane_pre.str_id_with_group);
    lane_pre.right_lane_str_id_with_group.emplace_back(lane->str_id_with_group);
  }
  // next_group->lanes.emplace_back(std::make_shared<Lane>(lane_pre));
  group->lanes.insert(group->lanes.begin(), lane_ptr);
}
void GroupMap::BuildVirtualLaneRight(Group::Ptr group) {
  if (group->lanes.empty()) {
    return;
  }
  // HLOG_DEBUG << "lane_in_curr name is " << lane_in_curr->str_id_with_group;
  auto lane_in_curr = group->lanes.back();
  if (lane_in_curr->center_line_param_front.empty()) {
    return;
  }
  Lane lane_pre;
  LineSegment right_bound;
  right_bound.id = lane_in_curr->right_boundary->id + 4000;
  right_bound.lanepos = em::LanePos::LanePositionType_OTHER;
  right_bound.type = lane_in_curr->right_boundary->type;
  right_bound.color = lane_in_curr->right_boundary->color;
  right_bound.isego = lane_in_curr->right_boundary->isego;
  right_bound.mean_end_heading = lane_in_curr->right_boundary->mean_end_heading;
  right_bound.pred_end_heading = lane_in_curr->right_boundary->pred_end_heading;
  right_bound.mean_end_heading_std_dev =
      lane_in_curr->right_boundary->mean_end_heading_std_dev;
  right_bound.mean_end_interval =
      lane_in_curr->right_boundary->mean_end_interval;
  float norm_line = std::hypot(lane_in_curr->center_line_param_front[1], 1);

  float detax =
      lane_in_curr->center_line_param_front[1] / norm_line * kLaneWidth;
  float detay = -1 / norm_line * kLaneWidth;
  // 判断现有的line是否已经存在
  size_t index_right = lane_in_curr->right_boundary->pts.size();
  if (index_right < 1) {
    return;
  }
  for (const auto& right_pt : lane_in_curr->right_boundary->pts) {
    Point right_pt_pred(PREDICTED, right_pt.pt.x() + detax,
                        right_pt.pt.y() + detay, right_pt.pt.z());
    right_bound.pts.emplace_back(right_pt_pred);
  }

  if (right_bound.pts.empty()) {
    HLOG_WARN << "left bound empty!";
    return;
  }

  lane_pre.str_id = std::to_string(lane_in_curr->right_boundary->id) + "_" +
                    std::to_string(right_bound.id);
  lane_pre.lanepos_id = std::to_string(lane_in_curr->right_boundary->lanepos) +
                        "_" + std::to_string(right_bound.lanepos);
  lane_pre.str_id_with_group = group->str_id + ":" + lane_pre.str_id;
  lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
  lane_pre.left_boundary = lane_in_curr->right_boundary;
  Lane::Ptr lane_ptr = std::make_shared<Lane>(lane_pre);
  // FitCenterLine(lane_ptr);
  ComputeCenterPoints(lane_ptr);
  if (lane_ptr->center_line_param.empty()) {
    lane_ptr->center_line_param = lane_in_curr->center_line_param;
  }
  if (lane_ptr->center_line_param_front.empty()) {
    lane_ptr->center_line_param_front = lane_in_curr->center_line_param;
  }

  if (right_bound.pts.size() < 2 ||
      lane_in_curr->right_boundary->pts.size() < 2) {
    lane_ptr->center_line_pts.emplace_back(
        lane_in_curr->center_line_pts.back());
  }
  if (lane_ptr->center_line_pts.empty()) {
    return;
  }
  for (const auto& lane : group->lanes) {
    lane->right_lane_str_id_with_group.emplace_back(lane_pre.str_id_with_group);
    lane_pre.left_lane_str_id_with_group.emplace_back(lane->str_id_with_group);
  }
  // next_group->lanes.emplace_back(std::make_shared<Lane>(lane_pre));
  group->lanes.emplace_back(lane_ptr);
}
void GroupMap::BuildVirtualLaneAfter(Group::Ptr curr_group,
                                     Group::Ptr next_group) {
  for (auto& lane_in_curr : curr_group->lanes) {
    // HLOG_DEBUG << "lane_in_curr name is " << lane_in_curr->str_id_with_group;
    if (lane_in_curr->next_lane_str_id_with_group.empty()) {
      Lane lane_pre;
      LineSegment left_bound;
      LineSegment right_bound;
      left_bound.id = lane_in_curr->left_boundary->id;
      left_bound.lanepos = lane_in_curr->left_boundary->lanepos;
      left_bound.type = lane_in_curr->left_boundary->type;
      left_bound.color = lane_in_curr->left_boundary->color;
      left_bound.isego = lane_in_curr->left_boundary->isego;
      left_bound.mean_end_heading =
          lane_in_curr->left_boundary->mean_end_heading;
      left_bound.pred_end_heading =
          lane_in_curr->left_boundary->pred_end_heading;
      left_bound.mean_end_heading_std_dev =
          lane_in_curr->left_boundary->mean_end_heading_std_dev;
      left_bound.mean_end_interval =
          lane_in_curr->left_boundary->mean_end_interval;
      for (auto& delete_id : lane_in_curr->left_boundary->deteled_ids) {
        left_bound.deteled_ids.emplace_back(delete_id);
      }
      right_bound.id = lane_in_curr->right_boundary->id;
      right_bound.lanepos = lane_in_curr->right_boundary->lanepos;
      right_bound.type = lane_in_curr->right_boundary->type;
      right_bound.color = lane_in_curr->right_boundary->color;
      right_bound.isego = lane_in_curr->right_boundary->isego;
      right_bound.mean_end_heading =
          lane_in_curr->right_boundary->mean_end_heading;
      right_bound.pred_end_heading =
          lane_in_curr->right_boundary->pred_end_heading;
      right_bound.mean_end_heading_std_dev =
          lane_in_curr->right_boundary->mean_end_heading_std_dev;
      right_bound.mean_end_interval =
          lane_in_curr->right_boundary->mean_end_interval;
      for (auto& delete_id : lane_in_curr->right_boundary->deteled_ids) {
        right_bound.deteled_ids.emplace_back(delete_id);
      }
      // HLOG_ERROR << "lane_in_curr->str_id_with_group = "
      //            << lane_in_curr->str_id_with_group
      //            << "  lane_in_curr->left_boundary->id = "
      //            << lane_in_curr->left_boundary->id
      //            << "  lane_in_curr->left_boundary->id_next = "
      //            << lane_in_curr->left_boundary->id_next
      //            << "  lane_in_curr->right_boundary->id = "
      //            << lane_in_curr->right_boundary->id
      //            << "  lane_in_curr->right_boundary->id_next = "
      //            << lane_in_curr->right_boundary->id_next;
      // 判断现有的line是否已经存在
      int left_bound_exist = 0, right_bound_exist = 0;
      for (auto& lane_in_next : next_group->lanes) {
        if (lane_in_next->left_boundary->id ==
                lane_in_curr->left_boundary->id_next &&
            !lane_in_next->left_boundary->pts.empty() &&
            lane_in_next->left_boundary->pts[0].type == gm::RAW &&
            !left_bound_exist) {
          // left_bound = *(lane_in_next->left_boundary);
          // FillLineSegment(lane_in_next->left_boundary, &left_bound);
          for (auto& line_pt : lane_in_next->left_boundary->pts) {
            if (line_pt.type != gm::RAW) {
              break;
            }
            Point pt_pre(gm::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = 1;
        } else if (lane_in_next->right_boundary->id ==
                       lane_in_curr->left_boundary->id_next &&
                   !lane_in_next->right_boundary->pts.empty() &&
                   lane_in_next->right_boundary->pts[0].type == gm::RAW &&
                   !left_bound_exist) {
          // left_bound = *(lane_in_next->right_boundary);
          // FillLineSegment(lane_in_next->right_boundary, &left_bound);
          for (auto& line_pt : lane_in_next->right_boundary->pts) {
            if (line_pt.type != gm::RAW) {
              break;
            }
            Point pt_pre(gm::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = 1;
        } else if (lane_in_next->left_boundary->id ==
                       lane_in_curr->right_boundary->id_next &&
                   !lane_in_next->left_boundary->pts.empty() &&
                   lane_in_next->left_boundary->pts[0].type == gm::RAW &&
                   !right_bound_exist) {
          // right_bound = *(lane_in_next->left_boundary);
          // FillLineSegment(lane_in_next->left_boundary, &right_bound);
          for (auto& line_pt : lane_in_next->left_boundary->pts) {
            if (line_pt.type != gm::RAW) {
              break;
            }
            Point pt_pre(gm::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = 1;
        } else if (lane_in_next->right_boundary->id ==
                       lane_in_curr->right_boundary->id_next &&
                   !lane_in_next->right_boundary->pts.empty() &&
                   lane_in_next->right_boundary->pts[0].type == gm::RAW &&
                   !right_bound_exist) {
          // right_bound = *(lane_in_next->right_boundary);
          // FillLineSegment(lane_in_next->right_boundary, &right_bound);
          for (auto& line_pt : lane_in_next->right_boundary->pts) {
            if (line_pt.type != gm::RAW) {
              break;
            }
            Point pt_pre(gm::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = 1;
        }
      }
      // 青鸾号:1308782
      // 防止错车道的前后继关联
      if (left_bound_exist &&
          PointToLineDis(left_bound,
                         lane_in_curr->left_boundary->pts.back().pt.x(),
                         lane_in_curr->left_boundary->pts.back().pt.y()) >
              conf_.min_lane_width + 0.5) {
        left_bound.pts.clear();
        left_bound_exist = 0;
      }
      if (right_bound_exist &&
          PointToLineDis(right_bound,
                         lane_in_curr->right_boundary->pts.back().pt.x(),
                         lane_in_curr->right_boundary->pts.back().pt.y()) >
              conf_.min_lane_width + 0.5) {
        right_bound.pts.clear();
        right_bound_exist = 0;
      }

      // HLOG_DEBUG << "left_bound_exist " << left_bound_exist
      //           << "  right_bound_exist  " << right_bound_exist;
      if (left_bound_exist == 1 && right_bound_exist == 1 &&
          ((!DistanceInferenceLane(left_bound, right_bound)) &&
           (lane_in_curr->left_boundary->id != ego_line_id_.left_id ||
            lane_in_curr->right_boundary->id != ego_line_id_.right_id))) {
        HLOG_WARN
            << "left bound exist and right bound exist but distance not valid!";
        return;
      } else if (left_bound_exist == 1 && right_bound_exist == 0) {
        size_t index_right = lane_in_curr->right_boundary->pts.size();
        if (index_right < 1) {
          return;
        }
        Point right_pt_pred(
            PREDICTED,
            lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
            lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
            lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
        right_bound.pts.emplace_back(right_pt_pred);
        if (left_bound.pts.size() < 2) {
          return;
        }
        if ((right_bound.pts[0].pt - left_bound.pts[0].pt).norm() > 4.0) {
          right_pt_pred.pt.x() = right_pt_pred.pt.x() +
                                 left_bound.pts[0].pt.x() -
                                 lane_in_curr->left_boundary->pts.back().pt.x();
          right_pt_pred.pt.y() = right_pt_pred.pt.y() +
                                 left_bound.pts[0].pt.y() -
                                 lane_in_curr->left_boundary->pts.back().pt.y();
          right_bound.pts.emplace_back(right_pt_pred);
        }
        size_t left_index = 1;
        size_t left_bound_size = left_bound.pts.size();
        float dx = left_bound.pts[left_index].pt.x() -
                   left_bound.pts[left_index - 1].pt.x();
        float dy = left_bound.pts[left_index].pt.y() -
                   left_bound.pts[left_index - 1].pt.y();
        while (right_pt_pred.pt.x() <
                   next_group->group_segments.back()->end_slice.po.x() &&
               left_index < left_bound_size &&
               (dx > 0 && dx * dx + dy * dy > 0.4 || dx * dx + dy * dy < 0.4)) {
          dx = left_bound.pts[left_index].pt.x() -
               left_bound.pts[left_index - 1].pt.x();
          dy = left_bound.pts[left_index].pt.y() -
               left_bound.pts[left_index - 1].pt.y();
          float pre_x = right_pt_pred.pt.x() + dx;
          float pre_y = right_pt_pred.pt.y() + dy;
          right_pt_pred =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          right_bound.pts.emplace_back(right_pt_pred);
          left_index++;
        }
        if (!BoundaryIsValid(right_bound)) {
          HLOG_WARN << "boundary not valid";
          continue;
        }
        if (right_bound.pts.empty()) {
          HLOG_WARN << "right bound empty!";
          return;
        }
      } else if (left_bound_exist == 0 && right_bound_exist == 1) {
        size_t index_left = lane_in_curr->left_boundary->pts.size();
        if (index_left < 1) {
          return;
        }
        Point left_pt_pred(
            PREDICTED, lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
            lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
            lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
        size_t right_index = 1;
        size_t right_bound_size = right_bound.pts.size();
        left_bound.pts.emplace_back(left_pt_pred);
        if (right_bound_size < 2) {
          return;
        }
        if ((right_bound.pts[0].pt - left_bound.pts[0].pt).norm() > 4.0) {
          left_pt_pred.pt.x() = left_pt_pred.pt.x() +
                                right_bound.pts[0].pt.x() -
                                lane_in_curr->right_boundary->pts.back().pt.x();
          left_pt_pred.pt.y() = left_pt_pred.pt.y() +
                                right_bound.pts[0].pt.y() -
                                lane_in_curr->right_boundary->pts.back().pt.y();
          left_bound.pts.emplace_back(left_pt_pred);
        }
        float dx = right_bound.pts[right_index].pt.x() -
                   right_bound.pts[right_index - 1].pt.x();
        float dy = right_bound.pts[right_index].pt.y() -
                   right_bound.pts[right_index - 1].pt.y();
        // HLOG_ERROR << "dx = " << dx << " dy = " << dy;
        while (left_pt_pred.pt.x() <
                   next_group->group_segments.back()->end_slice.po.x() &&
               right_index < right_bound_size &&
               (dx > 0 && dx * dx + dy * dy > 0.4 || dx * dx + dy * dy < 0.4)) {
          dx = right_bound.pts[right_index].pt.x() -
               right_bound.pts[right_index - 1].pt.x();
          dy = right_bound.pts[right_index].pt.y() -
               right_bound.pts[right_index - 1].pt.y();
          // HLOG_ERROR << "dx = " << dx << " dy = " << dy;
          float pre_x = left_pt_pred.pt.x() + dx;
          float pre_y = left_pt_pred.pt.y() + dy;
          left_pt_pred =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          left_bound.pts.emplace_back(left_pt_pred);
          right_index++;
        }
        if (!BoundaryIsValid(left_bound)) {
          HLOG_WARN << "boundary not valid";
          continue;
        }
        if (left_bound.pts.empty()) {
          HLOG_WARN << "left bound empty!";
          return;
        }
      } else if (left_bound_exist == 0 && right_bound_exist == 0) {
        if (lane_in_curr->center_line_param.empty()) {
          HLOG_WARN << "center_line_param isn't exist";
          return;
        }
        size_t index_left = lane_in_curr->left_boundary->pts.size();
        if (index_left < 1) {
          return;
        }
        Point left_pt_pred(
            PREDICTED, lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
            lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
            lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
        double b_left =
            left_pt_pred.pt.y() -
            lane_in_curr->center_line_param[1] * left_pt_pred.pt.x();
        left_bound.pts.emplace_back(left_pt_pred);
        while (left_pt_pred.pt.x() <
               next_group->group_segments.back()->end_slice.po.x()) {
          float pre_x = left_pt_pred.pt.x() + 1.0;
          float pre_y = b_left + lane_in_curr->center_line_param[1] * pre_x;
          left_pt_pred =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          left_bound.pts.emplace_back(left_pt_pred);
        }
        if (left_bound.pts.empty()) {
          HLOG_WARN << "left bound empty!";
          return;
        }
        size_t index_right = lane_in_curr->right_boundary->pts.size();
        if (index_right < 1) {
          return;
        }
        Point right_pt_pred(
            PREDICTED,
            lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
            lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
            lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
        double b_right =
            right_pt_pred.pt.y() -
            lane_in_curr->center_line_param[1] * right_pt_pred.pt.x();
        right_bound.pts.emplace_back(right_pt_pred);
        while (right_pt_pred.pt.x() <
               next_group->group_segments.back()->end_slice.po.x()) {
          float pre_x = right_pt_pred.pt.x() + 1.0;
          float pre_y = b_right + lane_in_curr->center_line_param[1] * pre_x;
          right_pt_pred =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          right_bound.pts.emplace_back(right_pt_pred);
        }
        if (right_bound.pts.empty()) {
          HLOG_WARN << "right bound empty!";
          return;
        }
      }

      lane_pre.str_id = lane_in_curr->str_id;
      lane_pre.lanepos_id = lane_in_curr->lanepos_id;
      lane_pre.str_id_with_group = next_group->str_id + ":" + lane_pre.str_id;
      lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
      lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
      Lane::Ptr lane_ptr = std::make_shared<Lane>(lane_pre);
      // FitCenterLine(lane_ptr);
      ComputeCenterPoints(lane_ptr);
      if (lane_ptr->center_line_param.empty()) {
        // lane_ptr->center_line_para_line_pts.size() = "
        //            << lane_pre.center_line_pts.size() << "   "
        //            << lane_ptr->center_line_pts.size()
        //            << "  lane_pre.left_boundary = "
        //            << lane_pre.left_boundary->pts.size()
        //            << "  lane_pre.right_boundary = "
        //            << lane_pre.right_boundary->pm =
        //            lane_in_curr->center_line_param;
        lane_ptr->center_line_param = lane_in_curr->center_line_param;
      }
      if (lane_ptr->center_line_param_front.empty()) {
        lane_ptr->center_line_param_front = lane_in_curr->center_line_param;
      }
      // HLOG_ERROR << "lane_pre.center_line_pts.size() = "
      //            << lane_pre.center_line_pts.size() << "   "
      //            << lane_ptr->center_line_pts.size()
      //            << "  lane_pre.left_boundary = "
      //            << lane_pre.left_boundary->pts.size()
      //            << "  lane_pre.right_boundary = "
      //            << lane_pre.right_boundary->pts.size();
      if (left_bound.pts.size() < 2 || right_bound.pts.size() < 2) {
        lane_ptr->center_line_pts.emplace_back(
            lane_in_curr->center_line_pts.back());
      }
      if (lane_ptr->center_line_pts.empty()) {
        return;
      }
      // lane_pre.center_line_param = lane_in_curr->center_line_param;
      // lane_pre.center_line_param_front = lane_in_curr->center_line_param;
      // lane_pre.center_line_pts = ctr_pts;
      // lane_pre.prev_lane_str_id_with_group.emplace_back(
      //     lane_in_curr->str_id_with_group);
      lane_in_curr->next_lane_str_id_with_group.emplace_back(
          lane_pre.str_id_with_group);

      for (const auto& lane_right :
           lane_in_curr->right_lane_str_id_with_group) {
        size_t index = lane_right.find(":");
        std::string right_str =
            next_group->str_id +
            lane_right.substr(index, lane_right.size() - index);
        lane_pre.right_lane_str_id_with_group.emplace_back(right_str);
      }
      for (const auto& lane_left : lane_in_curr->left_lane_str_id_with_group) {
        size_t index = lane_left.find(":");
        std::string left_str =
            next_group->str_id +
            lane_left.substr(index, lane_left.size() - index);
        lane_pre.left_lane_str_id_with_group.emplace_back(left_str);
      }

      // next_group->lanes.emplace_back(std::make_shared<Lane>(lane_pre));
      next_group->lanes.emplace_back(lane_ptr);
    }
  }
}

void GroupMap::BuildVirtualLaneBefore(Group::Ptr curr_group,
                                      Group::Ptr next_group) {
  for (auto& lane_in_next : next_group->lanes) {
    if (lane_in_next->prev_lane_str_id_with_group.empty()) {
      //  && !lane_in_next->center_line_param_front.empty()
      Lane lane_pre;
      LineSegment left_bound;
      LineSegment right_bound;
      left_bound.id = lane_in_next->left_boundary->id;
      left_bound.lanepos = lane_in_next->left_boundary->lanepos;
      left_bound.type = lane_in_next->left_boundary->type;
      left_bound.color = lane_in_next->left_boundary->color;
      left_bound.isego = lane_in_next->left_boundary->isego;
      left_bound.mean_end_heading =
          lane_in_next->left_boundary->mean_end_heading;
      left_bound.pred_end_heading =
          lane_in_next->left_boundary->pred_end_heading;
      left_bound.mean_end_heading_std_dev =
          lane_in_next->left_boundary->mean_end_heading_std_dev;
      left_bound.mean_end_interval =
          lane_in_next->left_boundary->mean_end_interval;
      for (auto& delete_id : lane_in_next->left_boundary->deteled_ids) {
        left_bound.deteled_ids.emplace_back(delete_id);
      }
      right_bound.id = lane_in_next->right_boundary->id;
      right_bound.lanepos = lane_in_next->right_boundary->lanepos;
      right_bound.type = lane_in_next->right_boundary->type;
      right_bound.color = lane_in_next->right_boundary->color;
      right_bound.isego = lane_in_next->right_boundary->isego;
      right_bound.mean_end_heading =
          lane_in_next->right_boundary->mean_end_heading;
      right_bound.pred_end_heading =
          lane_in_next->right_boundary->pred_end_heading;
      right_bound.mean_end_heading_std_dev =
          lane_in_next->right_boundary->mean_end_heading_std_dev;
      right_bound.mean_end_interval =
          lane_in_next->right_boundary->mean_end_interval;
      for (auto& delete_id : lane_in_next->right_boundary->deteled_ids) {
        right_bound.deteled_ids.emplace_back(delete_id);
      }
      // 判断左右line是否已经存在
      int left_bound_exist = 0, right_bound_exist = 0;
      for (auto& lane_in_curr : curr_group->lanes) {
        if (lane_in_curr->left_boundary->id ==
            lane_in_next->left_boundary->id) {
          // left_bound = *(lane_in_curr->left_boundary);
          for (auto& line_pt : lane_in_curr->left_boundary->pts) {
            Point pt_pre(gm::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = 1;
        } else if (lane_in_curr->left_boundary->id ==
                   lane_in_next->right_boundary->id) {
          // right_bound = *(lane_in_curr->left_boundary);
          for (auto& line_pt : lane_in_curr->left_boundary->pts) {
            Point pt_pre(gm::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = 1;
        } else if (lane_in_curr->right_boundary->id ==
                   lane_in_next->left_boundary->id) {
          // left_bound = *(lane_in_curr->right_boundary);
          for (auto& line_pt : lane_in_curr->right_boundary->pts) {
            Point pt_pre(gm::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = 1;
        } else if (lane_in_curr->right_boundary->id ==
                   lane_in_next->right_boundary->id) {
          // right_bound = *(lane_in_curr->right_boundary);
          for (auto& line_pt : lane_in_curr->right_boundary->pts) {
            Point pt_pre(gm::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = 1;
        }
      }
      if (left_bound_exist == 1 && right_bound_exist == 0) {
        if (left_bound.pts.size() < 2 ||
            lane_in_next->right_boundary->pts.empty()) {
          return;
        }
        Point right_pt_pred(PREDICTED,
                            lane_in_next->right_boundary->pts[0].pt.x(),
                            lane_in_next->right_boundary->pts[0].pt.y(),
                            lane_in_next->right_boundary->pts[0].pt.z());
        int left_index = static_cast<int>(left_bound.pts.size()) - 1;
        right_bound.pts.insert(right_bound.pts.begin(), right_pt_pred);
        float dx = left_bound.pts[left_index].pt.x() -
                   left_bound.pts[left_index - 1].pt.x();
        float dy = left_bound.pts[left_index].pt.y() -
                   left_bound.pts[left_index - 1].pt.y();
        while (right_pt_pred.pt.x() >
                   curr_group->group_segments[0]->start_slice.po.x() &&
               left_index > 0 &&
               (dx > 0 && dx * dx + dy * dy > 0.4 || dx * dx + dy * dy < 0.4)) {
          dx = left_bound.pts[left_index].pt.x() -
               left_bound.pts[left_index - 1].pt.x();
          dy = left_bound.pts[left_index].pt.y() -
               left_bound.pts[left_index - 1].pt.y();
          float pre_x = right_pt_pred.pt.x() - dx;
          float pre_y = right_pt_pred.pt.y() - dy;
          right_pt_pred =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          right_bound.pts.insert(right_bound.pts.begin(), right_pt_pred);
          left_index--;
        }
        if (right_bound.pts.empty()) {
          return;
        }
      } else if (left_bound_exist == 0 && right_bound_exist == 1) {
        if (right_bound.pts.size() < 2 ||
            lane_in_next->left_boundary->pts.empty()) {
          return;
        }
        Point left_pt_pred(PREDICTED,
                           lane_in_next->left_boundary->pts[0].pt.x(),
                           lane_in_next->left_boundary->pts[0].pt.y(),
                           lane_in_next->left_boundary->pts[0].pt.z());
        int right_index = static_cast<int>(right_bound.pts.size()) - 1;
        left_bound.pts.insert(left_bound.pts.begin(), left_pt_pred);
        float dx = right_bound.pts[right_index].pt.x() -
                   right_bound.pts[right_index - 1].pt.x();
        float dy = right_bound.pts[right_index].pt.y() -
                   right_bound.pts[right_index - 1].pt.y();
        while (left_pt_pred.pt.x() >
                   curr_group->group_segments[0]->start_slice.po.x() &&
               right_index > 0 &&
               (dx > 0 && dx * dx + dy * dy > 0.4 || dx * dx + dy * dy < 0.4)) {
          dx = right_bound.pts[right_index].pt.x() -
               right_bound.pts[right_index - 1].pt.x();
          dy = right_bound.pts[right_index].pt.y() -
               right_bound.pts[right_index - 1].pt.y();
          float pre_x = left_pt_pred.pt.x() - dx;
          float pre_y = left_pt_pred.pt.y() - dy;
          left_pt_pred =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          left_bound.pts.insert(left_bound.pts.begin(), left_pt_pred);
          right_index--;
        }
        if (left_bound.pts.empty()) {
          return;
        }
      } else if (left_bound_exist == 0 && right_bound_exist == 0) {
        if (lane_in_next->center_line_param_front.empty() ||
            lane_in_next->left_boundary->pts.empty() ||
            lane_in_next->right_boundary->pts.empty()) {
          return;
        }
        Point left_pt_pred(PREDICTED,
                           lane_in_next->left_boundary->pts[0].pt.x(),
                           lane_in_next->left_boundary->pts[0].pt.y(),
                           lane_in_next->left_boundary->pts[0].pt.z());
        double b_left =
            left_pt_pred.pt.y() -
            lane_in_next->center_line_param_front[1] * left_pt_pred.pt.x();
        while (left_pt_pred.pt.x() >
               curr_group->group_segments[0]->start_slice.po.x()) {
          left_bound.pts.insert(left_bound.pts.begin(), left_pt_pred);
          float pre_x = left_pt_pred.pt.x() - 1.0;
          float pre_y =
              b_left + lane_in_next->center_line_param_front[1] * pre_x;
          left_pt_pred =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
        }
        if (left_bound.pts.empty()) {
          return;
        }
        Point right_pt_pred(PREDICTED,
                            lane_in_next->right_boundary->pts[0].pt.x(),
                            lane_in_next->right_boundary->pts[0].pt.y(),
                            lane_in_next->right_boundary->pts[0].pt.z());
        double b_right =
            right_pt_pred.pt.y() -
            lane_in_next->center_line_param_front[1] * right_pt_pred.pt.x();
        while (right_pt_pred.pt.x() >
               curr_group->group_segments[0]->start_slice.po.x()) {
          right_bound.pts.insert(right_bound.pts.begin(), right_pt_pred);
          float pre_x = right_pt_pred.pt.x() - 1.0;
          float pre_y =
              b_right + lane_in_next->center_line_param_front[1] * pre_x;
          right_pt_pred =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
        }
        if (right_bound.pts.empty()) {
          return;
        }
      }
      lane_pre.str_id = lane_in_next->str_id;
      lane_pre.lanepos_id = lane_in_next->lanepos_id;
      lane_pre.str_id_with_group = curr_group->str_id + ":" + lane_pre.str_id;
      lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
      lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
      // lane_pre.center_line_param = lane_in_next->center_line_param_front;
      // lane_pre.center_line_param_front =
      // lane_in_next->center_line_param_front; lane_pre.center_line_pts =
      // ctr_pts;
      Lane::Ptr lane_ptr = std::make_shared<Lane>(lane_pre);
      // FitCenterLine(lane_ptr);
      ComputeCenterPoints(lane_ptr);
      if (lane_ptr->center_line_param.empty()) {
        lane_ptr->center_line_param = lane_in_next->center_line_param_front;
      }
      if (lane_ptr->center_line_param_front.empty()) {
        lane_ptr->center_line_param_front =
            lane_in_next->center_line_param_front;
      }
      HLOG_DEBUG << "lane_pre.center_line_pts.size() = "
                 << lane_pre.center_line_pts.size() << "   "
                 << lane_ptr->center_line_pts.size()
                 << "  lane_pre.left_boundary = "
                 << lane_pre.left_boundary->pts.size()
                 << "  lane_pre.right_boundary = "
                 << lane_pre.right_boundary->pts.size();
      if (lane_ptr->center_line_pts.empty()) {
        return;
      }
      // lane_pre.next_lane_str_id_with_group.emplace_back(
      //     lane_in_next->str_id_with_group);
      lane_in_next->prev_lane_str_id_with_group.emplace_back(
          lane_pre.str_id_with_group);
      for (const auto& lane_right :
           lane_in_next->right_lane_str_id_with_group) {
        size_t index = lane_right.find(":");
        std::string right_str =
            curr_group->str_id +
            lane_right.substr(index, lane_right.size() - index);
        lane_pre.right_lane_str_id_with_group.emplace_back(right_str);
      }
      for (const auto& lane_left : lane_in_next->left_lane_str_id_with_group) {
        size_t index = lane_left.find(":");
        std::string left_str =
            curr_group->str_id +
            lane_left.substr(index, lane_left.size() - index);
        lane_pre.left_lane_str_id_with_group.emplace_back(left_str);
      }
      curr_group->lanes.emplace_back(lane_ptr);
    }
  }
}

void GroupMap::BuildVirtualMergeLane(Group::Ptr curr_group,
                                     Group::Ptr next_group, size_t curr_lane,
                                     size_t curr_lane_next) {
  size_t next_lane = -1;
  for (size_t i = 0; i < next_group->lanes.size(); ++i) {
    if (next_group->lanes[i]->str_id_with_group ==
        curr_group->lanes[curr_lane_next]->next_lane_str_id_with_group[0]) {
      next_lane = i;
      break;
    }
  }
  if (next_lane == -1 ||
      next_group->lanes[next_lane]->center_line_param_front.empty() ||
      next_group->lanes[next_lane]->center_line_pts.size() < 11) {
    return;
  }
  size_t grp_seg_size = curr_group->group_segments.size();
  float y1 = -1.0, y2 = -1.0;
  for (const auto& lane_iter :
       curr_group->group_segments[grp_seg_size - 1]->lane_segments) {
    if (lane_iter->str_id == curr_group->lanes[curr_lane]->str_id) {
      y1 = Dist(lane_iter->left_boundary->center,
                lane_iter->right_boundary->center);
    }
  }
  for (const auto& lane_iter :
       curr_group->group_segments[grp_seg_size - 3]->lane_segments) {
    if (lane_iter->str_id == curr_group->lanes[curr_lane]->str_id) {
      y2 = Dist(lane_iter->left_boundary->center,
                lane_iter->right_boundary->center);
    }
  }
  if (y2 < y1 + 0.2) {
    return;
  }
  Lane lane_pre;
  LineSegment left_bound;
  auto& lane_in_curr = curr_group->lanes[curr_lane];
  auto& lane_in_next = next_group->lanes[next_lane];
  left_bound.id = lane_in_curr->left_boundary->id;
  left_bound.lanepos = lane_in_curr->left_boundary->lanepos;
  left_bound.type = lane_in_curr->left_boundary->type;
  left_bound.color = lane_in_curr->left_boundary->color;
  left_bound.isego = lane_in_curr->left_boundary->isego;
  left_bound.is_near_road_edge = lane_in_curr->left_boundary->is_near_road_edge;
  left_bound.mean_end_heading = lane_in_curr->left_boundary->mean_end_heading;
  left_bound.pred_end_heading = lane_in_curr->left_boundary->pred_end_heading;
  left_bound.mean_end_heading_std_dev =
      lane_in_curr->left_boundary->mean_end_heading_std_dev;
  left_bound.mean_end_interval = lane_in_curr->left_boundary->mean_end_interval;
  for (const auto& delete_id : lane_in_curr->left_boundary->deteled_ids) {
    left_bound.deteled_ids.emplace_back(delete_id);
  }
  size_t index_left = lane_in_curr->left_boundary->pts.size();

  Point left_pt_pred(PREDICTED,
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
  // float b_left = left_pt_pred.pt.y() -
  //                lane_in_curr->center_line_param[1] * left_pt_pred.pt.x();

  // float b_next_left = lane_in_next->left_boundary->pts[0].pt.y() -
  //                     lane_in_next->center_line_param_front[1] *
  //                         lane_in_next->left_boundary->pts[0].pt.x();

  // float st_sl_x = left_pt_pred.pt.x();
  // float distance = st_sl_x * lane_in_curr->center_line_param[1] + b_left -
  //                  st_sl_x * lane_in_next->center_line_param_front[1] -
  //                  b_next_left;
  // int flag = 1;
  // while (abs(distance) > 0.2 && flag > 0) {
  //   left_bound.pts.emplace_back(left_pt_pred);
  //   st_sl_x += 1.0;
  //   float pre_y = b_left + lane_in_curr->center_line_param[1] * st_sl_x;
  //   left_pt_pred = Point(PREDICTED, st_sl_x, pre_y, float(0.0));
  //   float distance2 = pre_y -
  //                     st_sl_x * lane_in_next->center_line_param_front[1] -
  //                     b_next_left;
  //   flag = (abs(distance2) < abs(distance)) ? 1 : 0;
  //   distance = distance2;
  // }
  auto dis_left = lane_in_next->left_boundary->pts[9].pt - left_pt_pred.pt;
  for (int i = 0; i < 10; ++i) {
    left_pt_pred =
        Point(PREDICTED, left_pt_pred.pt.x() + dis_left.x() * i,
              left_pt_pred.pt.y() + dis_left.y() * i, static_cast<float>(0.0));
    left_bound.pts.emplace_back(left_pt_pred);
  }

  if (left_bound.pts.empty()) {
    return;
  }
  LineSegment right_bound;
  right_bound.id = lane_in_curr->right_boundary->id;
  right_bound.lanepos = lane_in_curr->right_boundary->lanepos;
  right_bound.type = lane_in_curr->right_boundary->type;
  right_bound.color = lane_in_curr->right_boundary->color;
  right_bound.isego = lane_in_curr->right_boundary->isego;
  right_bound.is_near_road_edge =
      lane_in_curr->right_boundary->is_near_road_edge;
  right_bound.mean_end_heading = lane_in_curr->right_boundary->mean_end_heading;
  right_bound.pred_end_heading = lane_in_curr->right_boundary->pred_end_heading;
  right_bound.mean_end_heading_std_dev =
      lane_in_curr->right_boundary->mean_end_heading_std_dev;
  right_bound.mean_end_interval =
      lane_in_curr->right_boundary->mean_end_interval;
  for (const auto& delete_id : lane_in_curr->right_boundary->deteled_ids) {
    right_bound.deteled_ids.emplace_back(delete_id);
  }
  size_t index_right = lane_in_curr->right_boundary->pts.size();
  Point right_pt_pred(
      PREDICTED, lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
      lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
      lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
  // float b_right = right_pt_pred.pt.y() -
  //                 lane_in_curr->center_line_param[1] *
  //                 right_pt_pred.pt.x();
  float b_next_right = lane_in_next->right_boundary->pts[0].pt.y() -
                       lane_in_next->center_line_param_front[1] *
                           lane_in_next->right_boundary->pts[0].pt.x();
  // st_sl_x = right_pt_pred.pt.x();
  // distance = st_sl_x * lane_in_curr->center_line_param[1] + b_right -
  //            st_sl_x * lane_in_next->center_line_param_front[1] -
  //            b_next_right;
  // flag = 1;
  // while (abs(distance) > 0.2 && flag > 0) {
  //   right_bound.pts.emplace_back(right_pt_pred);
  //   st_sl_x += 1.0;
  //   float pre_y = b_right + lane_in_curr->center_line_param[1] * st_sl_x;
  //   right_pt_pred = Point(PREDICTED, st_sl_x, pre_y, float(0.0));
  //   float distance2 = pre_y -
  //                     st_sl_x * lane_in_next->center_line_param_front[1] -
  //                     b_next_right;
  //   flag = (abs(distance2) < abs(distance)) ? 1 : 0;
  //   distance = distance2;
  // }
  Eigen::Vector3f poly_right(lane_in_next->left_boundary->pts[9].pt.x(),
                             lane_in_next->left_boundary->pts[9].pt.x() *
                                     lane_in_next->center_line_param_front[1] +
                                 b_next_right,
                             0.0);
  auto dis_right = poly_right - right_pt_pred.pt;
  for (int i = 0; i < 10; ++i) {
    right_pt_pred = Point(
        PointType::PREDICTED, right_pt_pred.pt.x() + dis_right.x() * i,
        right_pt_pred.pt.y() + dis_right.y() * i, static_cast<float>(0.0));
    right_bound.pts.emplace_back(right_pt_pred);
  }

  if (right_bound.pts.empty()) {
    return;
  }

  std::vector<Point> ctr_pts;
  size_t index_center = lane_in_curr->center_line_pts.size();
  Point center_pt_pred(PointType::PREDICTED,
                       lane_in_curr->center_line_pts[index_center - 1].pt.x(),
                       lane_in_curr->center_line_pts[index_center - 1].pt.y(),
                       lane_in_curr->center_line_pts[index_center - 1].pt.z());
  Eigen::Vector3f poly_center(lane_in_next->left_boundary->pts[9].pt.x(),
                              lane_in_next->left_boundary->pts[9].pt.x() *
                                      lane_in_next->center_line_param_front[1] +
                                  lane_in_next->center_line_param_front[0],
                              0.0);
  auto dis_center = poly_center - center_pt_pred.pt;
  for (int i = 0; i < 10; ++i) {
    center_pt_pred = Point(
        PointType::PREDICTED, center_pt_pred.pt.x() + dis_center.x() * i,
        center_pt_pred.pt.y() + dis_center.y() * i, static_cast<float>(0.0));
    ctr_pts.emplace_back(center_pt_pred);
  }
  // st_sl_x = center_pt_pred.pt.x();
  // distance = lane_in_curr->center_line_param[1] * st_sl_x +
  //            lane_in_curr->center_line_param[0] -
  //            lane_in_next->center_line_param_front[1] * st_sl_x -
  //            lane_in_next->center_line_param_front[0];
  // flag = 1;
  // while (abs(distance) > 0.2 && flag > 0) {
  //   ctr_pts.emplace_back(center_pt_pred);
  //   st_sl_x += 1.0;
  //   float pre_y = lane_in_curr->center_line_param[1] * st_sl_x +
  //                 lane_in_curr->center_line_param[0];
  //   center_pt_pred = Point(PREDICTED, st_sl_x, pre_y, float(0.0));
  //   float distance2 = pre_y -
  //                     st_sl_x * lane_in_next->center_line_param_front[1] -
  //                     lane_in_next->center_line_param_front[0];
  //   flag = (abs(distance2) < abs(distance)) ? 1 : 0;
  //   distance = distance2;
  // }

  // HLOG_ERROR <<"distance = "<<distance;
  if (ctr_pts.empty()) {
    return;
  }
  lane_pre.str_id = lane_in_curr->str_id;
  lane_pre.lanepos_id = lane_in_curr->lanepos_id;
  lane_pre.str_id_with_group = next_group->str_id + ":" + lane_pre.str_id;
  lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
  lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
  lane_pre.center_line_pts = ctr_pts;

  lane_pre.prev_lane_str_id_with_group.emplace_back(
      lane_in_curr->str_id_with_group);
  lane_in_curr->next_lane_str_id_with_group.emplace_back(
      lane_pre.str_id_with_group);
  lane_pre.next_lane_str_id_with_group.emplace_back(
      lane_in_next->str_id_with_group);
  if (lane_pre.left_boundary->pts.size() > 2 &&
      lane_pre.right_boundary->pts.size() > 2) {
    next_group->lanes.emplace_back(std::make_shared<Lane>(lane_pre));
  }
}
// 判断车道线是否需要预测，当前预测的条件是：
// 1.线的末端点必须在车前方；
// 2.线的末端点距离自车距离在可信赖的感知范围之外，即可信赖的感知范围的线是不会预测的；
// 3.线末端平均heading足够小，即heading与自车行驶方向差异较大的线也不会预测；
// 4.线末端heading的标准差足够小（太大说明heading变换较大，可能是曲线）；
// 5.线末端平均点间距足够大；
//! TBD：当前仅对比较平直的线进行直线预测，后续考虑对弯道进行预测
bool GroupMap::LaneLineNeedToPredict(const LineSegment& line, bool check_back) {
  if (line.pts.empty()) {
    return false;
  }
  Eigen::Vector2f back_pt = line.pts.back().pt.head<2>();
  auto mean_heading = line.mean_end_heading;
  auto mean_heading_std = line.mean_end_heading_std_dev;
  auto mean_interval = line.mean_end_interval;
  float dist_to_veh = back_pt.norm();
  bool valid_back = true;
  if (check_back && back_pt.x() < -conf_.junction_predict_distance) {
    valid_back = false;
  }
  if (valid_back &&
      // dist_to_veh > conf_.robust_percep_dist &&
      dist_to_veh < conf_.predict_farthest_dist &&
      mean_heading < conf_.max_heading_rad &&
      // mean_heading_std < conf_.max_heading_std_dev &&
      mean_interval > conf_.min_predict_interval) {
    return true;
  }
  return false;
}

// 对line从最后一个点开始，沿着heading方向直线预测
//! TBD：当前直接按直线预测，后续考虑对弯道继续非直线预测，比如按曲率
void GroupMap::PredictLaneLine(std::vector<Lane::Ptr>* pred_lane,
                               const Lane::Ptr curr_lane) {
  // 预测左边线
  Lane lane_pre;
  LineSegment left_bound;
  left_bound.id = curr_lane->left_boundary->id;
  left_bound.lanepos = curr_lane->left_boundary->lanepos;
  left_bound.type = em::LaneType_DASHED;
  left_bound.color = em::WHITE;
  left_bound.isego = curr_lane->left_boundary->isego;
  left_bound.is_near_road_edge = curr_lane->left_boundary->is_near_road_edge;

  left_bound.mean_end_heading = curr_lane->left_boundary->mean_end_heading;
  left_bound.mean_end_heading_std_dev =
      curr_lane->left_boundary->mean_end_heading_std_dev;
  left_bound.mean_end_interval = curr_lane->left_boundary->mean_end_interval;
  for (const auto& delete_id : curr_lane->left_boundary->deteled_ids) {
    left_bound.deteled_ids.emplace_back(delete_id);
  }
  if (curr_lane->left_boundary != nullptr &&
      !curr_lane->left_boundary->pts.empty()) {
    Eigen::Vector2f back_pt = curr_lane->left_boundary->pts.back().pt.head<2>();
    auto mean_interval = curr_lane->left_boundary->mean_end_interval;
    float dist_to_veh = back_pt.norm();
    auto pred_length = conf_.predict_farthest_dist - dist_to_veh;
    auto pred_counts = static_cast<int>(pred_length / mean_interval);

    double left_kappa = std::get<1>(curr_lane->left_boundary->pred_end_heading);
    double left_heading =
        std::get<0>(curr_lane->left_boundary->pred_end_heading);
    double new_kappa = 0.0;

    for (int i = 0; i <= pred_counts; ++i) {
      new_kappa =
          left_kappa + std::get<2>(curr_lane->left_boundary->pred_end_heading);
      new_kappa = std::min(new_kappa, static_cast<double>(1.0 / 150.0));
      // new_kappa = left_kappa;
      if (math::DoubleHasSameSign(
              new_kappa,
              std::get<1>(curr_lane->left_boundary->pred_end_heading))) {
        left_kappa = new_kappa;
      }
      if (left_heading >= 0.) {
        left_heading += left_kappa;
      } else {
        left_heading -= left_kappa;
      }

      Eigen::Vector2f n(std::cos(left_heading), std::sin(left_heading));
      Eigen::Vector2f pt = back_pt + i * mean_interval * n;
      Point pred_pt;
      pred_pt.pt << pt.x(), pt.y(), 0;
      pred_pt.type = gm::VIRTUAL;
      left_bound.pts.emplace_back(pred_pt);
    }
  }

  // 预测右边线
  LineSegment right_bound;
  right_bound.id = curr_lane->right_boundary->id;
  right_bound.lanepos = curr_lane->right_boundary->lanepos;
  right_bound.type = em::LaneType_DASHED;
  right_bound.color = em::WHITE;
  right_bound.isego = curr_lane->right_boundary->isego;
  right_bound.is_near_road_edge = curr_lane->right_boundary->is_near_road_edge;
  right_bound.mean_end_heading = curr_lane->right_boundary->mean_end_heading;
  right_bound.mean_end_heading_std_dev =
      curr_lane->right_boundary->mean_end_heading_std_dev;
  right_bound.mean_end_interval = curr_lane->right_boundary->mean_end_interval;
  for (const auto& delete_id : curr_lane->right_boundary->deteled_ids) {
    right_bound.deteled_ids.emplace_back(delete_id);
  }
  if (curr_lane->right_boundary != nullptr &&
      !curr_lane->right_boundary->pts.empty()) {
    Eigen::Vector2f back_pt =
        curr_lane->right_boundary->pts.back().pt.head<2>();
    auto mean_interval = curr_lane->right_boundary->mean_end_interval;
    float dist_to_veh = back_pt.norm();
    auto pred_length = conf_.predict_farthest_dist - dist_to_veh;
    auto pred_counts = static_cast<int>(pred_length / mean_interval);

    double right_kappa =
        std::get<1>(curr_lane->right_boundary->pred_end_heading);
    double right_heading =
        std::get<0>(curr_lane->right_boundary->pred_end_heading);
    double new_kappa = 0.0;

    for (int i = 0; i <= pred_counts; ++i) {
      new_kappa = right_kappa +
                  std::get<2>(curr_lane->right_boundary->pred_end_heading);
      new_kappa = std::min(new_kappa, static_cast<double>(1.0 / 150.0));
      // new_kappa = right_kappa;
      if (math::DoubleHasSameSign(
              right_kappa,
              std::get<1>(curr_lane->right_boundary->pred_end_heading))) {
        right_kappa = new_kappa;
      }
      if (right_heading >= 0.) {
        right_heading += right_kappa;
      } else {
        right_heading -= right_kappa;
      }

      Eigen::Vector2f n(std::cos(right_heading), std::sin(right_heading));
      Eigen::Vector2f pt = back_pt + i * mean_interval * n;
      Point pred_pt;
      pred_pt.pt << pt.x(), pt.y(), 0;
      pred_pt.type = gm::VIRTUAL;
      right_bound.pts.emplace_back(pred_pt);
    }
  }

  // 预测中心线
  std::vector<Point> ctr_pts;
  if (!curr_lane->center_line_pts.empty()) {
    Eigen::Vector2f back_pt = curr_lane->center_line_pts.back().pt.head<2>();
    auto mean_interval = (curr_lane->left_boundary->mean_end_interval +
                          curr_lane->right_boundary->mean_end_interval) /
                         2;
    float dist_to_veh = back_pt.norm();
    auto pred_length = conf_.predict_farthest_dist - dist_to_veh;
    auto pred_counts = static_cast<int>(pred_length / mean_interval);

    double center_kappa =
        (std::get<1>(curr_lane->left_boundary->pred_end_heading) +
         std::get<1>(curr_lane->right_boundary->pred_end_heading)) /
        2;
    double center_heading =
        (std::get<0>(curr_lane->left_boundary->pred_end_heading) +
         std::get<0>(curr_lane->right_boundary->pred_end_heading)) /
        2;
    double new_kappa = 0.0;

    for (int i = 0; i <= pred_counts; ++i) {
      new_kappa = center_kappa +
                  (std::get<2>(curr_lane->left_boundary->pred_end_heading) +
                   std::get<2>(curr_lane->right_boundary->pred_end_heading)) /
                      2;
      new_kappa = std::min(new_kappa, static_cast<double>(1.0 / 150.0));
      // new_kappa = center_kappa;
      if (math::DoubleHasSameSign(
              center_kappa,
              (std::get<1>(curr_lane->left_boundary->pred_end_heading) +
               std::get<1>(curr_lane->right_boundary->pred_end_heading)) /
                  2)) {
        center_kappa = new_kappa;
      }
      if (center_heading >= 0.) {
        center_heading += center_kappa;
      } else {
        center_heading -= center_kappa;
      }

      Eigen::Vector2f n(std::cos(center_heading), std::sin(center_heading));
      Eigen::Vector2f pt = back_pt + i * mean_interval * n;
      Point pred_pt;
      pred_pt.pt << pt.x(), pt.y(), 0;
      pred_pt.type = gm::VIRTUAL;
      ctr_pts.emplace_back(pred_pt);
    }
  }

  lane_pre.str_id = curr_lane->str_id;
  lane_pre.lanepos_id = curr_lane->lanepos_id;
  size_t index = curr_lane->str_id_with_group.find(":");
  lane_pre.str_id_with_group =
      curr_lane->str_id_with_group.substr(0, index) + "P:" + lane_pre.str_id;
  lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
  lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
  lane_pre.center_line_param = curr_lane->center_line_param;
  lane_pre.center_line_param_front = curr_lane->center_line_param;
  lane_pre.center_line_pts = ctr_pts;
  // 添加预测lane的前后继
  lane_pre.prev_lane_str_id_with_group.emplace_back(
      curr_lane->str_id_with_group);
  curr_lane->next_lane_str_id_with_group.emplace_back(
      lane_pre.str_id_with_group);
  // 添加预测lane的左右邻
  std::vector<std::string> pre_str_id_with_group;
  for (const auto& str_id : curr_lane->left_lane_str_id_with_group) {
    size_t index = str_id.find(":");
    auto str_id_with_group =
        str_id.substr(0, index) + "P" + str_id.substr(index);
    pre_str_id_with_group.emplace_back(str_id_with_group);
  }
  lane_pre.left_lane_str_id_with_group = pre_str_id_with_group;

  pre_str_id_with_group.clear();
  for (const auto& str_id : curr_lane->right_lane_str_id_with_group) {
    size_t index = str_id.find(":");
    auto str_id_with_group =
        str_id.substr(0, index) + "P" + str_id.substr(index);
    pre_str_id_with_group.emplace_back(str_id_with_group);
  }
  lane_pre.right_lane_str_id_with_group = pre_str_id_with_group;
  if (lane_pre.left_boundary->pts.size() > 1 &&
      lane_pre.right_boundary->pts.size() > 1 &&
      lane_pre.center_line_pts.size() > 1) {
    pred_lane->emplace_back(std::make_shared<Lane>(lane_pre));
  }
}

// 判断车道是否收缩，即宽度越来越小
bool GroupMap::IsLaneShrink(Lane::Ptr lane) {
  // 为空或点太少，默认非收缩
  if (lane == nullptr || lane->left_boundary->pts.size() < 3 ||
      lane->right_boundary->pts.size() < 3) {
    return false;
  }
  const auto& left = lane->left_boundary->pts;
  const auto& right = lane->right_boundary->pts;
  const auto& left_back_pt = left.back().pt;
  const auto& left_mid_pt = left.at(left.size() / 2).pt;

  const auto& right_back_v_pt0 =
      right.at(static_cast<int>(right.size()) - 2).pt;
  const auto& right_back_v_pt1 =
      right.at(static_cast<int>(right.size()) - 1).pt;

  size_t right_mid_idx = right.size() / 2;
  const auto& right_mid_v_pt0 = right.at(right_mid_idx - 1).pt;
  const auto& right_mid_v_pt1 = right.at(right_mid_idx).pt;

  Eigen::Vector2f lfbt(left_back_pt.x(), left_back_pt.y());
  Eigen::Vector2f lmpt(left_mid_pt.x(), left_mid_pt.y());
  Eigen::Vector2f rbvpt0(right_back_v_pt0.x(), right_back_v_pt0.y());
  Eigen::Vector2f rbvpt1(right_back_v_pt1.x(), right_back_v_pt1.y());
  Eigen::Vector2f rmvpt0(right_mid_v_pt0.x(), right_mid_v_pt0.y());
  Eigen::Vector2f rmvpt1(right_mid_v_pt1.x(), right_mid_v_pt1.y());

  auto left_back_dist = PointToVectorDist(rbvpt0, rbvpt1, lfbt);
  auto left_mid_dist = PointToVectorDist(rmvpt0, rmvpt1, lmpt);
  auto left_diff_dist = left_mid_dist - left_back_dist;
  const float kShrinkDiffThreshold = 0.5;
  if (left_diff_dist > 0 && std::abs(left_diff_dist) > kShrinkDiffThreshold &&
      left_back_dist < conf_.min_lane_width + 0.5) {
    return true;
  }

  return false;
}

double GroupMap::CalcLaneLength(Lane::Ptr lane) {
  if (lane == nullptr || lane->center_line_pts.size() < 2) {
    return 0;
  }
  double len = 0;
  for (int i = 0; i < static_cast<int>(lane->center_line_pts.size()) - 1; ++i) {
    Eigen::Vector3f p0(lane->center_line_pts.at(i).pt.x(),
                       lane->center_line_pts.at(i).pt.y(), 0);
    Eigen::Vector3f p1(lane->center_line_pts.at(i + 1).pt.x(),
                       lane->center_line_pts.at(i + 1).pt.y(), 0);
    len += Dist(p0, p1);
  }
  return len;
}

std::shared_ptr<hozon::mp::mf::em::ElementMapOut> GroupMap::ConvertToElementMap(
    const std::vector<Group::Ptr>& groups, const KinePose::Ptr& curr_pose,
    const em::ElementMap::Ptr& ele) {
  for (const auto& grp : groups) {
    if (grp == nullptr) {
      HLOG_ERROR << "found nullptr group";
      return nullptr;
    }
  }
  auto ele_map = std::make_shared<hozon::mp::mf::em::ElementMapOut>();
  ele_map->map_info.stamp = groups.front()->stamp;
  int lane_id = 0;
  std::map<std::string, int> lane_id_hash;
  for (const auto& grp : groups) {
    for (const auto& lane : grp->lanes) {
      lane_id_hash.insert_or_assign(lane->str_id_with_group, lane_id++);
    }
  }

  std::vector<std::vector<int32_t>> lane_ids_in_group;

  std::map<std::string, int> node_id_hash;
  int boundary_id = 0, node_id = 0, center_id = 0;
  for (const auto& grp : groups) {
    lane_ids_in_group.emplace_back(std::vector<int32_t>());
    for (const auto& lane : grp->lanes) {
      em::Lane ele_lane;
      ele_lane.left_track_id = lane->left_boundary->id;
      ele_lane.right_track_id = lane->right_boundary->id;
      // id
      ele_lane.id = lane_id_hash[lane->str_id_with_group];
      lane_ids_in_group.back().emplace_back(ele_lane.id);
      // central_curve
      em::CenterLineDetailed centerline;
      centerline.id = center_id++;
      double length = 0;
      em::Point prev_pt;
      for (size_t i = 0; i < lane->center_line_pts.size(); ++i) {
        const auto& pt = lane->center_line_pts[i];
        if (i > 0) {
          length += Dist(pt.pt, prev_pt);
        }
        prev_pt = pt.pt;
        em::PointDetailed center_pt;
        center_pt.point = pt.pt;
        if (pt.type == gm::RAW) {
          center_pt.pointtype = em::RAW;
        } else if (pt.type == gm::INTERPOLATED) {
          center_pt.pointtype = em::INTERPOLATED;
        } else {
          center_pt.pointtype = em::PREDICTED;
        }
        centerline.points.emplace_back(center_pt);
        // HLOG_ERROR<<"center_pt.pointtype = "<<center_pt.pointtype;
      }
      ele_lane.center_line_id = centerline.id;
      ele_map->center_lines[centerline.id] =
          std::make_shared<em::CenterLineDetailed>(centerline);

      // leftboundary
      em::BoundaryDetailed left_bound;
      {
        left_bound.id = boundary_id++;
        for (size_t i = 0; i < lane->left_boundary->pts.size(); ++i) {
          const auto& pt = lane->left_boundary->pts[i];
          em::PointDetailed left_pt;
          left_pt.point = pt.pt;
          if (pt.type == gm::RAW) {
            left_pt.pointtype = em::RAW;
          } else if (pt.type == gm::INTERPOLATED) {
            left_pt.pointtype = em::INTERPOLATED;
          } else {
            left_pt.pointtype = em::PREDICTED;
          }
          left_bound.nodes.emplace_back(left_pt);
        }
        left_bound.color = lane->left_boundary->color;
        left_bound.is_ego = lane->left_boundary->isego;
        left_bound.is_near_road_edge = lane->left_boundary->is_near_road_edge;
        left_bound.linetype = lane->left_boundary->type;
        left_bound.lanepos = lane->left_boundary->lanepos;
        for (const auto& delete_id : lane->left_boundary->deteled_ids) {
          left_bound.delete_ids.emplace_back(delete_id);
        }
        ele_lane.left_boundary_ids.emplace_back(left_bound.id);
        ele_map->boundaries[left_bound.id] =
            std::make_shared<em::BoundaryDetailed>(left_bound);
      }
      // rightboundary
      em::BoundaryDetailed right_bound;
      {
        right_bound.id = boundary_id++;
        for (size_t i = 0; i < lane->right_boundary->pts.size(); ++i) {
          const auto& pt = lane->right_boundary->pts[i];
          em::PointDetailed left_pt;
          left_pt.point = pt.pt;
          if (pt.type == gm::RAW) {
            left_pt.pointtype = em::RAW;
          } else if (pt.type == gm::INTERPOLATED) {
            left_pt.pointtype = em::INTERPOLATED;
          } else {
            left_pt.pointtype = em::PREDICTED;
          }
          right_bound.nodes.emplace_back(left_pt);
        }
        right_bound.color = lane->right_boundary->color;
        right_bound.is_ego = lane->right_boundary->isego;
        right_bound.is_near_road_edge = lane->right_boundary->is_near_road_edge;
        right_bound.linetype = lane->right_boundary->type;
        right_bound.lanepos = lane->right_boundary->lanepos;
        for (const auto& delete_id : lane->right_boundary->deteled_ids) {
          right_bound.delete_ids.emplace_back(delete_id);
        }
        ele_lane.right_boundary_ids.emplace_back(right_bound.id);
        ele_map->boundaries[right_bound.id] =
            std::make_shared<em::BoundaryDetailed>(right_bound);
      }
      ele_lane.length = static_cast<float>(length);
      for (size_t i = 0; i < lane->next_lane_str_id_with_group.size(); ++i) {
        ele_lane.next_lane_ids.emplace_back(
            lane_id_hash[lane->next_lane_str_id_with_group[i]]);
      }
      for (size_t i = 0; i < lane->prev_lane_str_id_with_group.size(); ++i) {
        ele_lane.prev_lane_ids.emplace_back(
            lane_id_hash[lane->prev_lane_str_id_with_group[i]]);
      }
      for (size_t i = 0; i < lane->left_lane_str_id_with_group.size(); ++i) {
        ele_lane.left_forward_lane_ids.emplace_back(
            lane_id_hash[lane->left_lane_str_id_with_group[i]]);
      }
      for (size_t i = 0; i < lane->right_lane_str_id_with_group.size(); ++i) {
        ele_lane.right_forward_lane_ids.emplace_back(
            lane_id_hash[lane->right_lane_str_id_with_group[i]]);
      }
      ele_map->lanes[ele_lane.id] = std::make_shared<em::Lane>(ele_lane);
    }
  }
  if (lane_ids_in_group.size() != groups.size()) {
    HLOG_ERROR << "lane_ids_in_group size not matched with groups, "
               << lane_ids_in_group.size() << ", " << groups.size()
               << ", not add roads";
    return ele_map;
  }

  int grp_idx = -1;
  int grp_size = lane_ids_in_group.size();
  for (const auto& grp : lane_ids_in_group) {
    grp_idx += 1;
    if (grp.empty()) {
      continue;
    }
    em::Road ele_road;
    ele_road.id = grp_idx;
    for (const auto& lane_ids : grp) {
      ele_road.lane_ids.emplace_back(lane_ids);
    }
    if (grp_idx > 1) {
      ele_road.prev_road_ids.emplace_back(grp_idx - 1);
    }
    if (grp_idx < grp_size) {
      ele_road.next_road_ids.emplace_back(grp_idx + 1);
    }
    ele_map->roads[ele_road.id] = std::make_shared<em::Road>(ele_road);
  }

  // stop lines arrows crosswalk
  ele_map->stop_lines = ele->stop_lines;
  ele_map->arrows = ele->arrows;
  ele_map->cross_walks = ele->cross_walks;
  ele_map->occ_roads = ele->occ_roads;
  ele_map->objs = ele->objs;
  return ele_map;
}

// 把Group里元素转换成map proto，并且坐标变换到local系
std::shared_ptr<hozon::hdmap::Map> GroupMap::ConvertToProtoMap(
    const std::vector<Group::Ptr>& groups, const KinePose::Ptr& curr_pose,
    const em::ElementMap::Ptr& ele_map, HistoryId* history_id) {
  for (const auto& grp : groups) {
    if (grp == nullptr) {
      HLOG_ERROR << "found nullptr group";
      return nullptr;
    }
  }

  auto map = std::make_shared<hozon::hdmap::Map>();
  map->Clear();
  // 填充header
  map->mutable_header()->mutable_header()->set_data_stamp(
      groups.front()->stamp);

  int lane_id = history_id->lane_id;  // 0;
  std::map<std::string, int> lane_id_hash;
  for (const auto& grp : groups) {
    for (const auto& lane : grp->lanes) {
      lane_id++;
      lane_id = lane_id % history_id->cicle;
      lane_id_hash.insert_or_assign(lane->str_id_with_group, lane_id);
    }
  }
  history_id->lane_id = lane_id;
  std::vector<std::vector<std::string>> lane_ids_in_group;

  for (const auto& grp : groups) {
    lane_ids_in_group.emplace_back(std::vector<std::string>());
    for (const auto& lane : grp->lanes) {
      auto* proto_lane = map->add_lane();
      // id
      auto proto_lane_id =
          std::to_string(lane_id_hash[lane->str_id_with_group]);
      proto_lane->mutable_id()->set_id(proto_lane_id);
      lane_ids_in_group.back().emplace_back(proto_lane_id);

      // central_curve
      auto* central_seg = proto_lane->mutable_central_curve()->add_segment();
      double length = 0;
      em::Point prev_pt;
      for (size_t i = 0; i < lane->center_line_pts.size(); ++i) {
        const auto& pt = lane->center_line_pts[i].pt;
        if (i > 0) {
          length += Dist(pt, prev_pt);
        }
        prev_pt = pt;
        em::Point pt_local = curr_pose->TransformPoint(pt);
        auto* proto_pt = central_seg->mutable_line_segment()->add_point();
        proto_pt->set_x(pt_local.x());
        proto_pt->set_y(pt_local.y());
        proto_pt->set_z(pt_local.z());
      }
      central_seg->set_length(length);
      //! TBD: heading
      // central_seg->set_heading();

      // left_boundary
      bool is_virtual = false;
      {
        auto* left_boundary = proto_lane->mutable_left_boundary();
        auto* left_seg = left_boundary->mutable_curve()->add_segment();
        length = 0;
        prev_pt.setZero();
        for (size_t i = 0; i < lane->left_boundary->pts.size(); ++i) {
          const auto& pt = lane->left_boundary->pts[i].pt;
          if (i > 0) {
            length += Dist(pt, prev_pt);
          }
          if (i == 1 && lane->left_boundary->pts[i].type == VIRTUAL) {
            is_virtual = true;
          }
          prev_pt = pt;
          em::Point pt_local = curr_pose->TransformPoint(pt);
          auto* proto_pt = left_seg->mutable_line_segment()->add_point();
          proto_pt->set_x(pt_local.x());
          proto_pt->set_y(pt_local.y());
          proto_pt->set_z(pt_local.z());
        }
        if (lane->left_boundary->pts.size() == 1 &&
            lane->left_boundary->pts[0].type == VIRTUAL) {
          is_virtual = true;
        }
        left_seg->set_length(length);
        //! TBD: heading
        // left_seg->set_heading();
        left_boundary->set_virtual_(is_virtual);
        auto* boundary_type = left_boundary->add_boundary_type();
        //! TBD: 这里s设为整个线的长度行不行?
        boundary_type->set_s(0.0);
        auto type = ProtoBoundType::UNKNOWN;
        std::set<em::LineType> dotted = {
            em::LaneType_DASHED,
            em::LaneType_SHORT_DASHED,
            em::LaneType_DOUBLE_DASHED,
            em::LaneType_FISHBONE_DASHED,
            em::LaneType_INTERSECTION_VIRTUAL_MARKING,
        };
        std::set<em::LineType> solid = {
            em::LaneType_SOLID,
            em::LaneType_FISHBONE_SOLID,
        };
        if (lane->left_boundary->lanepos > 0) {
          dotted.insert(em::LaneType_RIGHT_SOLID_LEFT_DASHED);
          solid.insert(em::LaneType_LEFT_SOLID_RIGHT_DASHED);
        } else {
          dotted.insert(em::LaneType_LEFT_SOLID_RIGHT_DASHED);
          solid.insert(em::LaneType_RIGHT_SOLID_LEFT_DASHED);
        }
        if (dotted.find(lane->left_boundary->type) != dotted.end() &&
            lane->left_boundary->color == em::YELLOW) {
          type = ProtoBoundType::DOTTED_YELLOW;
        } else if (dotted.find(lane->left_boundary->type) != dotted.end() &&
                   lane->left_boundary->color == em::WHITE) {
          type = ProtoBoundType::DOTTED_WHITE;
        } else if (solid.find(lane->left_boundary->type) != solid.end() &&
                   lane->left_boundary->color == em::YELLOW) {
          type = ProtoBoundType::SOLID_YELLOW;
        } else if (solid.find(lane->left_boundary->type) != solid.end() &&
                   lane->left_boundary->color == em::WHITE) {
          type = ProtoBoundType::SOLID_WHITE;
        } else if (lane->left_boundary->type == em::LaneType_DOUBLE_SOLID &&
                   lane->left_boundary->color == em::YELLOW) {
          type = ProtoBoundType::DOUBLE_YELLOW;
        }
        boundary_type->add_types(type);
      }
      // right_boundary
      is_virtual = false;
      {
        auto* right_boundary = proto_lane->mutable_right_boundary();
        auto* right_seg = right_boundary->mutable_curve()->add_segment();
        length = 0;
        prev_pt.setZero();
        for (size_t i = 0; i < lane->right_boundary->pts.size(); ++i) {
          const auto& pt = lane->right_boundary->pts[i].pt;
          if (i > 0) {
            length += Dist(pt, prev_pt);
          }
          if (i == 1 && lane->right_boundary->pts[i].type == VIRTUAL) {
            is_virtual = true;
          }
          prev_pt = pt;
          em::Point pt_local = curr_pose->TransformPoint(pt);
          auto* proto_pt = right_seg->mutable_line_segment()->add_point();
          proto_pt->set_x(pt_local.x());
          proto_pt->set_y(pt_local.y());
          proto_pt->set_z(pt_local.z());
        }
        if (lane->right_boundary->pts.size() == 1 &&
            lane->right_boundary->pts[0].type == VIRTUAL) {
          is_virtual = true;
        }
        right_seg->set_length(length);
        //! TBD: heading
        // right_seg->set_heading();
        right_boundary->set_virtual_(is_virtual);
        auto* boundary_type = right_boundary->add_boundary_type();
        //! TBD: 这里s设为整个线的长度行不行?
        boundary_type->set_s(0.0);
        auto type = ProtoBoundType::UNKNOWN;
        std::set<em::LineType> dotted = {
            em::LaneType_DASHED,
            em::LaneType_SHORT_DASHED,
            em::LaneType_DOUBLE_DASHED,
            em::LaneType_FISHBONE_DASHED,
        };
        std::set<em::LineType> solid = {
            em::LaneType_SOLID,
            em::LaneType_FISHBONE_SOLID,
        };
        if (lane->right_boundary->lanepos > 0) {
          dotted.insert(em::LaneType_RIGHT_SOLID_LEFT_DASHED);
          solid.insert(em::LaneType_LEFT_SOLID_RIGHT_DASHED);
        } else {
          dotted.insert(em::LaneType_LEFT_SOLID_RIGHT_DASHED);
          solid.insert(em::LaneType_RIGHT_SOLID_LEFT_DASHED);
        }
        if (dotted.find(lane->right_boundary->type) != dotted.end() &&
            lane->right_boundary->color == em::YELLOW) {
          type = ProtoBoundType::DOTTED_YELLOW;
        } else if (dotted.find(lane->right_boundary->type) != dotted.end() &&
                   lane->right_boundary->color == em::WHITE) {
          type = ProtoBoundType::DOTTED_WHITE;
        } else if (solid.find(lane->right_boundary->type) != solid.end() &&
                   lane->right_boundary->color == em::YELLOW) {
          type = ProtoBoundType::SOLID_YELLOW;
        } else if (solid.find(lane->right_boundary->type) != solid.end() &&
                   lane->right_boundary->color == em::WHITE) {
          type = ProtoBoundType::SOLID_WHITE;
        } else if (lane->right_boundary->type == em::LaneType_DOUBLE_SOLID &&
                   lane->right_boundary->color == em::YELLOW) {
          type = ProtoBoundType::DOUBLE_YELLOW;
        }
        boundary_type->add_types(type);
      }

      // length
      //! TBD: 设为中心线长度行不行？
      proto_lane->set_length(central_seg->length());
      //! TBD: speed_limit
      proto_lane->set_speed_limit(IsSpeedLimitValid(speed_limit_)
                                      ? speed_limit_.first
                                      : conf_.lane_speed_limit_kmph / 3.6);

      // left_neighbor_forward_lane_id
      for (const auto& left_id : lane->left_lane_str_id_with_group) {
        int id = -1;
        if (lane_id_hash.find(left_id) != lane_id_hash.end()) {
          id = lane_id_hash[left_id];
          proto_lane->add_left_neighbor_forward_lane_id()->set_id(
              std::to_string(id));
        } else {
          HLOG_DEBUG << "cannot find " << left_id << " in lane_id_hash";
        }
      }

      // right_neighbor_forward_lane_id
      for (const auto& right_id : lane->right_lane_str_id_with_group) {
        int id = -1;
        if (lane_id_hash.find(right_id) != lane_id_hash.end()) {
          id = lane_id_hash[right_id];
          proto_lane->add_right_neighbor_forward_lane_id()->set_id(
              std::to_string(id));
        } else {
          HLOG_DEBUG << "cannot find " << right_id << " in lane_id_hash";
        }
      }

      // successor_id
      for (const auto& next_id : lane->next_lane_str_id_with_group) {
        int id = -1;
        if (lane_id_hash.find(next_id) != lane_id_hash.end()) {
          id = lane_id_hash[next_id];
          proto_lane->add_successor_id()->set_id(std::to_string(id));
        } else {
          HLOG_DEBUG << "cannot find " << next_id << " in lane_id_hash";
        }
      }

      // predecessor_id
      for (const auto& prev_id : lane->prev_lane_str_id_with_group) {
        int id = -1;
        if (lane_id_hash.find(prev_id) != lane_id_hash.end()) {
          id = lane_id_hash[prev_id];
          proto_lane->add_predecessor_id()->set_id(std::to_string(id));
        } else {
          HLOG_DEBUG << "cannot find " << prev_id << " in lane_id_hash";
        }
      }

      //! TBD: LaneType都设为城区行不行?
      proto_lane->set_type(hozon::hdmap::Lane_LaneType_CITY_DRIVING);

      //! TBD: LaneTurn暂时都设为NOTURN行不行?
      proto_lane->set_turn(hozon::hdmap::Lane_LaneTurn_NO_TURN);
      //! TBD: MapLaneType暂时都设为normal行不行?
      proto_lane->mutable_map_lane_type()->set_normal(true);
    }
  }

  if (lane_ids_in_group.size() != groups.size()) {
    HLOG_ERROR << "lane_ids_in_group size not matched with groups, "
               << lane_ids_in_group.size() << ", " << groups.size()
               << ", not add roads";
    return map;
  }

  //! TBD: 当前认为始终一条Road，每个group对应一个RoadSection
  //! TBD: 暂时未加RoadBoundary
  auto* proto_road = map->add_road();
  proto_road->mutable_id()->set_id("0");
  int grp_idx = history_id->road_id;  // -1;
  for (const auto& grp : lane_ids_in_group) {
    grp_idx += 1;
    grp_idx = grp_idx % history_id->cicle;
    if (grp.empty()) {
      continue;
    }
    auto* proto_road_section = proto_road->add_section();
    proto_road_section->set_max_max_speed(
        IsSpeedLimitValid(speed_limit_) ? speed_limit_.second
                                        : conf_.road_max_max_speed_kmph / 3.6);
    proto_road_section->set_min_max_speed(conf_.road_min_max_speed_kmph / 3.6);
    proto_road_section->mutable_id()->set_id(std::to_string(grp_idx));
    for (const auto& id : grp) {
      auto* proto_lane_id = proto_road_section->add_lane_id();
      proto_lane_id->set_id(id);
    }
  }
  history_id->road_id = grp_idx;

  // stop lines
  for (const auto& stopline_it : ele_map->stop_lines) {
    auto* stop_line = map->add_stop_line();
    stop_line->set_id("01" + std::to_string(stopline_it.first));
    Eigen::Vector3f left_point =
        curr_pose->TransformPoint(stopline_it.second->points[0]);
    auto* point_left = stop_line->mutable_shape()->add_point();
    point_left->set_x(static_cast<double>(left_point.x()));
    point_left->set_y(static_cast<double>(left_point.y()));
    point_left->set_z(static_cast<double>(left_point.z()));
    Eigen::Vector3f right_point =
        curr_pose->TransformPoint(stopline_it.second->points[1]);

    auto* point_right = stop_line->mutable_shape()->add_point();
    point_right->set_x(static_cast<double>(right_point.x()));
    point_right->set_y(static_cast<double>(right_point.y()));
    point_right->set_z(static_cast<double>(right_point.z()));

    auto* signal = map->add_signal();
    signal->mutable_id()->set_id("01" + std::to_string(stopline_it.first));
    auto* segment = signal->add_stop_line()->add_segment();
    auto* point_l = segment->mutable_line_segment()->add_point();
    point_l->set_x(static_cast<double>(left_point.x()));
    point_l->set_y(static_cast<double>(left_point.y()));
    point_l->set_z(static_cast<double>(left_point.z()));
    auto* point_r = segment->mutable_line_segment()->add_point();
    point_r->set_x(static_cast<double>(right_point.x()));
    point_r->set_y(static_cast<double>(right_point.y()));
    point_r->set_z(static_cast<double>(right_point.z()));
    auto* start_position = segment->mutable_start_position();
    start_position->set_x(static_cast<double>(left_point.x()));
    start_position->set_y(static_cast<double>(left_point.y()));
    start_position->set_z(static_cast<double>(left_point.z()));

    for (const auto& lane_id_it : stopline_[stopline_it.first]->lane_id) {
      if (lane_id_hash.find(lane_id_it) != lane_id_hash.end()) {
        stop_line->add_lane_id(std::to_string(lane_id_hash[lane_id_it]));

        // 对停止线和车道线关联对构建overlap
        auto* stopline_overlap = map->add_overlap();
        auto id = "01" + std::to_string(stopline_it.first) + "_" +
                  std::to_string(lane_id_hash[lane_id_it]);
        stopline_overlap->mutable_id()->set_id(id);

        signal->add_overlap_id()->set_id(id);
        auto* object_stopline = stopline_overlap->add_object();
        object_stopline->mutable_id()->set_id(
            "01" + std::to_string(stopline_it.first));
        object_stopline->mutable_signal_overlap_info();

        auto* object_lane = stopline_overlap->add_object();
        object_lane->mutable_id()->set_id(
            std::to_string(lane_id_hash[lane_id_it]));
        auto* laneinfo = object_lane->mutable_lane_overlap_info();
        // 判断是否关联对是进路口还是出路口关联形式
        auto target_point =
            (stopline_it.second->points[0] + stopline_it.second->points[1]) / 2;
        auto length_to_start =
            LengthToLaneStart(groups, lane_id_it, target_point);
        laneinfo->set_start_s(length_to_start);
        laneinfo->set_end_s(length_to_start + 0.1);
        // 给map.lane.overlap_id赋值
        for (int i = 0; i < map->lane_size(); ++i) {
          auto* pro_lane = map->mutable_lane(i);
          if (pro_lane->id().id() == std::to_string(lane_id_hash[lane_id_it])) {
            auto* lane_overlap_id = pro_lane->add_overlap_id();
            lane_overlap_id->set_id(id);
          }
        }
      }
    }
  }

  // for (const auto& stopline_it : ele_map->stop_lines) {
  //   auto* signal = map->add_signal();
  //   signal->mutable_id()->set_id(std::to_string(stopline_it.first));
  //   Eigen::Vector3f left_point =
  //       curr_pose->TransformPoint(stopline_it.second->points[0]);
  //   auto* point_left = signal->add_stop_line()
  //                          ->add_segment()
  //                          ->mutable_line_segment()
  //                          ->add_point();
  //   point_left->set_x(static_cast<double>(left_point.x()));
  //   point_left->set_y(static_cast<double>(left_point.y()));
  //   point_left->set_z(static_cast<double>(left_point.z()));
  //   Eigen::Vector3f right_point =
  //       curr_pose->TransformPoint(stopline_it.second->points[1]);
  //   auto* point_right = signal->add_stop_line()
  //                           ->add_segment()
  //                           ->mutable_line_segment()
  //                           ->add_point();
  //   point_right->set_x(static_cast<double>(right_point.x()));
  //   point_right->set_y(static_cast<double>(right_point.y()));
  //   point_right->set_z(static_cast<double>(right_point.z()));
  // }

  // crosswalk
  for (const auto& crosswalk_it : ele_map->cross_walks) {
    if (crosswalk_it.second->polygon.points.size() != 4) {
      continue;
    }
    auto* cross_walk = map->add_crosswalk();
    cross_walk->mutable_id()->set_id("02" + std::to_string(crosswalk_it.first));
    for (const auto& point_it : crosswalk_it.second->polygon.points) {
      Eigen::Vector3f pt = curr_pose->TransformPoint(point_it);
      auto* cross_walk_pt = cross_walk->mutable_polygon()->add_point();
      cross_walk_pt->set_x(static_cast<double>(pt.x()));
      cross_walk_pt->set_y(static_cast<double>(pt.y()));
      cross_walk_pt->set_z(static_cast<double>(pt.z()));
    }

    for (const auto& lane_id_it : zebra_[crosswalk_it.first]->lane_id) {
      if (lane_id_hash.find(lane_id_it) != lane_id_hash.end()) {
        // 对斑马线和车道线关联对构建overlap
        auto* cross_walk_overlap = map->add_overlap();
        cross_walk_overlap->mutable_id()->set_id(
            "02" + std::to_string(crosswalk_it.first) + "_" +
            std::to_string(lane_id_hash[lane_id_it]));
        auto* object_cross = cross_walk_overlap->add_object();
        object_cross->mutable_id()->set_id("02" +
                                           std::to_string(crosswalk_it.first));
        object_cross->mutable_crosswalk_overlap_info();

        auto* object_lane = cross_walk_overlap->add_object();
        object_lane->mutable_id()->set_id(
            std::to_string(lane_id_hash[lane_id_it]));
        auto* laneinfo = object_lane->mutable_lane_overlap_info();
        // 判断是否关联对是进路口还是出路口关联形式
        auto target_point = (crosswalk_it.second->polygon.points[1] +
                             crosswalk_it.second->polygon.points[2]) /
                            2;
        if (CloseToLaneEnd(groups, lane_id_it, target_point)) {
          auto lane_length = GetLaneLength(groups, lane_id_it);
          laneinfo->set_start_s(lane_length);
          laneinfo->set_end_s(lane_length + 2.0);
        } else {
          laneinfo->set_start_s(0.0);
          laneinfo->set_end_s(2.0);
        }

        // 给map.lane.overlap_id赋值
        for (int i = 0; i < map->lane_size(); ++i) {
          auto* pro_lane = map->mutable_lane(i);
          if (pro_lane->id().id() == std::to_string(lane_id_hash[lane_id_it])) {
            auto* lane_overlap_id = pro_lane->add_overlap_id();
            lane_overlap_id->set_id("02" + std::to_string(crosswalk_it.first) +
                                    "_" +
                                    std::to_string(lane_id_hash[lane_id_it]));
          }
        }
      }
    }
  }

  // arrows
  for (const auto& arrow_it : ele_map->arrows) {
    if (arrow_it.second->polygon.points.size() != 4) {
      continue;
    }
    auto* arrow = map->add_arraw();
    arrow->set_id(std::to_string(arrow_it.first));
    double center_x = 0.0, center_y = 0.0;
    for (const auto& point_it : arrow_it.second->polygon.points) {
      Eigen::Vector3f pt = curr_pose->TransformPoint(point_it);
      auto* arrow_pt = arrow->mutable_shape()->add_point();
      arrow_pt->set_x(static_cast<double>(pt.x()));
      arrow_pt->set_y(static_cast<double>(pt.y()));
      arrow_pt->set_z(static_cast<double>(pt.z()));
      center_x += pt.x();
      center_y += pt.y();
    }
    arrow->mutable_center_point()->set_x(center_x / 4);
    arrow->mutable_center_point()->set_y(center_y / 4);
    Eigen::Quaternionf quat_arrow_in_veh(
        Eigen::AngleAxisf(arrow_it.second->heading, Eigen::Vector3f::UnitZ()));
    Eigen::Quaternionf quat_arrow_in_local_enu =
        curr_pose->quat * quat_arrow_in_veh;
    Eigen::Vector3f euler_arrow =
        quat_arrow_in_local_enu.toRotationMatrix().eulerAngles(2, 0, 1);
    arrow->set_heading(static_cast<double>(euler_arrow[0]));
    arrow->set_type(FillArrowType(arrow_it.second->type));
  }
  return map;
}
double GroupMap::GetLaneLength(const std::vector<Group::Ptr>& groups,
                               std::string str_id) {
  double length = 0;
  for (const auto& grp : groups) {
    for (const auto& lane : grp->lanes) {
      if (lane->str_id_with_group == str_id) {
        em::Point prev_pt;
        for (size_t i = 0; i < lane->center_line_pts.size(); ++i) {
          const auto& pt = lane->center_line_pts[i].pt;
          if (i > 0) {
            length += Dist(pt, prev_pt);
          }
          prev_pt = pt;
        }
        return length;
      }
    }
  }
  return length;
}

hozon::hdmap::ArrowData_Type GroupMap::FillArrowType(em::ArrowType arrowtype) {
  switch (arrowtype) {
    case em::ArrowType::UNKNOWN_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_UNKNOWN_TURN;
    case em::ArrowType::STRAIGHT_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT;
    case em::ArrowType::RIGHT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_TURN;
    case em::ArrowType::LEFT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_TURN;
    case em::ArrowType::U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_U_TURN;
    case em::ArrowType::STRAIGHT_LEFT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_LEFT_TURN;
    case em::ArrowType::STRAIGHT_RIGHT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_RIGHT_TURN;
    case em::ArrowType::STRAIGHT_U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_U_TURN;
    case em::ArrowType::LEFT_U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_U_TURN;
    case em::ArrowType::LEFT_RIGHT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_RIGHT_TURN;
    case em::ArrowType::LEFT_FRONT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_FRONT_TURN;
    case em::ArrowType::RIGHT_FRONT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_FRONT_TURN;
    case em::ArrowType::STRAIGHT_LEFT_RIGHT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::
          ArrowData_Type_STRAIGHT_LEFT_RIGHT_TURN;
    case em::ArrowType::STRAIGHT_LEFT_U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_LEFT_U_TURN;
    case em::ArrowType::RIGHT_U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_U_TURN;
    case em::ArrowType::FORBID_LEFT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_LEFT_TURN;
    case em::ArrowType::FORBID_RIGHT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_RIGHT_TURN;
    case em::ArrowType::FORBID_U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_U_TURN;
    default:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_FRONT_NEAR_CROSSWALK;
  }
  return hozon::hdmap::ArrowData_Type::ArrowData_Type_UNKNOWN_TURN;
}

// 获取内部Group
void GroupMap::GetGroups(std::vector<Group::Ptr>* groups) {
  if (groups == nullptr) {
    return;
  }
  groups->reserve(groups->size());
  for (const auto& grp : groups_) {
    groups->emplace_back(grp);
  }
}

void GroupMap::SetSpeedLimit(const std::pair<double, double>& map_speed_limit) {
  speed_limit_ = map_speed_limit;
}

// 导出为map proto
std::shared_ptr<hozon::hdmap::Map> GroupMap::Export(
    const em::ElementMap::Ptr& ele_map, HistoryId* history_id) {
  if (curr_pose_ == nullptr || groups_.empty()) {
    return nullptr;
  }
  auto map = ConvertToProtoMap(groups_, curr_pose_, ele_map, history_id);
  return map;
}

std::shared_ptr<hozon::mp::mf::em::ElementMapOut> GroupMap::AddElementMap(
    const em::ElementMap::Ptr& ele_map) {
  if (curr_pose_ == nullptr || groups_.empty()) {
    return nullptr;
  }
  auto ele_m = ConvertToElementMap(groups_, curr_pose_, ele_map);
  return ele_m;
}

void GroupMap::UpdatePathInCurrPose(const std::vector<KinePose::Ptr>& path,
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
  incross_before_virtual_lane_length_ =
      std::max(static_cast<float>(curr_pose.vel.head<2>().norm() * 0.4),
               incross_before_virtual_lane_length_);
}

void GroupMap::RemoveNullGroup(std::vector<Group::Ptr>* groups) {
  for (size_t grp_idx = 0; grp_idx < groups->size(); ++grp_idx) {
    auto& curr_group = groups->at(grp_idx);
    HLOG_DEBUG << "REMOVE1 curr_group->group_segments.size()"
               << curr_group->group_segments.size()
               << "REMOVE1 curr_group->lanes.size()"
               << curr_group->lanes.size();
    if (curr_group->group_segments.size() < 2 || curr_group->lanes.size() < 1) {
      HLOG_DEBUG << "REMOVE curr_group->group_segments.size()"
                 << curr_group->group_segments.size()
                 << "REMOVE curr_group->lanes.size()"
                 << curr_group->lanes.size();
      groups->erase(groups->begin() + grp_idx);
    }
  }
}

bool GroupMap::IsAngleOkOfCurGrpAndNextGrp(Group::Ptr curr_group,
                                           Group::Ptr next_group) {
  int curr_grp_gs_size = curr_group->group_segments.size();
  auto cur_group_v =
      (curr_group->group_segments[curr_grp_gs_size - 1]->end_slice.po -
       curr_group->group_segments[curr_grp_gs_size - 2]->end_slice.po)
          .normalized();
  auto next_group_v = (next_group->group_segments[1]->end_slice.po -
                       next_group->group_segments[0]->end_slice.po)
                          .normalized();
  return cur_group_v.dot(next_group_v) > 0.707 ? true : false;
}

bool GroupMap::AreAdjacentLaneGroupsDisconnected(Group::Ptr curr_group,
                                                 Group::Ptr next_group) {
  // 判断curr_group的最后一个groupsegment的中心点和next_group的第一个groupsegment的中心点的距离
  HLOG_DEBUG << "curr_group->group_segments size:"
             << curr_group->group_segments.size()
             << "next_group->group_segments size:"
             << next_group->group_segments.size();
  double group_distance = (curr_group->group_segments.back()->end_slice.po -
                           next_group->group_segments.front()->start_slice.po)
                              .norm();

  double curgrp_lane_to_next_min_dis = DBL_MAX;
  for (auto& cur_group_lane : curr_group->lanes) {
    if (cur_group_lane->center_line_pts.empty()) {
      continue;
    }
    double calcu_dis = (cur_group_lane->center_line_pts.back().pt -
                        next_group->group_segments.front()->start_slice.po)
                           .norm();
    if (calcu_dis < curgrp_lane_to_next_min_dis) {
      curgrp_lane_to_next_min_dis = calcu_dis;
    }
  }

  if (group_distance > 8.0 && curgrp_lane_to_next_min_dis > 8.0) {
    return true;
  }

  return false;
}

void GroupMap::RemainOnlyOneForwardCrossWalk(std::vector<Group::Ptr>* groups) {
  if (groups->size() < 2) {
    return;
  }

  // 存在两个及以上的车道组,考虑删除其他路口， 只保留前向一个路口。
  std::vector<int> junction_group_idxs;
  for (int grp_idx = 0; grp_idx < static_cast<int>(groups->size()) - 1;
       ++grp_idx) {
    auto& curr_group = groups->at(grp_idx);
    auto& next_group = groups->at(grp_idx + 1);
    if (curr_group->group_segments.empty() ||
        next_group->group_segments.empty()) {
      continue;
    }
    if (AreAdjacentLaneGroupsDisconnected(curr_group, next_group)) {
      if (!IsAngleOkOfCurGrpAndNextGrp(curr_group, next_group)) {
        // 路口转弯等场景则跳过。
        continue;
      }

      Eigen::Vector2f next_start_pl(
          next_group->group_segments.front()->start_slice.pl.x(),
          next_group->group_segments.front()->start_slice.pl.y());
      Eigen::Vector2f next_start_pr(
          next_group->group_segments.front()->start_slice.pr.x(),
          next_group->group_segments.front()->start_slice.pr.y());
      Eigen::Vector2f ego_pos(0.0, 0.0);

      // 如果路口出现在车子的后面，
      if (PointInVectorSide(next_start_pr, next_start_pl, ego_pos) > 0) {
        continue;
      }
      junction_group_idxs.emplace_back(grp_idx);
    }
  }

  if (junction_group_idxs.size() >= 2) {
    int second_grp_idx = junction_group_idxs[1];
    groups->erase(groups->begin() + second_grp_idx + 1, groups->end());
  }
}

}  // namespace gm
}  // namespace mf
}  // namespace mp
}  // namespace hozon
