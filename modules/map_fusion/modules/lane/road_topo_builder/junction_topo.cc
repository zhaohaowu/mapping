/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： junction_topo.h
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#include "modules/map_fusion/modules/lane/road_topo_builder/junction_topo.h"

#include <cmath>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "base/element_base.h"
#include "base/group.h"
#include "base/utils/log.h"
#include "common/calc_util.h"
#include "modules/lane/road_topo_builder/utils.h"
#include "pcl/impl/point_types.hpp"

namespace hozon {
namespace mp {
namespace mf {

void JunctionTopoConstruct::Init(const LaneFusionProcessOption& conf) {
  conf_ = conf;
  HLOG_INFO << "Junction construct init";
}

void JunctionTopoConstruct::Clear() {
  is_connect_ = false;
  next_lane_left_ = -1000;
  next_lane_right_ = -1000;
  next_satisefy_lane_seg_.clear();
  path_in_curr_pose_.clear();
}

void JunctionTopoConstruct::ConstructTopology(
    double stamp, std::vector<Group::Ptr>* groups,
    const std::shared_ptr<std::vector<KinePosePtr>>& path,
    const KinePosePtr& curr_pose) {
  UpdatePathInCurrPose(*path, *curr_pose);

  std::vector<Group::Ptr> group_virtual;
  int erase_grp_idx = -1;
  if (groups->front()->start_slice.po.x() > 0.0) {
    auto& next_group = groups->at(0);
    // 路口太大了 本车道没有
    Lane::Ptr best_next_lane = nullptr;
    auto next_grp_start_slice = next_group->start_slice;
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
        curr_group->lanes.size() < 1 || next_group->lanes.size() < 1) {
      continue;
    }

    // 判断curr_group的end slice的中心点和next_group的start
    // slice的中心点的距离
    double group_distance =
        (curr_group->end_slice.po - next_group->start_slice.po).norm();

    double currgrp_nearest_mindis_to_nextgrp = DBL_MAX;
    for (auto& cur_group_lane : curr_group->lanes) {
      double calcu_dis = (cur_group_lane->center_line_pts.back().pt -
                          next_group->start_slice.po)
                             .norm();
      if (calcu_dis < currgrp_nearest_mindis_to_nextgrp) {
        currgrp_nearest_mindis_to_nextgrp = calcu_dis;
      }
    }

    // > 10m 大路口
    if (group_distance > 10 && currgrp_nearest_mindis_to_nextgrp > 10) {
      TicToc tic3;
      float angle =
          abs((curr_group->end_slice.po - curr_group->end_slice.pl)
                  .normalized()
                  .dot((next_group->start_slice.po - next_group->start_slice.pl)
                           .normalized()));
      HLOG_INFO << "angle: " << angle;
      // 生成curr_group和next_group的中线的角度值。
      if (angle > 0.707) {
        // 直行场景
        auto next_grp_start_slice = next_group->start_slice;
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
        //           << "  next_group_name = " << next_group->str_id;
        Lane::Ptr ego_curr_lane = nullptr;
        FindNearestLaneToHisVehiclePosition(curr_group, &ego_curr_lane);
        ego_curr_lane_ = ego_curr_lane;
        // HLOG_DEBUG << "ego_curr_lane is " <<
        // ego_curr_lane->str_id_with_group;
        std::vector<Lane::Ptr> history_satisfy_lane_;
        FindSatisefyNextLane(next_group, dist_to_slice, &history_satisfy_lane_);
        if (ego_curr_lane != nullptr && !history_satisfy_lane_.empty() &&
            (dist_to_slice <= conf_.next_group_max_distance || is_connect_)) {
          is_connect_ = true;
          if (curr_group->end_slice.po.x() < -2.0) {
            along_path_dis_ =
                ego_curr_lane
                    ->center_line_pts[ego_curr_lane->center_line_pts.size() - 1]
                    .pt;
            // along_path_dis_ = curr_group->end_slice.po;
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
    }
    if (!group_virtual.empty() &&
        groups->at(i)->str_id + 'V' == group_virtual.back()->str_id) {
      groups->insert(groups->begin() + i + 1, group_virtual.back());
      group_virtual.pop_back();
    }
  }

  AvoidSplitMergeLane(groups);
}

void JunctionTopoConstruct::UpdatePathInCurrPose(
    const std::vector<KinePosePtr>& path, const KinePose& curr_pose) {
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

void JunctionTopoConstruct::FindBestNextLane(const Group::Ptr& next_group,
                                             float dist_to_slice,
                                             Lane::Ptr* best_next_lane) {
  Eigen::Vector2f thresh_v(
      std::cos(conf_.junction_heading_diff * M_PI / 180.0),
      std::sin(conf_.junction_heading_diff * M_PI / 180.0));
  Eigen::Vector2f n(1, 0);  // 车前向量
  float thresh_len = std::abs(thresh_v.transpose() * n);

  float max_len = 0;
  for (auto& next_lane : next_group->lanes) {
    if (next_lane->right_boundary->isego == IsEgo::Other_Road ||
        next_lane->left_boundary->isego == IsEgo::Other_Road) {
      HLOG_DEBUG << "Lane " << next_lane->left_boundary->id << " or "
                 << next_lane->right_boundary->id << " is other road";
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
    HLOG_DEBUG << "len = " << acos(len) * 180 / M_PI;
  }
  HLOG_DEBUG << "angle between veh & next lane: " << max_len;
  float min_offset = FLT_MAX;
  const float kOffsetThreshold = 3.5 * 0.7;  // 0.7个3.5米车道宽度
  float check_offset_thresh = -10000;
  if (conf_.junction_heading_diff * M_PI / 180.0 > 0.001) {
    check_offset_thresh =
        kOffsetThreshold / std::tan(conf_.junction_heading_diff * M_PI / 180.0);
  }
  // 如果此时距离足够近，并且通过角度未找到合适的next
  // lane，此时通过找最小的横向偏移来确定next lane
  if (*best_next_lane == nullptr && dist_to_slice <= check_offset_thresh) {
    for (auto& next_lane : next_group->lanes) {
      if (next_lane->right_boundary->isego == IsEgo::Other_Road ||
          next_lane->left_boundary->isego == IsEgo::Other_Road) {
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
    if (next_lane_left_ != -1000 || next_lane_right_ != -1000) {
      // 如果上一帧存在best_lane
      for (auto& lane_ : next_group->lanes) {
        if (lane_->right_boundary->isego == IsEgo::Other_Road ||
            lane_->left_boundary->isego == IsEgo::Other_Road) {
          continue;
        }
        if (TopoUtils::LineIdConsistant(lane_->left_boundary,
                                        next_lane_left_) ||
            TopoUtils::LineIdConsistant(lane_->right_boundary,
                                        next_lane_right_)) {
          *best_next_lane = lane_;
          return;
        }
      }
    }
    return;
  }
  // 2.第一帧连接的best_lane 存储一下左右边线并返回
  if (next_lane_left_ == -1000 || next_lane_right_ == -1000) {
    next_lane_left_ = (*best_next_lane)->left_boundary->id;
    next_lane_right_ = (*best_next_lane)->right_boundary->id;
    return;
  }
  // HLOG_ERROR << " best is equal to exist "
  //            << LineIdConsistant((*best_next_lane)->left_boundary,
  //                                next_lane_left_)
  //            << "  "
  //            << LineIdConsistant((*best_next_lane)->right_boundary,
  //                                next_lane_right_);
  // 3.跟上一帧连接的车道一致，直接返回
  if (TopoUtils::LineIdConsistant((*best_next_lane)->left_boundary,
                                  next_lane_left_) ||
      TopoUtils::LineIdConsistant((*best_next_lane)->right_boundary,
                                  next_lane_right_)) {
    return;
  }
  // 4.跟上一帧连接的车道不一致，看看是否需要更新best_lane
  int exist = -1;
  for (int i = 0; i < static_cast<int>(next_group->lanes.size()); ++i) {
    auto& lane_ = next_group->lanes[i];
    if (lane_->right_boundary->isego == IsEgo::Other_Road ||
        lane_->left_boundary->isego == IsEgo::Other_Road) {
      continue;
    }
    if (TopoUtils::LineIdConsistant(lane_->left_boundary, next_lane_left_) &&
        TopoUtils::LineIdConsistant(lane_->right_boundary, next_lane_right_)) {
      exist = i;
      history_best_lane_ = lane_;
      break;
    }
  }
  if (exist == -1) {
    next_lane_left_ = (*best_next_lane)->left_boundary->id;
    next_lane_right_ = (*best_next_lane)->right_boundary->id;
    return;
  }
}

void JunctionTopoConstruct::GenerateLane(const Lane::Ptr& lane_next,
                                         const Lane::Ptr& transition_lane) {
  LineSegment& left_bound = *transition_lane->left_boundary;
  left_bound.Copy(*lane_next->left_boundary, LineType::LaneType_DASHED,
                  Color::WHITE);
  Point left_pt_pred(VIRTUAL, 0.0, 0.0, 0.0);
  left_pt_pred.pt =
      lane_next->left_boundary->pts[0].pt - lane_next->center_line_pts[0].pt;

  LineSegment& right_bound = *transition_lane->right_boundary;
  right_bound.Copy(*lane_next->right_boundary, LineType::LaneType_DASHED,
                   Color::WHITE);
  Point right_pt_pred(VIRTUAL, 0.0, 0.0, 0.0);
  right_pt_pred.pt =
      lane_next->right_boundary->pts[0].pt - lane_next->center_line_pts[0].pt;

  std::vector<Point>& ctr_pts = transition_lane->center_line_pts;

  Point center_pt_pred(VIRTUAL, 0.0, 0.0, 0.0);

  Eigen::Vector3f vec_left =
      lane_next->left_boundary->pts[0].pt - left_pt_pred.pt;
  while (vec_left.norm() > 1.0) {
    left_bound.pts.emplace_back(left_pt_pred);
    float pre_x = left_pt_pred.pt.x() + vec_left.x() / vec_left.norm();
    float pre_y = left_pt_pred.pt.y() + vec_left.y() / vec_left.norm();
    left_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    vec_left = lane_next->left_boundary->pts[0].pt - left_pt_pred.pt;
  }

  Eigen::Vector3f vec_right =
      lane_next->right_boundary->pts[0].pt - right_pt_pred.pt;
  while (vec_right.norm() > 1.0) {
    right_bound.pts.emplace_back(right_pt_pred);
    float pre_x = right_pt_pred.pt.x() + vec_right.x() / vec_right.norm();
    float pre_y = right_pt_pred.pt.y() + vec_right.y() / vec_right.norm();
    right_pt_pred = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    vec_right = lane_next->right_boundary->pts[0].pt - right_pt_pred.pt;
  }

  Eigen::Vector3f vec_center =
      lane_next->center_line_pts[0].pt - center_pt_pred.pt;
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

void JunctionTopoConstruct::FindSatisefyNextLane(
    const Group::Ptr& next_group, float dist_to_slice,
    std::vector<Lane::Ptr>* satisfy_next_lane) {
  Eigen::Vector2f thresh_v(
      std::cos(conf_.junction_heading_diff * M_PI / 180.0),
      std::sin(conf_.junction_heading_diff * M_PI / 180.0));
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
    if (next_lane->right_boundary->isego == IsEgo::Other_Road ||
        next_lane->left_boundary->isego == IsEgo::Other_Road) {
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

  float check_offset_thresh = -10000;
  if (conf_.junction_heading_diff * M_PI / 180.0 > 0.001) {
    check_offset_thresh =
        kOffsetThreshold / std::tan(conf_.junction_heading_diff * M_PI / 180.0);
  }
  // 如果此时距离足够近，并且通过角度未找到合适的next
  // lane，此时通过找最小的横向偏移来确定next lane
  if ((*satisfy_next_lane).empty() && dist_to_slice <= check_offset_thresh) {
    for (auto& next_lane : next_group->lanes) {
      if (next_lane->right_boundary->isego == IsEgo::Other_Road ||
          next_lane->left_boundary->isego == IsEgo::Other_Road) {
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
    for (auto& his_lane : next_satisefy_lane_seg_) {
      size_t index = his_lane.find("_");
      int32_t left = atoi(his_lane.substr(0, index).c_str());
      int32_t right = atoi(his_lane.substr(index + 1).c_str());
      for (auto& lane_ : next_group->lanes) {
        if (lane_->right_boundary->isego == IsEgo::Other_Road ||
            lane_->left_boundary->isego == IsEgo::Other_Road) {
          continue;
        }
        if (TopoUtils::LineIdConsistant(lane_->left_boundary, left) &&
            TopoUtils::LineIdConsistant(lane_->right_boundary, right)) {
          satisfy_next_lane->emplace_back(lane_);
        }
      }
    }
    return;
  }
  // 2.第一帧连接的best_lane 存储一下左右边线并返回
  for (auto& sat_lane : *satisfy_next_lane) {
    if (sat_lane->right_boundary->isego == IsEgo::Other_Road ||
        sat_lane->left_boundary->isego == IsEgo::Other_Road) {
      continue;
    }
    int exist = 0;
    for (auto& his_lane : next_satisefy_lane_seg_) {
      size_t index = his_lane.find("_");
      int32_t left = atoi(his_lane.substr(0, index).c_str());
      int32_t right = atoi(his_lane.substr(index + 1).c_str());
      if (TopoUtils::LineIdConsistant(sat_lane->left_boundary, left) &&
          TopoUtils::LineIdConsistant(sat_lane->right_boundary, right)) {
        exist = 1;
        break;
      }
    }
    if (!exist) {
      next_satisefy_lane_seg_.insert(sat_lane->str_id);
    }
  }
  // history 的lane存储到satisfy里
  for (auto& his_lane : next_satisefy_lane_seg_) {
    int exist = 0;
    size_t index = his_lane.find("_");
    int32_t left = atoi(his_lane.substr(0, index).c_str());
    int32_t right = atoi(his_lane.substr(index + 1).c_str());
    for (auto& sat_lane : *satisfy_next_lane) {
      if (sat_lane->right_boundary->isego == IsEgo::Other_Road ||
          sat_lane->left_boundary->isego == IsEgo::Other_Road) {
        continue;
      }
      if (TopoUtils::LineIdConsistant(sat_lane->left_boundary, left) &&
          TopoUtils::LineIdConsistant(sat_lane->right_boundary, right)) {
        exist = 1;
        break;
      }
    }
    if (!exist) {
      for (auto& lane_ : next_group->lanes) {
        if (lane_->right_boundary->isego == IsEgo::Other_Road ||
            lane_->left_boundary->isego == IsEgo::Other_Road) {
          continue;
        }
        if (TopoUtils::LineIdConsistant(lane_->left_boundary, left) &&
            TopoUtils::LineIdConsistant(lane_->right_boundary, right)) {
          satisfy_next_lane->emplace_back(lane_);
        }
      }
    }
  }
}

void JunctionTopoConstruct::FindNearestLaneToHisVehiclePosition(
    const Group::Ptr& curr_group, Lane::Ptr* ego_curr_lane) {
  auto curr_grp_start_slice = curr_group->start_slice;
  auto curr_grp_end_slice = curr_group->end_slice;
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

void JunctionTopoConstruct::GenerateAllSatisfyTransitionLane(
    const Lane::Ptr& lane_in_curr, std::vector<Lane::Ptr>* virtual_lanes,
    const std::vector<Lane::Ptr>& history_satisfy_lane_,
    float dist_to_next_group_slice) {
  // HLOG_ERROR << "dist_to_next_group_slice = " << dist_to_next_group_slice;
  if (along_path_dis_.norm() > 2.0 &&
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
    auto& lane_in_next = history_satisfy_lane_[0];
    GenerateTransitionLaneToAfter(transition_lane, lane_in_next,
                                  transition_lane_after1);
    GenerateTransitionLaneToPo(transition_lane, lane_in_next,
                               transition_lane_after1, 0);
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
        GenerateTransitionLaneToPo(transition_lane, lane_next_history,
                                   transition_lane_after2, 1);
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
      GenerateTransitionLaneToPo(lane_in_curr, lane_next_history,
                                 transition_lane_after2, 2);
      if (transition_lane_after2->center_line_pts.size() > 1) {
        (*virtual_lanes).emplace_back(transition_lane_after2);
      }
    }
  }
}

void JunctionTopoConstruct::BuildVirtualGroup(
    const std::vector<Lane::Ptr>& virtual_lanes,
    std::vector<Group::Ptr>* group_virtual, double stamp) {
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

void JunctionTopoConstruct::GenerateTransitionLaneToBefore(
    const Lane::Ptr& lane_in_curr, const Lane::Ptr& transition_lane) {
  LineSegment& left_bound = *transition_lane->left_boundary;
  left_bound.Copy(*lane_in_curr->right_boundary, LineType::LaneType_DASHED,
                  Color::WHITE);
  size_t index_left = lane_in_curr->left_boundary->pts.size();
  Point left_pt_pred(VIRTUAL,
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
  LineSegment& right_bound = *transition_lane->right_boundary;
  right_bound.Copy(*lane_in_curr->right_boundary, LineType::LaneType_DASHED,
                   Color::WHITE);
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
  along_path_dis_.x() =
      along_path_dis_.x() -
      incross_before_virtual_lane_length_;  // conf_.incross_before_virtual_lane_length
  float len = along_path_dis_.norm();
  left_bound.pts.emplace_back(left_pt_pred);
  right_bound.pts.emplace_back(right_pt_pred);
  ctr_pts.emplace_back(center_pt_pred);
  while (i < len - 1.0) {
    i = i + 1.0;
    float minus_x = 1 / len * along_path_dis_.x();
    float minus_y = 1 / len * along_path_dis_.y();
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

void JunctionTopoConstruct::GenerateTransitionLaneToAfter(
    const Lane::Ptr& lane_in_curr, const Lane::Ptr& lane_in_next,
    const Lane::Ptr& transition_lane) {
  LineSegment& left_bound = *transition_lane->left_boundary;
  left_bound.Copy(*lane_in_curr->left_boundary, LineType::LaneType_DASHED,
                  Color::WHITE);
  size_t index_left = lane_in_curr->left_boundary->pts.size();
  Point left_pt_pred(VIRTUAL,
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
  LineSegment& right_bound = *transition_lane->right_boundary;
  right_bound.Copy(*lane_in_curr->right_boundary, LineType::LaneType_DASHED,
                   Color::WHITE);
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
  Eigen::Vector3f param_left =
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
  Eigen::Vector3f param_right =
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
  Eigen::Vector3f param_center =
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

void JunctionTopoConstruct::GenerateTransitionLaneToPo(
    const Lane::Ptr& lane_in_curr, const Lane::Ptr& lane_in_next,
    const Lane::Ptr& transition_lane, int type) {
  switch (type) {
    case 0: {
      transition_lane->str_id = lane_in_curr->str_id;
      transition_lane->lanepos_id = lane_in_curr->lanepos_id;
      size_t index = lane_in_curr->str_id_with_group.find(":");
      transition_lane->str_id_with_group =
          lane_in_curr->str_id_with_group.substr(0, index) +
          "V:" + transition_lane->str_id;
      break;
    }
    case 1: {
      transition_lane->str_id = lane_in_next->str_id;
      transition_lane->lanepos_id = lane_in_next->lanepos_id;
      size_t index = lane_in_curr->str_id_with_group.find("V");
      transition_lane->str_id_with_group =
          lane_in_curr->str_id_with_group.substr(0, index) +
          "2VV:" + transition_lane->str_id;
      break;
    }
    case 2: {
      transition_lane->str_id = lane_in_next->str_id;
      transition_lane->lanepos_id = lane_in_next->lanepos_id;
      size_t index = lane_in_curr->str_id_with_group.find(":");
      transition_lane->str_id_with_group =
          lane_in_curr->str_id_with_group.substr(0, index) +
          "V:" + transition_lane->str_id;
      break;
    }
    default:
      break;
  }

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

void JunctionTopoConstruct::GenerateTransitionLaneGeo(
    const Lane::Ptr& lane_in_curr, const Lane::Ptr& lane_in_next,
    const Lane::Ptr& transition_lane) {
  LineSegment& left_bound = *transition_lane->left_boundary;
  left_bound.Copy(*lane_in_curr->left_boundary, LineType::LaneType_DASHED,
                  Color::WHITE);
  size_t index_left = lane_in_curr->left_boundary->pts.size();
  if (index_left < 1) {
    return;
  }
  Point left_pt_pred(VIRTUAL,
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                     lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
  LineSegment& right_bound = *transition_lane->right_boundary;
  right_bound.Copy(*lane_in_curr->right_boundary, LineType::LaneType_DASHED,
                   Color::WHITE);
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

  std::vector<Point> tmp_pts;
  tmp_pts.emplace_back(lane_in_curr->left_boundary->pts[index_left - 1]);
  tmp_pts.emplace_back(lane_in_next->left_boundary->pts[0]);
  std::vector<double> param_left = math::FitLaneline(tmp_pts);
  while (left_pt_pred.pt.x() < lane_in_next->left_boundary->pts[0].pt.x()) {
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
  param_left = math::FitLaneline(tmp_pts);
  while (right_pt_pred.pt.x() < lane_in_next->right_boundary->pts[0].pt.x()) {
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
  param_left = math::FitLaneline(tmp_pts);
  while (center_pt_pred.pt.x() < lane_in_next->center_line_pts[0].pt.x()) {
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

void JunctionTopoConstruct::FindGroupNextLane(const Group::Ptr& curr_group,
                                              const Group::Ptr& next_group) {
  // 如果next_group的lanes比curr_group的lanes多或者相等的话，currgroup->lane有一个后继
  // 如果next_group的lanes比curr_group的lanes少的话，next->lane有一个前驱
  // 如果左右车道线有一根的trackid相等的话，有连接关系

  // 用hash先存下关联关系
  std::map<int, std::vector<int>>
      curr_group_next_lane;  // 当前车道groupindex,后继在下个group的index
  std::map<int, std::vector<int>> next_group_prev_lane;
  if (curr_group->lanes.size() >= next_group->lanes.size()) {
    HLOG_DEBUG << "CUR >= NEXT";
    for (int i = 0; i < static_cast<int>(curr_group->lanes.size()); ++i) {
      auto& lane_in_curr = curr_group->lanes[i];
      if (lane_in_curr->next_lane_str_id_with_group.empty()) {
        int trackid_same_find = 0;
        for (int j = 0; j < static_cast<int>(next_group->lanes.size()); ++j) {
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
          for (int j = 0; j < static_cast<int>(next_group->lanes.size()); ++j) {
            has_next_lane = TopoUtils::NeedToConnect(curr_group, next_group, i,
                                                     j, &curr_group_next_lane,
                                                     &next_group_prev_lane) ||
                            has_next_lane;
          }
          if (!has_next_lane) {
            // 目前只考虑最左或最右线关联问题
            // 这里仅增加收缩的lane，目的是为了排除正常宽度的lane也被认为是merge的lane
            double curr_len = TopoUtils::CalcLaneLength(lane_in_curr);
            bool shrink =
                TopoUtils::IsShrinkLane(lane_in_curr, conf_.min_lane_width);
            const float dis_thresh = 4.5;
            if (i == 0 && curr_len > kMergeLengthThreshold && shrink &&
                TopoUtils::IsAccessLane(lane_in_curr, next_group->lanes[0]) &&
                TopoUtils::LaneDist(lane_in_curr, next_group->lanes[0]) <
                    dis_thresh &&
                TopoUtils::CalcLaneLength(next_group->lanes[0]) >
                    kMergeLengthThreshold) {
              curr_group_next_lane[i].emplace_back(0);
              next_group_prev_lane[0].emplace_back(i);
            } else if (i == static_cast<int>(curr_group->lanes.size()) - 1 &&
                       curr_len > kMergeLengthThreshold && shrink &&
                       TopoUtils::IsAccessLane(lane_in_curr,
                                               next_group->lanes.back()) &&
                       TopoUtils::LaneDist(lane_in_curr,
                                           next_group->lanes.back()) <
                           dis_thresh &&
                       TopoUtils::CalcLaneLength(next_group->lanes.back()) >
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
              float angle1 = TopoUtils::Calculate2CenterlineAngle(
                  lane_in_next, lane_in_curr, sizet);
              float angle2 = TopoUtils::Calculate2CenterlineAngle(
                  lane_best, lane_in_curr, sizet);
              float dis1 = TopoUtils::CalculatePoint2CenterLine(lane_in_next,
                                                                lane_in_curr);
              float dis2 =
                  TopoUtils::CalculatePoint2CenterLine(lane_best, lane_in_curr);
              if (angle2 > angle1 + 1 * 180 / M_PI && dis2 > dis1 + 0.2) {
                best = next_lane_idx;
              }
            } else {
              float dis_pt =
                  TopoUtils::CalculateDistPt(lane_in_next, lane_in_curr, sizet);
              float dis_pt2 =
                  TopoUtils::CalculateDistPt(lane_best, lane_in_curr, sizet);
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
    for (int i = 0; i < static_cast<int>(next_group->lanes.size()); ++i) {
      auto& lane_in_next = next_group->lanes[i];
      if (lane_in_next->prev_lane_str_id_with_group.empty()) {
        int trackid_same_find = 0;
        for (int j = 0; j < static_cast<int>(curr_group->lanes.size()); ++j) {
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
          for (int j = 0; j < static_cast<int>(curr_group->lanes.size()); ++j) {
            has_prev_lane = TopoUtils::NeedToConnect(curr_group, next_group, j,
                                                     i, &curr_group_next_lane,
                                                     &next_group_prev_lane) ||
                            has_prev_lane;
          }
          double next_len = TopoUtils::CalcLaneLength(lane_in_next);
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
                  TopoUtils::IsAccessLane(lane_in_curr, lane_in_next)) {
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
                  TopoUtils::IsAccessLane(lane_in_curr, lane_in_next)) {
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
              float angle1 = TopoUtils::Calculate2CenterlineAngle(
                  lane_in_next, lane_in_curr, sizet1);
              float angle2 = TopoUtils::Calculate2CenterlineAngle(
                  lane_in_next, lane_best, sizet2);
              float dis1 = TopoUtils::CalculatePoint2CenterLine(lane_in_next,
                                                                lane_in_curr);
              float dis2 =
                  TopoUtils::CalculatePoint2CenterLine(lane_in_next, lane_best);
              if (angle2 > angle1 + 1 * 180 / M_PI && dis2 > dis1 + 0.2) {
                best = prev_lane_idx;
              }
            } else {
              float dis_pt = TopoUtils::CalculateDistPt(lane_in_next,
                                                        lane_in_curr, sizet1);
              float dis_pt2 =
                  TopoUtils::CalculateDistPt(lane_in_next, lane_best, sizet2);
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
  //                                 lane_in_next->center_line_pts[0].pt.x()))
  //                                 /
  //                       sqrt(pow(lane_in_curr->center_line_param[0], 2) +
  //                             pow(lane_in_curr->center_line_param[1], 2));
  //           float angle =
  //               atan((lane_in_next->center_line_pts[0].pt.y() -
  //                     lane_in_curr->center_line_pts[sizet - 1].pt.y()) /
  //                     (lane_in_next->center_line_pts[0].pt.x() -
  //                     lane_in_curr->center_line_pts[sizet - 1].pt.x())) -
  //               atan(lane_in_curr->center_line_param[1]);
  //           // HLOG_ERROR << "angle = "<<angle*180/pi_;
  //           if ((dis < dis_thresh && abs(angle) * 180 / pi_ <
  //           angel_thresh))
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

void JunctionTopoConstruct::NextGroupLaneConnect(
    const Group::Ptr& curr_group, const Group::Ptr& next_group,
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
        // tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num -
        // 5].pt); tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num
        // - 4].pt);
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
          // 2].pt);
          // tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num
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

void JunctionTopoConstruct::BuildVirtualGroup2(
    const std::vector<Lane::Ptr>& virtual_lanes,
    std::vector<Group::Ptr>* group_virtual, double stamp) {
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

void JunctionTopoConstruct::AvoidSplitMergeLane(
    std::vector<Group::Ptr>* groups) {
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

void JunctionTopoConstruct::ErasePrevRelation(const Group::Ptr& curr_group,
                                              int curr_erase_index,
                                              const Group::Ptr& next_group,
                                              int next_index) {
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

}  // namespace mf
}  // namespace mp
}  // namespace hozon
