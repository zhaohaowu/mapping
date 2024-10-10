/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_topo.cc
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#include "modules/map_fusion/modules/lane/road_topo_builder/lane_topo.h"

#include <chrono>
#include <unordered_set>
#include <vector>

#include "base/utils/log.h"
#include "common/calc_util.h"
#include "modules/lane/road_topo_builder/utils.h"

namespace hozon {
namespace mp {
namespace mf {

void LaneTopoConstruct::Init(const LaneFusionProcessOption& conf) {
  conf_ = conf;

  HLOG_INFO << "Lane construct init";
}

void LaneTopoConstruct::Clear() {}

void LaneTopoConstruct::ConstructTopology(std::vector<Group::Ptr>* groups) {
  SmoothCenterline(groups);
  LeftRightTopoProcess(groups);

  // 从前往后
  for (int i = 0; i < static_cast<int>(groups->size()) - 1; ++i) {
    auto& curr_group = groups->at(i);
    auto& next_group = groups->at(i + 1);
    if (curr_group->lanes.empty() || next_group->lanes.empty()) {
      continue;
    }
    // if (curr_group->group_segments.empty() ||
    //     next_group->group_segments.empty()) {
    //   continue;
    // }
    bool is_any_next_lane_exist =
        false;  // 前后group是否存在某个车道根据trackid关联
    bool is_all_next_lane_exist =
        true;  // currgroup的车道是否全部都有nextgroup关联
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

    if (group_distance > 10 && currgrp_nearest_mindis_to_nextgrp > 10) {
      // 路口场景不补全
      continue;
    }

    ForwardTopoProcess(curr_group, next_group, &is_any_next_lane_exist,
                       &is_all_next_lane_exist);

    if (is_any_next_lane_exist) {
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

      // if (max_length_lane <= 10.0 && is_all_next_lane_exist == 0) {
      //   BuildVirtualLaneAfter(curr_group, next_group);
      //   // groups->erase(groups->begin() + i + 1);
      //   // i--;
      // }
      double group_distance =
          (curr_group->end_slice.po - next_group->start_slice.po).norm();
      if (!is_all_next_lane_exist && group_distance < 10.0) {
        UpdateLaneBoundaryId(curr_group);
        // 后侧车道线补齐
        // BuildVirtualLaneAfter(curr_group, next_group);
        // EraseIntersectLane(curr_group, next_group);
        // BuildVirtualLaneLeftRight(curr_group, next_group);
      }
      DelLaneNextStrIdInGroup(curr_group);
    } else {
      if (!ContainEgoLane(groups, i + 1) &&
          curr_group->end_slice.po.x() > 2.0) {
        groups->erase(groups->begin() + i + 1, groups->end());
        break;
      }
    }
  }

  // 从后往前
  for (int i = static_cast<int>(groups->size()) - 1; i > 0; --i) {
    auto& curr_group = groups->at(i - 1);
    auto& next_group = groups->at(i);
    // if (curr_group->group_segments.empty() ||
    //     next_group->group_segments.empty()) {
    //   continue;
    // }
    bool is_any_next_lane_exist =
        false;  // 前后group是否存在某个车道根据trackid关联
    bool is_all_next_lane_exist =
        true;  // currgroup的车道是否全部都有nextgroup关联

    BackwardTopoProcess(curr_group, next_group, &is_any_next_lane_exist,
                        &is_all_next_lane_exist);

    if (is_any_next_lane_exist || ContainEgoLane(groups, i)) {
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
          (curr_group->end_slice.po - next_group->start_slice.po).norm();
      // HLOG_DEBUG << "max_length_lane = " << max_length_lane;
      if (((IsGroupsNoEgoLane(groups, i - 1) && max_length_lane <= 50.0) ||
           max_length_lane <= 15.0) &&
          !is_all_next_lane_exist && group_distance < 10.0) {
        // HLOG_DEBUG << " is connect";
        // BuildVirtualLaneBefore(curr_group, next_group);
        // groups->erase(groups->begin() + i - 1);
      }
      DelLanePrevStrIdInGroup(next_group);
    }
  }

  // 设置lane属性(is_ego、is_tran)
  SetLaneStatus(groups);
}

bool LaneTopoConstruct::LeftRightTopoProcess(std::vector<Group::Ptr>* groups) {
  for (auto& grp : *groups) {
    int Lane_size = static_cast<int>(grp->lanes.size());
    HLOG_INFO << "group lane size: " << Lane_size;
    if (Lane_size > 1) {
      for (int i = 0; i < Lane_size; ++i) {
        auto& curr = grp->lanes.at(i);
        HLOG_INFO << "cur lane: " << curr->str_id_with_group;
        if (i - 1 >= 0) {
          const auto& left = grp->lanes.at(i - 1);
          curr->left_lane_str_id_with_group.emplace_back(
              left->str_id_with_group);
          HLOG_INFO << "l lane: " << left->str_id_with_group;
        }

        if (i + 1 < Lane_size) {
          const auto& right = grp->lanes.at(i + 1);
          curr->right_lane_str_id_with_group.emplace_back(
              right->str_id_with_group);
          HLOG_INFO << "r lane: " << right->str_id_with_group;
        }
      }
    }
  }
  return true;
}

void LaneTopoConstruct::ForwardTopoProcess(const Group::Ptr& curr_group,
                                           const Group::Ptr& next_group,
                                           bool* is_any_next_lane_exist,
                                           bool* is_all_next_lane_exist) {
  //! TBD:
  //! 把分合流场景判断进来，使得分合流的场景不向前延伸。FindGroupNextLane不用再判断一遍前后继了直接沿用
  for (auto& lane_in_curr : curr_group->lanes) {
    bool next_lane_exist = false;  // currlane是否有后继
    // HLOG_ERROR << "curr+lane = " << lane_in_curr->str_id_with_group;
    for (auto& lane_in_next : next_group->lanes) {
      if (lane_in_curr->str_id == lane_in_next->str_id ||
          lane_in_curr->left_boundary->id == lane_in_next->left_boundary->id ||
          lane_in_curr->right_boundary->id ==
              lane_in_next->right_boundary->id) {
        // 目前这个是否有next是为了记录是否需要补道，如果有后继就不用补道
        lane_in_curr->next_lane_str_id_with_group.emplace_back(
            lane_in_next->str_id_with_group);
        // HLOG_ERROR << "the next lane is " <<
        // lane_in_next->str_id_with_group;
        lane_in_curr->left_boundary->id_next = lane_in_next->left_boundary->id;
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
        *is_any_next_lane_exist = true;
        next_lane_exist = true;
        if (lane_in_next->center_line_param.empty()) {
          lane_in_next->center_line_param = lane_in_curr->center_line_param;
        }
        if (lane_in_next->center_line_param_front.empty()) {
          lane_in_next->center_line_param_front =
              lane_in_next->center_line_param;
        }
        break;
      }
      if (TopoUtils::NeedToConnect(lane_in_curr, lane_in_next)) {
        lane_in_curr->next_lane_str_id_with_group.emplace_back(
            lane_in_next->str_id_with_group);
        lane_in_curr->left_boundary->id_next = lane_in_next->left_boundary->id;
        lane_in_curr->right_boundary->id_next =
            lane_in_next->right_boundary->id;
        // HLOG_ERROR << "the next lane is " <<
        // lane_in_next->str_id_with_group;
        *is_any_next_lane_exist = true;
        next_lane_exist = true;
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
    if (!next_lane_exist) {
      double curr_len = TopoUtils::CalcLaneLength(lane_in_curr);
      bool shrink = TopoUtils::IsShrinkLane(lane_in_curr, conf_.min_lane_width);
      const float dis_thresh = 4.5;
      if (lane_in_curr->str_id_with_group ==
              curr_group->lanes[0]->str_id_with_group &&
          curr_len > kMergeLengthThreshold && shrink &&
          TopoUtils::IsAccessLane(lane_in_curr, next_group->lanes[0]) &&
          TopoUtils::LaneDist(lane_in_curr, next_group->lanes[0]) <
              dis_thresh &&
          TopoUtils::CalcLaneLength(next_group->lanes[0]) >
              kMergeLengthThreshold) {
        lane_in_curr->next_lane_str_id_with_group.emplace_back(
            next_group->lanes[0]->str_id_with_group);
        next_lane_exist = true;
        // HLOG_ERROR << "the next lane is "
        //            << next_group->lanes[0]->str_id_with_group;
      } else if (lane_in_curr->str_id_with_group ==
                     curr_group->lanes.back()->str_id_with_group &&
                 curr_len > kMergeLengthThreshold && shrink &&
                 TopoUtils::IsAccessLane(lane_in_curr,
                                         next_group->lanes.back()) &&
                 TopoUtils::LaneDist(lane_in_curr, next_group->lanes.back()) <
                     dis_thresh &&
                 TopoUtils::CalcLaneLength(next_group->lanes.back()) >
                     kMergeLengthThreshold) {
        lane_in_curr->next_lane_str_id_with_group.emplace_back(
            next_group->lanes.back()->str_id_with_group);
        next_lane_exist = true;
        // HLOG_ERROR << "the next lane is "
        //            << next_group->lanes.back()->str_id_with_group;
      }
    }
    if (!next_lane_exist) {
      *is_all_next_lane_exist = false;
    }
  }
}

void LaneTopoConstruct::BackwardTopoProcess(const Group::Ptr& curr_group,
                                            const Group::Ptr& next_group,
                                            bool* is_any_next_lane_exist,
                                            bool* is_all_next_lane_exist) {
  for (auto& lane_in_next : next_group->lanes) {
    bool next_lane_exist = false;  // next_group是否有前驱（0:没有,1:有）
    // HLOG_ERROR << "next+lane = " << lane_in_next->str_id_with_group;
    for (auto& lane_in_curr : curr_group->lanes) {
      if (lane_in_curr->str_id == lane_in_next->str_id ||
          lane_in_curr->left_boundary->id == lane_in_next->left_boundary->id ||
          lane_in_curr->right_boundary->id ==
              lane_in_next->right_boundary->id) {
        *is_any_next_lane_exist = true;
        next_lane_exist = true;
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
      if (TopoUtils::NeedToConnect(lane_in_curr, lane_in_next)) {
        *is_any_next_lane_exist = true;
        next_lane_exist = true;
        lane_in_next->prev_lane_str_id_with_group.emplace_back(
            lane_in_curr->str_id_with_group);
        // HLOG_ERROR << "the prev lane is " <<
        // lane_in_curr->str_id_with_group; if
        // (lane_in_curr->center_line_param.empty()) {
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
    if (!next_lane_exist) {
      *is_all_next_lane_exist = false;
    }
  }
}

void LaneTopoConstruct::SmoothCenterline(std::vector<Group::Ptr>* groups) {
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
        std::vector<Point> res = TopoUtils::SigmoidFunc(centerpt, 5.0);
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
        std::vector<Point> first_ctl =
            TopoUtils::SigmoidFunc(next_centerpt, 5.0);
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
        std::vector<Point> second_ctl =
            TopoUtils::SigmoidFunc(next_centerpt, 5.0);
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

void LaneTopoConstruct::SetLaneStatus(std::vector<Group::Ptr>* groups) {
  // 添加主路和是否当前朝向属性
  for (auto& group : *groups) {
    int flag = 0;
    for (auto& lane : group->lanes) {
      if (lane->left_boundary->isego == IsEgo::Ego_Road) {
        lane->is_ego = 1;
      }
      if (flag == 0) {
        if (lane->left_boundary->color == Color::YELLOW) {
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
}

void LaneTopoConstruct::UpdateLaneBoundaryId(const Group::Ptr& curr_group) {
  std::unordered_map<Id, Id> line_next;
  for (auto& lane : curr_group->lanes) {
    if (lane->left_boundary->id_next != -1000) {
      line_next[lane->left_boundary->id] = lane->left_boundary->id_next;
    }
    if (lane->right_boundary->id_next != -1000) {
      line_next[lane->right_boundary->id] = lane->right_boundary->id_next;
    }
  }
  for (auto& lane : curr_group->lanes) {
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

bool LaneTopoConstruct::ContainEgoLane(std::vector<Group::Ptr>* groups,
                                       int next_grp_index) {
  // 青鸾号:1250597
  for (int grp_idx = next_grp_index; grp_idx < static_cast<int>(groups->size());
       ++grp_idx) {
    auto& next_groups = groups->at(grp_idx);
    for (const auto& next_grp_lane : next_groups->lanes) {
      if (next_grp_lane->lanepos_id == "-1_1") {
        return true;
      }
    }
  }
  return false;
}

bool LaneTopoConstruct::IsGroupsNoEgoLane(std::vector<Group::Ptr>* groups,
                                          int curr_group_index) {
  if (curr_group_index >= static_cast<int>(groups->size())) {
    return false;
  }
  if (groups->at(curr_group_index)->end_slice.po.x() < 2.0) {
    // 不关心自车后方group
    return false;
  }
  for (int i = curr_group_index; i >= 0; i--) {
    auto curr_group = groups->at(curr_group_index);
    if (curr_group->end_slice.po.x() < -2.0) {
      break;
    }
    if (!IsGroupNoEgoLane(curr_group)) {
      return false;
    }
    if (curr_group->start_slice.po.x() > 0.0) {
      // 到自车group为止都没有egolane
      return true;
    }
  }

  return true;
}

bool LaneTopoConstruct::IsGroupNoEgoLane(const Group::Ptr& group) {
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

void LaneTopoConstruct::DelLaneNextStrIdInGroup(const Group::Ptr& curr_group) {
  for (auto& lane_in_curr : curr_group->lanes) {
    lane_in_curr->next_lane_str_id_with_group.clear();
  }
}

void LaneTopoConstruct::DelLanePrevStrIdInGroup(const Group::Ptr& curr_group) {
  for (auto& lane_in_curr : curr_group->lanes) {
    lane_in_curr->prev_lane_str_id_with_group.clear();
  }
}

void LaneTopoConstruct::BuildVirtualLaneAfter(const Group::Ptr& curr_group,
                                              const Group::Ptr& next_group) {
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
      HLOG_ERROR << "lane_in_curr->str_id_with_group = "
                 << lane_in_curr->str_id_with_group
                 << "  lane_in_curr->left_boundary->id = "
                 << lane_in_curr->left_boundary->id
                 << "  lane_in_curr->left_boundary->id_next = "
                 << lane_in_curr->left_boundary->id_next
                 << "  lane_in_curr->right_boundary->id = "
                 << lane_in_curr->right_boundary->id
                 << "  lane_in_curr->right_boundary->id_next = "
                 << lane_in_curr->right_boundary->id_next;
      // 判断现有的line是否已经存在
      bool left_bound_exist = false;
      bool right_bound_exist = false;
      for (auto& lane_in_next : next_group->lanes) {
        if (lane_in_next->left_boundary->id ==
                lane_in_curr->left_boundary->id_next &&
            !lane_in_next->left_boundary->pts.empty() &&
            lane_in_next->left_boundary->pts[0].type == PointType::RAW &&
            !left_bound_exist) {
          // left_bound = *(lane_in_next->left_boundary);
          // FillLineSegment(lane_in_next->left_boundary, &left_bound);
          for (auto& line_pt : lane_in_next->left_boundary->pts) {
            if (line_pt.type != PointType::RAW) {
              break;
            }
            Point pt_pre(PointType::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = true;
        } else if (lane_in_next->right_boundary->id ==
                       lane_in_curr->left_boundary->id_next &&
                   !lane_in_next->right_boundary->pts.empty() &&
                   lane_in_next->right_boundary->pts[0].type ==
                       PointType::RAW &&
                   !left_bound_exist) {
          // left_bound = *(lane_in_next->right_boundary);
          // FillLineSegment(lane_in_next->right_boundary, &left_bound);
          for (auto& line_pt : lane_in_next->right_boundary->pts) {
            if (line_pt.type != PointType::RAW) {
              break;
            }
            Point pt_pre(PointType::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = true;
        } else if (lane_in_next->left_boundary->id ==
                       lane_in_curr->right_boundary->id_next &&
                   !lane_in_next->left_boundary->pts.empty() &&
                   lane_in_next->left_boundary->pts[0].type == PointType::RAW &&
                   !right_bound_exist) {
          // right_bound = *(lane_in_next->left_boundary);
          // FillLineSegment(lane_in_next->left_boundary, &right_bound);
          for (auto& line_pt : lane_in_next->left_boundary->pts) {
            if (line_pt.type != PointType::RAW) {
              break;
            }
            Point pt_pre(PointType::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = true;
        } else if (lane_in_next->right_boundary->id ==
                       lane_in_curr->right_boundary->id_next &&
                   !lane_in_next->right_boundary->pts.empty() &&
                   lane_in_next->right_boundary->pts[0].type ==
                       PointType::RAW &&
                   !right_bound_exist) {
          // right_bound = *(lane_in_next->right_boundary);
          // FillLineSegment(lane_in_next->right_boundary, &right_bound);
          for (auto& line_pt : lane_in_next->right_boundary->pts) {
            if (line_pt.type != PointType::RAW) {
              break;
            }
            Point pt_pre(PointType::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = true;
        }
      }
      // 青鸾号:1308782
      // 防止错车道的前后继关联
      if (left_bound_exist &&
          TopoUtils::PointToLineDis(
              left_bound, lane_in_curr->left_boundary->pts.back().pt.x(),
              lane_in_curr->left_boundary->pts.back().pt.y()) >
              conf_.min_lane_width + 0.5) {
        left_bound.pts.clear();
        left_bound_exist = false;
      }
      if (right_bound_exist &&
          TopoUtils::PointToLineDis(
              right_bound, lane_in_curr->right_boundary->pts.back().pt.x(),
              lane_in_curr->right_boundary->pts.back().pt.y()) >
              conf_.min_lane_width + 0.5) {
        right_bound.pts.clear();
        right_bound_exist = false;
      }

      int left_ego_line_id = LOCATION_MANAGER->GetEgoLane().left_id;
      int right_ego_line_id = LOCATION_MANAGER->GetEgoLane().right_id;
      if (left_bound_exist && right_bound_exist &&
          ((!DistanceInferenceLane(left_bound, right_bound)) &&
           (lane_in_curr->left_boundary->id != left_ego_line_id ||
            lane_in_curr->right_boundary->id != right_ego_line_id))) {
        HLOG_WARN
            << "left bound exist and right bound exist but distance not valid!";
        return;
      } else if (left_bound_exist && !right_bound_exist) {
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
        while (
            right_pt_pred.pt.x() < next_group->end_slice.po.x() &&
            left_index < left_bound_size &&
            ((dx > 0 && dx * dx + dy * dy > 0.4) || dx * dx + dy * dy < 0.4)) {
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
        if (!TopoUtils::IsBoundaryValid(right_bound)) {
          HLOG_WARN << "boundary not valid";
          continue;
        }
        if (right_bound.pts.empty()) {
          HLOG_WARN << "right bound empty!";
          return;
        }
      } else if (!left_bound_exist && right_bound_exist) {
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
        while (
            left_pt_pred.pt.x() < next_group->end_slice.po.x() &&
            right_index < right_bound_size &&
            ((dx > 0 && dx * dx + dy * dy > 0.4) || dx * dx + dy * dy < 0.4)) {
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
        if (!TopoUtils::IsBoundaryValid(left_bound)) {
          HLOG_WARN << "boundary not valid";
          continue;
        }
        if (left_bound.pts.empty()) {
          HLOG_WARN << "left bound empty!";
          return;
        }
      } else if (!left_bound_exist && !right_bound_exist) {
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
        while (left_pt_pred.pt.x() < next_group->end_slice.po.x()) {
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
        while (right_pt_pred.pt.x() < next_group->end_slice.po.x()) {
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
      TopoUtils::FitCenterLine(lane_ptr);
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

void LaneTopoConstruct::BuildVirtualLaneBefore(const Group::Ptr& curr_group,
                                               const Group::Ptr& next_group) {
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
      bool left_bound_exist = false;
      bool right_bound_exist = false;
      for (auto& lane_in_curr : curr_group->lanes) {
        if (lane_in_curr->left_boundary->id ==
            lane_in_next->left_boundary->id) {
          // left_bound = *(lane_in_curr->left_boundary);
          for (auto& line_pt : lane_in_curr->left_boundary->pts) {
            Point pt_pre(PointType::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = true;
        } else if (lane_in_curr->left_boundary->id ==
                   lane_in_next->right_boundary->id) {
          // right_bound = *(lane_in_curr->left_boundary);
          for (auto& line_pt : lane_in_curr->left_boundary->pts) {
            Point pt_pre(PointType::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = true;
        } else if (lane_in_curr->right_boundary->id ==
                   lane_in_next->left_boundary->id) {
          // left_bound = *(lane_in_curr->right_boundary);
          for (auto& line_pt : lane_in_curr->right_boundary->pts) {
            Point pt_pre(PointType::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = true;
        } else if (lane_in_curr->right_boundary->id ==
                   lane_in_next->right_boundary->id) {
          // right_bound = *(lane_in_curr->right_boundary);
          for (auto& line_pt : lane_in_curr->right_boundary->pts) {
            Point pt_pre(PointType::RAW, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = true;
        }
      }
      if (left_bound_exist && !right_bound_exist) {
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
        while (
            right_pt_pred.pt.x() > curr_group->end_slice.po.x() &&
            left_index > 0 &&
            ((dx > 0 && dx * dx + dy * dy > 0.4) || dx * dx + dy * dy < 0.4)) {
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
      } else if (!left_bound_exist && right_bound_exist) {
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
        while (
            left_pt_pred.pt.x() > curr_group->end_slice.po.x() &&
            right_index > 0 &&
            ((dx > 0 && dx * dx + dy * dy > 0.4) || dx * dx + dy * dy < 0.4)) {
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
      } else if (!left_bound_exist && !right_bound_exist) {
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
        while (left_pt_pred.pt.x() > curr_group->end_slice.po.x()) {
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
        while (right_pt_pred.pt.x() > curr_group->end_slice.po.x()) {
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
      TopoUtils::FitCenterLine(lane_ptr);
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

void LaneTopoConstruct::BuildVirtualLaneLeftRight(
    const Group::Ptr& curr_group, const Group::Ptr& next_group) {
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
  int curr_group_lane_size = static_cast<int>(curr_group->lanes.size());
  for (int i = 0; i < curr_group_lane_size; ++i) {
    curr_lane_index[curr_group->lanes[i]->str_id_with_group] = i;
  }
  for (int curr_index = 0; curr_index < curr_group_lane_size; ++curr_index) {
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
      lane_next->right_lane_str_id_with_group.emplace_back(right_lane_next_id);
    }
  }

  // 存在少变多场景，导致误删左右邻。青鸾问题:1232877
  if (next_group->lanes.size() > 1) {
    std::vector<int> next_group_lanes;  // lane对应group的下标
    for (auto& lane : next_group->lanes) {
      // HLOG_DEBUG << "lane->str_id is " << lane->str_id
      //           << "  next_group->str_id is " << next_group->str_id;
      if (next_group->str_id.find(lane->str_id) < next_group->str_id.length()) {
        // int index = next_group->str_id.find(lane->str_id);
        // HLOG_DEBUG << "lane->str_id is " << lane->str_id
        //           << "  next_group->str_id is " << next_group->str_id
        //           << "  index = " << index;
        next_group_lanes.emplace_back(next_lane_index[lane->str_id_with_group]);
      }
    }
    if (next_group_lanes.empty()) {
      return;
    }

    int next_group_lanes_size = static_cast<int>(next_group_lanes.size());
    for (int i = 0; i < next_group_lanes_size; i++) {
      int cur_lane_index = next_group_lanes[i];
      if (cur_lane_index >= next_group_lanes_size) {
        continue;
      }
      for (int j = i + 1; j < next_group_lanes_size; ++j) {
        int right_lane_inex = next_group_lanes[j];
        if (right_lane_inex >= next_group_lanes_size) {
          continue;
        }
        bool exist =
            TopoUtils::IsRightLane(next_group, cur_lane_index, right_lane_inex);

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
    for (int i = next_group_lanes.back() + 1; i < next_group_lanes_size; ++i) {
      int index =
          next_group_lanes_size;  // 补全的lane相对与实际lane所在的index，物理意义：实际从下标index开始的lane的左边
      for (int j = 0; j < next_group_lanes_size; ++j) {
        int cur_lane_index = next_group_lanes[j];
        bool exist_left = TopoUtils::IsLeftLane(next_group, cur_lane_index, i);

        if (exist_left) {
          index = j;
          break;
        }
      }
      // 将这根虚拟道next_group->lanes[i]，添加到其左边的右邻
      for (int j = 0; j < index; ++j) {
        int cur_lane_index = next_group_lanes[j];
        bool exist_right = TopoUtils::IsRightLane(next_group, cur_lane_index,
                                                  i);  // 是否已经存在右邻
        if (!exist_right) {
          next_group->lanes[cur_lane_index]
              ->right_lane_str_id_with_group.emplace_back(
                  next_group->lanes[i]->str_id_with_group);
          next_group->lanes[i]->left_lane_str_id_with_group.emplace_back(
              next_group->lanes[cur_lane_index]->str_id_with_group);
        }
      }
      // 将这根虚拟道next_group->lanes[i]，添加到其右边的左邻
      for (int j = index; j < next_group_lanes_size; ++j) {
        int cur_lane_index = next_group_lanes[j];
        bool exist_left = TopoUtils::IsLeftLane(next_group, cur_lane_index,
                                                i);  // 是否已经存在左邻
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

void LaneTopoConstruct::EraseIntersectLane(Group::Ptr curr_group,
                                           Group::Ptr next_group) {
  std::unordered_set<std::string> erase_next_lane_str_id;
  double real_lane_mean_end_heading = 0.0;
  for (auto& lane : next_group->lanes) {
    if (!lane->right_boundary->pts.empty() &&
        lane->right_boundary->pts[0].type == PointType::RAW) {
      real_lane_mean_end_heading = lane->right_boundary->mean_end_heading;
      break;
    }
  }

  for (int i = 0; i < static_cast<int>(next_group->lanes.size()); ++i) {
    for (int j = i + 1; j < static_cast<int>(next_group->lanes.size()); ++j) {
      bool is_intersect =
          TopoUtils::IsIntersect(next_group->lanes[i], next_group->lanes[j]);
      // HLOG_ERROR << "lane_i = " << next_group->lanes[i]->str_id_with_group
      //            << "  lane_j = " <<
      //            next_group->lanes[j]->str_id_with_group
      //            << "  is_intersect = " << is_intersect;
      if (is_intersect) {
        int left_ego_line_id = LOCATION_MANAGER->GetEgoLane().left_id;
        int right_ego_line_id = LOCATION_MANAGER->GetEgoLane().right_id;
        auto& lane_i = next_group->lanes[i];
        auto& lane_j = next_group->lanes[j];
        if ((lane_i->left_boundary->pts[0].type == PointType::RAW ||
             lane_i->right_boundary->pts[0].type == PointType::RAW) &&
            (lane_j->left_boundary->pts[0].type == PointType::RAW ||
             lane_j->right_boundary->pts[0].type == PointType::RAW)) {
          continue;
        } else if (lane_i->left_boundary->id == left_ego_line_id &&
                   lane_i->right_boundary->id == right_ego_line_id) {
          erase_next_lane_str_id.insert(lane_j->str_id_with_group);
        } else if (lane_j->left_boundary->id == left_ego_line_id &&
                   lane_j->right_boundary->id == right_ego_line_id) {
          erase_next_lane_str_id.insert(lane_i->str_id_with_group);
        } else if ((lane_i->left_boundary->pts[0].type != PointType::RAW) &&
                   (lane_i->right_boundary->pts[0].type != PointType::RAW) &&
                   (lane_j->left_boundary->pts[0].type == PointType::RAW ||
                    lane_j->right_boundary->pts[0].type == PointType::RAW)) {
          erase_next_lane_str_id.insert(lane_i->str_id_with_group);
        } else if ((lane_j->left_boundary->pts[0].type != PointType::RAW) &&
                   (lane_j->right_boundary->pts[0].type != PointType::RAW) &&
                   (lane_i->left_boundary->pts[0].type == PointType::RAW ||
                    lane_i->right_boundary->pts[0].type == PointType::RAW)) {
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

bool LaneTopoConstruct::DistanceInferenceLane(const LineSegment& left_line,
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

bool LaneTopoConstruct::Distanceline(const LineSegment& left_line,
                                     float line_front_x, float line_front_y) {
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

      return (dis < conf_.max_lane_width && dis > conf_.min_lane_width);
    }
  }
  return false;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
