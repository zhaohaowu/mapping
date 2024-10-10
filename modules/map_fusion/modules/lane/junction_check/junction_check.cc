/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： detect_cut_pt.h
 *   author     ： pengwei
 *   date       ： 2024.05
 ******************************************************************************/
#include "modules/map_fusion/modules/lane/junction_check/junction_check.h"

#include "base/junction.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

int JunctionCheck::Init(const LaneFusionProcessOption& conf) {
  near_junction_dis_thresh_ = conf.near_junction_dis_thresh;
  return 0;
}

int JunctionCheck::Process(const std::vector<Group::Ptr>& groups) {
  if (groups.empty()) {
    type_convetor_.Update(RoadSceneType::IN_JUNCTION);
    junc_info_.road_scene_type = type_convetor_.GetRoadSceneType();
    junc_info_.along_path_vec = Eigen::Vector3d(0, 0, 0);
  } else {
    Eigen::Vector3d ego_pos(0.0, 0.0, 0.0);
    auto& first_group = groups.front();
    auto& last_group = groups.back();
    if (last_group->end_slice.po.x() <= 0) {
      type_convetor_.Update(RoadSceneType::IN_JUNCTION);
      junc_info_.road_scene_type = type_convetor_.GetRoadSceneType();
      Eigen::Vector3d end_po(last_group->end_slice.po.x(),
                             last_group->end_slice.po.y(), 0);
      junc_info_.along_path_vec = ego_pos - end_po;
    } else if (first_group->start_slice.po.x() > 0) {
      type_convetor_.Update(RoadSceneType::IN_JUNCTION);
      junc_info_.road_scene_type = type_convetor_.GetRoadSceneType();
      junc_info_.along_path_vec = Eigen::Vector3d(0, 0, 0);
    } else {
      int ego_group_index = -1;
      for (int i = static_cast<int>(groups.size()) - 1; i >= 0; i--) {
        Eigen::Vector3d start_po(groups[i]->start_slice.po.x(),
                                 groups[i]->start_slice.po.y(), 0);
        Eigen::Vector3d end_po(groups[i]->end_slice.po.x(),
                               groups[i]->end_slice.po.y(), 0);

        if (start_po.x() < 0 && end_po.x() > 0) {
          ego_group_index = i;
          break;
        }
      }
      if (ego_group_index != -1) {
        // 即使在路口中也可能找到自车坐在的group,后续可以结合停止线等来做
        Eigen::Vector3d end_po(groups[ego_group_index]->end_slice.po.x(),
                               groups[ego_group_index]->end_slice.po.y(), 0);
        if (end_po.x() > near_junction_dis_thresh_) {
          type_convetor_.Update(RoadSceneType::GENERAL_ROAD);
          junc_info_.road_scene_type = type_convetor_.GetRoadSceneType();
          junc_info_.along_path_vec = ego_pos - end_po;
        } else {
          if (ego_group_index == static_cast<int>(groups.size()) - 1) {
            type_convetor_.Update(RoadSceneType::NEAR_JUNCTION);
            junc_info_.road_scene_type = type_convetor_.GetRoadSceneType();
            junc_info_.along_path_vec = ego_pos - end_po;
          } else {
            int next_group_index = ego_group_index + 1;
            if (fabs(groups[ego_group_index]->end_slice.po.x() -
                     groups[next_group_index]->start_slice.po.x()) > 1.0) {
              type_convetor_.Update(RoadSceneType::NEAR_JUNCTION);
              junc_info_.road_scene_type = type_convetor_.GetRoadSceneType();
              junc_info_.along_path_vec = ego_pos - end_po;
            } else {
              type_convetor_.Update(RoadSceneType::GENERAL_ROAD);
              junc_info_.road_scene_type = type_convetor_.GetRoadSceneType();
              junc_info_.along_path_vec = ego_pos - end_po;
            }
          }
        }
      } else {
        type_convetor_.Update(RoadSceneType::IN_JUNCTION);
        junc_info_.road_scene_type = type_convetor_.GetRoadSceneType();
        int before_group_index = -1;
        for (int i = static_cast<int>(groups.size()) - 1; i >= 1; i--) {
          Eigen::Vector3d end_po(groups[i]->end_slice.po.x(),
                                 groups[i]->end_slice.po.y(), 0);
          if (end_po.x() < 0) {
            junc_info_.along_path_vec = ego_pos - end_po;
            break;
          }
        }
      }
    }
  }
  JUNC_MANAGER->UpdateStatus(junc_info_);
  HLOG_INFO << "junction check:"
            << static_cast<int>(junc_info_.road_scene_type);

  return static_cast<int>(junc_info_.road_scene_type);
}

JunctionInfo JunctionCheck::GetJunctionInfo() { return junc_info_; }

void JunctionCheck::Clear() {
  junc_info_.road_scene_type = RoadSceneType::GENERAL_ROAD;
  junc_info_.last_road_scene_type = RoadSceneType::GENERAL_ROAD;
  junc_info_.along_path_vec = Eigen::Vector3d(0, 0, 0);
  junc_info_.cross_after_lane_num = 0;
  junc_info_.cross_before_lane_num = 0;
  junc_info_.next_satisefy_lane_seg.clear();
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
