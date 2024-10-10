/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cvt_map.cc
 *   author     ： zhangzhike
 *   date       ： 2024.09
 ******************************************************************************/

#include "modules/map_fusion/data_convert/data_convert.h"
namespace hozon {
namespace mp {
namespace mf {
using ProtoBoundType = hozon::hdmap::LaneBoundaryType;
bool DataConvert::CloseToLaneEnd(const std::vector<Group::Ptr>& groups,
                                 std::string str_id,
                                 const Eigen::Vector3f& target_point) {
  for (const auto& grp : groups) {
    for (const auto& lane : grp->lanes) {
      if (lane->str_id_with_group == str_id) {
        if (lane->center_line_pts.size() >= 2) {
          Eigen::Vector3f front_pt = lane->center_line_pts.front().pt;
          Eigen::Vector3f end_pt = lane->center_line_pts.back().pt;
          auto length_1 = (target_point - front_pt).norm();
          auto length_2 = (target_point - end_pt).norm();
          if (length_2 < length_1) {
            return true;
          }
        }
      }
    }
  }
  return false;
}
double DataConvert::GetLaneLength(const std::vector<Group::Ptr>& groups,
                                  std::string str_id) {
  double length = 0;
  for (const auto& grp : groups) {
    for (const auto& lane : grp->lanes) {
      if (lane->str_id_with_group == str_id) {
        Eigen::Vector3f prev_pt;
        for (size_t i = 0; i < lane->center_line_pts.size(); ++i) {
          const auto& pt = lane->center_line_pts[i].pt;
          if (i > 0) {
            length += (pt - prev_pt).norm();
          }
          prev_pt = pt;
        }
        return length;
      }
    }
  }
  return length;
}
float DataConvert::LengthToLaneStart(const std::vector<Group::Ptr>& groups,
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
          Eigen::Vector3f prev_pt = lane->center_line_pts[0].pt;
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
std::shared_ptr<hozon::hdmap::Map> DataConvert::ConvertToProtoMap(
    const std::vector<Group::Ptr>& groups, const ElementMap::Ptr& ele_map,
    HistoryId* history_id, const std::map<Id, Zebra::Ptr>& zebra,
    const std::map<Id, Stpl::Ptr>& stopline) {
  if (groups.empty()) {
    return nullptr;
  }
  for (const auto& grp : groups) {
    if (grp == nullptr) {
      HLOG_ERROR << "found nullptr group";
      return nullptr;
    }
  }
  auto cur_T_w_v_ = LOCATION_MANAGER->GetCurrentPose();
  auto* config_manager = hozon::perception::lib::ConfigManager::Instance();
  const hozon::perception::lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("LaneFusionPipeline", &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: "
               << "LaneFusionPipeline";
  }
  LaneFusionProcessOption options_;
  if (!model_config->get_value("lane_speed_limit_kmph",
                               &options_.lane_speed_limit_kmph)) {
    HLOG_ERROR << "Get lane_speed_limit_kmph failed!";
  }
  if (!model_config->get_value("road_min_max_speed_kmph",
                               &options_.road_min_max_speed_kmph)) {
    HLOG_ERROR << "Get road_min_max_speed_kmph failed!";
  }
  if (!model_config->get_value("road_max_max_speed_kmph",
                               &options_.road_max_max_speed_kmph)) {
    HLOG_ERROR << "Get road_max_max_speed_kmph failed!";
  }
  std::pair<double, double> speed_limit{0., 0.};
  std::shared_ptr<hozon::hdmap::Map> map =
      std::make_shared<hozon::hdmap::Map>();
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
      Eigen::Vector3f prev_pt;
      for (size_t i = 0; i < lane->center_line_pts.size(); ++i) {
        const auto& pt = lane->center_line_pts[i].pt;
        if (i > 0) {
          length += (pt - prev_pt).norm();
        }
        prev_pt = pt;
        Eigen::Vector3d pt_tmp = pt.cast<double>();
        Eigen::Vector3f pt_local(static_cast<float>((cur_T_w_v_ * pt_tmp).x()),
                                 static_cast<float>((cur_T_w_v_ * pt_tmp).y()),
                                 0.0f);
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
            length += (pt - prev_pt).norm();
          }
          if (i == 1 && lane->left_boundary->pts[i].type == VIRTUAL) {
            is_virtual = true;
          }
          prev_pt = pt;
          Eigen::Vector3d pt_tmp = pt.cast<double>();
          Eigen::Vector3f pt_local(
              static_cast<float>((cur_T_w_v_ * pt_tmp).x()),
              static_cast<float>((cur_T_w_v_ * pt_tmp).y()), 0.0f);
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
        std::set<LineType> dotted = {
            LaneType_DASHED,
            LaneType_SHORT_DASHED,
            LaneType_DOUBLE_DASHED,
            LaneType_FISHBONE_DASHED,
            LaneType_INTERSECTION_VIRTUAL_MARKING,
        };
        std::set<LineType> solid = {
            LaneType_SOLID,
            LaneType_FISHBONE_SOLID,
        };
        if (lane->left_boundary->lanepos > 0) {
          dotted.insert(LaneType_RIGHT_SOLID_LEFT_DASHED);
          solid.insert(LaneType_LEFT_SOLID_RIGHT_DASHED);
        } else {
          dotted.insert(LaneType_LEFT_SOLID_RIGHT_DASHED);
          solid.insert(LaneType_RIGHT_SOLID_LEFT_DASHED);
        }
        if (dotted.find(lane->left_boundary->type) != dotted.end() &&
            lane->left_boundary->color == YELLOW) {
          type = ProtoBoundType::DOTTED_YELLOW;
        } else if (dotted.find(lane->left_boundary->type) != dotted.end() &&
                   lane->left_boundary->color == WHITE) {
          type = ProtoBoundType::DOTTED_WHITE;
        } else if (solid.find(lane->left_boundary->type) != solid.end() &&
                   lane->left_boundary->color == YELLOW) {
          type = ProtoBoundType::SOLID_YELLOW;
        } else if (solid.find(lane->left_boundary->type) != solid.end() &&
                   lane->left_boundary->color == WHITE) {
          type = ProtoBoundType::SOLID_WHITE;
        } else if (lane->left_boundary->type == LaneType_DOUBLE_SOLID &&
                   lane->left_boundary->color == YELLOW) {
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
            length += (pt - prev_pt).norm();
          }
          if (i == 1 && lane->right_boundary->pts[i].type == VIRTUAL) {
            is_virtual = true;
          }
          prev_pt = pt;
          Eigen::Vector3d pt_tmp = pt.cast<double>();
          Eigen::Vector3f pt_local(
              static_cast<float>((cur_T_w_v_ * pt_tmp).x()),
              static_cast<float>((cur_T_w_v_ * pt_tmp).y()), 0.0f);
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
        std::set<LineType> dotted = {
            LaneType_DASHED,
            LaneType_SHORT_DASHED,
            LaneType_DOUBLE_DASHED,
            LaneType_FISHBONE_DASHED,
            LaneType_INTERSECTION_VIRTUAL_MARKING,
        };
        std::set<LineType> solid = {
            LaneType_SOLID,
            LaneType_FISHBONE_SOLID,
        };
        if (lane->right_boundary->lanepos > 0) {
          dotted.insert(LaneType_RIGHT_SOLID_LEFT_DASHED);
          solid.insert(LaneType_LEFT_SOLID_RIGHT_DASHED);
        } else {
          dotted.insert(LaneType_LEFT_SOLID_RIGHT_DASHED);
          solid.insert(LaneType_RIGHT_SOLID_LEFT_DASHED);
        }
        if (dotted.find(lane->right_boundary->type) != dotted.end() &&
            lane->right_boundary->color == YELLOW) {
          type = ProtoBoundType::DOTTED_YELLOW;
        } else if (dotted.find(lane->right_boundary->type) != dotted.end() &&
                   lane->right_boundary->color == WHITE) {
          type = ProtoBoundType::DOTTED_WHITE;
        } else if (solid.find(lane->right_boundary->type) != solid.end() &&
                   lane->right_boundary->color == YELLOW) {
          type = ProtoBoundType::SOLID_YELLOW;
        } else if (solid.find(lane->right_boundary->type) != solid.end() &&
                   lane->right_boundary->color == WHITE) {
          type = ProtoBoundType::SOLID_WHITE;
        } else if (lane->right_boundary->type == LaneType_DOUBLE_SOLID &&
                   lane->right_boundary->color == YELLOW) {
          type = ProtoBoundType::DOUBLE_YELLOW;
        }
        boundary_type->add_types(type);
      }

      // length
      //! TBD: 设为中心线长度行不行？
      proto_lane->set_length(central_seg->length());
      //! TBD: speed_limit
      proto_lane->set_speed_limit(IsSpeedLimitValid(speed_limit)
                                      ? speed_limit.first
                                      : options_.lane_speed_limit_kmph / 3.6);

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
        IsSpeedLimitValid(speed_limit)
            ? speed_limit.second
            : options_.road_max_max_speed_kmph / 3.6);
    proto_road_section->set_min_max_speed(options_.road_min_max_speed_kmph /
                                          3.6);
    proto_road_section->mutable_id()->set_id(std::to_string(grp_idx));
    for (const auto& id : grp) {
      auto* proto_lane_id = proto_road_section->add_lane_id();
      proto_lane_id->set_id(id);
    }
  }
  history_id->road_id = grp_idx;
  // // stop lines
  // for (const auto& stopline_it : ele_map->stop_lines) {
  //   auto* stop_line = map->add_stop_line();
  //   stop_line->set_id("01" + std::to_string(stopline_it.first));
  //   auto& pt = stopline_it.second->points[0];
  //   Eigen::Vector3d pt_tmp = pt.cast<double>();
  //   Eigen::Vector3f left_point(static_cast<float>((cur_T_w_v_ * pt_tmp).x()),
  //                              static_cast<float>((cur_T_w_v_ * pt_tmp).y()),
  //                              0.0f);
  //   auto* point_left = stop_line->mutable_shape()->add_point();
  //   point_left->set_x(static_cast<double>(left_point.x()));
  //   point_left->set_y(static_cast<double>(left_point.y()));
  //   point_left->set_z(static_cast<double>(left_point.z()));
  //   pt = stopline_it.second->points[1];
  //   pt_tmp = pt;
  //   Eigen::Vector3f right_point(static_cast<float>((cur_T_w_v_ * pt).x()),
  //                               static_cast<float>((cur_T_w_v_ * pt).y()),
  //                               0.0f);

  //   auto* point_right = stop_line->mutable_shape()->add_point();
  //   point_right->set_x(static_cast<double>(right_point.x()));
  //   point_right->set_y(static_cast<double>(right_point.y()));
  //   point_right->set_z(static_cast<double>(right_point.z()));

  //   auto* signal = map->add_signal();
  //   signal->mutable_id()->set_id("01" + std::to_string(stopline_it.first));
  //   auto* segment = signal->add_stop_line()->add_segment();
  //   auto* point_l = segment->mutable_line_segment()->add_point();
  //   point_l->set_x(static_cast<double>(left_point.x()));
  //   point_l->set_y(static_cast<double>(left_point.y()));
  //   point_l->set_z(static_cast<double>(left_point.z()));
  //   auto* point_r = segment->mutable_line_segment()->add_point();
  //   point_r->set_x(static_cast<double>(right_point.x()));
  //   point_r->set_y(static_cast<double>(right_point.y()));
  //   point_r->set_z(static_cast<double>(right_point.z()));
  //   auto* start_position = segment->mutable_start_position();
  //   start_position->set_x(static_cast<double>(left_point.x()));
  //   start_position->set_y(static_cast<double>(left_point.y()));
  //   start_position->set_z(static_cast<double>(left_point.z()));

  //   for (const auto& lane_id_it : stopline[stopline_it.first]->lane_id) {
  //     if (lane_id_hash.find(lane_id_it) != lane_id_hash.end()) {
  //       stop_line->add_lane_id(std::to_string(lane_id_hash[lane_id_it]));

  //       // 对停止线和车道线关联对构建overlap
  //       auto* stopline_overlap = map->add_overlap();
  //       auto id = "01" + std::to_string(stopline_it.first) + "_" +
  //                 std::to_string(lane_id_hash[lane_id_it]);
  //       stopline_overlap->mutable_id()->set_id(id);

  //       signal->add_overlap_id()->set_id(id);
  //       auto* object_stopline = stopline_overlap->add_object();
  //       object_stopline->mutable_id()->set_id(
  //           "01" + std::to_string(stopline_it.first));
  //       object_stopline->mutable_signal_overlap_info();

  //       auto* object_lane = stopline_overlap->add_object();
  //       object_lane->mutable_id()->set_id(
  //           std::to_string(lane_id_hash[lane_id_it]));
  //       auto* laneinfo = object_lane->mutable_lane_overlap_info();
  //       // 判断是否关联对是进路口还是出路口关联形式
  //       auto target_point =
  //           (stopline_it.second->points[0] + stopline_it.second->points[1]) /
  //           2;
  //       auto length_to_start =
  //           LengthToLaneStart(groups, lane_id_it, target_point);
  //       laneinfo->set_start_s(length_to_start);
  //       laneinfo->set_end_s(length_to_start + 0.1);
  //       // 给map.lane.overlap_id赋值
  //       for (int i = 0; i < map->lane_size(); ++i) {
  //         auto* pro_lane = map->mutable_lane(i);
  //         if (pro_lane->id().id() ==
  //         std::to_string(lane_id_hash[lane_id_it])) {
  //           auto* lane_overlap_id = pro_lane->add_overlap_id();
  //           lane_overlap_id->set_id(id);
  //         }
  //       }
  //     }
  //   }
  // }

  // // crosswalk
  // for (const auto& crosswalk_it : ele_map->cross_walks) {
  //   if (crosswalk_it.second->polygon.points.size() != 4) {
  //     continue;
  //   }
  //   auto* cross_walk = map->add_crosswalk();
  //   cross_walk->mutable_id()->set_id("02" +
  //   std::to_string(crosswalk_it.first)); for (const auto& point_it :
  //   crosswalk_it.second->polygon.points) {

  // chongxinxiugai
  //     Eigen::Vector3f pt(static_cast<float>((cur_T_w_v_ * point_it).x()),
  //                        static_cast<float>((cur_T_w_v_ * point_it).y()),
  //                        0.0f);
  //     auto* cross_walk_pt = cross_walk->mutable_polygon()->add_point();
  //     cross_walk_pt->set_x(static_cast<double>(pt.x()));
  //     cross_walk_pt->set_y(static_cast<double>(pt.y()));
  //     cross_walk_pt->set_z(static_cast<double>(pt.z()));
  //   }

  //   for (const auto& lane_id_it : zebra[crosswalk_it.first]->lane_id) {
  //     if (lane_id_hash.find(lane_id_it) != lane_id_hash.end()) {
  //       // 对斑马线和车道线关联对构建overlap
  //       auto* cross_walk_overlap = map->add_overlap();
  //       cross_walk_overlap->mutable_id()->set_id(
  //           "02" + std::to_string(crosswalk_it.first) + "_" +
  //           std::to_string(lane_id_hash[lane_id_it]));
  //       auto* object_cross = cross_walk_overlap->add_object();
  //       object_cross->mutable_id()->set_id("02" +
  //                                          std::to_string(crosswalk_it.first));
  //       object_cross->mutable_crosswalk_overlap_info();

  //       auto* object_lane = cross_walk_overlap->add_object();
  //       object_lane->mutable_id()->set_id(
  //           std::to_string(lane_id_hash[lane_id_it]));
  //       auto* laneinfo = object_lane->mutable_lane_overlap_info();
  //       // 判断是否关联对是进路口还是出路口关联形式
  //       auto target_point = (crosswalk_it.second->polygon.points[1] +
  //                            crosswalk_it.second->polygon.points[2]) /
  //                           2;
  //       if (CloseToLaneEnd(groups, lane_id_it, target_point)) {
  //         auto lane_length = GetLaneLength(groups, lane_id_it);
  //         laneinfo->set_start_s(lane_length);
  //         laneinfo->set_end_s(lane_length + 2.0);
  //       } else {
  //         laneinfo->set_start_s(0.0);
  //         laneinfo->set_end_s(2.0);
  //       }

  //       // 给map.lane.overlap_id赋值
  //       for (int i = 0; i < map->lane_size(); ++i) {
  //         auto* pro_lane = map->mutable_lane(i);
  //         if (pro_lane->id().id() ==
  //         std::to_string(lane_id_hash[lane_id_it])) {
  //           auto* lane_overlap_id = pro_lane->add_overlap_id();
  //           lane_overlap_id->set_id("02" + std::to_string(crosswalk_it.first)
  //           +
  //                                   "_" +
  //                                   std::to_string(lane_id_hash[lane_id_it]));
  //         }
  //       }
  //     }
  //   }
  // }

  //! TBD: arrow rewrite
  // // arrows
  // for (const auto& arrow_it : ele_map->arrows) {
  //   if (arrow_it.second->polygon.points.size() != 4) {
  //     continue;
  //   }
  //   auto* arrow = map->add_arraw();
  //   arrow->set_id(std::to_string(arrow_it.first));
  //   double center_x = 0.0, center_y = 0.0;
  //   for (const auto& point_it : arrow_it.second->polygon.points) {
  //     Eigen::Vector3f pt(static_cast<float>((cur_T_w_v_ * point_it).x()),
  //                                  static_cast<float>((cur_T_w_v_ *
  //                                  point_it).y()), 0.0f);
  //     auto* arrow_pt = arrow->mutable_shape()->add_point();
  //     arrow_pt->set_x(static_cast<double>(pt.x()));
  //     arrow_pt->set_y(static_cast<double>(pt.y()));
  //     arrow_pt->set_z(static_cast<double>(pt.z()));
  //     center_x += pt.x();
  //     center_y += pt.y();
  //   }
  //   arrow->mutable_center_point()->set_x(center_x / 4);
  //   arrow->mutable_center_point()->set_y(center_y / 4);
  //   Eigen::Quaternionf quat_arrow_in_veh(
  //       Eigen::AngleAxisf(arrow_it.second->heading,
  //       Eigen::Vector3f::UnitZ()));
  //   Eigen::Quaternionf quat_arrow_in_local_enu =
  //       curr_pose->quat * quat_arrow_in_veh;
  //   Eigen::Vector3f euler_arrow =
  //       quat_arrow_in_local_enu.toRotationMatrix().eulerAngles(2, 0, 1);
  //   arrow->set_heading(static_cast<double>(euler_arrow[0]));
  //   arrow->set_type(FillArrowType(arrow_it.second->type));
  // }
  return map;
}

template <typename T, typename std::enable_if<
                          std::is_base_of<google::protobuf::Message, T>::value,
                          int>::type = 0>
void DataConvert::FillHeader(const std::string& module_name, T* msg) {
  static std::atomic<uint64_t> sequence_num = {0};
  auto* header = msg->mutable_header();
  double timestamp = ::hozon::common::Clock::NowInSeconds();
  header->set_frame_id(module_name);
  header->set_publish_stamp(timestamp);
  header->set_seq(static_cast<unsigned int>(sequence_num.fetch_add(1)));
}

std::shared_ptr<hozon::routing::RoutingResponse> DataConvert::SetRouting(
    const std::shared_ptr<hozon::hdmap::Map>& percep_map) {
  int lane_num = percep_map->lane_size();
  if (lane_num < 1) {
    HLOG_ERROR << "empty lane in percep_map";
    return nullptr;
  }
  std::shared_ptr<hozon::routing::RoutingResponse> routing =
      std::make_shared<hozon::routing::RoutingResponse>();
  // <lane_id, lane_length>
  std::map<std::string, double> lane_length_hash;
  for (const auto& lane : percep_map->lane()) {
    lane_length_hash.insert_or_assign(lane.id().id(), lane.length());
  }

  for (const auto& road : percep_map->road()) {
    for (const auto& section : road.section()) {
      if (section.lane_id().empty()) {
        continue;
      }

      auto* routing_road = routing->add_road();
      routing_road->set_id(section.id().id());
      routing_road->mutable_passage()->Reserve(section.lane_id_size());
      for (const auto& lane_id : section.lane_id()) {
        double length = 0;
        if (lane_length_hash.find(lane_id.id()) == lane_length_hash.end()) {
          continue;
        }
        length = lane_length_hash[lane_id.id()];
        auto* passage = routing_road->add_passage();
        passage->set_can_exit(true);
        passage->set_change_lane_type(routing::ChangeLaneType::FORWARD);
        auto* segment = passage->add_segment();
        segment->set_id(lane_id.id());
        segment->set_start_s(0.0);
        segment->set_end_s(length);
      }
    }
  }

  auto adc_lane_segment_points = percep_map->lane(lane_num - 1)
                                     .central_curve()
                                     .segment()[0]
                                     .line_segment()
                                     .point();
  common::PointENU start_point = adc_lane_segment_points[0];
  int max_index = static_cast<int>(adc_lane_segment_points.size()) - 1;
  common::PointENU end_point = adc_lane_segment_points[max_index];
  auto* routing_request = routing->mutable_routing_request();
  routing::LaneWaypoint waypoint;
  waypoint.set_id(percep_map->lane(lane_num - 1).id().id());
  waypoint.mutable_pose()->set_x(start_point.x());
  waypoint.mutable_pose()->set_y(start_point.y());
  waypoint.set_s(0.0);
  routing_request->add_waypoint()->CopyFrom(waypoint);
  waypoint.set_s(percep_map->lane(lane_num - 1).length());
  waypoint.mutable_pose()->set_x(end_point.x());
  waypoint.mutable_pose()->set_y(end_point.y());
  routing_request->add_waypoint()->CopyFrom(waypoint);

  // auto* routing_request = routing_->mutable_routing_request();
  FillHeader("from_adaptive_cruise_routingrequest", routing_request);
  FillHeader("from_adaptive_cruise_routing", &(*routing));
  return routing;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
