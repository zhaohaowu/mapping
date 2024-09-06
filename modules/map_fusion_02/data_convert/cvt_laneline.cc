/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cvt_laneline.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include <numeric>

#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/data_convert/data_convert.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

void FillLanePos(hozon::mp::mf::Boundary* lane_line,
                 hozon::mapping::LanePositionType lanepostype) {
  switch (lanepostype) {
    case hozon::mapping::LanePositionType::LanePositionType_BOLLARD_LEFT:
      lane_line->lanepos =
          hozon::mp::mf::LanePos::LanePositionType_BOLLARD_LEFT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT:
      lane_line->lanepos = hozon::mp::mf::LanePos::LanePositionType_FOURTH_LEFT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT:
      lane_line->lanepos = hozon::mp::mf::LanePos::LanePositionType_THIRD_LEFT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT:
      lane_line->lanepos =
          hozon::mp::mf::LanePos::LanePositionType_ADJACENT_LEFT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT:
      lane_line->lanepos = hozon::mp::mf::LanePos::LanePositionType_EGO_LEFT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT:
      lane_line->lanepos = hozon::mp::mf::LanePos::LanePositionType_EGO_RIGHT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT:
      lane_line->lanepos =
          hozon::mp::mf::LanePos::LanePositionType_ADJACENT_RIGHT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT:
      lane_line->lanepos = hozon::mp::mf::LanePos::LanePositionType_THIRD_RIGHT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT:
      lane_line->lanepos =
          hozon::mp::mf::LanePos::LanePositionType_FOURTH_RIGHT;
      break;
    case hozon::mapping::LanePositionType::LanePositionType_BOLLARD_RIGHT:
      lane_line->lanepos =
          hozon::mp::mf::LanePos::LanePositionType_BOLLARD_RIGHT;
      break;
    default:
      lane_line->lanepos = hozon::mp::mf::LanePos::LanePositionType_OTHER;
      break;
  }
}

void FillLaneType(hozon::mp::mf::Boundary* lane_line,
                  hozon::mapping::LaneType lanetype) {
  switch (lanetype) {
    case hozon::mapping::LaneType::LaneType_UNKNOWN:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_UNKNOWN;
      break;
    case hozon::mapping::LaneType::LaneType_SOLID:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_SOLID;
      break;
    case hozon::mapping::LaneType::LaneType_DASHED:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_DASHED;
      break;
    case hozon::mapping::LaneType::LaneType_SHORT_DASHED:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_SHORT_DASHED;
      break;
    case hozon::mapping::LaneType::LaneType_DOUBLE_SOLID:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_DOUBLE_SOLID;
      break;
    case hozon::mapping::LaneType::LaneType_DOUBLE_DASHED:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_DOUBLE_DASHED;
      break;
    case hozon::mapping::LaneType::LaneType_LEFT_SOLID_RIGHT_DASHED:
      lane_line->linetype =
          hozon::mp::mf::LineType::LaneType_LEFT_SOLID_RIGHT_DASHED;
      break;
    case hozon::mapping::LaneType::LaneType_RIGHT_SOLID_LEFT_DASHED:
      lane_line->linetype =
          hozon::mp::mf::LineType::LaneType_RIGHT_SOLID_LEFT_DASHED;
      break;
    case hozon::mapping::LaneType::LaneType_SHADED_AREA:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_SHADED_AREA;
      break;
    case hozon::mapping::LaneType::LaneType_LANE_VIRTUAL_MARKING:
      lane_line->linetype =
          hozon::mp::mf::LineType::LaneType_LANE_VIRTUAL_MARKING;
      break;
    case hozon::mapping::LaneType::LaneType_INTERSECTION_VIRTUAL_MARKING:
      lane_line->linetype =
          hozon::mp::mf::LineType::LaneType_INTERSECTION_VIRTUAL_MARKING;
      break;
    case hozon::mapping::LaneType::LaneType_CURB_VIRTUAL_MARKING:
      lane_line->linetype =
          hozon::mp::mf::LineType::LaneType_CURB_VIRTUAL_MARKING;
      break;
    case hozon::mapping::LaneType::LaneType_UNCLOSED_ROAD:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_UNCLOSED_ROAD;
      break;
    case hozon::mapping::LaneType::LaneType_ROAD_VIRTUAL:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_ROAD_VIRTUAL;
      break;
    case hozon::mapping::LaneType::LaneType_LANE_CHANG_VIRTUAL:
      lane_line->linetype =
          hozon::mp::mf::LineType::LaneType_LANE_CHANG_VIRTUAL;
      break;
    case hozon::mapping::LaneType::LaneType_FISHBONE_SOLID:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_FISHBONE_SOLID;
      break;
    case hozon::mapping::LaneType::LaneType_FISHBONE_DASHED:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_FISHBONE_DASHED;
      break;
    default:
      lane_line->linetype = hozon::mp::mf::LineType::LaneType_OTHER;
      break;
  }
}

void FillLaneColor(hozon::mp::mf::Boundary* lane_line,
                   hozon::mapping::Color lanecolor) {
  switch (lanecolor) {
    case hozon::mapping::Color::UNKNOWN:
      lane_line->color = UNKNOWN_COLOR;
      break;
    case hozon::mapping::Color::WHITE:
      lane_line->color = WHITE;
      break;
    case hozon::mapping::Color::YELLOW:
      lane_line->color = YELLOW;
      break;
    case hozon::mapping::Color::GREEN:
      lane_line->color = GREEN;
      break;
    case hozon::mapping::Color::RED:
      lane_line->color = RED;
      break;
    case hozon::mapping::Color::BLACK:
      lane_line->color = BLACK;
      break;
    default:
      break;
  }
}

void FillIsNearRoadLine(hozon::mp::mf::Boundary* lane_line,
                        const std::vector<std::vector<Eigen::Vector3d>>& roads,
                        const hozon::mapping::LaneLine& local_line) {
  // 判断是否是靠近路沿的线
  if (roads.empty()) {
    return;
  }
  std::vector<Eigen::Vector3d> line_it;
  for (const auto& pt : local_line.points()) {
    Eigen::Vector3d ptt(pt.x(), pt.y(), pt.z());
    line_it.emplace_back(ptt);
  }
  for (const auto& road_it : roads) {
    double avg_dis = 0.0;
    if (!math::ComputerLineDis(line_it, road_it, &avg_dis)) {
      continue;
    }
    if (avg_dis < 1.0) {
      lane_line->is_near_road_edge = true;
      break;
    }
  }
}

bool DataConvert::CvtLine2Boundary(const GeoLineInfo& line,
                                   Boundary::Ptr boundary_line) {
  if (line.line_pts.empty()) {
    return false;
  }
  int node_id = 1;
  Eigen::Vector3f last_point;
  bool last_point_added = false;
  if (!boundary_line->nodes.empty()) {
    boundary_line->nodes.clear();
  }
  for (auto& point : line.line_pts) {
    if (std::isnan(point.x()) || std::isnan(point.y()) ||
        std::isnan(point.z())) {
      continue;
    }
    if (!last_point_added) {
      last_point = point;
      last_point_added = true;
    } else {
      if ((last_point - point).norm() < 0.1) {
        continue;
      }
      last_point = point;
    }
    BoundaryNode::Ptr node = std::make_shared<BoundaryNode>();
    node->point.x() = point.x();
    node->point.y() = point.y();
    node->point.z() = point.z();
    node->id = node_id;
    boundary_line->nodes.emplace_back(node);
    ++node_id;
  }
  boundary_line->color = line.color;
  boundary_line->id = line.line_track_id;
  boundary_line->linetype = line.line_type;
  boundary_line->is_ego = line.is_ego;
  return true;
}

void DataConvert::ElemMapAppendLaneBoundary(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
    ElementMap::Ptr element_map_ptr) {
  int node_name = 1;
  for (const auto& lane_line_it : local_map->lane_lines()) {
    // 过滤空的点
    if (lane_line_it.points_size() <= 0) {
      continue;
    }
    hozon::mp::mf::Boundary lane_line;

    auto point_size = lane_line_it.points().size();
    Eigen::Vector3f last_point;  // 为了把太近的点过滤掉
    int last_point_exisit = 0;
    for (const auto& line_point_it : lane_line_it.points()) {
      if (std::isnan(line_point_it.x()) || std::isnan(line_point_it.y()) ||
          std::isnan(line_point_it.z())) {
        HLOG_ERROR << "found nan in localmap";
        continue;
      }
      Eigen::Vector3f point_local(static_cast<float>(line_point_it.x()),
                                  static_cast<float>(line_point_it.y()),
                                  static_cast<float>(line_point_it.z()));
      if (last_point_exisit == 0) {
        last_point = point_local;
        last_point_exisit = 1;
      } else {
        if ((last_point - point_local).norm() < 0.1) {
          continue;
        }
        last_point = point_local;
      }
      // Eigen::Vector3d point_enu = T_U_V_ * point_local;
      hozon::mp::mf::BoundaryNode node;
      node.point = point_local;
      node.id = node_name;
      element_map_ptr->boundary_nodes[node_name] =
          std::make_shared<hozon::mp::mf::BoundaryNode>(node);
      lane_line.nodes.push_back(element_map_ptr->boundary_nodes[node_name]);
      node_name++;
    }
    lane_line.id = lane_line_it.track_id();
    for (const auto& delete_id : lane_line_it.deleted_track_ids()) {
      lane_line.delete_ids.emplace_back(delete_id);
    }
    FillLanePos(&lane_line, lane_line_it.lanepos());
    FillLaneType(&lane_line, lane_line_it.lanetype());
    FillLaneColor(&lane_line, lane_line_it.color());
    element_map_ptr->lane_boundaries[lane_line.id] =
        std::make_shared<hozon::mp::mf::Boundary>(lane_line);
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
