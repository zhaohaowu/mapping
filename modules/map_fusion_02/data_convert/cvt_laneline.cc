/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： broken_point_search.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/map_fusion_02/base/element_base.h"

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

}  // namespace mf
}  // namespace mp
}  // namespace hozon
