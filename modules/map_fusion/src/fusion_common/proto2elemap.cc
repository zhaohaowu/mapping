/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： proto2elemap.cc
 *   author     ： hozon
 *   date       ： 2024.08
 ******************************************************************************/

#include "map_fusion/fusion_common/proto2elemap.h"

namespace hozon {
namespace mp {
namespace mf {
namespace em {

void SetLanePos(const hozon::mp::mf::em::Boundary& lane_line,
                hozon::mapping::LanePositionType* lanepostype) {
  switch (lane_line.lanepos) {
    case hozon::mp::mf::em::LanePos::LanePositionType_BOLLARD_LEFT:
      *lanepostype =
          hozon::mapping::LanePositionType::LanePositionType_BOLLARD_LEFT;
      break;
    case hozon::mp::mf::em::LanePos::LanePositionType_FOURTH_LEFT:
      *lanepostype =
          hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT;
      break;
    case hozon::mp::mf::em::LanePos::LanePositionType_THIRD_LEFT:
      *lanepostype =
          hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT;
      break;
    case hozon::mp::mf::em::LanePos::LanePositionType_ADJACENT_LEFT:
      *lanepostype =
          hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT;
      break;
    case hozon::mp::mf::em::LanePos::LanePositionType_EGO_LEFT:
      *lanepostype =
          hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT;
      break;
    case hozon::mp::mf::em::LanePos::LanePositionType_EGO_RIGHT:
      *lanepostype =
          hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT;
      break;
    case hozon::mp::mf::em::LanePos::LanePositionType_ADJACENT_RIGHT:
      *lanepostype =
          hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT;
      break;
    case hozon::mp::mf::em::LanePos::LanePositionType_THIRD_RIGHT:
      *lanepostype =
          hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT;
      break;
    case hozon::mp::mf::em::LanePos::LanePositionType_FOURTH_RIGHT:
      *lanepostype =
          hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT;
      break;
    case hozon::mp::mf::em::LanePos::LanePositionType_BOLLARD_RIGHT:
      *lanepostype =
          hozon::mapping::LanePositionType::LanePositionType_BOLLARD_RIGHT;
      break;
    default:
      *lanepostype = hozon::mapping::LanePositionType::LanePositionType_OTHER;
      break;
  }
}

void SetLaneColor(const hozon::mp::mf::em::Boundary& lane_line,
                  hozon::mapping::Color* lanecolor) {
  switch (lane_line.color) {
    case em::UNKNOWN_COLOR:
      *lanecolor = hozon::mapping::Color::UNKNOWN;
      break;
    case em::WHITE:
      *lanecolor = hozon::mapping::Color::WHITE;
      break;
    case em::YELLOW:
      *lanecolor = hozon::mapping::Color::YELLOW;
      break;
    case em::GREEN:
      *lanecolor = hozon::mapping::Color::GREEN;
      break;
    case em::RED:
      *lanecolor = hozon::mapping::Color::RED;
      break;
    case em::BLACK:
      *lanecolor = hozon::mapping::Color::BLACK;
      break;
    default:
      break;
  }
}

void SetLaneType(const hozon::mp::mf::em::Boundary& lane_line,
                 hozon::mapping::LaneType* lanetype) {
  switch (lane_line.linetype) {
    case hozon::mp::mf::em::LineType::LaneType_UNKNOWN:
      *lanetype = hozon::mapping::LaneType::LaneType_UNKNOWN;
      break;
    case hozon::mp::mf::em::LineType::LaneType_SOLID:
      *lanetype = hozon::mapping::LaneType::LaneType_SOLID;
      break;
    case hozon::mp::mf::em::LineType::LaneType_DASHED:
      *lanetype = hozon::mapping::LaneType::LaneType_DASHED;
      break;
    case hozon::mp::mf::em::LineType::LaneType_SHORT_DASHED:
      *lanetype = hozon::mapping::LaneType::LaneType_SHORT_DASHED;
      break;
    case hozon::mp::mf::em::LineType::LaneType_DOUBLE_SOLID:
      *lanetype = hozon::mapping::LaneType::LaneType_DOUBLE_SOLID;
      break;
    case hozon::mp::mf::em::LineType::LaneType_DOUBLE_DASHED:
      *lanetype = hozon::mapping::LaneType::LaneType_DOUBLE_DASHED;
      break;
    case hozon::mp::mf::em::LineType::LaneType_LEFT_SOLID_RIGHT_DASHED:
      *lanetype = hozon::mapping::LaneType::LaneType_LEFT_SOLID_RIGHT_DASHED;
      break;
    case hozon::mp::mf::em::LineType::LaneType_RIGHT_SOLID_LEFT_DASHED:
      *lanetype = hozon::mapping::LaneType::LaneType_RIGHT_SOLID_LEFT_DASHED;
      break;
    case hozon::mp::mf::em::LineType::LaneType_SHADED_AREA:
      *lanetype = hozon::mapping::LaneType::LaneType_SHADED_AREA;
      break;
    case hozon::mp::mf::em::LineType::LaneType_LANE_VIRTUAL_MARKING:
      *lanetype = hozon::mapping::LaneType::LaneType_LANE_VIRTUAL_MARKING;
      break;
    case hozon::mp::mf::em::LineType::LaneType_INTERSECTION_VIRTUAL_MARKING:
      *lanetype =
          hozon::mapping::LaneType::LaneType_INTERSECTION_VIRTUAL_MARKING;
      break;
    case hozon::mp::mf::em::LineType::LaneType_CURB_VIRTUAL_MARKING:
      *lanetype = hozon::mapping::LaneType::LaneType_CURB_VIRTUAL_MARKING;
      break;
    case hozon::mp::mf::em::LineType::LaneType_UNCLOSED_ROAD:
      *lanetype = hozon::mapping::LaneType::LaneType_UNCLOSED_ROAD;
      break;
    case hozon::mp::mf::em::LineType::LaneType_ROAD_VIRTUAL:
      *lanetype = hozon::mapping::LaneType::LaneType_ROAD_VIRTUAL;
      break;
    case hozon::mp::mf::em::LineType::LaneType_LANE_CHANG_VIRTUAL:
      *lanetype = hozon::mapping::LaneType::LaneType_LANE_CHANG_VIRTUAL;
      break;
    case hozon::mp::mf::em::LineType::LaneType_FISHBONE_SOLID:
      *lanetype = hozon::mapping::LaneType::LaneType_FISHBONE_SOLID;
      break;
    case hozon::mp::mf::em::LineType::LaneType_FISHBONE_DASHED:
      *lanetype = hozon::mapping::LaneType::LaneType_FISHBONE_DASHED;
      break;
    default:
      *lanetype = hozon::mapping::LaneType::LaneType_OTHER;
      break;
  }
}

}  // namespace em
}  // namespace mf
}  // namespace mp
}  // namespace hozon
