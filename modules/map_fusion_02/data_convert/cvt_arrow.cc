/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： broken_point_search.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "depend/proto/map/map.pb.h"
#include "modules/map_fusion_02/base/element_base.h"

namespace hozon {
namespace mp {
namespace mf {

void FillArrowType(hozon::mp::mf::Arrow* arrow, hozon::hdmap::ArrowData_Type) {
  switch (hozon::mp::mf::ArrowType) {
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_UNKNOWN_TURN:
      arrow->type = hozon::mp::mf::ArrowType::UNKNOWN_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT:
      arrow->type = hozon::mp::mf::ArrowType::STRAIGHT_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_TURN:
      arrow->type = hozon::mp::mf::ArrowType::RIGHT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_TURN:
      arrow->type = hozon::mp::mf::ArrowType::LEFT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_U_TURN:
      arrow->type = hozon::mp::mf::ArrowType::U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_LEFT_TURN:
      arrow->type = hozon::mp::mf::ArrowType::STRAIGHT_LEFT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_RIGHT_TURN:
      arrow->type = hozon::mp::mf::ArrowType::STRAIGHT_RIGHT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_U_TURN:
      arrow->type = hozon::mp::mf::ArrowType::STRAIGHT_U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_U_TURN:
      arrow->type = hozon::mp::mf::ArrowType::LEFT_U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_RIGHT_TURN:
      arrow->type = hozon::mp::mf::ArrowType::LEFT_RIGHT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_FRONT_TURN:
      arrow->type = hozon::mp::mf::ArrowType::LEFT_FRONT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_FRONT_TURN:
      arrow->type = hozon::mp::mf::ArrowType::RIGHT_FRONT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_LEFT_RIGHT_TURN:
      arrow->type = hozon::mp::mf::ArrowType::STRAIGHT_LEFT_RIGHT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_LEFT_U_TURN:
      arrow->type = hozon::mp::mf::ArrowType::STRAIGHT_LEFT_U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_U_TURN:
      arrow->type = hozon::mp::mf::ArrowType::RIGHT_U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_LEFT_TURN:
      arrow->type = hozon::mp::mf::ArrowType::FORBID_LEFT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_RIGHT_TURN:
      arrow->type = hozon::mp::mf::ArrowType::FORBID_RIGHT_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_U_TURN:
      arrow->type = hozon::mp::mf::ArrowType::FORBID_U_TURN_ARROW;
      break;
    case hozon::hdmap::ArrowData_Type::ArrowData_Type_FRONT_NEAR_CROSSWALK:
      arrow->type = hozon::mp::mf::ArrowType::FRONT_NEAR_CROSSWALK_ARROW;
      break;
    default:
      break;
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
