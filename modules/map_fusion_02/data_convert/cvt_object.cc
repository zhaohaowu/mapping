/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： broken_point_search.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "depend/proto/perception/perception_obstacle.pb.h"
#include "modules/map_fusion_02/base/element_base.h"

namespace hozon {
namespace mp {
namespace mf {
void FillObjType(hozon::mp::mf::Obj* obj,
                 hozon::perception::PerceptionObstacle_Type objtype) {
  switch (objtype) {
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_UNKNOWN:
      obj->type = hozon::mp::mf::ObjType::UNKNOWN;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_UNKNOWN_UNMOVABLE:
      obj->type = hozon::mp::mf::ObjType::UNKNOWN_UNMOVABLE;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_UNKNOWN_MOVABLE:
      obj->type = hozon::mp::mf::ObjType::UNKNOWN_MOVABLE;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_PEDESTRIAN:
      obj->type = hozon::mp::mf::ObjType::PEDESTRIAN;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_BICYCLE:
      obj->type = hozon::mp::mf::ObjType::BICYCLE;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_VEHICLE:
      obj->type = hozon::mp::mf::ObjType::VEHICLE;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_CYCLIST:
      obj->type = hozon::mp::mf::ObjType::CYCLIST;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_STATIC_OBSTACLE:
      obj->type = hozon::mp::mf::ObjType::STATIC_OBSTACLE;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_TRANSPORT_ELEMENT:
      obj->type = hozon::mp::mf::ObjType::TRANSPORT_ELEMENT;
      break;
    case hozon::perception::PerceptionObstacle_Type::
        PerceptionObstacle_Type_ANIMAL:
      obj->type = hozon::mp::mf::ObjType::ANIMAL;
      break;
    default:
      obj->type = hozon::mp::mf::ObjType::UNKNOWN;
      break;
  }
}
}  // namespace mf
}  // namespace mp
}  // namespace hozon
