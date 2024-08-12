/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cvt_object.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "depend/proto/perception/perception_obstacle.pb.h"
#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/data_convert/data_convert.h"

namespace hozon {
namespace mp {
namespace mf {
void FillObjType(hozon::mp::mf::Object* obj,
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
void cvtPb2obj(const ::hozon::perception::PerceptionObstacle& obj,
               Object::Ptr elem_obj) {
  elem_obj->id = obj.track_id();
  FillObjType(elem_obj.get(), obj.type());
  Eigen::Vector3d point(obj.position().x(), obj.position().y(),
                        obj.position().z());
  elem_obj->position = point;
  Eigen::Vector3f v(static_cast<float>(obj.velocity().x()),
                    static_cast<float>(obj.velocity().y()),
                    static_cast<float>(obj.velocity().z()));
  elem_obj->velocity = v;
  for (const auto& pt : obj.polygon_point()) {
    Eigen::Vector3f obj_pts(static_cast<float>(pt.x()),
                            static_cast<float>(pt.y()),
                            static_cast<float>(pt.z()));
    elem_obj->polygon.points.emplace_back(obj_pts);
  }
  elem_obj->heading = obj.theta();
  elem_obj->length = obj.length();
  elem_obj->width = obj.width();
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
