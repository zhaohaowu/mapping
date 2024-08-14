/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： data_convert.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <memory>

#include "depend/proto/local_mapping/local_map.pb.h"
#include "depend/proto/localization/localization.pb.h"
#include "depend/proto/perception/perception_obstacle.pb.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

class DataConvert {
 public:
  DataConvert() = default;
  ~DataConvert() = default;

  static bool LocalMap2ElmentMap(
      const std::shared_ptr<hozon::mapping::LocalMap>& msg,
      ElementMap::Ptr elem_map);
  static bool Localization2LocInfo(
      const std::shared_ptr<hozon::localization::Localization>& loc_msg,
      LocInfo::Ptr loc_info);
  static void ElemMapAppendLaneBoundary(
      const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
      ElementMap::Ptr element_map_ptr);
  static void ElemMapAppendRoadEdge(
      const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
      ElementMap::Ptr element_map_ptr);
  static void ElemMapAppendArrows(
      const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
      ElementMap::Ptr element_map_ptr);
  static void ElemMapAppendStopLine(
      const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
      ElementMap::Ptr element_map_ptr);
  static void ElemMapAppendZebra(
      const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
      ElementMap::Ptr element_map_ptr);
  static void cvtPb2obj(const ::hozon::perception::PerceptionObstacle& obj,
                        Object::Ptr elem_obj);
  static void ElemMapAppendOcc(
      const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
      ElementMap::Ptr element_map_ptr);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
