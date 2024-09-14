/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： data_convert.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <memory>

#include "common/time/clock.h"
#include "depend/proto/local_mapping/local_map.pb.h"
#include "depend/proto/localization/localization.pb.h"
#include "depend/proto/perception/perception_obstacle.pb.h"
#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/interface_option.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/data_manager/location_data_manager.h"
#include "modules/util/include/util/mapping_log.h"
#include "perception-lib/lib/config_manager/config_manager.h"
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
  static bool CvtLine2Boundary(const GeoLineInfo& line,
                               Boundary::Ptr boundary_line);
  static void ElemMapAppendOcc(
      const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
      ElementMap::Ptr element_map_ptr);
  static std::shared_ptr<hozon::hdmap::Map> ConvertToProtoMap(
      const std::vector<Group::Ptr>& groups, const ElementMap::Ptr& ele_map,
      HistoryId* history_id, const std::map<Id, Zebra::Ptr>& zebra,
      const std::map<Id, Stpl::Ptr>& stopline);
  static bool IsSpeedLimitValid(const std::pair<double, double>& speed_limit) {
    return (speed_limit.first > 0. && speed_limit.second > 0.);
  }
  template <
      typename T,
      typename std::enable_if<
          std::is_base_of<google::protobuf::Message, T>::value, int>::type = 0>
  static void FillHeader(const std::string& module_name, T* msg);
  static std::shared_ptr<hozon::routing::RoutingResponse> SetRouting(
      const std::shared_ptr<hozon::hdmap::Map>& percep_map);
  static bool CloseToLaneEnd(const std::vector<Group::Ptr>& groups,
                             std::string str_id,
                             const Eigen::Vector3f& target_point);
  static double GetLaneLength(const std::vector<Group::Ptr>& groups,
                              std::string str_id_with_group);
  static float LengthToLaneStart(const std::vector<Group::Ptr>& groups,
                                 std::string str_id,
                                 const Eigen::Vector3f& target_point);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
