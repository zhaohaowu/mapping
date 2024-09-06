/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cvt_occ.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/data_convert/data_convert.h"

namespace hozon {
namespace mp {
namespace mf {
void DataConvert::ElemMapAppendOcc(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
    ElementMap::Ptr element_map_ptr) {
  for (const auto& occ : local_map->occs()) {
    // 过滤point_size小于2和起始点x小于0的occ
    if (occ.points_size() < 2 || occ.points().at(0).x() < 0) {
      continue;
    }
    // 根据曲线参数和起始点进行采样
    LineCubic curve_params;
    if (occ.has_lane_param() && occ.lane_param().cubic_curve_set_size() == 1) {
      curve_params.start = occ.lane_param().cubic_curve_set(0).start_point_x();
      curve_params.end = occ.lane_param().cubic_curve_set(0).end_point_x();
      auto c0 = occ.lane_param().cubic_curve_set(0).c0();
      auto c1 = occ.lane_param().cubic_curve_set(0).c1();
      auto c2 = occ.lane_param().cubic_curve_set(0).c2();
      auto c3 = occ.lane_param().cubic_curve_set(0).c3();
      curve_params.coefs = {c0, c1, c2, c3};
    }
    if (curve_params.coefs.size() != 4) {
      continue;
    }
    std::vector<Eigen::Vector3d> new_line_pts;
    math::SamplingCubic(curve_params, 1.0, &new_line_pts);
    if (new_line_pts.size() < 2) {
      continue;
    }
    OccRoad occ_road;
    for (const auto& pt : occ.points()) {
      if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
        continue;
      }
      Eigen::Vector3d p(pt.x(), pt.y(), pt.z());
      occ_road.ori_detect_points.emplace_back(p);
    }
    for (const auto& pt : new_line_pts) {
      Eigen::Vector3d point(pt.x(), pt.y(), pt.z());
      occ_road.road_points.emplace_back(point);
    }
    occ_road.track_id = occ.track_id();
    occ_road.detect_id = occ.detect_id();
    occ_road.curve_params = curve_params;
    element_map_ptr->occ_roads[occ_road.track_id] =
        std::make_shared<OccRoad>(occ_road);
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
