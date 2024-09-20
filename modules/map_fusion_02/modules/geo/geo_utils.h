/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： geo_utils.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <memory>
#include <vector>
#include <boost/circular_buffer.hpp>

#include "modules/map_fusion_02/base/element_base.h"

namespace hozon {
namespace mp {
namespace mf {

// occ 拟合误差
double OccLineFitError(OccRoad::Ptr occ);
// 根据障碍物过滤逆向线
bool CheckOppositeLineByObj(const std::vector<Eigen::Vector3f>& line_points,
                            const std::vector<Eigen::Vector3f>& obj_points);
// 计算车道线heading
void ComputeLaneLineHeading(const std::vector<Eigen::Vector3f>& line_pts,
                            Eigen::Vector3f* avg_heading);
void ComputeAngleBetweenVectors(const Eigen::Vector3f& v1,
                                const Eigen::Vector3f& v2, float* angle_deg);
}  // namespace mf
}  // namespace mp
}  // namespace hozon
