/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cut_point.cc
 *   author     ： mahaijun
 *   date       ： 2024.06
 ******************************************************************************/

#include "modules/map_fusion_02/modules/lane/road_builder/cut_point.h"

namespace hozon {
namespace mp {
namespace mf {

void CutPoint::SetId(const id_t id) { id_ = id; }

id_t CutPoint::GetId() const { return id_; }

void CutPoint::SetPoint(const Point3D& pt) { point_ = pt; }

Point3D CutPoint::GetPoint() const { return point_; }

void CutPoint::SetType(const CutPointType& type) { cut_type_ = type; }

CutPointType CutPoint::GetType() const { return cut_type_; }

void CutPoint::SetMainLineId(const id_t id) { main_line_id_ = id; }

id_t CutPoint::GetMainLineId() const { return main_line_id_; }

void CutPoint::SetTargetLineId(const id_t id) { target_line_id_ = id; }

id_t CutPoint::GetTargetLineId() const { return target_line_id_; }

void CutPoint::SetLineIds(const std::vector<id_t>& line_ids) {
  line_ids_ = line_ids;
}

std::vector<id_t> CutPoint::GetLineIds() const { return line_ids_; }

void CutPoint::SetParams(const Eigen::Vector3d& params) {
  cut_dir_params_ = params;
}

Eigen::Vector3d CutPoint::GetParams() const { return cut_dir_params_; }

}  // namespace mf
}  // namespace mp
}  // namespace hozon
