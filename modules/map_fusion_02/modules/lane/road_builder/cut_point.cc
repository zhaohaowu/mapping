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
CutPoint::CutPoint(const id_t id, const Eigen::Vector3d& point,
                   const CutPointType& cut_type, const id_t main_line_id,
                   const Eigen::Vector3d& cut_dir_params)
    : id_(id),
      point_(point[0], point[1], point[2], 1.0),
      cut_type_(cut_type),
      main_line_id_(main_line_id),
      cut_dir_params_(cut_dir_params) {
  Eigen::Vector3d start_f = -cut_dir_params_ * extend_dist_ + point;
  Eigen::Vector3d end_f = cut_dir_params_ * extend_dist_ + point;
  extre_point_.emplace_back(start_f[0], start_f[1], start_f[2]);
  extre_point_.emplace_back(end_f[0], end_f[1], end_f[2]);
}

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
