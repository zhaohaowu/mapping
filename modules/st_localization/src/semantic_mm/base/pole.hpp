/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include "localization/data_type/base.hpp"
#include "localization/data_type/road_structure.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "semantic_mm/base/base_element.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class Pole : public BaseElement {
 public:
  DEFINE_SMART_PTR(Pole)
  DEFINE_PTR_CONTAINER(Pole)

  Pole() = default;
  Pole(id_t id, const Point3D_t& bottom_point, PoleType type)
      : BaseElement(id), bottom_point_(bottom_point), type_(type) {}
  explicit Pole(const Pole& other)
      : BaseElement(other.id_),
        bottom_point_(other.bottom_point_),
        type_(other.type_) {}
  explicit Pole(const PoleData& data)
      : BaseElement(data.id),
        bottom_point_(data.bottom_point),
        type_(data.type) {}

  ~Pole() override {}

  void SetBottomPoint(const Point3D_t& bottom_point) {
    bottom_point_ = bottom_point;
  }
  Point3D_t GetBottomPoint() const { return bottom_point_; }
  Point3D_t& GetBottomPoint() { return bottom_point_; }

  void SetPointCov(const Eigen::Matrix3d& point_cov) {
    point_cov_.noalias() = point_cov;
  }
  Eigen::Matrix3d GetPointCov() const { return point_cov_; }
  Eigen::Matrix3d& GetPointCov() { return point_cov_; }

  void SetType(const PoleType& type) { type_ = type; }
  PoleType GetType() const { return type_; }

 private:
  Point3D_t bottom_point_;
  PoleType type_ = PoleType::Unknown;

  Eigen::Matrix3d point_cov_ = Eigen::Matrix3d::Identity();

  // NOTE: following data are currently not provided by HD map
  id_t lane_id_ = 0;
  Point3D_t top_point_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
