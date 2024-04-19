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

class TrafficSign : public BaseElement {
 public:
  DEFINE_SMART_PTR(TrafficSign)
  DEFINE_PTR_CONTAINER(TrafficSign)

  TrafficSign() = default;
  TrafficSign(id_t id, const Point3D_t& center, TrafficSignType type)
      : BaseElement(id), center_(center), type_(type) {}
  explicit TrafficSign(const TrafficSign& other)
      : BaseElement(other.id_), center_(other.center_), type_(other.type_) {}
  explicit TrafficSign(const TrafficSignData& data)
      : BaseElement(data.id), center_(data.centroid), type_(data.type) {}

  ~TrafficSign() override {}

  void SetCenter(const Point3D_t& center) { center_ = center; }
  Point3D_t GetCenter() const { return center_; }
  Point3D_t& GetCenter() { return center_; }

  void SetCenterCov(const Eigen::Matrix3d& center_cov) {
    center_cov_.noalias() = center_cov;
  }
  Eigen::Matrix3d GetCenterCov() const { return center_cov_; }
  Eigen::Matrix3d& GetCenterCov() { return center_cov_; }

  void SetType(const TrafficSignType& type) { type_ = type; }
  TrafficSignType GetType() const { return type_; }

 private:
  Point3D_t center_;
  TrafficSignType type_ = TrafficSignType::Unknown;

  Eigen::Matrix3d center_cov_ = Eigen::Matrix3d::Identity();

  // NOTE: following data are currently not provided by HD map
  id_t lane_id_ = 0;
  Quaternion_t quaternion_;
  float32_t length_ = 0;
  float32_t width_ = 0;
  float32_t height_ = 0;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
