/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Chen Longquan <chenlongquan1@sensetime.com>
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <ceres/ceres.h>

#include <vector>

#include "localization/common/log.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

class Vertex : public ceres::LocalParameterization {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(Vertex);

  Vertex(uint64_t id, int dimension) : id_(id), dimension_(dimension) {
    parameter_ = new double[dimension_]();
  }
  ~Vertex() override { delete[] parameter_; }

  /////////////////////////// set & get interface/////////////////////////////

  void SetId(uint64_t id) { id_ = id; }
  uint64_t GetId() const { return id_; }

  void SetFixed(bool fixed = true) { fixed_ = fixed; }
  bool IsFixed() const { return fixed_; }

  // @brief: parameter interface
  void SetParameter(const Eigen::VectorXd& parameter) {
    if (dimension_ != static_cast<int>(parameter.size()))
      throw std::invalid_argument(
          "dimension is not equal with parameter size!");
    memcpy(parameter_, parameter.data(), dimension_ * sizeof(double));
  }
  double* RawParameter() const { return parameter_; }
  Eigen::VectorXd GetParameter() const {
    return Eigen::Map<Eigen::VectorXd>(parameter_, dimension_, 1);
  }

  // @brief: parameter bound interface
  void SetParameterConstant(const size_t index) {
    if (index >= dimension_ || index < 0) return;
    constant_ids_.emplace_back(index);
  }
  void SetParameterConstants(const std::vector<size_t>& indexes) {
    for (const auto& idx : indexes) {
      if (idx >= dimension_ || idx < 0) return;
    }
    constant_ids_ = indexes;
  }
  void SetParameterLowerBound(const size_t index, const double value) {
    if (index >= dimension_ || index < 0) return;
    bound_lower_ids_.emplace_back(index);
    bound_lower_values_.emplace_back(value);
  }
  void SetParameterUpperBound(const size_t index, const double value) {
    if (index >= dimension_ || index < 0) return;
    bound_upper_ids_.emplace_back(index);
    bound_upper_values_.emplace_back(value);
  }
  const std::vector<size_t>& GetConstantIDs() const { return constant_ids_; }
  const std::vector<size_t>& GetBoundLowerIDs() const {
    return bound_lower_ids_;
  }
  const std::vector<size_t>& GetBoundUpperIDs() const {
    return bound_upper_ids_;
  }
  const std::vector<double>& GetBoundLowerValues() const {
    return bound_lower_values_;
  }
  const std::vector<double>& GetBoundUpperValues() const {
    return bound_upper_values_;
  }

  //////////////////////////// override interface/////////////////////////////

  int GlobalSize() const override { return dimension_; }
  int LocalSize() const override { return dimension_; }

  bool Plus(const double* x, const double* delta,
            double* x_plus_delta) const override = 0;

  bool ComputeJacobian(const double* x, double* jacobian) const override {
    ceres::MatrixRef(jacobian, dimension_, dimension_) =
        ceres::Matrix::Identity(dimension_, dimension_);
    return true;
  };

 protected:
  uint64_t id_;         // unique id for this vertex
  bool fixed_ = false;  // if fixed, parameter is not optimied

  int dimension_;      // parameter dimension
  double* parameter_;  // parameter pointer

  std::vector<size_t> constant_ids_;        // constant parameter indexes
  std::vector<size_t> bound_lower_ids_;     // has bound lower parameter indexes
  std::vector<size_t> bound_upper_ids_;     // has bound upper parameter indexes
  std::vector<double> bound_lower_values_;  // bound lower parameter velues
  std::vector<double> bound_upper_values_;  // bound upper parameter velues
};

}  // namespace localization
}  // namespace senseAD
