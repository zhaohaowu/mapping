/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Chen Longquan <chenlongquan1@sensetime.com>
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <ceres/ceres.h>

#include <memory>
#include <vector>

#include "factor_optimizer/factor/vertex.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

//////////////////////////////////// Edge //////////////////////////////////////

class Edge : public ceres::CostFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(Edge)

  explicit Edge(uint64_t id) : id_(id) {}
  ~Edge() override {}

  void SetId(uint64_t id) { id_ = id; }
  uint64_t GetId() const { return id_; }

  void SetOutlier(bool outlier = true) { is_outlier_ = outlier; }
  bool IsOutlier() const { return is_outlier_; }

  void SetResidual(const Eigen::Matrix<double, -1, 1>& residual) {
    residual_ = residual;
  }
  const Eigen::Matrix<double, -1, 1>& GetResidual() const { return residual_; }

  void SetInfomation(const Eigen::Matrix<double, -1, -1, 1>& infomation) {
    sqrt_infomation_ = Eigen::LLT<Eigen::Matrix<double, -1, -1, 1>>(infomation)
                           .matrixL()
                           .transpose();
  }
  const Eigen::Matrix<double, -1, -1, 1>& GetSqrtInfomation() const {
    return sqrt_infomation_;
  }

  void SetJacobians(
      const std::vector<Eigen::Matrix<double, -1, -1, 1>>& jacobians) {
    jacobians_ = jacobians;
  }
  const std::vector<Eigen::Matrix<double, -1, -1, 1>>& GetJacobians() const {
    return jacobians_;
  }

  void AddVertex(const std::shared_ptr<Vertex>& vertex) {
    vertexs_.emplace_back(vertex);
  }
  void SetVertexs(const std::vector<std::shared_ptr<Vertex>>& vertexs) {
    vertexs_ = vertexs;
  }
  const std::vector<std::shared_ptr<Vertex>>& GetVertexs() const {
    return vertexs_;
  }

  void SetLossFunction(const std::shared_ptr<ceres::LossFunction>& ptr) {
    loss_function_ = ptr;
  }
  std::shared_ptr<ceres::LossFunction> GetLossFunction() const {
    return loss_function_;
  }

  double Chi2() const { return residual_.transpose() * residual_; }

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const final {
    bool status = InnerEvaluate(parameters, residuals, jacobians);
    int param_size = parameter_block_sizes().size();
    Eigen::Map<Eigen::Matrix<double, -1, 1>> tp(residuals, num_residuals());
    residual_ = tp;
    if (jacobians) {
      for (size_t i = 0; i < jacobians_.size(); ++i) {
        Eigen::Map<Eigen::Matrix<double, -1, -1, 1>> tp(
            jacobians[i], num_residuals(), parameter_block_sizes()[i]);
        jacobians_[i] = tp;
      }
    }

    return status;
  }

 protected:
  virtual bool InnerEvaluate(double const* const* parameters, double* residuals,
                             double** jacobians) const = 0;

 protected:
  uint64_t id_;              // unique id for this edge
  bool is_outlier_ = false;  // if outlier, not used in optimization

  // optim core parameter
  mutable Eigen::Matrix<double, -1, 1> residual_;
  mutable std::vector<Eigen::Matrix<double, -1, -1, 1>> jacobians_;
  Eigen::Matrix<double, -1, -1, 1> sqrt_infomation_;

  // associated vertexs
  std::vector<std::shared_ptr<Vertex>> vertexs_;

  // loss function
  std::shared_ptr<ceres::LossFunction> loss_function_;
};

/////////////////////// EdgeBase (template derived) ////////////////////////////

template <int kNumResiduals, int... Ns>
class EdgeBase : public Edge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(EdgeBase);

  static_assert(kNumResiduals > 0 || kNumResiduals == ceres::DYNAMIC,
                "Cost functions must have at least one residual block.");
  static_assert(ceres::internal::StaticParameterDims<Ns...>::kIsValid,
                "Invalid parameter block dimension detected. Each parameter "
                "block dimension must be bigger than zero.");

  explicit EdgeBase(uint64_t id) : Edge(id) {
    set_num_residuals(kNumResiduals);
    *mutable_parameter_block_sizes() = std::vector<int32_t>{Ns...};

    int dim = num_residuals();
    int param_size = parameter_block_sizes().size();
    residual_ = Eigen::Matrix<double, -1, 1>(dim);
    residual_.setZero();
    sqrt_infomation_ = Eigen::Matrix<double, -1, -1, 1>(dim, dim);
    sqrt_infomation_.setIdentity();
    jacobians_.reserve(param_size);
    for (size_t i = 0; i < param_size; ++i) {
      jacobians_.emplace_back(
          Eigen::Matrix<double, -1, -1, 1>(dim, parameter_block_sizes()[i]));
      jacobians_.back().setZero();
    }
  }
  ~EdgeBase() override {}

  bool InnerEvaluate(double const* const* parameters, double* residuals,
                     double** jacobians) const override = 0;
};

//////////////////// AutoDiffEdgeBase (template derived) ///////////////////////

template <typename CostFunctor, int kNumResiduals, int... Ns>
class AutoDiffEdgeBase : public Edge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(AutoDiffEdgeBase)

  static_assert(kNumResiduals > 0 || kNumResiduals == ceres::DYNAMIC,
                "Cost functions must have at least one residual block.");
  static_assert(ceres::internal::StaticParameterDims<Ns...>::kIsValid,
                "Invalid parameter block dimension detected. Each parameter "
                "block dimension must be bigger than zero.");
  using ParameterDims = ceres::internal::StaticParameterDims<Ns...>;

  explicit AutoDiffEdgeBase(uint64_t id) : Edge(id) {
    set_num_residuals(kNumResiduals);
    *mutable_parameter_block_sizes() = std::vector<int32_t>{Ns...};

    int dim = num_residuals();
    int param_size = parameter_block_sizes().size();
    residual_ = Eigen::Matrix<double, -1, 1>(dim);
    residual_.setZero();
    sqrt_infomation_ = Eigen::Matrix<double, -1, -1, 1>(dim, dim);
    sqrt_infomation_.setIdentity();
    jacobians_.reserve(param_size);
    for (size_t i = 0; i < param_size; ++i) {
      jacobians_.emplace_back(
          Eigen::Matrix<double, -1, -1, 1>(dim, parameter_block_sizes()[i]));
      jacobians_.back().setZero();
    }
  }
  ~AutoDiffEdgeBase() override {}

  bool InnerEvaluate(double const* const* parameters, double* residuals,
                     double** jacobians) const override {
    if (!jacobians) {
      return ceres::internal::VariadicEvaluate<ParameterDims>(
          *functor_, parameters, residuals);
    }
    return ceres::internal::AutoDifferentiate<ParameterDims>(
        *functor_, parameters, this->num_residuals(), residuals, jacobians);
  }

 protected:
  std::unique_ptr<CostFunctor> functor_;
};

}  // namespace localization
}  // namespace senseAD
