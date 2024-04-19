/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "factor_optimizer/factor_optimizer.hpp"

#include <utility>
#include <vector>

#include "factor_optimizer/factor/edge.hpp"
#include "factor_optimizer/factor/vertex.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

FactorOptimizer::FactorOptimizer() {
  // If TAKE_OWNERSHIP is passed, the called object takes ownership of the
  // pointer argument, and will call delete on it upon completion.
  problem_options_.local_parameterization_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problem_options_.loss_function_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problem_options_.cost_function_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;

  // disable line search during trust region optimization
  solver_options_.max_num_line_search_step_size_iterations = 0;
}

void FactorOptimizer::SetNumIters(int iter_num) {
  solver_options_.max_num_iterations = iter_num;
}

void FactorOptimizer::SetLinearSolverType(ceres::LinearSolverType type) {
  solver_options_.linear_solver_type = type;
}

bool FactorOptimizer::AddVertex(const Vertex::Ptr& vertex) {
  if (vertexs_.find(vertex->GetId()) != vertexs_.end()) {
    LC_LERROR(OPTIM) << "vertex id: " << vertex->GetId() << " has existed.";
    return false;
  }
  vertexs_.insert(std::make_pair(vertex->GetId(), vertex));
  return true;
}

bool FactorOptimizer::AddEdge(const Edge::Ptr& edge) {
  if (edges_.find(edge->GetId()) != edges_.end()) {
    LC_LERROR(OPTIM) << "edge id: " << edge->GetId() << " has existed.";
    return false;
  }
  edges_.insert(std::make_pair(edge->GetId(), edge));
  return true;
}

bool FactorOptimizer::GetVertex(uint64_t id, Vertex::Ptr* vertex) const {
  if (!vertex) {
    LC_LERROR(OPTIM) << "input nullptr.";
    return false;
  }
  auto iter = vertexs_.find(id);
  if (iter == vertexs_.end()) {
    LC_LERROR(OPTIM) << "vertex find failed, id: " << id;
    return false;
  }
  *vertex = iter->second;
  return true;
}

const std::unordered_map<uint64_t, Vertex::Ptr>& FactorOptimizer::GetVertexs()
    const {
  return vertexs_;
}

std::unordered_map<uint64_t, Vertex::Ptr>& FactorOptimizer::GetVertexs() {
  return vertexs_;
}

bool FactorOptimizer::GetEdge(uint64_t id, Edge::Ptr* edge) const {
  if (!edge) {
    LC_LERROR(OPTIM) << "input nullptr.";
    return false;
  }
  auto iter = edges_.find(id);
  if (iter == edges_.end()) {
    LC_LERROR(OPTIM) << "edge find failed, id: " << id;
    return false;
  }
  *edge = iter->second;
  return true;
}

const std::unordered_map<uint64_t, Edge::Ptr>& FactorOptimizer::GetEdges()
    const {
  return edges_;
}

std::unordered_map<uint64_t, Edge::Ptr>& FactorOptimizer::GetEdges() {
  return edges_;
}

bool FactorOptimizer::GetEdgeCosts(bool get_mahalanobis_cost,
                                   std::vector<double>* costs) const {
  if (!problem_) {
    LC_LERROR(OPTIM) << "problem is nullptr.";
    return false;
  }
  if (!costs) {
    LC_LERROR(OPTIM) << "input costs is nullptr.";
    return false;
  }
  costs->clear();
  costs->reserve(edges_.size());
  for (auto& item : edges_) {
    auto& edge = item.second;
    if (edge->IsOutlier()) continue;
    const auto& vertexs = edge->GetVertexs();
    if (vertexs.empty()) continue;
    const Eigen::Matrix<double, -1, 1>& residual = edge->GetResidual();
    if (get_mahalanobis_cost) {
      costs->emplace_back(residual.norm());
    } else {
      const Eigen::Matrix<double, -1, -1, 1>& sqrt_infomation =
          edge->GetSqrtInfomation();
      Eigen::Matrix<double, -1, 1> no_info_residual(residual.rows());
      no_info_residual.noalias() = sqrt_infomation.inverse() * residual;
      costs->emplace_back(no_info_residual.norm());
    }
  }
  return true;
}

bool FactorOptimizer::GetHessian(int param_block_index, bool normalize,
                                 Eigen::MatrixXd* hessian) const {
  if (!problem_) {
    LC_LERROR(OPTIM) << "problem is nullptr.";
    return false;
  }
  if (!hessian) {
    LC_LERROR(OPTIM) << "input hessian is nullptr.";
    return false;
  }
  if (edges_.empty()) {
    LC_LERROR(OPTIM) << "edges is empty.";
    return false;
  }
  if (param_block_index < 0 ||
      param_block_index >=
          edges_.begin()->second->parameter_block_sizes().size()) {
    LC_LERROR(OPTIM) << "input param block index is out of range.";
    return false;
  }
  // TODO(xx): now assume all edges are same, but actually a problem may
  // has different edges with different param blocks
  int param_size =
      edges_.begin()->second->parameter_block_sizes()[param_block_index];
  Eigen::Matrix<double, -1, -1, 1> whole_hessian(param_size, param_size);
  whole_hessian.setZero();
  int cnt = 0;
  for (auto& item : edges_) {
    auto& edge = item.second;
    if (edge->IsOutlier()) continue;
    const auto& vertexs = edge->GetVertexs();
    if (vertexs.empty()) continue;
    const auto& jacobians = edge->GetJacobians();
    if (jacobians.empty() || param_block_index >= jacobians.size()) continue;
    const Eigen::Matrix<double, -1, -1, 1>& jacobian =
        jacobians[param_block_index];
    if (jacobian.cols() != param_size) continue;
    Eigen::Matrix<double, -1, -1, 1> sub_hessian(param_size, param_size);
    if (normalize) {
      const Eigen::Matrix<double, -1, -1, 1>& sqrt_infomation =
          edge->GetSqrtInfomation();
      Eigen::Matrix<double, -1, -1, 1> no_info_jacobian(jacobian.rows(),
                                                        jacobian.cols());
      no_info_jacobian.noalias() = sqrt_infomation.inverse() * jacobian;
      sub_hessian.noalias() = no_info_jacobian.transpose() * no_info_jacobian;
    } else {
      sub_hessian.noalias() = jacobian.transpose() * jacobian;
    }
    whole_hessian += sub_hessian;
    ++cnt;
  }
  if (cnt <= 0) {
    LC_LERROR(OPTIM) << "no valid edge.";
    return false;
  }
  if (normalize) {
    hessian->noalias() = whole_hessian / cnt;
  } else {
    *hessian = whole_hessian;
  }

  return true;
}

bool FactorOptimizer::GetCovariance(int param_block_index,
                                    Eigen::MatrixXd* covariance) const {
  if (!problem_) {
    LC_LERROR(OPTIM) << "problem is nullptr.";
    return false;
  }
  if (problem_->NumResiduals() == 0) {
    LC_LERROR(OPTIM) << "problem has no residuals";
    return false;
  }
  if (problem_->NumResiduals() < problem_->NumParameters()) {
    LC_LERROR(OPTIM) << "problem is under determined";
    return false;
  }

  Eigen::MatrixXd hessian;
  bool status = GetHessian(param_block_index, true, &hessian);
  if (!status) {
    LC_LERROR(OPTIM) << "calculate hessian fail";
    return false;
  }

  std::vector<double> costs;
  status = GetEdgeCosts(false, &costs);
  if (!status) {
    LC_LERROR(OPTIM) << "calculate edge costs fail";
    return false;
  }
  if (costs.empty()) {
    LC_LERROR(OPTIM) << "no valid edge cost";
    return false;
  }

  if (hessian.determinant() < 1.e-5) {
    LC_LERROR(OPTIM) << "hessian's determinant is close to 0";
    return false;
  } else {
    double cost_sum = 0.0;
    for (const double& cost : costs) {
      cost_sum += cost;
    }
    double cost_mean = cost_sum / costs.size();
    double cost_var = 0.0;
    for (const double& cost : costs) {
      double delta_cost = cost - cost_mean;
      cost_var += delta_cost * delta_cost;
    }
    cost_var /= costs.size();
    covariance->noalias() = hessian.inverse() * cost_var;
  }
  return true;
}

bool FactorOptimizer::Optimize(bool verbose) {
  problem_.reset(new ceres::Problem(problem_options_));
  if (!problem_) {
    LC_LERROR(OPTIM) << "problem allocate failed.";
    return false;
  }

  // add vertexs
  for (auto& item : vertexs_) {
    auto& vertex = item.second;
    // add parameter block
    double* raw_param = vertex->RawParameter();
    problem_->AddParameterBlock(raw_param, vertex->GlobalSize(), vertex.get());

    // set lower/upper bound of parameters
    const auto& bound_upper_ids = vertex->GetBoundUpperIDs();
    for (size_t i = 0; i < bound_upper_ids.size(); ++i) {
      problem_->SetParameterUpperBound(raw_param, bound_upper_ids[i],
                                       vertex->GetBoundUpperValues()[i]);
    }
    const auto& bound_lower_ids = vertex->GetBoundLowerIDs();
    for (size_t i = 0; i < bound_lower_ids.size(); ++i) {
      problem_->SetParameterLowerBound(raw_param, bound_lower_ids[i],
                                       vertex->GetBoundLowerValues()[i]);
    }

    // set (subset) parameters constant
    if (vertex->IsFixed()) {
      problem_->SetParameterBlockConstant(raw_param);
    } else {
      for (const auto& constant_id : vertex->GetConstantIDs()) {
        problem_->SetParameterUpperBound(raw_param, constant_id, 1e-8);
        problem_->SetParameterLowerBound(raw_param, constant_id, -1e-8);
      }
    }
  }
  // add edges
  for (auto& item : edges_) {
    auto& edge = item.second;
    if (edge->IsOutlier()) continue;
    // add residual block
    const auto& vertexs = edge->GetVertexs();
    if (vertexs.empty()) continue;
    std::vector<double*> parameters;
    for (const auto& vertex : vertexs) {
      parameters.emplace_back(vertex->RawParameter());
    }
    problem_->AddResidualBlock(edge.get(), edge->GetLossFunction().get(),
                               parameters);
  }

  if (problem_->NumResiduals() < problem_->NumParameters()) {
    LC_LINFO_EVERY(OPTIM, 20) << "problem maybe under determined";
  }

  // solve the problem
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options_, problem_.get(), &summary);
  if (verbose) {
    LC_LDEBUG(OPTIM) << "optimization report: " << summary.BriefReport();
  }

  return true;
}

}  // namespace localization
}  // namespace senseAD
