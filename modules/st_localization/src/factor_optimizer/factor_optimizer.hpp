/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once
#include <ceres/ceres.h>

#include <memory>
#include <unordered_map>
#include <vector>

#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

class Vertex;
class Edge;

// class holds the factor optimizer, contains add vertexs/edges, construct
// problem and optimize it
class FactorOptimizer {
 public:
  DEFINE_SMART_PTR(FactorOptimizer)

  FactorOptimizer();
  ~FactorOptimizer() = default;

  /////////////////////////// set & get interface ////////////////////////////

  // @brief: set problem options param (support extended)
  void SetNumIters(int iter_num);
  void SetLinearSolverType(ceres::LinearSolverType type);

  // @brief: add and get vertex/dege
  bool AddVertex(const std::shared_ptr<Vertex>& vertex);
  bool AddEdge(const std::shared_ptr<Edge>& edge);

  bool GetVertex(uint64_t id, std::shared_ptr<Vertex>* vertex) const;
  const std::unordered_map<uint64_t, std::shared_ptr<Vertex>>& GetVertexs()
      const;
  std::unordered_map<uint64_t, std::shared_ptr<Vertex>>& GetVertexs();

  bool GetEdge(uint64_t id, std::shared_ptr<Edge>* edge) const;
  const std::unordered_map<uint64_t, std::shared_ptr<Edge>>& GetEdges() const;
  std::unordered_map<uint64_t, std::shared_ptr<Edge>>& GetEdges();

  bool GetCovariance(int param_block_index, Eigen::MatrixXd* covariance) const;

  //////////////////////// optimize related interface ////////////////////////

  bool Optimize(bool verbose = false);

 private:
  bool GetEdgeCosts(bool get_mahalanobis_cost,
                    std::vector<double>* costs) const;

  bool GetHessian(int param_block_index, bool normalize,
                  Eigen::MatrixXd* hessian) const;

 private:
  // problem and solver option params
  ceres::Solver::Options solver_options_;
  ceres::Problem::Options problem_options_;

  // optimized problem
  std::unique_ptr<ceres::Problem> problem_ = nullptr;

  // factor graphs
  std::unordered_map<uint64_t, std::shared_ptr<Vertex>> vertexs_;
  std::unordered_map<uint64_t, std::shared_ptr<Edge>> edges_;
};

}  // namespace localization
}  // namespace senseAD
