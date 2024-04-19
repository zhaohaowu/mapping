/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Zhu Kaiying <zhukaiying@senseauto.com>
 */

#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <memory>
#include <vector>

#include <Sophus/se2.hpp>
#include <Sophus/so2.hpp>

#include "factor_optimizer/factor/edge.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

///////////////////////////// EdgeP2P2DSpace ///////////////////////////////////

class EdgeP2P2DSpace : public EdgeBase<2, 2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(EdgeP2P2DSpace)

  EdgeP2P2DSpace(uint64_t id, const Eigen::Vector2d& Pt,
                 const Eigen::Vector2d& mPt, const Eigen::Matrix2d& Pt_cov)
      : EdgeBase<2, 2, 1>(id), Pt_(Pt), mPt_(mPt) {
    SetInfomation(Pt_cov.inverse());
  }

  void Reset(uint64_t id, const Eigen::Vector2d& Pt, const Eigen::Vector2d& mPt,
             const Eigen::Matrix2d& Pt_cov) {
    // edge base
    SetId(id);
    SetOutlier(false);
    SetInfomation(Pt_cov.inverse());
    vertexs_.clear();
    loss_function_ = nullptr;
    // derived
    Pt_ = Pt;
    mPt_ = mPt;
  }

  bool InnerEvaluate(double const* const* parameters, double* residuals,
                     double** jacobians) const override {
    // car-center tranformation from optimized to predicted
    double y = *(parameters[0] + 0);
    double theta = *(parameters[0] + 1);
    double x = *parameters[1];

    double cosh = ceres::cos(theta);
    double sinh = ceres::sin(theta);
    rotation_ << cosh, -sinh, sinh, cosh;
    new_Pt_.noalias() = rotation_ * Pt_ + Eigen::Vector2d(x, y);

    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual.noalias() = sqrt_infomation_ * (new_Pt_ - mPt_);

    if (jacobians) {
      if (jacobians[0]) {
        // jacobian of point w.r.t y and theta
        Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>>
            jacobian_res_yh(jacobians[0]);

        double dx_dlateral = 0.0;
        double dx_dtheta = -sinh * Pt_(0) - cosh * Pt_(1);
        double dy_dlateral = 1.0;
        double dy_dtheta = cosh * Pt_(0) - sinh * Pt_(1);
        jac_mPt_yh_ << dx_dlateral, dx_dtheta, dy_dlateral, dy_dtheta;
        jacobian_res_yh.noalias() = sqrt_infomation_ * jac_mPt_yh_;
      }

      if (jacobians[1]) {
        // jacobian of point w.r.t x
        Eigen::Map<Eigen::Vector2d> jacobian_res_x(jacobians[1]);
        double dx_dlong = 1.0;
        double dy_dlong = 0.0;
        jac_mPt_x_ << dx_dlong, dy_dlong;
        jacobian_res_x.noalias() = sqrt_infomation_ * jac_mPt_x_;
      }
    }

    return true;
  }

 private:
  Eigen::Vector2d Pt_;   // percepted 2d point in Bird View
  Eigen::Vector2d mPt_;  // map 2d point in Bird View
  // update terms during iteration, pre-allocated memory
  mutable Eigen::Matrix2d rotation_;
  mutable Eigen::Matrix2d jac_mPt_yh_;
  mutable Eigen::Vector2d jac_mPt_x_;
  mutable Eigen::Vector2d new_Pt_;
};

}  // namespace localization
}  // namespace senseAD
