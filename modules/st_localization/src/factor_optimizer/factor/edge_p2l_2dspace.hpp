/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
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

template <typename T>
inline T Point2LineDist(const Eigen::Matrix<T, 2, 1>& p,
                        const Eigen::Matrix<T, 2, 1>& q1,
                        const Eigen::Matrix<T, 2, 1>& q2) {
  Eigen::Matrix<T, 2, 1> ptpt1 = p - q1;
  Eigen::Matrix<T, 2, 1> ptpt2 = p - q2;
  Eigen::Matrix<T, 2, 1> pt1pt2 = q1 - q2;
  T pt1pt2_norm = pt1pt2.norm();
  T cross = ptpt1.x() * ptpt2.y() - ptpt2.x() * ptpt1.y();
  return ceres::abs(cross) / pt1pt2_norm;
}

template <typename T>
inline Eigen::Matrix<T, 2, 1> Point2LineVector(
    const Eigen::Matrix<T, 2, 1>& p, const Eigen::Matrix<T, 2, 1>& q1,
    const Eigen::Matrix<T, 2, 1>& q2) {
  Eigen::Matrix<T, 2, 1> line_q1q2 = q2 - q1;
  Eigen::Matrix<T, 2, 1> line_q1p = p - q1;
  line_q1q2 = line_q1q2 / line_q1q2.norm();
  Eigen::Matrix<T, 1, 1> project_dist = line_q1p.transpose() * line_q1q2;
  Eigen::Matrix<T, 2, 1> line_q1proj = line_q1q2 * project_dist;
  Eigen::Matrix<T, 2, 1> line_pproj = line_q1proj - line_q1p;
  return line_pproj;
}

///////////////////////////// EdgeP2L2DSpace ///////////////////////////////////

class EdgeP2L2DSpace : public EdgeBase<2, 2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(EdgeP2L2DSpace)

  EdgeP2L2DSpace(uint64_t id, const Eigen::Vector2d& Pt,
                 const Eigen::Vector2d& mPt1, const Eigen::Vector2d& mPt2,
                 const Eigen::Matrix2d& Pt_cov)
      : EdgeBase<2, 2>(id), Pt_(Pt), mPt1_(mPt1), mPt2_(mPt2) {
    SetInfomation(Pt_cov.inverse());
    MakeConstant();
  }

  void Reset(uint64_t id, const Eigen::Vector2d& Pt,
             const Eigen::Vector2d& mPt1, const Eigen::Vector2d& mPt2,
             const Eigen::Matrix2d& Pt_cov) {
    // edge base
    SetId(id);
    SetOutlier(false);
    SetInfomation(Pt_cov.inverse());
    vertexs_.clear();
    loss_function_ = nullptr;
    // derived
    Pt_ = Pt;
    mPt1_ = mPt1;
    mPt2_ = mPt2;
    MakeConstant();
  }

  bool InnerEvaluate(double const* const* parameters, double* residuals,
                     double** jacobians) const override {
    // car-center tranformation from optimized to predicted
    double y(*parameters[0]);
    double theta = *(parameters[0] + 1);

    double cosh = ceres::cos(theta);
    double sinh = ceres::sin(theta);
    rotation_ << cosh, -sinh, sinh, cosh;
    new_Pt_.noalias() = rotation_ * Pt_ + Eigen::Vector2d(0, y);
    pt1pt_.noalias() = new_Pt_ - mPt1_;
    pt1proj_.noalias() = (pt1pt_.transpose() * norm_pt1pt2_) * norm_pt1pt2_;

    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual.noalias() = sqrt_infomation_ * (pt1proj_ - pt1pt_);

    if (jacobians && jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> jacobian_res_yh(
          jacobians[0]);
      // jacobian of point w.r.t y and theta
      double dx_dlateral = 0.0;
      double dx_dtheta = -sinh * Pt_(0) - cosh * Pt_(1);
      double dy_dlateral = 1.0;
      double dy_dtheta = cosh * Pt_(0) - sinh * Pt_(1);
      jac_mPt_yh_ << dx_dlateral, dx_dtheta, dy_dlateral, dy_dtheta;
      jacobian_res_yh.noalias() = sqrt_infomation_ * jac_res_mPt_ * jac_mPt_yh_;
    }

    return true;
  }

 private:
  void MakeConstant() {
    Eigen::Vector2d pt1pt2 = mPt2_ - mPt1_;
    norm_pt1pt2_ = pt1pt2 / pt1pt2.norm();
    // jacobian of residual w.r.t point
    jac_res_mPt_ =
        norm_pt1pt2_ * norm_pt1pt2_.transpose() - Eigen::Matrix2d::Identity();
  }

 private:
  Eigen::Vector2d Pt_;    // percepted 2d point in Bird View
  Eigen::Vector2d mPt1_;  // map 2d point1, has been transform to car-center
  Eigen::Vector2d mPt2_;  // map 2d point2, has been transform to car-center
  // constant terms during iteration, pre-allocated memory
  Eigen::Vector2d norm_pt1pt2_;
  Eigen::Matrix2d jac_res_mPt_;
  // update terms during iteration
  mutable Eigen::Matrix2d rotation_;
  mutable Eigen::Matrix2d jac_mPt_yh_;
  mutable Eigen::Vector2d new_Pt_;
  mutable Eigen::Vector2d pt1pt_;
  mutable Eigen::Vector2d pt1proj_;
};

/////////////////////// AutoDiffEdgeP2L2DSpace /////////////////////////////////

struct P2L2DSpaceCostFunction {
  P2L2DSpaceCostFunction(const Eigen::Vector2d& Pt, const Eigen::Vector2d& mPt1,
                         const Eigen::Vector2d& mPt2,
                         const Eigen::Matrix2d& Pt_cov)
      : Pt_(Pt), mPt1_(mPt1), mPt2_(mPt2) {
    sqrt_info_ =
        Eigen::LLT<Eigen::Matrix2d>(Pt_cov.inverse()).matrixL().transpose();
  }

  template <typename T>
  bool operator()(const T* parameters, T* residuals) const {
    // car-center tranformation from optimized to predicted
    T y = static_cast<T>(parameters[0]);
    T theta = static_cast<T>(parameters[1]);

    T cosh = static_cast<T>(ceres::cos(theta));
    T sinh = static_cast<T>(ceres::sin(theta));
    Eigen::Matrix<T, 2, 2> rotation;
    rotation << cosh, -sinh, sinh, cosh;

    Eigen::Matrix<T, 2, 1> mPt =
        rotation * Pt_.template cast<T>() + Eigen::Matrix<T, 2, 1>(0, y);
    Eigen::Matrix<T, 2, 1> TmPt1 = mPt1_.template cast<T>();
    Eigen::Matrix<T, 2, 1> TmPt2 = mPt2_.template cast<T>();

    Eigen::Map<Eigen::Matrix<T, 2, 1>> residual(residuals);
    residual = sqrt_info_ * Point2LineVector(mPt, TmPt1, TmPt2);

    return true;
  }

  Eigen::Vector2d Pt_;    // percepted 2d point in Bird View
  Eigen::Vector2d mPt1_;  // map 2d point1, has been transform to car-center
  Eigen::Vector2d mPt2_;  // map 2d point2, has been transform to car-center
  Eigen::Matrix2d sqrt_info_;  // sqrt information
};

class AutoDiffEdgeP2L2DSpace
    : public AutoDiffEdgeBase<P2L2DSpaceCostFunction, 2, 2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(AutoDiffEdgeP2L2DSpace)

  AutoDiffEdgeP2L2DSpace(uint64_t id, const Eigen::Vector2d& Pt,
                         const Eigen::Vector2d& mPt1,
                         const Eigen::Vector2d& mPt2,
                         const Eigen::Matrix2d& Pt_cov)
      : AutoDiffEdgeBase<P2L2DSpaceCostFunction, 2, 2>(id) {
    functor_.reset(new P2L2DSpaceCostFunction(Pt, mPt1, mPt2, Pt_cov));
    SetInfomation(Pt_cov.inverse());
  }
};

}  // namespace localization
}  // namespace senseAD
