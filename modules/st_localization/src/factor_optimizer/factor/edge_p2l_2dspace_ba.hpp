/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <memory>
#include <vector>

#include <Sophus/se3.hpp>

#include "factor_optimizer/factor/edge.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

// edge minimizing point-to-line distance by bundle-adjusting pose and
// homography
class EdgeP2L2DSpaceBA : public EdgeBase<2, 2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(EdgeP2L2DSpaceBA)

  EdgeP2L2DSpaceBA(uint64_t id, const Eigen::Vector2d& imgPt,
                   const Eigen::Vector2d& mPt1, const Eigen::Vector2d& mPt2,
                   const Eigen::Matrix2d& Pt_cov, const Eigen::Matrix3d& H_base,
                   const Eigen::Matrix3d& K_inv)
      : EdgeBase<2, 2, 1>(id),
        imgPt_(imgPt),
        mPt1_(mPt1),
        mPt2_(mPt2),
        H_base_(H_base),
        K_inv_(K_inv) {
    SetInfomation(Pt_cov.inverse());
  }

  bool InnerEvaluate(double const* const* parameters, double* residuals,
                     double** jacobians) const override {
    // car-center tranformation from optimized to predicted
    double y(*parameters[0]);
    double theta = *(parameters[0] + 1);

    // camera to ground delta pitch
    double h_pitch(*parameters[1]);
    // NOTE: actually roll of camera!
    Eigen::Matrix3d R_pitch;
    R_pitch << 1., 0., 0., 0., cos(h_pitch), -sin(h_pitch), 0., sin(h_pitch),
        cos(h_pitch);

    // BV point projection
    Eigen::Vector3d imgPt_homo(imgPt_(0), imgPt_(1), 1);
    Eigen::Matrix3d H_adapted = H_base_ * R_pitch * K_inv_;
    Eigen::Vector3d bvPt_homo = H_adapted * imgPt_homo;
    Eigen::Vector2d bvPt(bvPt_homo(0) / bvPt_homo(2),
                         bvPt_homo(1) / bvPt_homo(2));

    // bv point transformed into map coordinate
    double cosh = ceres::cos(theta);
    double sinh = ceres::sin(theta);
    Eigen::Matrix2d rotation;
    rotation << cosh, -sinh, sinh, cosh;
    Eigen::Vector2d mPt = rotation * bvPt + Eigen::Vector2d(0, y);

    // compute p2l residual
    Eigen::Vector2d pt1pt = mPt - mPt1_;
    Eigen::Vector2d pt1pt2 = mPt2_ - mPt1_;
    Eigen::Vector2d norm_pt1pt2 = pt1pt2 / pt1pt2.norm();
    Eigen::Matrix<double, 1, 1> project_dist = pt1pt.transpose() * norm_pt1pt2;
    Eigen::Vector2d pt1proj = norm_pt1pt2 * project_dist;
    Eigen::Vector2d ptproj = pt1proj - pt1pt;

    Eigen::Map<Eigen::Vector2d> residual(residuals);
    residual = sqrt_infomation_ * ptproj;

    if (jacobians) {
      // jacobian of residual w.r.t point in map coordinate
      Eigen::Matrix2d jac_res_mPt =
          norm_pt1pt2 * norm_pt1pt2.transpose() - Eigen::Matrix2d::Identity();

      if (jacobians[0]) {
        // jacobian of point in map coordinate w.r.t y and theta
        double dx_dlateral = 0.0;
        double dx_dtheta = -sinh * imgPt_(0) - cosh * imgPt_(1);
        double dy_dlateral = 1.0;
        double dy_dtheta = cosh * imgPt_(0) - sinh * imgPt_(1);
        Eigen::Matrix2d jac_mPt_yh;
        jac_mPt_yh << dx_dlateral, dx_dtheta, dy_dlateral, dy_dtheta;

        Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>>
            jacobian_res_yh(jacobians[0]);
        jacobian_res_yh = sqrt_infomation_ * jac_res_mPt * jac_mPt_yh;
      }

      if (jacobians[1]) {
        // jacobian of point in map w.r.t. bv coordinate
        Eigen::Matrix2d jac_mPt_bvPt = rotation;

        // jacobian of bv point w.r.t its homogenous form
        Eigen::Matrix<double, 2, 3> jac_bvPt_bvPtHomo;
        jac_bvPt_bvPtHomo.setZero();
        jac_bvPt_bvPtHomo(0, 0) = 1.0 / bvPt_homo(2);
        jac_bvPt_bvPtHomo(0, 2) = -bvPt_homo(0) / (bvPt_homo(2) * bvPt_homo(2));
        jac_bvPt_bvPtHomo(1, 1) = 1.0 / bvPt_homo(2);
        jac_bvPt_bvPtHomo(1, 2) = -bvPt_homo(1) / (bvPt_homo(2) * bvPt_homo(2));

        // jacobian of bv point in its homogenous form w.r.t. pitch in H
        Eigen::Matrix3d jac_R_pitch;
        jac_R_pitch << 1., 0., 0., 0., -sin(h_pitch), -cos(h_pitch), 0.,
            cos(h_pitch), -sin(h_pitch);

        Eigen::Vector3d jac_vbPtHomo_Hpitch =
            H_base_ * jac_R_pitch * K_inv_ * imgPt_homo;

        Eigen::Map<Eigen::Matrix<double, 2, 1>> jacobian_res_Hpitch(
            jacobians[1]);
        jacobian_res_Hpitch = sqrt_infomation_ * jac_res_mPt * jac_mPt_bvPt *
                              jac_bvPt_bvPtHomo * jac_vbPtHomo_Hpitch;
      }
    }

    return true;
  }

 private:
  Eigen::Vector2d imgPt_;   // percepted image point
  Eigen::Vector2d mPt1_;    // map 2d point1, has been transform to car-center
  Eigen::Vector2d mPt2_;    // map 2d point2, has been transform to car-center
  Eigen::Matrix3d H_base_;  // H base term built from T_gound_camera extrinsic
  Eigen::Matrix3d K_inv_;   // camera intrinsic inversion
};

}  // namespace localization
}  // namespace senseAD
