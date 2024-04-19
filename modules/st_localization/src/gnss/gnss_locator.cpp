/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */
#include "gnss/gnss_locator.hpp"

#include <algorithm>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "ad_time/ad_time.hpp"
#include "common/coordinate_converter.hpp"
#include "common/msf_common.hpp"
#include "common/transform_config.hpp"
#include "eval/evaluator_gnss_fault.hpp"
#include "gnss/gnss_fault_diagnosis.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t GNSSLocator::Init(const LocalizationParam& param) {
  if (param.gnss_param.enable_fault_diagnosis) {
    // init gnss fault dignostor
    gnss_fault_diagnostor_ = std::make_unique<GNSSFaultDiagnostor>();
    if (nullptr == gnss_fault_diagnostor_) {
      LC_LERROR(GNSS) << "failed to create GNSSFaultDiagnostor";
      return LOC_LOCALIZATION_ERROR;
    }
    // init gnss fault evaluator
    if (param.ci_param.enable_evaluation) {
      std::string setting_type = "GNSS_FAULT";
      gnss_fault_evaluator_ = std::make_shared<EvaluatorGnssFault>();
      if (nullptr == gnss_fault_evaluator_) {
        LC_LERROR(GNSS) << "Failed to create gnss fault evaluator.";
        return LOC_ALLOC_MEMORY_FAILED;
      }
      if (gnss_fault_evaluator_->Init(
              param.ci_param.results_save_dir, setting_type,
              param.ci_param.testcase_id) != LOC_SUCCESS) {
        LC_LERROR(GNSS) << "Failed to init gnss fault evaluator.";
        return LOC_LOCALIZATION_ERROR;
      }
    }
  }
  LC_LINFO(LOCALIZATION) << "Create GNSSLocator done";
  return LOC_SUCCESS;
}

adLocStatus_t GNSSLocator::SetState(const NavState& nav_state) {
  init_pose_state_ = nav_state;
  pose_state_ = init_pose_state_;
  return LOC_SUCCESS;
}

adLocStatus_t GNSSLocator::SetOdomState(const OdomState& odom_state) {
  init_odom_state_ = odom_state;
  return LOC_SUCCESS;
}

adLocStatus_t GNSSLocator::GetState(NavState* nav_state, double* confidence) {
  if (nullptr == nav_state) {
    LC_LERROR(GNSS) << "nullptr";
    return LOC_NULL_PTR;
  }
  *nav_state = pose_state_;
  // output location confidence
  if (confidence) *confidence = confidence_;
  return LOC_SUCCESS;
}

adLocStatus_t GNSSLocator::SwitchOriginProc() {
  return gnss_fault_diagnostor_->SwitchOriginProc();
}

adLocStatus_t GNSSLocator::Process(uint64_t timestamp,
                                   std::shared_ptr<Gnss> raw_gnss) {
  if (nullptr == raw_gnss) {
    LC_LERROR(GNSS) << "nullptr";
    return LOC_NULL_PTR;
  }

  pose_state_.state_source = GNSS;
  pose_state_.timestamp = timestamp;

  // pose
  // gnss antenna position (lat, lon, height)
  Eigen::Vector3d gnss_position;
  gnss_position << raw_gnss->position.lat, raw_gnss->position.lon,
      raw_gnss->position.height;
  // solve heading (in NED)
  double heading = 0.0, heading_std = 1e3;
  SolveHeading(raw_gnss, &heading, &heading_std);
  Eigen::Vector3d ypr(heading, 0, 0);
  Eigen::Vector3d t;
  t << gnss_position(0) * d2r, gnss_position(1) * d2r, gnss_position(2);
  // pose_state_.pose = SE3(SO3::exp(ypr), t); old Sophus
  pose_state_.pose = SE3d(SO3d::exp(ypr), t);

  // velocity
  Eigen::Vector3d gnss_velocity;
  gnss_velocity << raw_gnss->linear_velocity.x, raw_gnss->linear_velocity.y,
      raw_gnss->linear_velocity.z;
  pose_state_.linear_speed = gnss_velocity;

  // gnss fault diagnosis, the purpose is to adjust the covariance adaptively
  GnssFaultData gf_data;
  Eigen::Vector3d gnss_scale_factors = Eigen::Vector3d::Ones();
  if (gnss_fault_diagnostor_) {
    gnss_fault_diagnostor_->Process(timestamp, *raw_gnss, init_odom_state_,
                                    init_pose_state_);
    // set gnss fault data for eval
    gf_data.fault_type = gnss_fault_diagnostor_->GetGnssDiagnostictResult();
    gf_data.gnss_dr_rate = gnss_fault_diagnostor_->GetFluctuationChecksResult();
    gf_data.gnss_dr_consistency =
        gnss_fault_diagnostor_->GetConsistencyChecksResult();
    gf_data.gnss_data = *raw_gnss;

    gnss_scale_factors = gnss_fault_diagnostor_->GetScaleFactors();
  }
  if (gnss_fault_evaluator_) {
    gnss_fault_evaluator_->WriteResult(timestamp * kNanoSecToSec, gf_data);
  }

  // gnss measurement noise
  Eigen::Matrix<double, 6, 6> pose_cov;
  pose_cov.setZero();
  // Note parser order (lon, lat, height) to (lat, lon, height)
  // TODO(wangxiaofeng): I think current uncertainty may be over confident
  pose_cov(0, 0) = gnss_scale_factors[1] * raw_gnss->position_std_dev.y *
                   raw_gnss->position_std_dev.y * 10;
  pose_cov(1, 1) = gnss_scale_factors[0] * raw_gnss->position_std_dev.x *
                   raw_gnss->position_std_dev.x * 10;
  pose_cov(2, 2) = gnss_scale_factors[2] * raw_gnss->position_std_dev.z *
                   raw_gnss->position_std_dev.z * 10;
  pose_cov(3, 3) = heading_std * heading_std;
  pose_state_.pose_cov = pose_cov;

  LC_LDEBUG(GNSS) << "Gnss process: " << timestamp << "/"
                  << Time::ToString(timestamp)
                  << ", status: " << GetGnssStatusStr(raw_gnss->status)
                  << ", 4dof std: " << std::sqrt(pose_cov(0, 0)) << " "
                  << std::sqrt(pose_cov(1, 1)) << " "
                  << std::sqrt(pose_cov(2, 2)) << " " << heading_std;

  // consider init pose state is invalid, don't check position error
  if (init_pose_state_.timestamp == 0) {
    confidence_ = 0.3;
    return LOC_SUCCESS;
  }

  // calculate predicted gnss antenna position, consider lever-arm compensate
  PointLLH_t lla(gnss_position(1), gnss_position(0), gnss_position(2));
  PointENU_t enu;
  CoordinateConverter::GetInstance()->LLA2ENU(lla, &enu);
  Eigen::Vector3d gnss_enu(enu.x, enu.y, enu.z);

  Eigen::Vector3d llarm_RFU, llarm_FRD;
  llarm_FRD = TransformConfig::GetLeverArm();
  llarm_RFU << llarm_FRD(1), llarm_FRD(0), -llarm_FRD(2);
  Eigen::Matrix4d Tvb = TransformConfig::GetTvb().matrix();
  Eigen::Vector3d llarm_offset =
      init_pose_state_.pose.so3().matrix() *
      (Tvb.block<3, 3>(0, 0) * llarm_RFU + Tvb.block<3, 1>(0, 3));
  // calculate gnss error compare to predicted
  Eigen::Vector3d predict_gnss_position =
      init_pose_state_.pose.translation() + llarm_offset;
  Eigen::Vector3d position_err = gnss_enu - predict_gnss_position;
  double position_err_norm = position_err.norm();

  // calculate mah distance for position
  Eigen::Matrix3d predict_cov = init_pose_state_.pose_cov.topLeftCorner(3, 3);
  Eigen::Matrix3d gnss_cov = pose_cov.topLeftCorner(3, 3);
  std::swap(gnss_cov(0, 0), gnss_cov(1, 1));
  chi_square_ = position_err.transpose() * (predict_cov + gnss_cov).inverse() *
                position_err;
  if (chi_square_ > 100.0) {
    LC_LDEBUG(GNSS) << "Gnss has large chi square: " << chi_square_;
  }

  // output gnss localization confidence
  if (raw_gnss->status == GnssStatus::RTK_INTEGER && position_err_norm < 0.3) {
    confidence_ = 1.0;
  } else if ((raw_gnss->status == GnssStatus::RTK_INTEGER ||
              raw_gnss->status == GnssStatus::RTK_FLOAT ||
              raw_gnss->status == GnssStatus::PPP ||
              raw_gnss->status == GnssStatus::PSRDIFF ||
              raw_gnss->status == GnssStatus::SINGLE) &&
             position_err_norm < 0.6) {
    confidence_ = 0.7;
  } else {
    confidence_ = 0.3;
  }

  return LOC_SUCCESS;
}

double GNSSLocator::GetLocatorChiSquare() const { return chi_square_; }

adLocStatus_t GNSSLocator::ProcessForGtMode(const uint64_t timestamp,
                                            std::shared_ptr<Gnss> raw_gnss,
                                            NavState* nav_state) {
  if (nullptr == raw_gnss) {
    LC_LERROR(GNSS) << "nullptr";
    return LOC_NULL_PTR;
  }

  nav_state->state_source = GNSS;
  nav_state->timestamp = timestamp;

  // only consider convert position to enu coordinate
  PointENU_t enu;
  CoordinateConverter::GetInstance()->LLA2ENU(raw_gnss->position, &enu);
  nav_state->pose = SE3d(SO3d(), Eigen::Vector3d(enu.x, enu.y, enu.z));

  // only consider original gnss measurement noise
  Eigen::Matrix<double, 6, 6> pose_cov;
  pose_cov.setZero();
  // Note parser order (lon, lat, height) to (lat, lon, height)
  pose_cov(0, 0) =
      raw_gnss->position_std_dev.y * raw_gnss->position_std_dev.y * 10;
  pose_cov(1, 1) =
      raw_gnss->position_std_dev.x * raw_gnss->position_std_dev.x * 10;
  pose_cov(2, 2) =
      raw_gnss->position_std_dev.z * raw_gnss->position_std_dev.z * 10;
  nav_state->pose_cov = pose_cov;

  return LOC_SUCCESS;
}

adLocStatus_t GNSSLocator::SolveHeading(std::shared_ptr<Gnss> raw_gnss,
                                        double* heading, double* heading_std) {
  static Gnss last_gnss;
  static bool last_gnss_updated = false;

  // calculate heading from gnss position diff
  double heading_gnss_p = 0.0;
  adLocStatus_t p_status = LOC_INVALID;
  if (last_gnss_updated)
    p_status =
        SolveHeadingByGnssPosition(last_gnss, *raw_gnss, &heading_gnss_p);
  last_gnss = *raw_gnss;
  last_gnss_updated = true;

  // calculate heading from gnss velocity
  double heading_gnss_v = 0.0;
  double heading_gnss_v_std = 0.0;
  auto v_status = SolveHeadingByGnssVelocity(raw_gnss, &heading_gnss_v,
                                             &heading_gnss_v_std);
  if (v_status != LOC_SUCCESS) return LOC_LOCALIZATION_ERROR;

  *heading = heading_gnss_v;
  *heading_std = heading_gnss_v_std;
  if (p_status != LOC_SUCCESS) {
    *heading_std *= 1.5;
  } else {
    *heading_std =
        1.0 * (*heading_std) + 0.3 * std::fabs(heading_gnss_p - heading_gnss_v);
  }

  return LOC_SUCCESS;
}

adLocStatus_t GNSSLocator::SolveHeadingByGnssPosition(const Gnss& last_gnss,
                                                      const Gnss& cur_gnss,
                                                      double* heading) {
  static auto CheckGnss = [](const Gnss& gnss) {
    return gnss.position_std_dev.x < 0.2 && gnss.position_std_dev.y < 0.2;
  };
  if (!CheckGnss(last_gnss)) return LOC_LOCALIZATION_ERROR;
  if (!CheckGnss(cur_gnss)) return LOC_LOCALIZATION_ERROR;
  double time_gap =
      std::fabs(cur_gnss.measurement_time - last_gnss.measurement_time);
  if (time_gap > 5.0 || time_gap <= 0) return LOC_LOCALIZATION_ERROR;

  // convert LLH to ENU
  PointLLH_t last_llh;
  PointENU_t last_enu;
  last_llh.lat = last_gnss.position.lat;
  last_llh.lon = last_gnss.position.lon;
  last_llh.height = last_gnss.position.height;
  CoordinateConverter::GetInstance()->LLA2ENU(last_llh, &last_enu);

  PointLLH_t current_llh;
  PointENU_t current_enu;
  current_llh.lat = cur_gnss.position.lat;
  current_llh.lon = cur_gnss.position.lon;
  current_llh.height = cur_gnss.position.height;
  CoordinateConverter::GetInstance()->LLA2ENU(current_llh, &current_enu);
  // solve heading
  double de = current_enu.x - last_enu.x;
  double dn = current_enu.y - last_enu.y;
  double v_norm = std::sqrt(de * de + dn * dn) / time_gap;
  if (v_norm < 5.0) return LOC_LOCALIZATION_ERROR;
  *heading = std::atan2(de, std::fabs(dn) < 1E-4 ? 1E-4 : dn);

  return LOC_SUCCESS;
}

adLocStatus_t GNSSLocator::SolveHeadingByGnssVelocity(
    std::shared_ptr<Gnss> raw_gnss, double* heading, double* heading_std) {
  static std::deque<std::pair<double, Eigen::Vector2d>> v_datas;
  double timestamp = raw_gnss->measurement_time;
  Eigen::Vector2d v_meas;
  v_meas << raw_gnss->linear_velocity.x, raw_gnss->linear_velocity.y;
  if (v_meas.norm() < 5.0) return LOC_LOCALIZATION_ERROR;
  v_datas.emplace_back(timestamp, v_meas);
  while ((timestamp - v_datas.front().first) > 5.0) v_datas.pop_front();

  // calculate variance using statistical method
  std::vector<double> heading_arr;
  heading_arr.reserve(v_datas.size());
  double heading_ave = 0, heading_var = 0;
  for (size_t i = 0; i < v_datas.size(); ++i) {
    Eigen::Vector2d V = v_datas[i].second;
    double hd = std::atan2(V(0), std::fabs(V(1)) < 1E-4 ? 1E-4 : V(1));
    // process for critical value (pi or -pi)
    if (i != 0) {
      double gap = hd - heading_ave;
      if (gap > M_PI) hd -= 2.0 * M_PI;
      if (gap < -M_PI) hd += 2.0 * M_PI;
    }
    heading_arr.push_back(hd);
    heading_ave += (hd - heading_ave) / (i + 1);
  }
  for (const auto& hd : heading_arr) {
    heading_var += (hd - heading_ave) * (hd - heading_ave);
  }
  heading_var /= v_datas.size();
  double heading_sqvar = std::sqrt(heading_var);
  if (heading_sqvar > 5.0 * d2r) return LOC_LOCALIZATION_ERROR;

  // weight using count and velocity for heading std
  double cnt_weight = 1.0;
  cnt_weight = v_datas.size() == 2 ? 2.0 : cnt_weight;
  cnt_weight = v_datas.size() == 1 ? 6.0 : cnt_weight;
  double v_weight = 1.0 + 7.0 * std::exp(-v_meas.norm() / 5.0);

  *heading = heading_arr.back();
  *heading_std = cnt_weight * v_weight * std::max(0.5 * d2r, heading_sqvar);

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
