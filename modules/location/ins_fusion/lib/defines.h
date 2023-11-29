/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： defines.h
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#pragma once

#include <Eigen/Eigen>

namespace hozon {
namespace mp {
namespace loc {

enum class InsInitStatus : uint32_t { OK = 0, CONFIG_NOT_FOUND = 1 };

enum class InsSysStatus : uint32_t {
  INIT = 0,
  SATELLITE = 1,
  INTEGRATED = 2,
  INERTIAL = 3
};

enum class InsGpsStatus : uint32_t {
  NONE = 0,
  SPPO = 1,
  RTD_PO = 2,
  COMB_CAL = 3,
  RTK_STABLE_PO = 4,
  RTK_FLOAT_PO = 5,
  SPP = 6,
  RTD_P = 7,
  RTK_STABLE_P = 8,
  RTK_FLOAT_P = 9
};

struct Config {
  bool smooth = false;
  bool use_rviz_bridge = false;
  bool use_inspva = false;
  bool use_deflection = false;
  bool use_fixed_quat = false;
  uint32_t smooth_window_size = 0;
  double smooth_gcj02_enu_east_diff_thr = 0.0;
  double smooth_gcj02_enu_north_diff_thr = 0.0;
  double smooth_gcj02_enu_norm_diff_thr = 0.0;
  double smooth_momentum = 0.0;

  // ins gps status duration threshold without mm
  double gps_1_last_thr = 5.0;
  double gps_2_last_thr = 5.0;
  double gps_3_last_thr = 20.0;
  double gps_5_last_thr = 5.0;

  // ins gps status duration threshold with mm
  double gps_1_last_with_mm_thr = 0.0;
  double gps_2_last_with_mm_thr = 0.0;
  double gps_3_last_with_mm_thr = 0.0;
  double gps_5_last_with_mm_thr = 0.0;

  // gps 5 std threshold
  double gps_5_stdx_thr = 0.0;
  double gps_5_stdy_thr = 0.0;
  double gps_5_stdz_thr = 0.0;

  unsigned int ins84_deque_max_size = 10;
  unsigned int inspva_deque_max_size = 10;
  bool fix_deflection_repeat = false;
};

struct InsNode {
  uint32_t seq;
  double ticktime;
  Eigen::Vector3d refpoint;
  Eigen::Vector3d blh;
  Eigen::Vector3d org_blh;
  Eigen::Vector3d enu;
  Eigen::Vector3d orientation;
  Eigen::Vector3d velocity;
  Eigen::Vector3d linear_accel;

  InsNode() { Reset(); }

  void Reset() {
    seq = 0;
    ticktime = -1;
    refpoint = Eigen::Vector3d::Zero();
    blh = Eigen::Vector3d::Zero();
    org_blh = Eigen::Vector3d::Zero();
    enu = Eigen::Vector3d::Zero();
    orientation = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();
    linear_accel = Eigen::Vector3d::Zero();
  }
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
