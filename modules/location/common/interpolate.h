/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： interpolate.h
 *   author     ： lilanxing
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once

#include <deque>
#include <memory>

#include "modules/location/common/defines.h"

namespace hozon {
namespace mp {
namespace loc {
namespace cm {

bool IsInterpolable(const BaseNode& n1, const BaseNode& n2,
                    double dis_tol = 5.0, double ang_tol = 0.3,
                    double time_tol = 0.5) {
  const double dis_delta = (n1.enu - n2.enu).norm();
  const double ang_delta = (Sophus::SO3d::exp(n1.orientation).inverse() *
                            Sophus::SO3d::exp(n2.orientation)).log().norm();
  const double time_delta = fabs(n2.ticktime - n1.ticktime);
  if (dis_delta >= dis_tol || ang_delta >= ang_tol || time_delta >= time_tol) {
    return false;
  }
  return true;
}

bool Interpolate(double ticktime, const BaseNode& n1, const BaseNode& n2,
                 BaseNode* const n, double dis_tol = 5.0, double ang_tol = 0.3,
                 double time_tol = 0.5) {
  if (!n || n1.ticktime < 0 || n2.ticktime < 0 || ticktime < n1.ticktime ||
      ticktime > n2.ticktime || fabs(n1.ticktime - n2.ticktime) < 1e-3) {
    return false;
  }
  if (!IsInterpolable(n1, n2, dis_tol, ang_tol, time_tol)) {
    return false;
  }

  const auto& p1 = Node2SE3(n1);
  const auto& p2 = Node2SE3(n2);
  const auto& delta = p1.inverse() * p2;
  const double ratio = (ticktime - n1.ticktime) / (n2.ticktime - n1.ticktime);
  if (std::isnan(ratio) || std::isinf(ratio)) {
    return false;
  }

  const auto& pose =
      p1 * Sophus::SE3d(Sophus::SO3d::exp(ratio * delta.so3().log()),
                        ratio * delta.translation());
  n->ticktime = ticktime;
  n->enu = pose.translation();
  n->orientation = pose.so3().log();
  n->velocity = (1 - ratio) * n1.velocity + ratio * n2.velocity;

  return true;
}

bool Interpolate(double ticktime, const std::deque<BaseNode>& dq,
                 BaseNode* const n, double dis_tol = 5.0, double ang_tol = 0.3,
                 double time_tol = 0.5) {
  if (!n || dq.empty()) {
    return false;
  }

  std::size_t i = 0;
  for (; i < dq.size(); ++i) {
    if (fabs(dq[i].ticktime - ticktime) < 1e-3) {
      *n = dq[i];
      return true;
    }
    if (dq[i].ticktime > ticktime) {
      break;
    }
  }
  if (i == 0 || i == dq.size()) {
    return false;
  }

  return Interpolate(ticktime, dq[i - 1], dq[i], n, dis_tol, ang_tol, time_tol);
}

bool Interpolate(double ticktime,
                 const std::deque<std::shared_ptr<BaseNode>>& dq,
                 BaseNode* const n, double dis_tol = 5.0, double ang_tol = 0.3,
                 double time_tol = 0.5) {
  if (!n || dq.empty()) {
    return false;
  }

  std::size_t i = 0;
  for (; i < dq.size(); ++i) {
    if (fabs(dq[i]->ticktime - ticktime) < 1e-3) {
      *n = *(dq[i]);
      return true;
    }
    if (dq[i]->ticktime > ticktime) {
      break;
    }
  }
  if (i == 0 || i == dq.size()) {
    return false;
  }

  return Interpolate(ticktime, *(dq[i - 1]), *(dq[i]), n, dis_tol, ang_tol,
                     time_tol);
}

}  // namespace cm
}  // namespace loc
}  // namespace mp
}  // namespace hozon
