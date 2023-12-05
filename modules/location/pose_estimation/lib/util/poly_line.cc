/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： poly_line.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include <poly_line.h>
#include <limits>
#include <algorithm>
namespace hozon {
namespace mp {
namespace loc {

template <typename T>
PolyLine<T>::PolyLine(T t) {
  min_ = std::numeric_limits<double>::max();
  max_ = std::numeric_limits<double>::min();
  // min_ = t.lane_param().cubic_curve_set(0).start_point_x();
  // max_ = t.lane_param().cubic_curve_set(0).end_point_x();
  // c0_ = t.lane_param().cubic_curve_set(0).c0();
  // c1_ = t.lane_param().cubic_curve_set(0).c1();
  // c2_ = t.lane_param().cubic_curve_set(0).c2();
  // c3_ = t.lane_param().cubic_curve_set(0).c3();
  id_ = t.track_id();
  confidence_ = t.confidence();
  lane_position_type_ = static_cast<int>(t.lanepos());
  for (auto& p : t.points()) {
    if (p.x() < 1.0e-3) {
      continue;
    }
    min_ = std::min(p.x(), static_cast<double>(min_));
    max_ = std::max(p.x(), static_cast<double>(max_));
    points.emplace_back(hozon::mp::loc::V3{p.x(), p.y(), p.z()});
  }
}

template class PolyLine<hozon::mp::loc::LaneLine>;
// template class PolyLine<RoadEdge>;

}  // namespace loc
}  // namespace mp
}  // namespace hozon
