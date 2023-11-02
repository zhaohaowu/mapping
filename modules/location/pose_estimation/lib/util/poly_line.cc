/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： poly_line.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include <poly_line.h>

namespace hozon {
namespace mp {
namespace loc {

template <typename T>
PolyLine<T>::PolyLine(T t) {
  min_ = t.lane_param().cubic_curve_set(0).start_point_x();
  max_ = t.lane_param().cubic_curve_set(0).end_point_x();
  c0_ = t.lane_param().cubic_curve_set(0).c3();
  c1_ = t.lane_param().cubic_curve_set(0).c2();
  c2_ = t.lane_param().cubic_curve_set(0).c1();
  c3_ = t.lane_param().cubic_curve_set(0).c0();
  id_ = t.track_id();
  confidence_ = t.confidence();
}

template class PolyLine<LaneLine>;
// template class PolyLine<RoadEdge>;

}  // namespace loc
}  // namespace mp
}  // namespace hozon
