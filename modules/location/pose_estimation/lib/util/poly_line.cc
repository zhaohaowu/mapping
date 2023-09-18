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
  min_ = t.lane_fit().x_start_vrf();
  max_ = t.lane_fit().x_end_vrf();
  c0_ = t.lane_fit().coefficients().d();
  c1_ = t.lane_fit().coefficients().c();
  c2_ = t.lane_fit().coefficients().b();
  c3_ = t.lane_fit().coefficients().a();
  id_ = t.laneline_seq();
  confidence_ = t.geometry_confidence();
}

template class PolyLine<LaneLine>;
// template class PolyLine<RoadEdge>;

}  // namespace loc
}  // namespace mp
}  // namespace hozon
