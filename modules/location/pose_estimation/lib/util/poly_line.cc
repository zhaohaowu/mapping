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

}  // namespace loc
}  // namespace mp
}  // namespace hozon
