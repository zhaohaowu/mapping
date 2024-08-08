/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： broken_point_search.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/modules/lane/broken_point_search.h"

namespace hozon {
namespace mp {
namespace mf {
bool BrokenPointSearch::Init() { return true; }
bool BrokenPointSearch::Process(ElementMap::Ptr element_map_ptr) {
  return true;
}
void BrokenPointSearch::Search() { return; }
}  // namespace mf
}  // namespace mp
}  // namespace hozon
