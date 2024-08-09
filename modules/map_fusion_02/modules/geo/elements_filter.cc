/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： elements_filter.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/modules/geo/elements_filter.h"

namespace hozon {
namespace mp {
namespace mf {

bool ElementsFilter::Init() { return true; }
bool ElementsFilter::Process(ElementMap::Ptr element_map_ptr) { return true; }
void ElementsFilter::Clear() { return; }

}  // namespace mf
}  // namespace mp
}  // namespace hozon
