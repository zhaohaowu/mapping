/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： occ_process.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/modules/geo/occ_process.h"

namespace hozon {
namespace mp {
namespace mf {

bool OccProcessor::Init() { return true; }
bool OccProcessor::Process(ElementMap::Ptr element_map_ptr) { return true; }
void OccProcessor::Clear() { return; }

}  // namespace mf
}  // namespace mp
}  // namespace hozon
