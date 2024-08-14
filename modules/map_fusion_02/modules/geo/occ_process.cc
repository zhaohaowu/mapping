/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： occ_process.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/modules/geo/occ_process.h"

#include "modules/map_fusion_02/data_manager/percep_obj_manager.h"
#include "modules/map_fusion_02/modules/geo/geo_utils.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

bool OccProcessor::Init() { return true; }
bool OccProcessor::Process(ElementMap::Ptr element_map_ptr) {
  std::vector<std::pair<int, OccRoad>> vec_occ_line;
  for (const auto& occ_pair : element_map_ptr->occ_roads) {
    const OccRoad::Ptr& occ = occ_pair.second;
    if (OccLineFitError(occ) > 2.0) {
      continue;
    }
    if (CheckOppositeLineByObj(occ->road_points,
                               OBJECT_MANAGER->GetInverseHistoryObjs())) {
      HLOG_DEBUG << "CheckOppisiteLineByObj id: " << occ->track_id
                 << ", detect_id: " << occ->detect_id;
      continue;
    }
  }
  return true;
}
void OccProcessor::Clear() { return; }

}  // namespace mf
}  // namespace mp
}  // namespace hozon
