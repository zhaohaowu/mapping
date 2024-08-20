/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： broken_point_search.h
 *   author     ： cuijiayu
 *   date       ： 2024.08
 ******************************************************************************/

#pragma once

#include <cfloat>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Sophus/se3.hpp>
#include <Sophus/so3.hpp>

#include "common/math/vec2d.h"
#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/processor.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/common/common_data.h"
#include "modules/map_fusion_02/modules/lane/road_builder/ctp_util.h"
#include "modules/map_fusion_02/modules/lane/road_builder/detect_cut_pt.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

class BrokenPointSearch : ProcessorBase {
 public:
  BrokenPointSearch() = default;
  ~BrokenPointSearch() = default;
  bool Init() override;

  bool SearchCtp(const std::shared_ptr<std::vector<KinePosePtr>>& path,
                 const KinePosePtr& curr_pose, const ElementMap::Ptr& ele_map);
  std::vector<CutPoint> GetCutPoints();
  std::deque<Line::Ptr> GetLines();

  void Clear() override;

 private:
  void RetrieveBoundaries(const ElementMap::Ptr& ele_map, float interp_dist,
                          std::deque<Line::Ptr>* lines);
  bool PoseLineAdapter2Cpt(const std::deque<Line::Ptr>& lines,
                           Sophus::SE3d* Twv,
                           std::vector<LaneLine::Ptr>* lanelines);

  KinePosePtr curr_pose_ = nullptr;
  std::deque<Line::Ptr> lines_;
  std::vector<CutPoint> cutpoints_;
  std::shared_ptr<DetectCutPt> detect_cut_pt_;
};

using BrokenPointSearchPtr = std::unique_ptr<BrokenPointSearch>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
