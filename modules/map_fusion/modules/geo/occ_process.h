/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： occ_process.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <map>
#include <utility>
#include <vector>

#include "modules/map_fusion/base/element_map.h"
#include "modules/map_fusion/base/processor.h"

namespace hozon {
namespace mp {
namespace mf {
class OccProcessor : public ProcessorBase {
 public:
  OccProcessor() = default;
  ~OccProcessor() = default;
  OccProcessor(const OccProcessor&) = delete;
  OccProcessor& operator=(const OccProcessor&) = delete;
  bool Init() override;
  bool Process(ElementMap::Ptr element_map_ptr);
  void Clear() override;

 private:
  void ConstructLinePairs();
  void CompareGroupLines();
  static bool GetFirstOCCPoints(const OccRoad::Ptr& occ_road_ptr,
                                int* first_point_index);
  bool FindOCCGuidePoint();
  void CalDistNearOcc(bool left, const OccRoad::Ptr& occ_road_ptr, int* index,
                      double* distance);
  bool GetFirstNearIndex(const OccRoad::Ptr& occ_road_ptr,
                         int* first_point_index);

  void UpdateOCCRoadPoints();

 private:
  std::vector<std::pair<int, int>> line_pairs_;
  // 用于组group的occ
  std::vector<std::pair<int, OccRoad::Ptr>> vec_occ_line_;
  // 按照平均宽度对线进行分组
  std::vector<std::vector<std::pair<int, OccRoad::Ptr>>> grouped_lines_;
  // copy from element_map
  std::map<Id, OccRoad::Ptr> occ_roads_;
  // 通过slope找到第一个点的occ
  std::vector<OccRoad::Ptr> find_occ_;
  // 没有通过slope找到第一个点的occ
  std::vector<OccRoad::Ptr> lost_occ_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
