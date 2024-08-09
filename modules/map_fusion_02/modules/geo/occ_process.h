/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： occ_process.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/processor.h"

namespace hozon {
namespace mp {
namespace mf {
class OccProcessor : public ProcessorBase {
 public:
  OccProcessor() : point_num_(-1) {}
  ~OccProcessor() = default;
  OccProcessor(const OccProcessor&) = delete;
  OccProcessor& operator=(const OccProcessor&) = delete;
  bool Init() override;
  bool Process(ElementMap::Ptr element_map_ptr);
  void Clear() override;

 private:
  int point_num_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
