/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： elements_filter.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/processor.h"

namespace hozon {
namespace mp {
namespace mf {
class ElementsFilter : public ProcessorBase {
 public:
  ElementsFilter() : point_num_(-1) {}
  ~ElementsFilter() = default;
  ElementsFilter(const ElementsFilter&) = delete;
  ElementsFilter& operator=(const ElementsFilter&) = delete;
  bool Init() override;
  bool Process(ElementMap::Ptr element_map_ptr);
  void Clear() override;

 private:
  int point_num_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
