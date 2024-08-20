/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： interface_option.h
 *   author     ： hozon
 *   date       ： 2024.01
 ******************************************************************************/
#pragma once
#include "modules/map_fusion_02/base/interface_option.h"

namespace hozon {
namespace mp {
namespace mf {

class ProcessorBase {
 public:
  ProcessorBase() = default;
  virtual ~ProcessorBase() = default;
  ProcessorBase(const ProcessorBase&) = delete;
  ProcessorBase& operator=(const ProcessorBase&) = delete;
  virtual bool Init() {
    return true;
  }
  virtual void Clear() = 0;
};
}  // namespace mf
}  // namespace mp
}  // namespace hozon
