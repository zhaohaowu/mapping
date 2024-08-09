/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_fusion_pipeline.h
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <string>

namespace hozon {
namespace mp {
namespace mf {

class BaseFusionPipeline {
 public:
  virtual ~BaseFusionPipeline() = default;
  virtual bool Init() = 0;
  virtual void Clear() = 0;

  virtual std::string Name() const = 0;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
