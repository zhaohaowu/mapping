/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： tlr_fusion_pipeline.h
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/
#include <memory>

namespace hozon {
namespace mp {
namespace mf {
class TLRFusionPipline {
 public:
  typedef std::unique_ptr<TLRFusionPipline> Ptr;

  TLRFusionPipline();
  ~TLRFusionPipline();

  void Process();
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
