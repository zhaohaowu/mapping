/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_fusion_component.h
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#pragma once

#include <memory>

#include "cyber/component/component.h"
#include "lib/ins_fusion.h"

namespace hozon {
namespace mp {
namespace loc {

using adsfi_proto::hz_Adsfi::AlgInsInfo;
using adsfi_proto::internal::HafNodeInfo;
using hozon::localization::LocalizationEstimate;

class InsFusionComponent : public apollo::cyber::Component<> {
 public:
  InsFusionComponent() = default;
  ~InsFusionComponent() override;

  bool Init() override;
  void OnOriginIns(const std::shared_ptr<const AlgInsInfo>& msg);
  bool OnInspva(const std::shared_ptr<const HafNodeInfo>& msg);

 private:
  std::shared_ptr<apollo::cyber::Reader<AlgInsInfo>> origin_ins_reader_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<HafNodeInfo>> inspva_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<HafNodeInfo>> ins_writer_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<LocalizationEstimate>>
      localization_writer_ = nullptr;

  std::unique_ptr<InsFusion> ins_fusion_ = nullptr;
};

CYBER_REGISTER_COMPONENT(InsFusionComponent);

}  // namespace loc
}  // namespace mp
}  // namespace hozon
