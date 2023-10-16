/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_fusion_component.h
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#pragma once

#include <memory>

#include "cyber/component/component.h"
#include "modules/location/ins_fusion/lib/ins_fusion.h"

namespace hozon {
namespace mp {
namespace loc {

class InsFusionComponent : public apollo::cyber::Component<> {
 public:
  InsFusionComponent() = default;
  ~InsFusionComponent() override;

  bool Init() override;
  void OnOriginIns(const std::shared_ptr<const hozon::soc::ImuIns>& msg);
  bool OnInspva(
      const std::shared_ptr<const hozon::localization::HafNodeInfo>& msg);

 private:
  std::shared_ptr<apollo::cyber::Reader<hozon::soc::ImuIns>>
      origin_ins_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::HafNodeInfo>>
      inspva_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<hozon::localization::HafNodeInfo>>
      ins_writer_ = nullptr;
  std::unique_ptr<InsFusion> ins_fusion_ = nullptr;
};

CYBER_REGISTER_COMPONENT(InsFusionComponent);

}  // namespace loc
}  // namespace mp
}  // namespace hozon
