/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_fusion_component.h
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#pragma once

#include <memory>

#include "cyber/component/component.h"
#include "modules/location/dr_fusion/lib/dr_fusion.h"
namespace hozon {
namespace mp {
namespace loc {

class DrFusionComponent : public apollo::cyber::Component<> {
 public:
  DrFusionComponent() = default;
  ~DrFusionComponent() override;

  bool Init() override;

 private:
  bool OnInspva(
      const std::shared_ptr<const hozon::localization::HafNodeInfo>& msg);
  bool OnDr(const std::shared_ptr<const hozon::localization::HafNodeInfo>& msg);

 private:
  std::unique_ptr<DrFusion> dr_fusion_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::HafNodeInfo>>
      inspva_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::HafNodeInfo>>
      dr_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<hozon::localization::HafNodeInfo>>
      loc_dr_writer_ = nullptr;
};

CYBER_REGISTER_COMPONENT(DrFusionComponent);

}  // namespace loc
}  // namespace mp
}  // namespace hozon
