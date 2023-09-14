/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： prior_provider_component.h
 *   author     ： taoshaoyuan/zuodongsheng
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/internal/node_info.pb.h>
#include <cyber/cyber.h>
#include <depend/proto/map/map.pb.h>

#include <memory>

namespace hozon {
namespace mp {
namespace mf {

class PriorProvider;

class PriorProviderComponent final : public apollo::cyber::Component<> {
 public:
  PriorProviderComponent() = default;
  ~PriorProviderComponent() = default;

  bool Init() override;

 private:
  void OnInsNodeInfo(
      const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg);

  std::shared_ptr<apollo::cyber::Writer<hozon::hdmap::Map>> prior_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<adsfi_proto::internal::HafNodeInfo>>
      ins_reader_ = nullptr;
  std::shared_ptr<PriorProvider> provider_ = nullptr;
};

CYBER_REGISTER_COMPONENT(PriorProviderComponent);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
