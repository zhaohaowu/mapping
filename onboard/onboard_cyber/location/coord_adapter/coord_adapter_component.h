/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： coord_adapter_component.h
 *   author     ： lilanxing
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once

#include <cyber/cyber.h>
#include <cyber/component/component.h>

#include <deque>
#include <memory>

#include "proto/localization/node_info.pb.h"
#include "proto/local_mapping/local_map.pb.h"
#include "modules/location/coord_adapter/lib/coord_adapter.h"

namespace hozon {
namespace mp {
namespace loc {
namespace ca {

using apollo::cyber::Reader;
using apollo::cyber::Writer;
using hozon::mapping::LocalMap;

class CoordAdapterComponent : public apollo::cyber::Component<> {
 public:
  CoordAdapterComponent() = default;
  ~CoordAdapterComponent() = default;

  bool Init() override;
  void OnLocalMap(const std::shared_ptr<const LocalMap>& msg);
  void OnDrFusion(const std::shared_ptr<const HafNodeInfo>& msg);

 private:
  std::shared_ptr<Reader<LocalMap>> local_map_reader_ = nullptr;
  std::shared_ptr<Reader<HafNodeInfo>> dr_reader_ = nullptr;
  std::shared_ptr<Writer<HafNodeInfo>> init_dr_writer_ = nullptr;

  std::unique_ptr<CoordAdapter> coord_adapter_ = nullptr;
};

CYBER_REGISTER_COMPONENT(CoordAdapterComponent);

}  // namespace ca
}  // namespace loc
}  // namespace mp
}  // namespace hozon
