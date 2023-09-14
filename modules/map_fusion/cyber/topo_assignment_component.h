/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_assignment_component.h
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/internal/node_info.pb.h>
#include <adsfi_proto/map/local_map.pb.h>
#include <cyber/cyber.h>
#include <depend/proto/map/map.pb.h>

#include <memory>

namespace hozon {
namespace mp {
namespace mf {

class TopoAssignment;

class TopoAssignmentComponent final : public apollo::cyber::Component<> {
 public:
  TopoAssignmentComponent() = default;
  ~TopoAssignmentComponent() = default;

  bool Init() override;

 private:
  void OnInsNodeInfo(
      const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg);
  void OnHQMap(const std::shared_ptr<hozon::hdmap::Map>& msg);
  void OnLocalMap(const std::shared_ptr<LocalMap>& msg);

  std::shared_ptr<apollo::cyber::Writer<hozon::hdmap::Map>> topo_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<adsfi_proto::internal::HafNodeInfo>>
      ins_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::hdmap::Map>> hq_reader_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<LocalMap>> lm_reader_ = nullptr;

  std::shared_ptr<TopoAssignment> topo_assign_ = nullptr;
};

CYBER_REGISTER_COMPONENT(TopoAssignmentComponent);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
