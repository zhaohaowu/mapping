/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_assignment_component.h
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <cyber/cyber.h>
#include <depend/proto/local_mapping/local_map.pb.h>
#include <depend/proto/localization/localization.pb.h>
#include <depend/proto/localization/node_info.pb.h>
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
  void Clear() override;

 private:
  void OnInsNodeInfo(
      const std::shared_ptr<hozon::localization::HafNodeInfo>& msg);
  void OnLocalMap(const std::shared_ptr<hozon::mapping::LocalMap>& msg);

  std::shared_ptr<apollo::cyber::Writer<hozon::hdmap::Map>> topo_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::HafNodeInfo>>
      ins_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::mapping::LocalMap>> lm_reader_ =
      nullptr;

  std::shared_ptr<TopoAssignment> topo_assign_ = nullptr;
};

CYBER_REGISTER_COMPONENT(TopoAssignmentComponent);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
