/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_assignment.h
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/internal/node_info.pb.h>
#include <depend/map/hdmap/hdmap.h>
#include <depend/proto/local_mapping/local_map.pb.h>
#include <depend/proto/map/map.pb.h>

#include <memory>

#include "util/geo.h"

namespace hozon {
namespace mp {
namespace mf {

class TopoAssignment {
 public:
  TopoAssignment() = default;
  ~TopoAssignment() = default;

  int Init();

  void OnInsNodeInfo(
      const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg);
  void OnHQMap(const std::shared_ptr<hozon::hdmap::Map>& msg);
  void OnLocalMap(const std::shared_ptr<hozon::mapping::LocalMap>& msg);

  std::shared_ptr<hozon::hdmap::Map> GetTopoMap();

 private:
  std::shared_ptr<hozon::hdmap::Map> topo_map_ = nullptr;
  std::shared_ptr<hozon::hdmap::Map> hq_map_ = nullptr;
  std::shared_ptr<hozon::hdmap::HDMap> hq_map_server_ = nullptr;
  Eigen::Vector3d vehicle_pose_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
