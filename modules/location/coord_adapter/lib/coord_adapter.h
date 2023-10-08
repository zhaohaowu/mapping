/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： coord_adapter.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <deque>
#include <memory>
#include <string>

#include "modules/location/common/defines.h"
#include "modules/location/coord_adapter/lib/defines.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/localization/node_info.pb.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace loc {
namespace ca {

using hozon::localization::HafNodeInfo;
using hozon::mapping::LocalMap;
using hozon::perception::TransportElement;

class CoordAdapter {
 public:
  CoordAdapter() = default;
  ~CoordAdapter() = default;

  bool Init(const std::string& configfile);
  void SetCoordInitTimestamp(double t);
  double GetCoordInitTimestamp() const;
  bool IsCoordInitSucc() const;
  HafNodeInfo GetSysInitDrFusion() const;
  void OnLocalMap(const LocalMap& local_map);
  void OnPerception(const TransportElement& percep);
  void OnDrFusion(const HafNodeInfo& dr);

 private:
  bool LoadParams(const std::string& configfile);
  bool ConvertDrNode(const HafNodeInfo& msg, cm::BaseNode* const node);

 private:
  Params params_;
  double coord_init_timestamp_ = 0.0;
  cm::BaseNode init_dr_node_;
  bool sys_init_ = false;
  HafNodeInfo curr_raw_dr_;
  HafNodeInfo init_raw_dr_;

  hozon::perception::TransportElement curr_percep_;
  std::deque<std::shared_ptr<cm::BaseNode>> dr_deque_;
};

}  // namespace ca
}  // namespace loc
}  // namespace mp
}  // namespace hozon
