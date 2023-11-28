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

#include <Sophus/se3.hpp>

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
  static bool ConvertDrNode(const HafNodeInfo& msg, Node* const node);
  static bool IsInterpolable(const Node& n1, const Node& n2,
                             double dis_tol = 5.0, double ang_tol = 0.3,
                             double time_tol = 0.5);
  static bool Interpolate(double ticktime,
                          const std::deque<std::shared_ptr<Node>>& d,
                          Node* const node, double dis_tol = 5.0,
                          double ang_tol = 0.3, double time_tol = 0.5);
  static Sophus::SE3d Node2SE3(const Node& node);
  static Sophus::SE3d Node2SE3(const std::shared_ptr<Node>& node);

 private:
  Params params_;
  double coord_init_timestamp_ = 0.0;
  Node init_dr_node_;
  bool sys_init_ = false;
  HafNodeInfo curr_raw_dr_;
  HafNodeInfo init_raw_dr_;

  hozon::perception::TransportElement curr_percep_;
  std::deque<std::shared_ptr<Node>> dr_deque_;
};

}  // namespace ca
}  // namespace loc
}  // namespace mp
}  // namespace hozon
