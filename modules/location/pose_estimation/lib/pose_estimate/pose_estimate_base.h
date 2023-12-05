/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimate_base.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <math.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

#include "modules/location/pose_estimation/lib/hd_map/hd_map.h"
#include "modules/location/pose_estimation/lib/perception/perception.h"
#include "modules/util/include/util/temp_log.h"

#define DEBUG_POLE true
#define DEBUG_TRAFFIC_SIGN true

namespace hozon {
namespace mp {
namespace loc {

#define DOUBLE_MAX std::numeric_limits<double>::max()
#define DOUBLE_MIN std::numeric_limits<double>::min()

using HdMap = Map<hozon::hdmap::Map>;
using LaneLinePerceptionPtr = std::shared_ptr<PerceptionLaneLine>;
// using Perception = Perception<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut>;
// using PerceptionLaneLine =
//     PerceptionLaneLine<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut>;
// using PerceptionLaneLineList =
//     PerceptionLaneLineList<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut>;
// using PerceptionRoadEdge =
//     PerceptionRoadEdge<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut>;
// using RoadMarkingPerceptionRoadEdgeList =
//     PerceptionRoadEdgeList<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut>;
enum class LineSide { UNKNOWN_SIDE = 0, LEFT_SIDE = 1, RIGHT_SIDE = 2 };
enum CommonState { NO = 0, YES = 1, UNKNOW = 2 };
enum class PercepLineType {
  UNKOWN = 100000,
  L_LINE = -1,
  R_LINE = 1,
  LL_LINE = -1,
  RR_LINE = 2,
  EDGE_LINE = 8
};
enum class InsStatus {
  INVALID = 0,                        // 不定位不定向
  SINGLE_POINT_LOCATION_ORIEN = 1,    // 单点定位定向
  PSEUD_DIFF = 2,                     // 伪距差分
  COMB_EXTRAPOLATE = 3,               // 组合推算
  RTK_STABLE = 4,                     // RTK稳定解
  RTK_FLOAT = 5,                      // RTK浮点解
  SINGLE_POINT_LOCATION_UNORIEN = 6,  // 单点定位不定向
  PSEUD_DIFF_LOCATION_UNORIEN = 7,    // 伪距差分定位不定向
  RTK_STABLE_LOCATION_UNORIEN = 8,    // RTK稳定解定位不定向
  RTK_FLOAT_LOCATION_UNORIEN = 9      // RTK浮点解定位不定向
};
enum MapLineType {
  LaneChangeVirtualLine = adsfi_proto::internal::SubMap::LineType::
      SubMap_LineType_LineType_LaneChangeVirtualLine,
  CurbVirtualMarking = adsfi_proto::internal::SubMap::LineType::
      SubMap_LineType_LineType_CurbVirtualMarking,
  RoadVirtualLine = adsfi_proto::internal::SubMap::LineType::
      SubMap_LineType_LineType_RoadVirtualLine
};
template <class T>
struct PointV3Comp {
  bool operator()(const T &p1, const T &p2) const { return p1.x() < p2.x(); }
};
struct PointMatchPair {
  PointMatchPair() {
    type = PercepLineType::UNKOWN;
    weight = 0.f;
    map_pw = Eigen::Matrix<double, 3, 1>::Identity();
    pecep_pv = Eigen::Matrix<double, 3, 1>::Identity();
  }
  PointMatchPair(const V3 &map, const V3 &pecep, PercepLineType type,
                 float weight)
      : map_pw(map), pecep_pv(pecep), type(type), weight(weight) {}
  V3 map_pw;
  V3 pecep_pv;
  PercepLineType type;
  float weight;
};
// struct JacentMapEdge {
//   JacentMapEdge() {
//     nearest_p = V3::Identity();
//     idx = -1;
//     side = LineSide::UNKNOWN_SIDE;
//   }
//   JacentMapEdge(const V3 &nearest_p, size_t idx, LineSide side)
//       : nearest_p(nearest_p), idx(idx), side(side) {}
//   V3 nearest_p;
//   size_t idx;
//   LineSide side;
// };
struct JacentMapLine {
  JacentMapLine() {
    nearest_p = V3::Zero();
    last_p = V3::Zero();
    idx = "";
  }
  JacentMapLine(const V3 &nearest_p, const V3 &last_p, std::string idx)
      : nearest_p(nearest_p), last_p(last_p), idx(idx) {}
  V3 nearest_p;
  V3 last_p;
  std::string idx;
};
struct AlternativeMapLine {
  AlternativeMapLine() {
    ref_p = V3::Zero();
    idx = "";
  }
  AlternativeMapLine(const V3 &ref_p, std::string idx)
      : ref_p(ref_p), idx(idx) {}
  V3 ref_p;
  std::string idx;
};
// struct PercpEdgeItem {
//   PercpEdgeItem(const PerceptionLanelinePtr &edge, const LineSide &side)
//       : edge(edge), side(side) {}
//   PercpEdgeItem() {
//     side = LineSide::UNKNOWN_SIDE;
//     edge = nullptr;
//   }
//   LineSide side;
//   PerceptionLanelinePtr edge;
// };
struct LineMatchPair {
  LineMatchPair() {
    map_line_idxs.clear();
    pecep_line = nullptr;
  }
  LineMatchPair(const std::vector<std::string> &map_line_idxs,
                const LaneLinePerceptionPtr &pecep_line)
      : map_line_idxs(map_line_idxs), pecep_line(pecep_line) {}
  std::vector<std::string> map_line_idxs;
  LaneLinePerceptionPtr pecep_line;
};
// struct EdgeMatchPair {
//   EdgeMatchPair() { map_edge_idx = -1; }
//   EdgeMatchPair(int idx, const PercpEdgeItem &pecep_edge)
//       : map_edge_idx(idx), pecep_edge(pecep_edge) {}
//   int map_edge_idx;
//   PercpEdgeItem pecep_edge;
// };

}  // namespace loc
}  // namespace mp
}  // namespace hozon
