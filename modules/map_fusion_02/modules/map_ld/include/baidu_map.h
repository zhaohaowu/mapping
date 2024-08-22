/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ：
 *   author     ： zhangrui
 *   date       ： 2024.5
 ******************************************************************************/
#include <signal.h>
#include <unistd.h>

#include <climits>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <iterator>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include "map_engine/hdmap_enum.h"
#include "map_engine/map_engine.h"
#include "modules/map_fusion_02/modules/map_ld/include/coordinate_convertor.h"
#include "modules/map_fusion_02/modules/map_ld/include/global_ld_map.h"
#include "proto/common/error_code.pb.h"
#include "proto/map/map.pb.h"
#include "proto/map/map_lane.pb.h"
#include "proto/map/map_road.pb.h"
namespace hozon {
namespace mp {
namespace mf {
using hozon::common::math::LineSegment2d;
using hozon::common::math::Vec2d;

template <class Objs_um>
void SerachObjReleatedLanes(
    const std::unordered_map<uint32_t, baidu::imap::LinkPtr>& links_um,
    const std::unordered_map<uint32_t, baidu::imap::LanePtr>& lanes_um,
    Objs_um* obj_um) {
  for (auto& obj_lanes : *obj_um) {
    std::vector<uint32_t> link_ids;
    for (const auto& lane_id : obj_lanes.second) {
      if (lanes_um.count(lane_id) != 0) {
        link_ids.push_back(lanes_um.at(lane_id)->get_linkid());
      }
    }
    for (const auto& link_id : link_ids) {
      if (links_um.count(link_id) != 0) {
        for (const auto& land_id : links_um.at(link_id)->get_laneids()) {
          obj_lanes.second.insert(land_id);
        }
      }
    }
  }
}
struct INSPos {
  double x = 0.0;
  double y = 0.0;
};

class BaiDuMapEngine : public hozon::netaos::adf_lite::Executor {
 public:
  // BaiDuMapEngine(std::string& path, std::string& id) : dbpath_(path),
  // vid_(id) {
  //   // do nothing
  // }
  BaiDuMapEngine() {}

  ~BaiDuMapEngine();
  int32_t AlgInit() override;
  void AlgRelease() override;

  void UpdateBaiDuMap(const INSPos& pos);
  const hozon::hdmap::Map& GetNetaMap() const { return neta_map_; }

 private:
  // std::string dbpath_;
  // std::string vid_;
  void MapTransition(
      std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um);
  void LaneBuild(
      std::vector<uint32_t> left_id, std::vector<uint32_t> right_id,
      const baidu::imap::LanePtr& bd_lane, int serach_road_boundary,
      std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um,
      std::vector<baidu::imap::HdLaneBoundaryPtr>* bd_left_boundarys,
      std::vector<baidu::imap::HdLaneBoundaryPtr>* bd_right_boundarys);
  void SetCenterPoint(const baidu::imap::LanePtr& bd_lane,
                      hozon::hdmap::Curve* curve);
  void SetLaneBoundary(const baidu::imap::LanePtr& bd_lane,
                       hozon::hdmap::Lane* neta_lane);
  void SetBoundaryType(
      bool is_left, bool in_junction,
      const std::vector<baidu::imap::HdLaneBoundaryPtr>& bd_marks,
      hozon::hdmap::LaneBoundary* boundary);
  void SetBoundaryPoint(const baidu::imap::HdLaneBoundaryPtr& bd_mark,
                        hozon::hdmap::Curve* curve);
  void SetLaneType(const baidu::imap::LaneType& bd_lane_type,
                   hozon::hdmap::Lane* lane);
  void SetTurnType(uint16_t bd_turn_type, hozon::hdmap::Lane* lane);
  void SetDirection(const baidu::imap::Direction& bd_direction,
                    hozon::hdmap::Lane* lane);

  void SetTransitions(const baidu::imap::LaneConnectionType& bd_type,
                      hozon::hdmap::Lane* lane);

  void SetRoadType(const baidu::imap::LinkClass& bd_link_class,
                   hozon::hdmap::Road* road);
  void SetSectionType(uint32_t bd_link_type,
                      hozon::hdmap::RoadSection* section);

  void SetSectionBoundarys(
      const std::vector<baidu::imap::HdLaneBoundaryPtr>& bd_lefts,
      const std::vector<baidu::imap::HdLaneBoundaryPtr>& bd_rights,
      hozon::hdmap::RoadSection* section);
  void SetSectionBoundary(
      const std::vector<baidu::imap::HdLaneBoundaryPtr>& bd_boundarys,
      bool is_left, hozon::hdmap::BoundaryPolygon* outer_polygon);

  // crosswalk
  void SetCrossWalks(
      std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um);

  void SetCrossWalkOverlapPb(const std::string& id, uint64_t crw_id,
                             const std::pair<uint64_t, double>& lane_s,
                             double width);
  void SetCrossWalkPb(const std::shared_ptr<baidu::imap::IObject>& cross_walk,
                      const std::vector<std::string>& overlap_ids);
  // signal
  void SetOverlapSignal(
      std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um);

  void SetLaneOverlap(
      const std::string& lane_id, const std::string& overlap_id,
      std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um);

  void SetOverlap(
      const std::string& lane_id, const std::string& overlap_id,
      uint32_t stopline_id,
      std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um);
  void SetSignal(
      const std::unordered_map<uint32_t, std::unordered_set<std::string>>&
          line_overlap_id);
  void SetSignalStopLine(const std::vector<baidu::imap::Point3I>& geometries,
                         hozon::hdmap::LineSegment* segment);

  // setjunction
  void Setjunctions(
      const std::unordered_map<std::string, hozon::hdmap::Lane>& neta_lanes_um);
  void SetJunctionPb(const baidu::imap::JunctionPtr& bd_jun);
  void SetJunOverlapPb(
      const baidu::imap::JunctionPtr& bd_jun,
      const std::unordered_map<std::string, hozon::hdmap::Lane>& neta_lanes_um);
  Vec2d Geometry2Vec2d(const baidu::imap::Point3I& geo);
  void SetLenLine(const std::vector<baidu::imap::Point3I>& geometries,
                  hozon::common::math::LineSegment2d* l,
                  hozon::common::math::LineSegment2d* width);
  bool Get2lineInterPoint(const Vec2d& p1, const Vec2d& p2, const Vec2d& p3,
                          const Vec2d& p4, Vec2d* point);
  bool LinePointsIntersectAB(const LineSegment2d& l,
                             const std::vector<Vec2d>& points, double* s);
  bool LinePointsIntersect(const LineSegment2d& l,
                           const std::vector<Vec2d>& points, double* s);
  bool GetLineLaneIntersect(const LineSegment2d& l,
                            const hozon::hdmap::Lane& lane, double* s,
                            bool sure);
  void CacheLocalMapInfo(const baidu::imap::LocalMap& local_map);
  void Clear();
  std::unordered_map<uint32_t, baidu::imap::LinkPtr> links_um_;
  std::unordered_map<uint32_t, baidu::imap::LanePtr> lanes_um_;
  std::unordered_map<uint32_t, baidu::imap::ObjectPtr> objs_um_;
  std::unordered_map<uint32_t, baidu::imap::JunctionPtr> juns_um_;
  std::unordered_map<uint32_t, baidu::imap::ObjectPtr> cross_walk_um_;
  std::unordered_map<uint32_t, std::unordered_set<uint32_t>>
      cross_walk_lanes_um_;
  std::unordered_map<uint32_t, baidu::imap::ObjectPtr> stop_line_um_;
  std::unordered_map<uint32_t, std::unordered_set<uint32_t>>
      stop_line_lanes_um_;
  hozon::hdmap::Map neta_map_;
  baidu::imap::sdk::MapEnginePtr map_engine_ptr_ = nullptr;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
