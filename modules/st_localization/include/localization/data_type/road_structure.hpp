/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "localization/data_type/base.hpp"
#include "localization/data_type/semantic_type.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

// Header
struct Header {
  uint64_t timestamp_ns;
  uint64_t sensor_id = 0x0;  // unused param
  CoordinateType coord_type = CoordinateType::ENU;
};

// CurrentStatus
struct CurrentStatus {
  id_t cur_lane_id = 0;
  float32_t distance_to_ramp = std::numeric_limits<float32_t>::max();
};

////////////////////////// semantic map data ///////////////////////////////////

// LineSegmentData
struct LineSegmentData {
  id_t id = 0;
  float32_t confidence = 0.0;

  float32_t width = 0.15;
  Color color = Color::NONE;
  LineType line_type = LineType::Unknown;
  LineStyle line_style = LineStyle::Unknown;

  std::vector<Point3D_t> points;
};

// LineData
struct LineData {
  id_t id = 0;
  id_t left_lane_id = 0;
  id_t right_lane_id = 0;
  std::vector<LineSegmentData> line_segments;
};

// StopLineData
struct StopLineData {
  id_t id = 0;
  std::vector<Point3D_t> points;
  std::vector<id_t> lane_ids;
};

// PoleData
struct PoleData {
  id_t id = 0;
  uint8_t confidence = 0;

  id_t lane_id = 0;
  Point3D_t bottom_point;
  Point3D_t top_point;
  PoleType type = PoleType::Unknown;
};

// TrafficSignData
struct TrafficSignData {
  id_t id = 0;
  float32_t confidence = 0.0;

  id_t lane_id = 0;
  Point3D_t centroid;
  Quaternion_t quaternion;
  float32_t length = 0;
  float32_t width = 0;
  float32_t height = 0;

  TrafficSignType type = TrafficSignType::Unknown;
};

// SemanticMapData
struct SemanticMapData {
  std::vector<LineData> lines;
  std::vector<StopLineData> stop_lines;
  std::vector<PoleData> poles;
  std::vector<TrafficSignData> traffic_signs;
};

////////////////////////// route map data //////////////////////////////////////

/// LaneLinkData
struct LaneLinkData {
  id_t id = 0;
  id_t from_lane_id = 0;
  id_t to_lane_id = 0;
  LinkType link_type = LinkType::Unknown;
  std::vector<Point3D_t> ref_points;
};

// SectionLinkData
struct SectionLinkData {
  id_t id = 0;
  id_t from_section_id = 0;
  id_t to_section_id = 0;
};

// LaneData
struct LaneData {
  id_t id = 0;
  id_t section_id = 0;
  uint8_t sequence_id = 0;
  id_t intersection_id = 0;
  LaneType lane_type = LaneType::Unknown;

  // 16进制表示 按位组合
  uint32_t turn_type = 0x0;
  TransType trans_type = TransType::Unknown;

  id_t left_line_id = 0;
  id_t right_line_id = 0;
  id_t left_lane_id = 0;
  id_t right_lane_id = 0;
  float32_t length = 0.0;

  std::vector<Point3D_t> center_points;
  // all these vector's size equals to center_points.size();
  std::vector<float32_t> lane_widths;
  std::vector<float32_t> headings;
  std::vector<float32_t> longitudinal_slopes;  // 纵向坡度
  std::vector<float32_t> lateral_slopes;       // 横向坡度
  std::vector<float32_t> curvature_radius;

  float32_t max_speed = 0.0;  // m/s
  float32_t min_speed = 0.0;  // m/s

  std::vector<id_t> successor_link_ids;
  std::vector<id_t> predecessor_link_ids;
  // objects
  std::vector<id_t> stop_line_ids;
  std::vector<id_t> area_ids;
  std::vector<id_t> traffic_light_ids;
  std::vector<id_t> traffic_sign_ids;
  std::vector<id_t> road_marker_ids;
  std::vector<id_t> cross_walk_ids;
};

/// SectionData
struct SectionData {
  id_t id = 0;
  SectionClass section_class = SectionClass::Unknown;
  SectionType section_type = SectionType::Unknown;
  id_t intersection_id = 0;
  float32_t length = 0.0;
  // 从右至左排序，存储的lane id
  std::vector<id_t> lane_ids;
  // road edge and laneline
  // 左右侧边界，有可能是多个，例如由车道线和路边沿共同组成
  std::vector<id_t> left_boundary_ids;
  std::vector<id_t> right_boundary_ids;
  std::vector<id_t> successor_link_ids;
  std::vector<id_t> predecessor_link_ids;
  bool is_routing_section = false;
};

// RouteMapData
struct RouteMapData {
  std::vector<LaneData> lanes;
  std::vector<SectionData> sections;
  std::vector<LaneLinkData> lane_links;
  std::vector<SectionLinkData> section_links;
};

////////////////////////// road structure data /////////////////////////////////

// RoadStructure
class RoadStructure {
 public:
  DEFINE_PTR(RoadStructure)

  Header header;
  CurrentStatus cur_status;
  SemanticMapData semantic_map_data;
  RouteMapData route_map_data;
};

}  // namespace localization
}  // namespace senseAD
