/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ：
 *   author     ： zhangrui
 *   date       ： 2024.5
 ******************************************************************************/
#include "modules/map_fusion_02/modules/map_ld/baidu_map.h"

#include "base/utils/log.h"
#include "lib/environment/environment.h"
#include "util/mapping_log.h"
#include "util/rate.h"
#include "util/tic_toc.h"

bool SetProtoToBinaryFile(const google::protobuf::Message& message,
                          const std::string& file_name) {
  std::fstream output(file_name,
                      std::ios::out | std::ios::trunc | std::ios::binary);
  return message.SerializeToOstream(&output);
}
namespace hozon {
namespace mp {
namespace mf {
using hozon::common::math::Vec2d;
int32_t BaiDuMapEngine::AlgInit() {
  // 可放置baidu map路径
  if (map_engine_ptr_ == nullptr) {
    map_engine_ptr_ = baidu::imap::sdk::get_map_engine();
    baidu::imap::InitParameter init_param(
        "", "",
        baidu::imap::LogControl(5, 100 * 1024, "./hdmap_log",
                                baidu::imap::LogControl::INFO));
    std::string default_work_root = "/app/";
    std::string work_root =
        hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
    std::string dbpath = work_root + "/data/baidu_map/hdmap/";
#ifdef ISORIN
    dbpath = "/hd_map/ld_map/";
#endif
    std::string vid = "HeZhong2024010166";
    if (map_engine_ptr_->initialize(dbpath, vid, init_param) !=
        baidu::imap::IMAP_STATUS_OK) {
      HLOG_WARN << "Engine initialize failed!";
      if (map_engine_ptr_) {
        map_engine_ptr_->finalize_map();
      }
      map_engine_ptr_ = nullptr;  // 确保指针重置为nullptr
      return -1;                  // 返回错误码表示初始化失败
    }
  }
  return 0;
}
void BaiDuMapEngine::AlgRelease() {
  if (map_engine_ptr_ != nullptr) {
    map_engine_ptr_->finalize_map();
  }
}
BaiDuMapEngine::~BaiDuMapEngine() {
  if (map_engine_ptr_ != nullptr) {
    map_engine_ptr_->finalize_map();
  }
  Clear();
}

void BaiDuMapEngine::SetLenLine(
    const std::vector<baidu::imap::Point3I>& geometries,
    hozon::common::math::LineSegment2d* l,
    hozon::common::math::LineSegment2d* width) {
  if (geometries.size() < 4) {
    HLOG_WARN << "error: geometries size<4";
    return;
  }
  std::vector<Vec2d> points;
  for (const auto& geo : geometries) {
    points.emplace_back(Geometry2Vec2d(geo));
  }
  if (geometries.size() == 4) {
    if (points[1].DistanceSquareTo(points[0]) >
        points[1].DistanceSquareTo(points[2])) {
      *l = LineSegment2d((points[0] + points[3]) / 2,
                         (points[1] + points[2]) / 2);
      *width = LineSegment2d(points[1], points[2]);
    } else {
      *l = LineSegment2d((points[0] + points[1]) / 2,
                         (points[2] + points[3]) / 2);
      *width = LineSegment2d(points[1], points[0]);
    }
  } else {
    std::vector<hozon::common::math::LineSegment2d> segments;
    for (int i = 0; i + 1 < points.size(); i++) {
      segments.emplace_back(points[i], points[i + 1]);
    }
    double max_l(0.0);
    int index(-1);
    for (int i = 0; i < segments.size(); i++) {
      if (segments[i].length() > max_l) {
        max_l = segments[i].length();
        index = i;
      }
    }
    if (index != -1) {
      bool length_forward(false);
      if (index == 0) {
        length_forward = true;
        *width = segments[index + 1];
      } else if (index + 1 == segments.size()) {
        length_forward = false;
        *width = segments[index - 1];
      } else {
        if (segments[index - 1].length() > segments[index + 1].length()) {
          length_forward = false;
          *width = segments[index - 1];
        } else {
          length_forward = true;
          *width = segments[index + 1];
        }
      }

      if (length_forward) {
        *l = LineSegment2d(points[index] + (width->end() - width->start()) / 2,
                           width->center());
      } else {
        *l = LineSegment2d(
            width->center(),
            points[index + 1] + (width->start() - width->end()) / 2);
      }

    } else {
      HLOG_DEBUG << "crosswalk set length width fail";
    }
  }
}
// 寻找两条线的交叉点
bool BaiDuMapEngine::Get2lineInterPoint(const Vec2d& p1, const Vec2d& p2,
                                        const Vec2d& p3, const Vec2d& p4,
                                        Vec2d* point) {
  if (abs((p2 - p1).CrossProd(p4 - p3)) < 1e-6) {
    HLOG_DEBUG << "lines parallel";
    return false;
  }

  double a1(0.0);
  double a2(0.0);
  double a3(0.0);
  double b1(0.0);
  double b2(0.0);
  double b3(0.0);
  double t(0.0);
  a1 = p4.y() - p3.y();
  a2 = p3.y() - p1.y();
  a3 = p2.y() - p1.y();
  b1 = p4.x() - p3.x();
  b2 = p3.x() - p1.x();
  b3 = p2.x() - p1.x();
  t = (a3 * b2 - a2 * b3) / (a1 * b3 - a3 * b1);
  point->set_x((p4.x() - p3.x()) * t + p3.x());
  point->set_y((p4.y() - p3.y()) * t + p3.y());

  return true;
}

bool BaiDuMapEngine::LinePointsIntersectAB(const LineSegment2d& l,
                                           const std::vector<Vec2d>& points,
                                           double* s) {
  int point_size = points.size();
  Vec2d point;
  int number(-1);
  double dis = __DBL_MAX__;
  int best_index = -1;
  for (int i = 0; i < point_size - 1; ++i) {
    LineSegment2d l2(points.at(i), points.at(i + 1));
    Get2lineInterPoint(l.start(), l.end(), l2.start(), l2.end(), &point);
    if (point.DistanceTo(l2.start()) < dis) {
      dis = point.DistanceTo(l2.start());
      best_index = i;
    }
    if (l2.IsPointIn(point) && l.IsPointIn(point)) {
      number = i;
      break;
    }
  }

  // HLOG_ERROR << "intersection point: " << point.DebugString();
  if (number != -1) {
    *s = 0;
    std::vector<hozon::common::math::LineSegment2d> segments;
    for (int i = 0; i <= number; ++i) {
      if (i < number) {
        segments.emplace_back(points.at(i), points.at(i + 1));
      } else {
        segments.emplace_back(points.at(i), point);
      }
      *s += segments.back().length();
    }
    return true;
  }
  if (best_index == 0) {
    LineSegment2d l2(points.at(0), points.at(1));
    if (l2.unit_direction().InnerProd(point - l2.start()) < 0) {
      *s = -point.DistanceTo(points.front());
    } else {
      *s = point.DistanceTo(points.front());
    }
    return true;
  } else if (best_index > 0) {
    *s = 0;
    std::vector<hozon::common::math::LineSegment2d> segments;
    for (int i = 0; i <= number; ++i) {
      if (i < number) {
        segments.emplace_back(points.at(i), points.at(i + 1));
      } else {
        segments.emplace_back(points.at(i), point);
      }
      *s += segments.back().length();
    }
    return true;
  }

  HLOG_WARN << "not find inter point ";
  return false;
}

bool BaiDuMapEngine::LinePointsIntersect(const LineSegment2d& l,
                                         const std::vector<Vec2d>& points,
                                         double* s) {
  int point_size = points.size();
  Vec2d point;
  int number(-1);
  // std::cout << " cross line: " << l.DebugString();
  for (int i = 0; i < point_size - 1; ++i) {
    LineSegment2d l2(points.at(i), points.at(i + 1));
    Get2lineInterPoint(l.start(), l.end(), l2.start(), l2.end(), &point);
    if (l2.IsPointIn(point)) {
      number = i;
      break;
    }
  }
  if (number != -1) {
    *s = 0;
    std::vector<hozon::common::math::LineSegment2d> segments;
    for (int i = 0; i <= number; ++i) {
      if (i < number) {
        segments.emplace_back(points.at(i), points.at(i + 1));
      } else {
        segments.emplace_back(points.at(i), point);
      }
      *s += segments.back().length();
    }
    return true;
  }
  return false;
}

bool BaiDuMapEngine::GetLineLaneIntersect(const LineSegment2d& l,
                                          const hozon::hdmap::Lane& lane,
                                          double* s, bool sure) {
  if (lane.central_curve().segment().empty()) {
    HLOG_WARN << "lane curve is empty, lane id: " << lane.id().id();
  }
  std::vector<Vec2d> lane_points;
  for (const auto& segment : lane.central_curve().segment()) {
    for (const auto& point : segment.line_segment().point()) {
      lane_points.emplace_back(point.x(), point.y());
    }
  }
  if (sure) {
    return LinePointsIntersectAB(l, lane_points, s);
  }

  return LinePointsIntersect(l, lane_points, s);
}

void BaiDuMapEngine::MapTransition(
    std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um) {
  for (const auto& bd_link : links_um_) {
    auto* road = neta_map_.add_road();
    road->mutable_id()->set_id("bd" + std::to_string(bd_link.first));
    SetRoadType(bd_link.second->get_link_class(), road);

    auto* section = road->add_section();
    section->mutable_id()->set_id(std::to_string(bd_link.first));
    SetSectionType(bd_link.second->get_link_type(), section);
    auto* section_lane_id = section->mutable_lane_id();
    const auto& bd_lane_ids = bd_link.second->get_laneids();
    std::vector<baidu::imap::HdLaneBoundaryPtr> bd_left_road_boundarys;
    std::vector<baidu::imap::HdLaneBoundaryPtr> bd_right_road_boundarys;
    for (int i = 0; i < bd_lane_ids.size(); i++) {
      if (lanes_um_.count(bd_lane_ids[i]) != 0) {
        int search_road_boudary =
            INT_MAX;  // -1: right; 0: right+left; 1: left; 2: no serach
        std::vector<uint32_t> left_ids;
        std::vector<uint32_t> right_ids;
        section_lane_id->Add()->set_id(std::to_string(bd_lane_ids[i]));
        int right_i = i - 1;
        while (right_i >= 0) {
          right_ids.emplace_back(bd_lane_ids[right_i--]);
        }
        int left_i = i + 1;
        while (left_i < bd_lane_ids.size()) {
          left_ids.emplace_back(bd_lane_ids[left_i++]);
        }
        if (i == 0 && bd_lane_ids.size() == 1) {
          search_road_boudary = 0;
        } else if (i == 0 && bd_lane_ids.size() > 1) {
          search_road_boudary = -1;
        } else if (i + 1 == bd_lane_ids.size()) {
          search_road_boudary = 1;
        }
        LaneBuild(left_ids, right_ids, lanes_um_.at(bd_lane_ids[i]),
                  search_road_boudary, neta_lanes_um, &bd_left_road_boundarys,
                  &bd_right_road_boundarys);
      }
    }
    SetSectionBoundarys(bd_left_road_boundarys, bd_right_road_boundarys,
                        section);
  }
}

void BaiDuMapEngine::SetRoadType(const baidu::imap::LinkClass& bd_link_class,
                                 hozon::hdmap::Road* road) {
  switch (bd_link_class) {
    case baidu::imap::LC_EXPRESSWAY:
      road->set_type(::hozon::hdmap::Road_Type::Road_Type_HIGHWAY);
      break;

    case baidu::imap::LC_URBAN_EXPRESSWAY:
      road->set_type(::hozon::hdmap::Road_Type::Road_Type_CITY_HIGHWAY);
      break;

    case baidu::imap::LC_NATION_ROAD:
      road->set_type(::hozon::hdmap::Road_Type::Road_Type_NATIONAL_ROAD);
      break;

    case baidu::imap::LC_PROVINCE_ROAD:
      road->set_type(::hozon::hdmap::Road_Type::Road_Type_PROVINCIAL_ROAD);
      break;

    case baidu::imap::LC_COUNTY_ROAD:
      road->set_type(::hozon::hdmap::Road_Type::Road_Type_COUNTY_ROAD);
      break;

    case baidu::imap::LC_TOWN_ROAD:
      road->set_type(::hozon::hdmap::Road_Type::Road_Type_TOWN_ROAD);
      break;

    case baidu::imap::LC_WALK_ROAD:
      road->set_type(::hozon::hdmap::Road_Type::Road_Type_WALK_ROAD);
      break;

    case baidu::imap::LC_SPECIAL_ROAD:
    case baidu::imap::LC_PEOPLE_FERRY:
    case baidu::imap::LC_FERRY:
      road->set_type(::hozon::hdmap::Road_Type::Road_Type_CITY_ROAD);
      break;

    default:
      road->set_type(::hozon::hdmap::Road_Type::Road_Type_UNKNOWN);
      break;
  }
}

void BaiDuMapEngine::SetSectionType(uint32_t bd_link_type,
                                    hozon::hdmap::RoadSection* section) {
  // std::cout << "link id: " << section->id().id()
  //           << " bd link type: " << bd_link_type << std::endl;
  if (bd_link_type == 0X00000000 || bd_link_type & 0X00004000 ||
      bd_link_type & 0X00000001 || bd_link_type & 0X00000002) {
    section->set_type(
        ::hozon::hdmap::RoadSection_Type::RoadSection_Type_MultipleCarriageWay);
  } else if (bd_link_type & 0X00000004 || bd_link_type & 0X00000200 ||
             bd_link_type & 0X00000400 || bd_link_type & 0X00000800) {
    section->set_type(
        ::hozon::hdmap::RoadSection_Type::RoadSection_Type_Service);
  } else if (bd_link_type & 0X00001000) {
    section->set_type(
        ::hozon::hdmap::RoadSection_Type::RoadSection_Type_RoundaboutCircle);
  } else if (bd_link_type & 0X00000010 || bd_link_type & 0X00010000) {
    section->set_type(::hozon::hdmap::RoadSection_Type::RoadSection_Type_Ramp);
  } else if (bd_link_type & 0X00000020) {
    section->set_type(::hozon::hdmap::RoadSection_Type::RoadSection_Type_JCT);
  } else if (bd_link_type & 0X00000040) {
    section->set_type(
        ::hozon::hdmap::RoadSection_Type::RoadSection_Type_CarPark);
  } else if (bd_link_type & 0X00008000) {
    section->set_type(
        ::hozon::hdmap::RoadSection_Type::RoadSection_Type_SideRoad);
  } else {
    section->set_type(
        ::hozon::hdmap::RoadSection_Type::RoadSection_Type_UNKNOWN);
  }
}

void BaiDuMapEngine::SetSectionBoundarys(
    const std::vector<baidu::imap::HdLaneBoundaryPtr>& bd_lefts,
    const std::vector<baidu::imap::HdLaneBoundaryPtr>& bd_rights,
    hozon::hdmap::RoadSection* section) {
  auto* section_bounadry = section->mutable_boundary();
  auto* outer_polygon = section_bounadry->mutable_outer_polygon();
  if (!bd_lefts.empty()) {
    SetSectionBoundary(bd_lefts, true, outer_polygon);
  } else {
    HLOG_ERROR << "set left section boundary error, link id: "
               << section->id().id();
  }
  if (!bd_rights.empty()) {
    SetSectionBoundary(bd_rights, false, outer_polygon);
  } else {
    HLOG_ERROR << "set right section boundary error, link id: "
               << section->id().id();
  }
}

void BaiDuMapEngine::SetSectionBoundary(
    const std::vector<baidu::imap::HdLaneBoundaryPtr>& bd_boundarys,
    bool is_left, hozon::hdmap::BoundaryPolygon* outer_polygon) {
  for (const auto& bd_boundary : bd_boundarys) {
    auto* edge = outer_polygon->add_edge();
    if (is_left) {
      edge->set_type(
          hozon::hdmap::BoundaryEdge_Type::BoundaryEdge_Type_LEFT_BOUNDARY);
    } else {
      edge->set_type(
          hozon::hdmap::BoundaryEdge_Type::BoundaryEdge_Type_RIGHT_BOUNDARY);
    }
    if (bd_boundary->get_surface_type() == baidu::imap::SFT_NO_PLANE_SURFACE) {
      edge->set_is_plane(false);
    } else {
      edge->set_is_plane(true);
    }
    edge->mutable_id()->set_id(std::to_string(bd_boundary->get_id()));

    SetBoundaryPoint(bd_boundary, edge->mutable_curve());
  }
}

void BaiDuMapEngine::LaneBuild(
    std::vector<uint32_t> left_ids, std::vector<uint32_t> right_ids,
    const baidu::imap::LanePtr& bd_lane, int search_road_boundary,
    std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um,
    std::vector<baidu::imap::HdLaneBoundaryPtr>* bd_left_boundarys,
    std::vector<baidu::imap::HdLaneBoundaryPtr>* bd_right_boundarys) {
  hozon::hdmap::Lane neta_lane;
  neta_lane.mutable_id()->set_id(std::to_string(bd_lane->get_laneid()));

  neta_lane.set_length(bd_lane->get_length() / 100.0);

  auto* cent_curve = neta_lane.mutable_central_curve();
  SetCenterPoint(bd_lane, cent_curve);

  const auto& left_marks = bd_lane->get_left_marking_list();
  SetLaneBoundary(bd_lane, &neta_lane);
  if (search_road_boundary == -1 || search_road_boundary == 0) {
    for (const auto& bd_right : bd_lane->get_right_marking_list()) {
      if (bd_right->get_is_road_boundary()) {
        bd_right_boundarys->push_back(bd_right);
      }
    }
  }

  if (search_road_boundary == 1 || search_road_boundary == 0) {
    for (const auto& bd_left : bd_lane->get_left_marking_list()) {
      if (bd_left->get_is_road_boundary()) {
        bd_left_boundarys->push_back(bd_left);
      }
    }
  }
  // std::cout << "lane id: " << bd_lane->get_laneid()
  //           << " speed: " << bd_lane->get_max_speed_limit()
  //           << " connect type: " << bd_lane->get_lane_connection_type()
  //           << std::endl;
  if (bd_lane->get_max_speed_limit() > 0) {
    neta_lane.set_speed_limit(bd_lane->get_max_speed_limit() / 3.6);
  } else {
    neta_lane.set_speed_limit(40 / 3.6);
  }

  for (const auto& pred_id : bd_lane->get_precursor_lanes_ids()) {
    neta_lane.add_predecessor_id()->set_id(std::to_string(pred_id));
  }

  for (const auto& succ_id : bd_lane->get_successor_lanes_ids()) {
    neta_lane.add_successor_id()->set_id(std::to_string(succ_id));
  }

  for (auto left_id : left_ids) {
    neta_lane.add_left_neighbor_forward_lane_id()->set_id(
        std::to_string(left_id));
  }

  for (auto right_id : right_ids) {
    neta_lane.add_right_neighbor_forward_lane_id()->set_id(
        std::to_string(right_id));
  }

  SetLaneType(bd_lane->get_lane_type(), &neta_lane);

  SetTurnType(bd_lane->get_current_direction(), &neta_lane);

  SetDirection(bd_lane->get_travel_direction(), &neta_lane);

  SetTransitions(bd_lane->get_lane_connection_type(), &neta_lane);

  auto width = bd_lane->get_average_width();
  auto* left_sample = neta_lane.add_left_sample();
  left_sample->set_s(0);
  left_sample->set_width(width / 200.0);
  auto* right_sample = neta_lane.add_right_sample();
  right_sample->set_s(0);
  right_sample->set_width(width / 200.0);

  if (bd_lane->has_junction_id()) {
    auto* overlap = neta_lane.add_overlap_id();
    overlap->set_id(std::to_string(bd_lane->get_junction_id()) + "_" +
                    neta_lane.id().id());
  }

  (*neta_lanes_um)[neta_lane.id().id()] = neta_lane;
}

void BaiDuMapEngine::SetCenterPoint(const baidu::imap::LanePtr& bd_lane,
                                    hozon::hdmap::Curve* curve) {
  auto* segment = curve->add_segment();
  segment->set_length(bd_lane->get_length() / 100.0);
  segment->set_s(0.0);
  auto* line_segment = segment->mutable_line_segment();
  for (const auto& geo : bd_lane->get_geometry()) {
    auto x = geo.x * 1.0e-7;
    auto y = geo.y * 1.0e-7;
    auto* original_point = line_segment->add_original_point();
    original_point->set_x(x);
    original_point->set_y(y);
    original_point->set_z(0.0);
    GCS2UTM(51, &x, &y);
    auto* point = line_segment->add_point();
    point->set_x(x);
    point->set_y(y);
    point->set_z(0.0);
  }
}

void BaiDuMapEngine::SetLaneBoundary(const baidu::imap::LanePtr& bd_lane,
                                     hozon::hdmap::Lane* neta_lane) {
  auto* left_boundary = neta_lane->mutable_left_boundary();
  SetBoundaryType(true, bd_lane->has_junction_id(),
                  bd_lane->get_left_marking_list(), left_boundary);
  auto* right_boundary = neta_lane->mutable_right_boundary();
  SetBoundaryType(false, bd_lane->has_junction_id(),
                  bd_lane->get_right_marking_list(), right_boundary);
}

void BaiDuMapEngine::SetBoundaryType(
    bool is_left, bool in_junction,
    const std::vector<baidu::imap::HdLaneBoundaryPtr>& bd_marks,
    hozon::hdmap::LaneBoundary* boundary) {
  // HLOG_INFO << " left boundary: " << is_left << " bdsize: " <<
  // bd_marks.size(); for (const auto& bd_mark : bd_marks) {
  //   HLOG_INFO << " bd boundary type: " << bd_mark->get_type()
  //             << " is road boundary: " << bd_mark->get_is_road_boundary();
  // }
  auto* boundary_type = boundary->add_boundary_type();
  boundary_type->set_s(0.0);

  // if (in_junction) {
  //   boundary->set_virtual_(true);
  //   boundary_type->add_types(hozon::hdmap::LaneBoundaryType::DOTTED_WHITE);
  // } else {
  int flag_all_boundary = 1;  // 是否全是边沿
  baidu::imap::HdLaneBoundaryPtr bd_mark_bound;
  for (auto bd_mark : bd_marks) {
    if (bd_mark->get_is_road_boundary() == 0) {
      flag_all_boundary = 0;
      bd_mark_bound = bd_mark;
      break;
    }
  }

  if (!bd_marks.empty() && flag_all_boundary == 0) {
    switch (bd_mark_bound->get_type()) {
      case baidu::imap::DMT_MARKING_NONE:
        boundary->set_virtual_(true);
        boundary_type->add_types(hozon::hdmap::LaneBoundaryType::UNKNOWN);
        break;
      case baidu::imap::DMT_MARKING_UNKNOWN:
        boundary->set_virtual_(false);
        boundary_type->add_types(hozon::hdmap::LaneBoundaryType::UNKNOWN);
        break;

      case baidu::imap::DMT_MARKING_LONG_DASHED_LINE:
      case baidu::imap::DMT_MARKING_DOUBLE_DASHED_LINE:
        boundary->set_virtual_(false);
        boundary_type->add_types(hozon::hdmap::LaneBoundaryType::DOTTED_WHITE);
        break;

      case baidu::imap::DMT_MARKING_VIRTUAL_FIT:
      case baidu::imap::DMT_MARKING_VIRTUAL:
        boundary->set_virtual_(true);
        boundary_type->add_types(hozon::hdmap::LaneBoundaryType::DOTTED_WHITE);
        break;

      case baidu::imap::DMT_MARKING_DOUBLE_SOLID_LINE:
      case baidu::imap::DMT_MARKING_SINGLE_SOLID_LINE:
        boundary->set_virtual_(false);
        boundary_type->add_types(hozon::hdmap::LaneBoundaryType::SOLID_WHITE);
        break;

      case baidu::imap::DMT_MARKING_RIGHT_SOLID_LINE_LEFT_DASHED_LINE:
        boundary->set_virtual_(false);
        if (is_left) {
          boundary_type->add_types(hozon::hdmap::LaneBoundaryType::SOLID_WHITE);
        } else {
          boundary_type->add_types(
              hozon::hdmap::LaneBoundaryType::DOTTED_WHITE);
        }
        break;

      case baidu::imap::DMT_MARKING_LEFT_SOLID_LINE_RIGHT_DASHED_LINE:
        boundary->set_virtual_(false);
        if (is_left) {
          boundary_type->add_types(
              hozon::hdmap::LaneBoundaryType::DOTTED_WHITE);
        } else {
          boundary_type->add_types(hozon::hdmap::LaneBoundaryType::SOLID_WHITE);
        }
        break;
    }
  } else {
    HLOG_WARN << "bd lane has no boundary";
  }
  // }

  boundary->set_length(bd_marks.front()->get_length() / 100.0);
  boundary->add_id()->set_id(std::to_string(bd_marks.front()->get_id()));

  SetBoundaryPoint(bd_marks.front(), boundary->mutable_curve());
}

void BaiDuMapEngine::SetBoundaryPoint(
    const baidu::imap::HdLaneBoundaryPtr& bd_mark, hozon::hdmap::Curve* curve) {
  auto* segment = curve->add_segment();
  segment->set_length(bd_mark->get_length() / 100.0);
  segment->set_s(0.0);
  auto* line_segment = segment->mutable_line_segment();
  for (const auto& geo : bd_mark->get_geometry()) {
    auto x = geo.x * 1.0e-7;
    auto y = geo.y * 1.0e-7;
    auto* original_point = line_segment->add_original_point();
    original_point->set_x(x);
    original_point->set_y(y);
    original_point->set_z(0.0);
    GCS2UTM(51, &x, &y);
    auto* point = line_segment->add_point();
    point->set_x(x);
    point->set_y(y);
    point->set_z(0.0);
  }
}

void BaiDuMapEngine::SetLaneType(const baidu::imap::LaneType& bd_lane_type,
                                 hozon::hdmap::Lane* lane) {
  switch (bd_lane_type) {
    case baidu::imap::LAT_NORMAL:
    case baidu::imap::LAT_ENTRY:
    case baidu::imap::LAT_EXIT:
    case baidu::imap::LAT_VARIABLE_LANE:
    case baidu::imap::LAT_RIGHT_TURN_LANE:
      lane->set_type(hozon::hdmap::Lane_LaneType::Lane_LaneType_CITY_DRIVING);
      break;

    case baidu::imap::LAT_EMERGENCY:
      lane->set_type(hozon::hdmap::Lane_LaneType::Lane_LaneType_EMERGENCY_LANE);
      break;

    case baidu::imap::LAT_ON_RAMP:
    case baidu::imap::LAT_OFF_RAMP:
    case baidu::imap::LAT_CONNECT_RAMP:
    case baidu::imap::LAT_ACCELERATE:
    case baidu::imap::LAT_DECELERATE:
    case baidu::imap::LAT_EMERGENCY_PARKING_STRIP:
    case baidu::imap::LAT_CLIMBING:
    case baidu::imap::LAT_ESCAPE:
    case baidu::imap::LAT_DEDICATED_CUSTOMS:
    case baidu::imap::LAT_VIEWING_PLATFROM:
    case baidu::imap::LAT_PARALLEL_LANE:
    case baidu::imap::LAT_DIVERSION:
    case baidu::imap::LAT_PARKING:
      lane->set_type(
          hozon::hdmap::Lane_LaneType::Lane_LaneType_HIGHWAY_DRIVING);
      break;

    case baidu::imap::LAT_LEFT_TURN_AREA:
      lane->set_type(
          hozon::hdmap::Lane_LaneType::Lane_LaneType_LEFT_TURN_WAITING_ZONE);
      break;

    case baidu::imap::LAT_NON_MOTOR_LANE:
    case baidu::imap::LAT_BUS_STOP:
    case baidu::imap::LAT_NO_ENTRY_LANE:
      lane->set_type(hozon::hdmap::Lane_LaneType::Lane_LaneType_INVALID_LANE);
      break;
    default:
      lane->set_type(hozon::hdmap::Lane_LaneType::Lane_LaneType_NONE);
      break;
  }
}

void BaiDuMapEngine::SetTurnType(uint16_t bd_turn_type,
                                 hozon::hdmap::Lane* lane) {
  switch (bd_turn_type) {
    case baidu::imap::TURNDIR_STRAIGHT:
      lane->set_turn(::hozon::hdmap::Lane_LaneTurn::Lane_LaneTurn_NO_TURN);
      break;

    case baidu::imap::TURNDIR_LEFT:
      lane->set_turn(::hozon::hdmap::Lane_LaneTurn::Lane_LaneTurn_LEFT_TURN);
      break;

    case baidu::imap::TURNDIR_RIGHT:
      lane->set_turn(::hozon::hdmap::Lane_LaneTurn::Lane_LaneTurn_RIGHT_TURN);
      break;

    case baidu::imap::TURNDIR_TURN:
      lane->set_turn(::hozon::hdmap::Lane_LaneTurn::Lane_LaneTurn_U_TURN);
      break;
  }
}

void BaiDuMapEngine::SetDirection(const baidu::imap::Direction& bd_direction,
                                  hozon::hdmap::Lane* lane) {
  switch (bd_direction) {
    case baidu::imap::Direction::IN_POSITIVE_DIRECTION:
      lane->set_direction(
          ::hozon::hdmap::Lane_LaneDirection::Lane_LaneDirection_FORWARD);
      break;

    case baidu::imap::Direction::IN_NEGATIVE_DIRECTION:
      lane->set_direction(
          ::hozon::hdmap::Lane_LaneDirection::Lane_LaneDirection_BACKWARD);
      break;

    case baidu::imap::Direction::IN_BOTH_DIRECTIONS:
    case baidu::imap::Direction::IN_BOTH_DIRECTIONS_PROHIBIT:
      lane->set_direction(
          ::hozon::hdmap::Lane_LaneDirection::Lane_LaneDirection_BIDIRECTION);
      break;
  }
}

void BaiDuMapEngine::SetTransitions(
    const baidu::imap::LaneConnectionType& bd_type, hozon::hdmap::Lane* lane) {
  if (bd_type == 0) {
    lane->set_lane_transition(
        ::hozon::hdmap::Lane_LaneTransition::Lane_LaneTransition_CONTINUE);
  } else if (bd_type & baidu::imap::LAN_STATUS_SPLITING) {
    lane->set_lane_transition(
        ::hozon::hdmap::Lane_LaneTransition::Lane_LaneTransition_SPLITING);
  } else if (bd_type & baidu::imap::LAN_STATUS_MERGING) {
    lane->set_lane_transition(
        ::hozon::hdmap::Lane_LaneTransition::Lane_LaneTransition_MERGING);
  } else {
    lane->set_lane_transition(
        ::hozon::hdmap::Lane_LaneTransition::Lane_LaneTransition_UNKONOW);
  }
}

// crosswalk

void BaiDuMapEngine::SetCrossWalks(
    std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um) {
  for (const auto& cross_walk : cross_walk_um_) {
    // 确定长边的点
    hozon::common::math::LineSegment2d cen_line;
    hozon::common::math::LineSegment2d width;

    double crosswalk_width(0.0);
    std::vector<std::pair<uint64_t, double>> lane_s_pairs;
    std::vector<std::string> overlap_ids;
    if (cross_walk_lanes_um_.count(cross_walk.first) != 0) {
      for (const auto& lane_id : cross_walk_lanes_um_.at(cross_walk.first)) {
        if (neta_lanes_um->count(std::to_string(lane_id)) != 0) {
          SetLenLine(cross_walk.second->get_geometry(), &cen_line, &width);
          double s(0.0);
          if (GetLineLaneIntersect(cen_line,
                                   neta_lanes_um->at(std::to_string(lane_id)),
                                   &s, true)) {
            crosswalk_width = width.length();
            lane_s_pairs.emplace_back(lane_id, s);
            overlap_ids.push_back(std::to_string(lane_id) + "_" +
                                  std::to_string(cross_walk.first));
          } else if (GetLineLaneIntersect(
                         width, neta_lanes_um->at(std::to_string(lane_id)), &s,
                         true)) {
            crosswalk_width = cen_line.length();
            lane_s_pairs.emplace_back(lane_id, s);
            overlap_ids.push_back(std::to_string(lane_id) + "_" +
                                  std::to_string(cross_walk.first));
          }
          // HLOG_ERROR << "crosswalk id: " << cross_walk.first
          //            << " , laneId: " << lane_id << " ,s: " << s
          //            << " width: " << crosswalk_width;
        } else {
          HLOG_ERROR << "not find lane id: " << lane_id;
        }
      }
    }

    for (const auto& lane_s : lane_s_pairs) {
      std::string overlap_id(std::to_string(lane_s.first) + "_" +
                             std::to_string(cross_walk.first));

      SetCrossWalkOverlapPb(overlap_id, cross_walk.first, lane_s,
                            crosswalk_width);

      neta_lanes_um->at(std::to_string(lane_s.first))
          .add_overlap_id()
          ->set_id(overlap_id);
    }

    SetCrossWalkPb(cross_walk.second, overlap_ids);
  }
}

void BaiDuMapEngine::SetCrossWalkOverlapPb(
    const std::string& id, uint64_t crw_id,
    const std::pair<uint64_t, double>& lane_s, double width) {
  auto* overlap = neta_map_.mutable_overlap()->Add();
  overlap->mutable_id()->set_id(id);
  auto* crw_obj = overlap->mutable_object()->Add();
  crw_obj->mutable_id()->set_id(std::to_string(crw_id));
  crw_obj->mutable_crosswalk_overlap_info();

  auto* lane_object = overlap->add_object();
  lane_object->mutable_id()->set_id(std::to_string(lane_s.first));
  auto* laneoverlapinfo = lane_object->mutable_lane_overlap_info();
  laneoverlapinfo->set_start_s(lane_s.second - width / 2.0);
  laneoverlapinfo->set_end_s(lane_s.second + width / 2.0);
  laneoverlapinfo->set_is_merge(false);
}

void BaiDuMapEngine::SetCrossWalkPb(
    const std::shared_ptr<baidu::imap::IObject>& cross_walk,
    const std::vector<std::string>& overlap_ids) {
  auto* crosswalk = neta_map_.mutable_crosswalk()->Add();
  crosswalk->mutable_id()->set_id(std::to_string(cross_walk->get_id()));
  for (const auto& geo : cross_walk->get_geometry()) {
    auto init_point = Geometry2Vec2d(geo);
    auto* point = crosswalk->mutable_polygon()->add_point();
    point->set_x(init_point.x());
    point->set_y(init_point.y());
    point->set_z(0.0);
  }

  for (const auto& overlap_id : overlap_ids) {
    crosswalk->mutable_overlap_id()->Add()->set_id(overlap_id);
  }
}

// set signal
void BaiDuMapEngine::SetOverlapSignal(
    std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um) {
  std::unordered_map<uint32_t, std::unordered_set<std::string>> line_overlap_id;
  for (const auto& stop_line_lanes : stop_line_lanes_um_) {
    for (const auto& lane_id : stop_line_lanes.second) {
      std::string overlap_id(std::to_string(lane_id) + "_" +
                             std::to_string(stop_line_lanes.first));
      SetLaneOverlap(std::to_string(lane_id), overlap_id, neta_lanes_um);
      SetOverlap(std::to_string(lane_id), overlap_id, stop_line_lanes.first,
                 neta_lanes_um);
      line_overlap_id[stop_line_lanes.first].insert(overlap_id);
    }
  }
  SetSignal(line_overlap_id);
}

void BaiDuMapEngine::SetLaneOverlap(
    const std::string& lane_id, const std::string& overlap_id,
    std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um) {
  if (neta_lanes_um->count(lane_id) != 0) {
    (*neta_lanes_um)[lane_id].mutable_overlap_id()->Add()->set_id(overlap_id);
  } else {
    HLOG_ERROR << "total lane um not find lane id: " << lane_id;
  }
}

void BaiDuMapEngine::SetOverlap(
    const std::string& lane_id, const std::string& overlap_id,
    uint32_t stopline_id,
    std::unordered_map<std::string, hozon::hdmap::Lane>* neta_lanes_um) {
  if (stop_line_um_.count(stopline_id) != 0) {
    if (stop_line_um_.at(stopline_id)->get_geometry().size() < 2) {
      HLOG_ERROR << "stopline id: " << stopline_id << " has no 2 point\n"
                 << "size: "
                 << stop_line_um_.at(stopline_id)->get_geometry().size();
      return;
    }

    auto* overlap = neta_map_.mutable_overlap()->Add();
    overlap->mutable_id()->set_id(overlap_id);

    auto* signal_obj = overlap->mutable_object()->Add();
    signal_obj->mutable_id()->set_id(std::to_string(stopline_id));
    signal_obj->mutable_signal_overlap_info();

    // 确定长边的点
    double stopline_width(0.5);
    auto p1 =
        Geometry2Vec2d(stop_line_um_.at(stopline_id)->get_geometry().front());
    auto p2 =
        Geometry2Vec2d(stop_line_um_.at(stopline_id)->get_geometry().back());
    hozon::common::math::LineSegment2d cen_line(p1, p2);

    auto* lane_object = overlap->add_object();
    lane_object->mutable_id()->set_id(lane_id);
    auto* laneoverlapinfo = lane_object->mutable_lane_overlap_info();
    double s(0.0);
    if (neta_lanes_um->count(lane_id) != 0) {
      GetLineLaneIntersect(cen_line, neta_lanes_um->at(lane_id), &s, true);
    }
    laneoverlapinfo->set_start_s(s - stopline_width / 2);
    laneoverlapinfo->set_end_s(s + stopline_width / 2);
    laneoverlapinfo->set_is_merge(false);
  }
}

void BaiDuMapEngine::SetSignal(
    const std::unordered_map<uint32_t, std::unordered_set<std::string>>&
        line_overlap_id) {
  for (const auto& line_overlaps : line_overlap_id) {
    auto* signal = neta_map_.mutable_signal()->Add();
    signal->mutable_id()->set_id(std::to_string(line_overlaps.first));
    for (const auto& overlap_id : line_overlaps.second) {
      signal->mutable_overlap_id()->Add()->set_id(overlap_id);
    }

    if (stop_line_um_.count(line_overlaps.first) != 0) {
      auto* segment = signal->mutable_stop_line()
                          ->Add()
                          ->add_segment()
                          ->mutable_line_segment();
      SetSignalStopLine(stop_line_um_.at(line_overlaps.first)->get_geometry(),
                        segment);
    }
  }
}

void BaiDuMapEngine::SetSignalStopLine(
    const std::vector<baidu::imap::Point3I>& geometries,
    hozon::hdmap::LineSegment* segment) {
  for (const auto& geo : geometries) {
    auto* point = segment->add_point();
    double x(geo.x * 1e-7);
    double y(geo.y * 1e-7);
    GCS2UTM(51, &x, &y);
    point->set_x(x);
    point->set_y(y);
    point->set_z(0.0);
  }
}

void BaiDuMapEngine::Setjunctions(
    const std::unordered_map<std::string, hozon::hdmap::Lane>& neta_lanes_um) {
  for (const auto& bd_junction : juns_um_) {
    if (!bd_junction.second->get_laneids().empty()) {
      SetJunctionPb(bd_junction.second);
      SetJunOverlapPb(bd_junction.second, neta_lanes_um);
    }
  }
}

void BaiDuMapEngine::SetJunctionPb(const baidu::imap::JunctionPtr& bd_jun) {
  auto* junction = neta_map_.mutable_junction()->Add();
  junction->mutable_id()->set_id(std::to_string(bd_jun->get_id()));
  for (const auto& geo : bd_jun->get_geometry()) {
    auto init_point = Geometry2Vec2d(geo);
    auto* point = junction->mutable_polygon()->add_point();
    point->set_x(init_point.x());
    point->set_y(init_point.y());
    point->set_z(0.0);
  }
  for (const auto& lane_id : bd_jun->get_laneids()) {
    junction->mutable_overlap_id()->Add()->set_id(
        std::to_string(bd_jun->get_id()) + "_" + std::to_string(lane_id));
  }
}

void BaiDuMapEngine::SetJunOverlapPb(
    const baidu::imap::JunctionPtr& bd_jun,
    const std::unordered_map<std::string, hozon::hdmap::Lane>& neta_lanes_um) {
  for (const auto& id : bd_jun->get_laneids()) {
    std::string lane_id(std::to_string(id));
    std::string jun_id(std::to_string(bd_jun->get_id()));
    if (neta_lanes_um.count(lane_id) != 0) {
      auto* overlap = neta_map_.add_overlap();
      overlap->mutable_id()->set_id(jun_id + "_" + lane_id);
      auto* jun_obj = overlap->mutable_object()->Add();
      jun_obj->mutable_id()->set_id(jun_id);
      jun_obj->mutable_junction_overlap_info();
      auto* lane_obj = overlap->mutable_object()->Add();
      lane_obj->mutable_id()->set_id(lane_id);
      auto* laneoverlap_info = lane_obj->mutable_lane_overlap_info();
      laneoverlap_info->set_start_s(0.0);
      laneoverlap_info->set_end_s(neta_lanes_um.at(lane_id).length());
    }
  }
}
Vec2d BaiDuMapEngine::Geometry2Vec2d(const baidu::imap::Point3I& geo) {
  double x(geo.x * 1e-7);
  double y(geo.y * 1e-7);
  GCS2UTM(51, &x, &y);
  return {x, y};
}
void BaiDuMapEngine::CacheLocalMapInfo(const baidu::imap::LocalMap& local_map) {
  for (const auto& l : local_map.hd_links) {
    links_um_[l->get_id()] = l;
  }
  for (const auto& l : local_map.hd_lanes) {
    lanes_um_[l->get_laneid()] = l;
  }

  for (const auto& obj : local_map.hd_objects) {
    objs_um_[obj->get_id()] = obj;
    // HLOG_ERROR << "obj id: " << obj->get_id() << " type: " << obj->get_type()
    //            << " sub type: " << obj->get_subtype()
    //            << " lane id size: " << obj->get_laneids().size();
    // HLOG_ERROR << "obj geo size: " << obj->get_geometry().size();
    // HLOG_ERROR << "obj length: " << obj->get_length()
    //            << " width: " << obj->get_width() << "\n";
    // for (const auto& lane_id : obj->get_laneids()) {
    //   HLOG_ERROR << "releated lane id: " << lane_id << " ";
    // }

    if (obj->get_subtype() == baidu::imap::OST_ROADMAK_STOP_LINE) {
      stop_line_um_[obj->get_id()] = obj;
      for (const auto& lane_id : obj->get_laneids()) {
        stop_line_lanes_um_[obj->get_id()].insert(lane_id);
      }
      if (obj->get_geometry().size() != 2) {
        HLOG_ERROR << "stop line geo size!=2: " << obj->get_id();
      }
    }
    if (obj->get_subtype() == baidu::imap::OST_ROADMAK_PEDESTRIAN) {
      cross_walk_um_[obj->get_id()] = obj;
      for (const auto& lane_id : obj->get_laneids()) {
        cross_walk_lanes_um_[obj->get_id()].insert(lane_id);
      }
    }
  }

  // HLOG_ERROR << "junction size: " << local_map.hd_junctions.size();
  for (const auto& jc : local_map.hd_junctions) {
    juns_um_[jc->get_id()] = jc;
    // HLOG_ERROR << "junction id: " << jc->get_id()
    //            << " lane id size:" << jc->get_laneids().size();
    // for (const auto& lane_id : jc->get_laneids()) {
    //   HLOG_ERROR << "lane id: " << lane_id << ",";
    // }
  }
}
void BaiDuMapEngine::Clear() {
  links_um_.clear();
  lanes_um_.clear();
  objs_um_.clear();
  stop_line_um_.clear();
  stop_line_lanes_um_.clear();
  cross_walk_um_.clear();
  cross_walk_lanes_um_.clear();
  juns_um_.clear();
  neta_map_.Clear();
}
void BaiDuMapEngine::UpdateBaiDuMap(const INSPos& pos) {
  baidu::imap::GpsCoord gps;
  // gps.lon = 121.395844;
  // gps.lat = 31.224504;
  gps.lon = pos.y;
  gps.lat = pos.x;
  double radius = 500;
  baidu::imap::LocalMap local_map;
  std::string map_version;
  std::string engine_version;
  baidu::imap::LanePtr p_lane = nullptr;
  baidu::imap::LinkPtr p_link = nullptr;
  baidu::imap::ObjectPtr p_obj = nullptr;
  baidu::imap::JunctionPtr p_jc = nullptr;

  // uint32_t link_id = 17509173;
  // uint32_t lane_id = 162732831;

  // uint32_t obj_id = 214885465;
  // uint32_t jc_id = 4395287;

  if (map_engine_ptr_ == nullptr) {
    HLOG_ERROR << "baidu map engine is not initialized. Call AlgInit first.";
    return;
  }

  map_engine_ptr_->set_current_position(gps);
  uint32_t status = 0;
  while ((status = map_engine_ptr_->get_engine_status()) !=
         baidu::imap::IMAP_STATUS_OK) {
    HLOG_ERROR << "waiting buff loading ";
    HLOG_ERROR << "engine status: " << status;
    usleep(1e6);
  }
  uint32_t res = 0;
  res = map_engine_ptr_->get_local_map(gps, radius, local_map);
  if (res != baidu::imap::IMAP_STATUS_OK) {
    HLOG_ERROR << "get local map status: " << res;
    return;
  }
  Clear();
  CacheLocalMapInfo(local_map);

  std::unordered_map<std::string, hozon::hdmap::Lane> neta_lanes_um;
  MapTransition(&neta_lanes_um);

  // set crosswalk
  SerachObjReleatedLanes(
      links_um_, lanes_um_,
      &cross_walk_lanes_um_);  // 把所有的关联lane添加进去，因为百度只关联了道路的最左车道
  SetCrossWalks(&neta_lanes_um);

  // set stop line=singal
  SetOverlapSignal(&neta_lanes_um);

  Setjunctions(neta_lanes_um);
  for (const auto& lane : neta_lanes_um) {
    neta_map_.add_lane()->CopyFrom(lane.second);
  }
  hozon::mp::GlobalLDMap::Instance()->UpdateLDMap(neta_map_);
  // GLOBAL_LD_MAP->LoadMapFromProto(neta_map_);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
