/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_helper.cc
 *   author     ： taoshaoyuan
 *   date       ： 2024.02
 ******************************************************************************/

#include "modules/util/include/util/viz_helper.h"

#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace util {

#define PRINT_TBD                                                 \
  std::cout << "!!! TBD: " << __FILE__ << ": " << __LINE__ << " " \
            << __FUNCTION__ << std::endl;

#define FLOAT_RGB(IntRgb) (IntRgb / 255.0)
#define PI acos(-1)

Rgb ColorRgb(Color color) {
  // rgb value refers to: https://tool.oschina.net/commons?type=3
  std::map<Color, Rgb> color_map = {
      {WHITE, {FLOAT_RGB(255), FLOAT_RGB(255), FLOAT_RGB(255)}},
      {GREY, {FLOAT_RGB(190), FLOAT_RGB(190), FLOAT_RGB(190)}},
      {BLACK, {FLOAT_RGB(0), FLOAT_RGB(0), FLOAT_RGB(0)}},
      {RED, {FLOAT_RGB(255), FLOAT_RGB(0), FLOAT_RGB(0)}},
      {ORANGE, {FLOAT_RGB(255), FLOAT_RGB(165), FLOAT_RGB(0)}},
      {YELLOW, {FLOAT_RGB(255), FLOAT_RGB(255), FLOAT_RGB(0)}},
      {GREEN, {FLOAT_RGB(0), FLOAT_RGB(255), FLOAT_RGB(0)}},
      {BLUE, {FLOAT_RGB(0), FLOAT_RGB(0), FLOAT_RGB(255)}},
      {CYAN, {FLOAT_RGB(0), FLOAT_RGB(255), FLOAT_RGB(255)}},
      {PURPLE, {FLOAT_RGB(160), FLOAT_RGB(32), FLOAT_RGB(240)}},
  };
  if (color_map.find(color) == color_map.end()) {
    color = WHITE;
  }
  return color_map[color];
}

void SplitStamp(double secs, uint32_t* sec, uint32_t* nsec) {
  if (sec == nullptr || nsec == nullptr) {
    return;
  }
  if (secs < 0) {
    return;
  }
  auto s = static_cast<uint32_t>(secs);
  auto ns = static_cast<uint32_t>((secs - s) * 1e9);
  *sec = s;
  *nsec = ns;
}

std::shared_ptr<adsfi_proto::viz::CompressedImage> YUVNV12ImageToVizImage(
    const std::shared_ptr<hozon::soc::Image>& yuv_image, int quality,
    double resize_factor) {
  if (yuv_image == nullptr) {
    HLOG_ERROR << "nullptr input yuv";
    return nullptr;
  }
  auto width = static_cast<int>(yuv_image->width());
  auto height = static_cast<int>(yuv_image->height());
  size_t nv12_bytes_num = height * width * 3 / 2;
  if (yuv_image->data().size() != nv12_bytes_num) {
    HLOG_ERROR << "invalid yuv data size";
    return nullptr;
  }
  if (quality <= 0 || quality > 100) {
    HLOG_ERROR << "invalid quality, should in range (0, 100]";
    return nullptr;
  }
  if (resize_factor <= 0. || resize_factor > 1.) {
    HLOG_ERROR << "invalid resize factor, should in range (0, 1]";
    return nullptr;
  }
  cv::Mat yuv_nv12;
  yuv_nv12.create(height * 3 / 2, width, CV_8UC1);
  memcpy(yuv_nv12.data, yuv_image->data().data(), nv12_bytes_num);
  cv::Mat bgr;
  cv::cvtColor(yuv_nv12, bgr, cv::COLOR_YUV2BGR_NV12);
  if (resize_factor < 1.0) {
    auto resized_cols = static_cast<int>(bgr.cols * resize_factor);
    auto resized_rows = static_cast<int>(bgr.rows * resize_factor);
    cv::resize(bgr, bgr, cv::Size(resized_cols, resized_rows));
  }
  std::vector<int> param(2);
  param[0] = cv::IMWRITE_JPEG_QUALITY;
  param[1] = quality;
  std::vector<uchar> buf;
  cv::imencode(".jpg", bgr, buf, param);
  auto viz_image = std::make_shared<adsfi_proto::viz::CompressedImage>();
  viz_image->mutable_header()->set_seq(yuv_image->header().seq());
  viz_image->mutable_header()->set_frameid(yuv_image->header().frame_id());
  auto raw_secs = yuv_image->header().sensor_stamp().camera_stamp();
  auto sec = static_cast<uint32_t>(raw_secs);
  auto nsec = static_cast<uint32_t>((raw_secs - sec) * 1e9);
  viz_image->mutable_header()->mutable_timestamp()->set_sec(sec);
  viz_image->mutable_header()->mutable_timestamp()->set_nsec(nsec);
  viz_image->set_format("jpeg");
  viz_image->mutable_data()->resize(buf.size());
  memcpy(viz_image->mutable_data()->data(), buf.data(), buf.size());
  return viz_image;
}

void MapCurveToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                       const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                       const std::string& ns,
                       const adsfi_proto::viz::ColorRGBA& rgba,
                       double line_width, const hozon::hdmap::Curve& curve,
                       adsfi_proto::viz::MarkerArray* ma, bool is_dashed) {
  if (ma == nullptr) {
    return;
  }

  if (curve.segment().empty() ||
      curve.segment().begin()->line_segment().point().empty()) {
    // HLOG_WARN << "empty curve";
    return;
  }

  std::vector<hozon::common::PointENU> pts;
  for (const auto& seg : curve.segment()) {
    for (const auto& pt : seg.line_segment().point()) {
      pts.emplace_back(pt);
    }
  }

  if (pts.size() < 2) {
    HLOG_WARN << "no enough points: " << pts.size();
    return;
  }
  bool is_odd = (pts.size() % 2 != 0);
  // 虚线，但点数目是奇数时，补一个点使LINE_LIST有效
  if (is_dashed && is_odd) {
    pts.emplace_back(pts.back());
  }

  auto* marker_line = ma->add_markers();
  marker_line->mutable_header()->CopyFrom(header);
  marker_line->mutable_lifetime()->CopyFrom(lifetime);
  marker_line->set_ns(ns);
  marker_line->set_id(0);
  if (is_dashed) {
    marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
  } else {
    marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  }
  marker_line->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_line->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_w(1.);
  marker_line->mutable_scale()->set_x(line_width);
  marker_line->mutable_color()->CopyFrom(rgba);
  for (const auto& pt : pts) {
    auto* mpt = marker_line->add_points();
    mpt->set_x(pt.x());
    mpt->set_y(pt.y());
    mpt->set_z(pt.z());
  }

  const auto& start_pt = pts.front();
  auto* marker_start_pt = ma->add_markers();
  marker_start_pt->mutable_header()->CopyFrom(header);
  marker_start_pt->mutable_lifetime()->CopyFrom(lifetime);
  marker_start_pt->set_ns(ns);
  marker_start_pt->set_id(1);
  marker_start_pt->set_type(adsfi_proto::viz::MarkerType::SPHERE);
  marker_start_pt->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_start_pt->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_start_pt->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_start_pt->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_start_pt->mutable_pose()->mutable_orientation()->set_w(1.);
  marker_start_pt->mutable_pose()->mutable_position()->set_x(start_pt.x());
  marker_start_pt->mutable_pose()->mutable_position()->set_y(start_pt.y());
  marker_start_pt->mutable_pose()->mutable_position()->set_z(start_pt.z());
  marker_start_pt->mutable_scale()->set_x(line_width);
  marker_start_pt->mutable_scale()->set_y(line_width);
  marker_start_pt->mutable_scale()->set_z(line_width);
  marker_start_pt->mutable_color()->CopyFrom(rgba);
}

void MapLaneInfoToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                          const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                          const std::string& ns,
                          const adsfi_proto::viz::ColorRGBA& rgba,
                          const hozon::hdmap::Lane& lane,
                          adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  const auto& central_seg = lane.central_curve().segment();
  if (central_seg.empty() ||
      central_seg.begin()->line_segment().point().empty()) {
    HLOG_WARN << "empty central";
    return;
  }

  const auto first_center_pt =
      *central_seg.begin()->line_segment().point().begin();
  auto* marker = ma->add_markers();
  marker->mutable_header()->CopyFrom(header);
  marker->mutable_lifetime()->CopyFrom(lifetime);
  marker->set_ns(ns);
  marker->set_id(0);
  marker->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker->mutable_pose()->mutable_position()->set_x(first_center_pt.x());
  marker->mutable_pose()->mutable_position()->set_y(first_center_pt.y());
  marker->mutable_pose()->mutable_position()->set_z(first_center_pt.z());
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker->mutable_scale()->set_z(text_size);
  marker->mutable_color()->CopyFrom(rgba);

  std::string id = "id: " + lane.id().id();
  std::ostringstream out;
  out.precision(3);
  out << std::fixed << lane.length();
  std::string length = "length: " + out.str();
  out.str("");
  out << std::fixed << lane.speed_limit();
  std::string speed_limit = "speed_limit: " + out.str();
  std::string prev = "prev: ";
  std::string temp;
  for (const auto& it : lane.predecessor_id()) {
    if (temp.empty()) {
      temp += (it.id());
    } else {
      temp += ("," + it.id());
    }
  }
  prev += temp;
  std::string next = "next: ";
  temp.clear();
  for (const auto& it : lane.successor_id()) {
    if (temp.empty()) {
      temp += (it.id());
    } else {
      temp += ("," + it.id());
    }
  }
  next += temp;
  std::string left = "left: ";
  temp.clear();
  for (const auto& it : lane.left_neighbor_forward_lane_id()) {
    if (temp.empty()) {
      temp += (it.id());
    } else {
      temp += ("," + it.id());
    }
  }
  left += temp;
  std::string right = "right: ";
  temp.clear();
  for (const auto& it : lane.right_neighbor_forward_lane_id()) {
    if (temp.empty()) {
      temp += (it.id());
    } else {
      temp += ("," + it.id());
    }
  }
  right += temp;
  std::string type = "type: " + hozon::hdmap::Lane_LaneType_Name(lane.type());
  std::string turn = "turn: " + hozon::hdmap::Lane_LaneTurn_Name(lane.turn());

  auto* text = marker->mutable_text();
  *text = (id + "\n" + length + "\n" + speed_limit + "\n" + prev + "\n" + next +
           "\n" + left + "\n" + right + "\n" + type + "\n" + turn);
}

void MapLaneBoundaryToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                              const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                              const std::string& ns,
                              const adsfi_proto::viz::ColorRGBA& rgba,
                              const hozon::hdmap::LaneBoundary& boundary,
                              adsfi_proto::viz::MarkerArray* ma,
                              bool auto_color) {
  if (ma == nullptr) {
    return;
  }

  const auto& curve = boundary.curve();
  if (curve.segment().empty() ||
      curve.segment().begin()->line_segment().point().empty()) {
    // HLOG_WARN << "empty curve";
    return;
  }

  std::string type_txt;
  std::unordered_map<hozon::hdmap::LaneBoundaryType_Type, std::string>
      boundary_types = {
          {hozon::hdmap::LaneBoundaryType_Type_UNKNOWN, "UNKNOWN"},
          {hozon::hdmap::LaneBoundaryType_Type_DOTTED_YELLOW, "DOTTED_YELLOW"},
          {hozon::hdmap::LaneBoundaryType_Type_DOTTED_WHITE, "DOTTED_WHITE"},
          {hozon::hdmap::LaneBoundaryType_Type_SOLID_YELLOW, "SOLID_YELLOW"},
          {hozon::hdmap::LaneBoundaryType_Type_SOLID_WHITE, "SOLID_WHITE"},
          {hozon::hdmap::LaneBoundaryType_Type_DOUBLE_YELLOW, "DOUBLE_YELLOW"},
          {hozon::hdmap::LaneBoundaryType_Type_CURB, "CURB"},
          {hozon::hdmap::LaneBoundaryType_Type_BARRIER, "BARRIER"},
      };

  hozon::hdmap::LaneBoundaryType_Type boundary_type =
      hozon::hdmap::LaneBoundaryType_Type_UNKNOWN;

  //! 注意：这里只使用第一个boundary_type里的第一个type
  if (!boundary.boundary_type().empty() &&
      !boundary.boundary_type().begin()->types().empty()) {
    boundary_type = static_cast<hdmap::LaneBoundaryType_Type>(
        *boundary.boundary_type().begin()->types().begin());
    if (boundary_types.find(boundary_type) == boundary_types.end()) {
      type_txt = "INVALID_BOUNDARY_TYPE";
    } else {
      type_txt = boundary_types[boundary_type];
    }
  }

  Color color = WHITE;
  bool is_dashed = false;
  switch (boundary_type) {
    case hozon::hdmap::LaneBoundaryType_Type_DOTTED_YELLOW: {
      color = YELLOW;
      is_dashed = true;
    } break;
    case hozon::hdmap::LaneBoundaryType_Type_DOTTED_WHITE: {
      color = WHITE;
      is_dashed = true;
    } break;
    case hozon::hdmap::LaneBoundaryType_Type_SOLID_YELLOW: {
      color = YELLOW;
      is_dashed = false;
    } break;
    case hozon::hdmap::LaneBoundaryType_Type_SOLID_WHITE: {
      color = WHITE;
      is_dashed = false;
    } break;
    case hozon::hdmap::LaneBoundaryType_Type_DOUBLE_YELLOW: {
      color = YELLOW;
      is_dashed = false;
    } break;
    case hozon::hdmap::LaneBoundaryType_Type_CURB:
    case hozon::hdmap::LaneBoundaryType_Type_BARRIER: {
      color = RED;
      is_dashed = false;
    } break;
    case hozon::hdmap::LaneBoundaryType_Type_UNKNOWN:
    default: {
      color = GREY;
      is_dashed = false;
    } break;
  }

  adsfi_proto::viz::ColorRGBA used_rgba(rgba);
  if (auto_color) {
    Rgb rgb = ColorRgb(color);
    used_rgba.set_a(1.0);
    used_rgba.set_r(rgb.r);
    used_rgba.set_g(rgb.g);
    used_rgba.set_b(rgb.b);
  }

  const std::string curve_ns = ns + "/curve";
  const double line_width = 0.05;
  MapCurveToMarkers(header, lifetime, curve_ns, used_rgba, line_width, curve,
                    ma, is_dashed);

  const std::string info_ns = ns + "/info";
  const auto first_pt =
      *curve.segment().begin()->line_segment().point().begin();
  auto* marker = ma->add_markers();
  marker->mutable_header()->CopyFrom(header);
  marker->mutable_lifetime()->CopyFrom(lifetime);
  marker->set_ns(ns);
  marker->set_id(0);
  marker->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker->mutable_pose()->mutable_position()->set_x(first_pt.x());
  marker->mutable_pose()->mutable_position()->set_y(first_pt.y());
  marker->mutable_pose()->mutable_position()->set_z(first_pt.z());
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker->mutable_scale()->set_z(text_size);
  marker->mutable_color()->CopyFrom(used_rgba);

  std::string virtual_txt = "virtual: ";
  virtual_txt.append(boundary.virtual_() ? "TRUE" : "FALSE");
  type_txt = "boundary_type: " + type_txt;
  auto* text = marker->mutable_text();
  *text = (virtual_txt + "\n" + type_txt);
}

void MapLaneToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                      const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                      const hozon::hdmap::Lane& lane,
                      adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  const std::string lane_ns = "lane_" + lane.id().id();
  const double line_width = 0.05;

  // some lane info
  const auto& central_seg = lane.central_curve().segment();
  if (!central_seg.empty() &&
      !central_seg.begin()->line_segment().point().empty()) {
    const std::string text_ns = lane_ns + "/info";
    auto rgb = ColorRgb(CYAN);
    adsfi_proto::viz::ColorRGBA rgba;
    rgba.set_a(1.0);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    MapLaneInfoToMarkers(header, lifetime, text_ns, rgba, lane, ma);
  }

  // central_curve
  if (!central_seg.empty()) {
    const std::string center_ns = lane_ns + "/center";
    auto rgb = ColorRgb(CYAN);
    adsfi_proto::viz::ColorRGBA rgba;
    rgba.set_a(1.0);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    MapCurveToMarkers(header, lifetime, center_ns, rgba, line_width,
                      lane.central_curve(), ma, false);
  }

  // left_boundary
  const std::string left_ns = lane_ns + "/left";
  MapLaneBoundaryToMarkers(header, lifetime, left_ns,
                           adsfi_proto::viz::ColorRGBA(), lane.left_boundary(),
                           ma, true);

  // right_boundary
  const std::string right_ns = lane_ns + "/right";
  MapLaneBoundaryToMarkers(header, lifetime, right_ns,
                           adsfi_proto::viz::ColorRGBA(), lane.right_boundary(),
                           ma, true);
}

void MapRoadToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                      const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                      const hozon::hdmap::Road& road,
                      adsfi_proto::viz::MarkerArray* ma) {
  // id
  // section, 相邻section间用不同颜色区分
  //   id, lane_id, BoundaryEdge::type
  PRINT_TBD
}

void MapCrosswalkToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                           const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                           const hozon::hdmap::Crosswalk& crosswalk,
                           adsfi_proto::viz::MarkerArray* ma) {
  PRINT_TBD
}

void MapArrowToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                       const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                       const hozon::hdmap::ArrowData& arrow,
                       adsfi_proto::viz::MarkerArray* ma) {
  PRINT_TBD
}

void MapStopLineToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                          const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                          const hozon::hdmap::StopLine& stop_line,
                          adsfi_proto::viz::MarkerArray* ma) {
  PRINT_TBD
}

void MapToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                  const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                  const hozon::hdmap::Map& map,
                  adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  for (const auto& lane : map.lane()) {
    MapLaneToMarkers(header, lifetime, lane, ma);
  }

  for (const auto& road : map.road()) {
    MapRoadToMarkers(header, lifetime, road, ma);
  }
}

void RoutingToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                      const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                      const hozon::routing::RoutingResponse& routing,
                      const hozon::hdmap::Map& map,
                      adsfi_proto::viz::MarkerArray* ma) {
  std::map<std::string, hozon::routing::ChangeLaneType> lanes_in_routing;
  for (const auto& road : routing.road()) {
    for (const auto& psg : road.passage()) {
      for (const auto& seg : psg.segment()) {
        lanes_in_routing.insert_or_assign(seg.id(), psg.change_lane_type());
      }
    }
  }

  const std::string routing_ns = "routing";
  const double line_width = 1.0;
  auto rgb = ColorRgb(GREEN);
  adsfi_proto::viz::ColorRGBA rgba;
  rgba.set_a(0.1);
  rgba.set_r(rgb.r);
  rgba.set_g(rgb.g);
  rgba.set_b(rgb.b);
  for (const auto& lane : map.lane()) {
    if (lanes_in_routing.find(lane.id().id()) == lanes_in_routing.end()) {
      continue;
    }
    const auto& curve = lane.central_curve();
    if (curve.segment().empty() ||
        curve.segment().begin()->line_segment().point().empty()) {
      continue;
    }

    const std::string routing_lane_ns = routing_ns + "/lane_" + lane.id().id();
    MapCurveToMarkers(header, lifetime, routing_lane_ns, rgba, line_width,
                      lane.central_curve(), ma, false);
  }

  const std::string routing_way_pts_ns = routing_ns + "/way_pts";
  auto* marker_pts = ma->add_markers();
  marker_pts->mutable_header()->CopyFrom(header);
  marker_pts->mutable_lifetime()->CopyFrom(lifetime);
  marker_pts->set_ns(routing_way_pts_ns);
  marker_pts->set_id(0);
  marker_pts->set_type(adsfi_proto::viz::MarkerType::SPHERE_LIST);
  marker_pts->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_pts->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_pts->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_pts->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_pts->mutable_pose()->mutable_orientation()->set_w(1.);
  const double pts_scale = 2.0;
  auto pts_rgba = rgba;
  pts_rgba.set_a(0.5);
  marker_pts->mutable_scale()->set_x(pts_scale);
  marker_pts->mutable_scale()->set_y(pts_scale);
  marker_pts->mutable_scale()->set_z(pts_scale);
  marker_pts->mutable_color()->CopyFrom(pts_rgba);
  for (const auto& pt : routing.routing_request().waypoint()) {
    auto* mpt = marker_pts->add_points();
    mpt->set_x(pt.pose().x());
    mpt->set_y(pt.pose().y());
    mpt->set_z(pt.pose().z());
    auto* color = marker_pts->add_colors();
    color->CopyFrom(pts_rgba);
  }
}

void MapMsgStatusToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                           const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                           const hozon::navigation_hdmap::MapMsg& map_msg,
                           adsfi_proto::viz::MarkerArray* ma) {
  const auto& map = map_msg.hdmap();
  const auto& routing = map_msg.routing();

  std::set<std::string> lanes_in_map;
  for (const auto& lane : map.lane()) {
    lanes_in_map.insert(lane.id().id());
  }

  std::vector<std::string> lanes_not_in_map;
  for (const auto& road : routing.road()) {
    for (const auto& psg : road.passage()) {
      for (const auto& seg : psg.segment()) {
        if (lanes_in_map.count(seg.id()) != 0) {
          continue;
        }
        lanes_not_in_map.emplace_back(seg.id());
      }
    }
  }

  bool has_map_type = map_msg.has_map_type();
  std::string type_txt;
  if (has_map_type) {
    type_txt = hozon::navigation_hdmap::MapMsg_MapType_Name(map_msg.map_type());
  }

  auto rgb = ColorRgb(GREEN);
  if (has_map_type &&
      map_msg.map_type() ==
          hozon::navigation_hdmap::MapMsg_MapType::MapMsg_MapType_PERCEP_MAP) {
    rgb = ColorRgb(mp::util::YELLOW);
  } else if ((has_map_type && map_msg.map_type() ==
                                  hozon::navigation_hdmap::MapMsg_MapType::
                                      MapMsg_MapType_INVALID) ||
             routing.routing_request().waypoint().empty()) {
    rgb = ColorRgb(mp::util::RED);
  }

  adsfi_proto::viz::ColorRGBA rgba;
  rgba.set_a(1.0);
  rgba.set_r(rgb.r);
  rgba.set_g(rgb.g);
  rgba.set_b(rgb.b);

  const std::string status_ns = "map_msg_status";
  const double text_size = 1.0;

  auto* marker_txt = ma->add_markers();
  marker_txt->mutable_header()->CopyFrom(header);
  marker_txt->mutable_lifetime()->CopyFrom(lifetime);
  marker_txt->set_ns(status_ns);
  marker_txt->set_id(0);
  marker_txt->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_txt->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_txt->mutable_pose()->mutable_position()->set_x(0);
  marker_txt->mutable_pose()->mutable_position()->set_y(-10);
  marker_txt->mutable_pose()->mutable_position()->set_z(10);
  marker_txt->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_txt->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_txt->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_txt->mutable_pose()->mutable_orientation()->set_w(1.);
  marker_txt->mutable_scale()->set_z(text_size);
  marker_txt->mutable_color()->CopyFrom(rgba);

  std::string type_str;
  if (has_map_type) {
    type_str = "map_type: " + type_txt;
  }
  //! NOTE:
  //! routing包含三公里的lane，因此会有很多lane在map里找不到，因此这里不再关注.
  // std::string rout_lanes_not_found = "rout_lanes_not_found: ";
  // std::string temp;
  // for (const auto& it : lanes_not_in_map) {
  //   if (temp.empty()) {
  //     temp += it;
  //   } else {
  //     temp += ("," + it);
  //   }
  // }
  // rout_lanes_not_found += temp;

  std::string way_pts =
      "way_points_num: " +
      std::to_string(routing.routing_request().waypoint().size());

  std::string valid_str;
  if (map_msg.has_is_valid()) {
    valid_str = "valid: " + (map_msg.is_valid() ? std::string("TRUE")
                                                : std::string("FALSE"));
  }

  std::string level_str;
  if (map_msg.has_fault_level()) {
    level_str = "level: " + std::to_string(map_msg.fault_level());
  }

  auto* text = marker_txt->mutable_text();
  if (!type_str.empty()) {
    *text += (type_str + "\n");
  }
  if (!way_pts.empty()) {
    *text += (way_pts + "\n");
  }
  if (!valid_str.empty()) {
    *text += (valid_str + "\n");
  }
  if (!level_str.empty()) {
    *text += (level_str + "\n");
  }
}

void MapMsgToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                     const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                     const hozon::navigation_hdmap::MapMsg& map_msg,
                     adsfi_proto::viz::MarkerArray* map_ma,
                     adsfi_proto::viz::MarkerArray* routing_ma,
                     adsfi_proto::viz::MarkerArray* status_ma) {
  if (map_ma != nullptr) {
    MapToMarkers(header, lifetime, map_msg.hdmap(), map_ma);
  }

  if (routing_ma != nullptr) {
    RoutingToMarkers(header, lifetime, map_msg.routing(), map_msg.hdmap(),
                     routing_ma);
  }

  if (status_ma != nullptr) {
    auto status_header = header;
    status_header.set_frameid(kFrameVehicle);
    MapMsgStatusToMarkers(status_header, lifetime, map_msg, status_ma);
  }
}

void TeLaneToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                     const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                     const hozon::perception::LaneInfo& lane,
                     const std::string& ns_prefix,
                     const adsfi_proto::viz::ColorRGBA& rgba,
                     adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  if (lane.points().empty()) {
    return;
  }

  const std::string lane_ns = ns_prefix + "/" + std::to_string(lane.track_id());

  auto* marker_line = ma->add_markers();
  marker_line->mutable_header()->CopyFrom(header);
  marker_line->mutable_lifetime()->CopyFrom(lifetime);
  marker_line->set_ns(lane_ns);
  marker_line->set_id(0);

  marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker_line->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_line->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_w(1.);
  const double line_width = 0.05;
  marker_line->mutable_scale()->set_x(line_width);
  marker_line->mutable_color()->CopyFrom(rgba);
  for (const auto& pt : lane.points()) {
    auto* mpt = marker_line->add_points();
    mpt->set_x(pt.x());
    mpt->set_y(pt.y());
    mpt->set_z(pt.z());
  }

  const auto first_pt = *lane.points().begin();
  auto* marker_info = ma->add_markers();
  marker_info->mutable_header()->CopyFrom(header);
  marker_info->mutable_lifetime()->CopyFrom(lifetime);
  marker_info->set_ns(lane_ns);
  marker_info->set_id(1);

  marker_info->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_info->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_info->mutable_pose()->mutable_position()->set_x(first_pt.x());
  marker_info->mutable_pose()->mutable_position()->set_y(first_pt.y());
  marker_info->mutable_pose()->mutable_position()->set_z(first_pt.z());
  marker_info->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker_info->mutable_scale()->set_z(text_size);
  marker_info->mutable_color()->CopyFrom(marker_line->color());

  std::string track_id = "track_id: " + std::to_string(lane.track_id());
  std::string lane_type =
      "lane_type: " + hozon::perception::LaneType_Name(lane.lanetype());
  std::string lane_pos =
      "lane_pos: " + hozon::perception::LanePositionType_Name(lane.lanepos());
  std::ostringstream out;
  out.precision(3);
  out << std::fixed << lane.confidence();
  std::string confidence = "confidence: " + out.str();
  std::string use_type =
      "use_type: " + hozon::perception::LaneUseType_Name(lane.use_type());
  std::string color = "color: " + hozon::perception::Color_Name(lane.color());
  std::string scene_type =
      "scene_type: " +
      hozon::perception::LaneInfo::LaneLineSceneType_Name(lane.scene_type());
  auto* text = marker_info->mutable_text();
  *text = (track_id + "\n" + lane_type + "\n" + lane_pos + "\n" + confidence +
           "\n" + use_type + "\n" + color + "\n" + scene_type);
}

void TeRoadEdgeToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                         const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                         const hozon::perception::RoadEdge& edge,
                         const std::string& ns_prefix,
                         const adsfi_proto::viz::ColorRGBA& rgba,
                         adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  if (edge.points().empty()) {
    return;
  }

  const std::string edge_ns = ns_prefix + "/" + std::to_string(edge.id());

  auto* marker_line = ma->add_markers();
  marker_line->mutable_header()->CopyFrom(header);
  marker_line->mutable_lifetime()->CopyFrom(lifetime);
  marker_line->set_ns(edge_ns);
  marker_line->set_id(0);

  marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker_line->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_line->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_w(1.);
  const double line_width = 0.05;
  marker_line->mutable_scale()->set_x(line_width);
  marker_line->mutable_color()->CopyFrom(rgba);
  for (const auto& pt : edge.points()) {
    auto* mpt = marker_line->add_points();
    mpt->set_x(pt.x());
    mpt->set_y(pt.y());
    mpt->set_z(pt.z());
  }

  const auto first_pt = *edge.points().begin();
  auto* marker_info = ma->add_markers();
  marker_info->mutable_header()->CopyFrom(header);
  marker_info->mutable_lifetime()->CopyFrom(lifetime);
  marker_info->set_ns(edge_ns);
  marker_info->set_id(1);

  marker_info->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_info->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_info->mutable_pose()->mutable_position()->set_x(first_pt.x());
  marker_info->mutable_pose()->mutable_position()->set_y(first_pt.y());
  marker_info->mutable_pose()->mutable_position()->set_z(first_pt.z());
  marker_info->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker_info->mutable_scale()->set_z(text_size);
  marker_info->mutable_color()->CopyFrom(marker_line->color());

  std::string id = "id: " + std::to_string(edge.id());
  std::string type =
      "type: " + hozon::perception::RoadEdge_RoadEdgeType_Name(edge.type());
  std::ostringstream out;
  out.precision(3);
  out << std::fixed << edge.confidence();
  std::string confidence = "confidence: " + out.str();
  auto* text = marker_info->mutable_text();
  *text = (id + "\n" + type + "\n" + confidence);
}

void TeArrowToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                      const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                      const hozon::perception::Arrow& arrow,
                      const std::string& ns_prefix,
                      const adsfi_proto::viz::ColorRGBA& rgba,
                      adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  if (arrow.points().point().empty()) {
    return;
  }

  const std::string arrow_ns =
      ns_prefix + "/" + std::to_string(arrow.track_id());

  auto* marker_line = ma->add_markers();
  marker_line->mutable_header()->CopyFrom(header);
  marker_line->mutable_lifetime()->CopyFrom(lifetime);
  marker_line->set_ns(arrow_ns);
  marker_line->set_id(0);

  marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker_line->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_line->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_w(1.);
  const double line_width = 0.3;
  marker_line->mutable_scale()->set_x(line_width);
  marker_line->mutable_color()->CopyFrom(rgba);
  for (const auto& pt : arrow.points().point()) {
    auto* mpt = marker_line->add_points();
    mpt->set_x(pt.x());
    mpt->set_y(pt.y());
    mpt->set_z(pt.z());
  }

  // 再加一个点使其封闭
  const auto first_pt = *arrow.points().point().begin();
  auto* mpt = marker_line->add_points();
  mpt->set_x(first_pt.x());
  mpt->set_y(first_pt.y());
  mpt->set_z(first_pt.z());

  double cent_x = 0;
  double cent_y = 0;
  double cent_z = 0;
  for (const auto& pt : arrow.points().point()) {
    cent_x += pt.x();
    cent_y += pt.y();
    cent_z += pt.z();
  }
  cent_x /= arrow.points().point().size();
  cent_y /= arrow.points().point().size();
  cent_z /= arrow.points().point().size();

  auto* marker_info = ma->add_markers();
  marker_info->mutable_header()->CopyFrom(header);
  marker_info->mutable_lifetime()->CopyFrom(lifetime);
  marker_info->set_ns(arrow_ns);
  marker_info->set_id(1);

  marker_info->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_info->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_info->mutable_pose()->mutable_position()->set_x(cent_x);
  marker_info->mutable_pose()->mutable_position()->set_y(cent_y);
  marker_info->mutable_pose()->mutable_position()->set_z(cent_z);
  marker_info->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker_info->mutable_scale()->set_z(text_size);
  marker_info->mutable_color()->CopyFrom(marker_line->color());

  std::string track_id = "track_id: " + std::to_string(arrow.track_id());
  std::string type = "type: " + hozon::perception::ArrowType_Name(arrow.type());
  std::ostringstream out;
  out.precision(3);
  out << std::fixed << arrow.heading();
  std::string heading = "heading: " + out.str();
  out.str("");
  out << std::fixed << arrow.confidence();
  std::string confidence = "confidence: " + out.str();
  auto* text = marker_info->mutable_text();
  *text = (track_id + "\n" + type + "\n" + heading + "\n" + confidence);
}

void TeStopLineToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                         const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                         const hozon::perception::StopLine& stop_line,
                         const std::string& ns_prefix,
                         const adsfi_proto::viz::ColorRGBA& rgba,
                         adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  const std::string stop_line_ns =
      ns_prefix + "/" + std::to_string(stop_line.track_id());

  auto* marker_line = ma->add_markers();
  marker_line->mutable_header()->CopyFrom(header);
  marker_line->mutable_lifetime()->CopyFrom(lifetime);
  marker_line->set_ns(stop_line_ns);
  marker_line->set_id(0);

  marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker_line->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_line->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_w(1.);
  const double line_width = 0.4;
  marker_line->mutable_scale()->set_x(line_width);
  marker_line->mutable_color()->CopyFrom(rgba);

  auto* mpt = marker_line->add_points();
  mpt->set_x(stop_line.left_point().x());
  mpt->set_y(stop_line.left_point().y());
  mpt->set_z(stop_line.left_point().z());
  mpt = marker_line->add_points();
  mpt->set_x(stop_line.right_point().x());
  mpt->set_y(stop_line.right_point().y());
  mpt->set_z(stop_line.right_point().z());

  auto* marker_info = ma->add_markers();
  marker_info->mutable_header()->CopyFrom(header);
  marker_info->mutable_lifetime()->CopyFrom(lifetime);
  marker_info->set_ns(stop_line_ns);
  marker_info->set_id(1);

  marker_info->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_info->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_info->mutable_pose()->mutable_position()->set_x(
      stop_line.right_point().x());
  marker_info->mutable_pose()->mutable_position()->set_y(
      stop_line.right_point().y());
  marker_info->mutable_pose()->mutable_position()->set_z(
      stop_line.right_point().z());
  marker_info->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker_info->mutable_scale()->set_z(text_size);
  marker_info->mutable_color()->CopyFrom(marker_line->color());

  std::string track_id = "track_id: " + std::to_string(stop_line.track_id());
  std::ostringstream out;
  out.precision(3);
  out << std::fixed << stop_line.confidence();
  std::string confidence = "confidence: " + out.str();
  auto* text = marker_info->mutable_text();
  *text = (track_id + "\n" + confidence);
}

void TeCrossWalkToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                          const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                          const hozon::perception::ZebraCrossing& cross_walk,
                          const std::string& ns_prefix,
                          const adsfi_proto::viz::ColorRGBA& rgba,
                          adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  if (cross_walk.points().point().empty()) {
    return;
  }

  const std::string cross_walk_ns =
      ns_prefix + "/" + std::to_string(cross_walk.track_id());

  auto* marker_line = ma->add_markers();
  marker_line->mutable_header()->CopyFrom(header);
  marker_line->mutable_lifetime()->CopyFrom(lifetime);
  marker_line->set_ns(cross_walk_ns);
  marker_line->set_id(0);

  marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker_line->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_line->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_w(1.);
  const double line_width = 0.3;
  marker_line->mutable_scale()->set_x(line_width);
  marker_line->mutable_color()->CopyFrom(rgba);
  for (const auto& pt : cross_walk.points().point()) {
    auto* mpt = marker_line->add_points();
    mpt->set_x(pt.x());
    mpt->set_y(pt.y());
    mpt->set_z(pt.z());
  }

  // 再加一个点使其封闭
  const auto first_pt = *cross_walk.points().point().begin();
  auto* mpt = marker_line->add_points();
  mpt->set_x(first_pt.x());
  mpt->set_y(first_pt.y());
  mpt->set_z(first_pt.z());

  auto* marker_info = ma->add_markers();
  marker_info->mutable_header()->CopyFrom(header);
  marker_info->mutable_lifetime()->CopyFrom(lifetime);
  marker_info->set_ns(cross_walk_ns);
  marker_info->set_id(1);

  marker_info->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_info->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_info->mutable_pose()->mutable_position()->set_x(first_pt.x());
  marker_info->mutable_pose()->mutable_position()->set_y(first_pt.y());
  marker_info->mutable_pose()->mutable_position()->set_z(first_pt.z());
  marker_info->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker_info->mutable_scale()->set_z(text_size);
  marker_info->mutable_color()->CopyFrom(marker_line->color());

  std::string track_id = "track_id: " + std::to_string(cross_walk.track_id());
  std::ostringstream out;
  out.precision(3);
  out << std::fixed << cross_walk.heading();
  std::string heading = "heading: " + out.str();
  out.str("");
  out << std::fixed << cross_walk.confidence();
  std::string confidence = "confidence: " + out.str();
  auto* text = marker_info->mutable_text();
  *text = (track_id + "\n" + heading + "\n" + confidence);
}

void TransportElementToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                               const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                               const hozon::perception::TransportElement& te,
                               adsfi_proto::viz::MarkerArray* te_ma) {
  if (te_ma == nullptr) {
    return;
  }

  const std::string ns = "transport_element/";
  for (const auto& lane : te.lane()) {
    std::string lane_ns = ns + "lane/";
    adsfi_proto::viz::ColorRGBA rgba;
    Rgb rgb = ColorRgb(YELLOW);
    rgba.set_a(0.5);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    TeLaneToMarkers(header, lifetime, lane, lane_ns, rgba, te_ma);
  }

  for (const auto& edge : te.road_edges()) {
    std::string edge_ns = ns + "edge/";
    adsfi_proto::viz::ColorRGBA rgba;
    Rgb rgb = ColorRgb(RED);
    rgba.set_a(0.5);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    TeRoadEdgeToMarkers(header, lifetime, edge, edge_ns, rgba, te_ma);
  }

  for (const auto& arrow : te.arrow()) {
    std::string arrow_ns = ns + "arrow/";
    adsfi_proto::viz::ColorRGBA rgba;
    Rgb rgb = ColorRgb(ORANGE);
    rgba.set_a(0.5);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    TeArrowToMarkers(header, lifetime, arrow, arrow_ns, rgba, te_ma);
  }

  for (const auto& stop_line : te.stopline()) {
    std::string stop_line_ns = ns + "stop_line/";
    adsfi_proto::viz::ColorRGBA rgba;
    Rgb rgb = ColorRgb(YELLOW);
    rgba.set_a(0.5);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    TeStopLineToMarkers(header, lifetime, stop_line, stop_line_ns, rgba, te_ma);
  }

  for (const auto& cross_walk : te.zebra_crossing()) {
    std::string cross_walk_ns = ns + "cross_walk/";
    adsfi_proto::viz::ColorRGBA rgba;
    Rgb rgb = ColorRgb(YELLOW);
    rgba.set_a(0.5);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    TeCrossWalkToMarkers(header, lifetime, cross_walk, cross_walk_ns, rgba,
                         te_ma);
  }
}

void LmRoadEdgeToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                         const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                         const hozon::mapping::RoadEdge& line,
                         const std::string& ns_prefix,
                         const adsfi_proto::viz::ColorRGBA& rgba,
                         adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  if (line.points().empty()) {
    return;
  }

  const std::string lane_ns = ns_prefix + "/" + std::to_string(line.track_id());

  auto* marker_line = ma->add_markers();
  marker_line->mutable_header()->CopyFrom(header);
  marker_line->mutable_lifetime()->CopyFrom(lifetime);
  marker_line->set_ns(lane_ns);
  marker_line->set_id(0);

  marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker_line->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_line->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_w(1.);
  const double line_width = 0.05;
  marker_line->mutable_scale()->set_x(line_width);
  marker_line->mutable_color()->CopyFrom(rgba);
  for (const auto& pt : line.points()) {
    auto* mpt = marker_line->add_points();
    mpt->set_x(pt.x());
    mpt->set_y(pt.y());
    mpt->set_z(pt.z());
  }

  const auto first_pt = *line.points().begin();
  auto* marker_info = ma->add_markers();
  marker_info->mutable_header()->CopyFrom(header);
  marker_info->mutable_lifetime()->CopyFrom(lifetime);
  marker_info->set_ns(lane_ns);
  marker_info->set_id(1);

  marker_info->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_info->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_info->mutable_pose()->mutable_position()->set_x(first_pt.x());
  marker_info->mutable_pose()->mutable_position()->set_y(first_pt.y());
  marker_info->mutable_pose()->mutable_position()->set_z(first_pt.z());
  marker_info->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker_info->mutable_scale()->set_z(text_size);
  marker_info->mutable_color()->CopyFrom(marker_line->color());

  std::string track_id = "track_id: " + std::to_string(line.track_id());
  // std::string lane_type =
  //     "lane_type: " + hozon::mapping::LaneType_Name(line.type());
  std::string lane_type = "roadedge_type: ";
  std::string lane_pos =
      "lane_pos: " + hozon::mapping::LanePositionType_Name(line.lanepos());
  std::ostringstream out;
  out.precision(3);
  out << std::fixed << line.confidence();
  std::string confidence = "confidence: " + out.str();
  std::string use_type =
      "use_type: " + hozon::mapping::LaneUseType_Name(line.use_type());
  auto* text = marker_info->mutable_text();
  *text = (track_id + "\n" + lane_type + "\n" + lane_pos + "\n" + confidence +
           "\n" + use_type + "\n");
}

void LmLaneLineToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                         const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                         const hozon::mapping::LaneLine& line,
                         const std::string& ns_prefix,
                         const adsfi_proto::viz::ColorRGBA& rgba,
                         adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  if (line.points().empty()) {
    return;
  }

  const std::string lane_ns = ns_prefix + "/" + std::to_string(line.track_id());

  auto* marker_line = ma->add_markers();
  marker_line->mutable_header()->CopyFrom(header);
  marker_line->mutable_lifetime()->CopyFrom(lifetime);
  marker_line->set_ns(lane_ns);
  marker_line->set_id(0);

  marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker_line->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_line->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_w(1.);
  const double line_width = 0.05;
  marker_line->mutable_scale()->set_x(line_width);
  marker_line->mutable_color()->CopyFrom(rgba);
  for (const auto& pt : line.points()) {
    auto* mpt = marker_line->add_points();
    mpt->set_x(pt.x());
    mpt->set_y(pt.y());
    mpt->set_z(pt.z());
  }

  const auto first_pt = *line.points().begin();
  auto* marker_info = ma->add_markers();
  marker_info->mutable_header()->CopyFrom(header);
  marker_info->mutable_lifetime()->CopyFrom(lifetime);
  marker_info->set_ns(lane_ns);
  marker_info->set_id(1);

  marker_info->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_info->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_info->mutable_pose()->mutable_position()->set_x(first_pt.x());
  marker_info->mutable_pose()->mutable_position()->set_y(first_pt.y());
  marker_info->mutable_pose()->mutable_position()->set_z(first_pt.z());
  marker_info->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker_info->mutable_scale()->set_z(text_size);
  marker_info->mutable_color()->CopyFrom(marker_line->color());

  std::string track_id = "track_id: " + std::to_string(line.track_id());
  std::string lane_type =
      "lane_type: " + hozon::mapping::LaneType_Name(line.lanetype());
  std::string lane_pos =
      "lane_pos: " + hozon::mapping::LanePositionType_Name(line.lanepos());
  std::ostringstream out;
  out.precision(3);
  out << std::fixed << line.confidence();
  std::string confidence = "confidence: " + out.str();
  std::string use_type =
      "use_type: " + hozon::mapping::LaneUseType_Name(line.use_type());
  std::string color = "color: " + hozon::mapping::Color_Name(line.color());
  auto* text = marker_info->mutable_text();
  *text = (track_id + "\n" + lane_type + "\n" + lane_pos + "\n" + confidence +
           "\n" + use_type + "\n" + color);
}

void LmStopLineToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                         const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                         const hozon::mapping::StopLine& stop_line,
                         const std::string& ns_prefix,
                         const adsfi_proto::viz::ColorRGBA& rgba,
                         adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  const std::string stop_line_ns =
      ns_prefix + "/" + std::to_string(stop_line.track_id());

  auto* marker_line = ma->add_markers();
  marker_line->mutable_header()->CopyFrom(header);
  marker_line->mutable_lifetime()->CopyFrom(lifetime);
  marker_line->set_ns(stop_line_ns);
  marker_line->set_id(0);

  marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker_line->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_line->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_w(1.);
  const double line_width = 0.2;
  marker_line->mutable_scale()->set_x(line_width);
  marker_line->mutable_color()->CopyFrom(rgba);

  auto* mpt = marker_line->add_points();
  mpt->set_x(stop_line.left_point().x());
  mpt->set_y(stop_line.left_point().y());
  mpt->set_z(stop_line.left_point().z());
  mpt = marker_line->add_points();
  mpt->set_x(stop_line.right_point().x());
  mpt->set_y(stop_line.right_point().y());
  mpt->set_z(stop_line.right_point().z());

  auto* marker_info = ma->add_markers();
  marker_info->mutable_header()->CopyFrom(header);
  marker_info->mutable_lifetime()->CopyFrom(lifetime);
  marker_info->set_ns(stop_line_ns);
  marker_info->set_id(1);

  marker_info->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_info->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_info->mutable_pose()->mutable_position()->set_x(
      stop_line.right_point().x());
  marker_info->mutable_pose()->mutable_position()->set_y(
      stop_line.right_point().y());
  marker_info->mutable_pose()->mutable_position()->set_z(
      stop_line.right_point().z());
  marker_info->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker_info->mutable_scale()->set_z(text_size);
  marker_info->mutable_color()->CopyFrom(marker_line->color());

  std::string track_id = "track_id: " + std::to_string(stop_line.track_id());
  auto* text = marker_info->mutable_text();
  *text = track_id;
}

void LmCrossWalkToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                          const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                          const hozon::mapping::CrossWalk& cross_walk,
                          const std::string& ns_prefix,
                          const adsfi_proto::viz::ColorRGBA& rgba,
                          adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  if (cross_walk.points().point().empty()) {
    return;
  }

  const std::string cross_walk_ns =
      ns_prefix + "/" + std::to_string(cross_walk.track_id());

  auto* marker_line = ma->add_markers();
  marker_line->mutable_header()->CopyFrom(header);
  marker_line->mutable_lifetime()->CopyFrom(lifetime);
  marker_line->set_ns(cross_walk_ns);
  marker_line->set_id(0);

  marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker_line->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_line->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_w(1.);
  const double line_width = 0.2;
  marker_line->mutable_scale()->set_x(line_width);
  marker_line->mutable_color()->CopyFrom(rgba);
  for (const auto& pt : cross_walk.points().point()) {
    auto* mpt = marker_line->add_points();
    mpt->set_x(pt.x());
    mpt->set_y(pt.y());
    mpt->set_z(pt.z());
  }

  // 再加一个点使其封闭
  const auto first_pt = *cross_walk.points().point().begin();
  auto* mpt = marker_line->add_points();
  mpt->set_x(first_pt.x());
  mpt->set_y(first_pt.y());
  mpt->set_z(first_pt.z());

  auto* marker_info = ma->add_markers();
  marker_info->mutable_header()->CopyFrom(header);
  marker_info->mutable_lifetime()->CopyFrom(lifetime);
  marker_info->set_ns(cross_walk_ns);
  marker_info->set_id(1);

  marker_info->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_info->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_info->mutable_pose()->mutable_position()->set_x(first_pt.x());
  marker_info->mutable_pose()->mutable_position()->set_y(first_pt.y());
  marker_info->mutable_pose()->mutable_position()->set_z(first_pt.z());
  marker_info->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker_info->mutable_scale()->set_z(text_size);
  marker_info->mutable_color()->CopyFrom(marker_line->color());

  std::string track_id = "track_id: " + std::to_string(cross_walk.track_id());
  auto* text = marker_info->mutable_text();
  *text = track_id;
}

void LmArrowToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                      const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                      const hozon::mapping::Arrow& arrow,
                      const std::string& ns_prefix,
                      const adsfi_proto::viz::ColorRGBA& rgba,
                      adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  if (arrow.points().point().empty()) {
    return;
  }

  const std::string arrow_ns =
      ns_prefix + "/" + std::to_string(arrow.track_id());

  auto* marker_line = ma->add_markers();
  marker_line->mutable_header()->CopyFrom(header);
  marker_line->mutable_lifetime()->CopyFrom(lifetime);
  marker_line->set_ns(arrow_ns);
  marker_line->set_id(0);

  marker_line->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker_line->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_line->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_line->mutable_pose()->mutable_orientation()->set_w(1.);
  const double line_width = 0.2;
  marker_line->mutable_scale()->set_x(line_width);
  marker_line->mutable_color()->CopyFrom(rgba);
  for (const auto& pt : arrow.points().point()) {
    auto* mpt = marker_line->add_points();
    mpt->set_x(pt.x());
    mpt->set_y(pt.y());
    mpt->set_z(pt.z());
  }

  // 再加一个点使其封闭
  const auto first_pt = *arrow.points().point().begin();
  auto* mpt = marker_line->add_points();
  mpt->set_x(first_pt.x());
  mpt->set_y(first_pt.y());
  mpt->set_z(first_pt.z());

  double cent_x = 0;
  double cent_y = 0;
  double cent_z = 0;
  for (const auto& pt : arrow.points().point()) {
    cent_x += pt.x();
    cent_y += pt.y();
    cent_z += pt.z();
  }
  cent_x /= arrow.points().point().size();
  cent_y /= arrow.points().point().size();
  cent_z /= arrow.points().point().size();

  auto* marker_info = ma->add_markers();
  marker_info->mutable_header()->CopyFrom(header);
  marker_info->mutable_lifetime()->CopyFrom(lifetime);
  marker_info->set_ns(arrow_ns);
  marker_info->set_id(1);

  marker_info->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker_info->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker_info->mutable_pose()->mutable_position()->set_x(cent_x);
  marker_info->mutable_pose()->mutable_position()->set_y(cent_y);
  marker_info->mutable_pose()->mutable_position()->set_z(cent_z);
  marker_info->mutable_pose()->mutable_orientation()->set_x(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_y(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_z(0.);
  marker_info->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 0.3;
  marker_info->mutable_scale()->set_z(text_size);
  marker_info->mutable_color()->CopyFrom(marker_line->color());

  std::string track_id = "track_id: " + std::to_string(arrow.track_id());
  std::string type =
      "type: " + hozon::hdmap::ArrowData_Type_Name(arrow.arrow_type());
  std::ostringstream out;
  out.precision(3);
  out << std::fixed << arrow.heading();
  std::string heading = "heading: " + out.str();
  auto* text = marker_info->mutable_text();
  *text = (track_id + "\n" + type + "\n" + heading);
}

void LocalMapToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                       const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                       const hozon::mapping::LocalMap& local_map,
                       adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }

  const std::string& ns = "local_map/";
  for (const auto& lane_line : local_map.lane_lines()) {
    std::string lane_line_ns = ns + "lane_line/";
    adsfi_proto::viz::ColorRGBA rgba;
    Rgb rgb = ColorRgb(GREEN);
    rgba.set_a(0.8);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    LmLaneLineToMarkers(header, lifetime, lane_line, lane_line_ns, rgba, ma);
  }

  for (const auto& edge_line : local_map.road_edges()) {
    std::string edge_line_ns = ns + "edge_line/";
    adsfi_proto::viz::ColorRGBA rgba;
    Rgb rgb = ColorRgb(PURPLE);
    rgba.set_a(0.8);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    LmRoadEdgeToMarkers(header, lifetime, edge_line, edge_line_ns, rgba, ma);
  }

  for (const auto& stop_line : local_map.stop_lines()) {
    std::string stop_line_ns = ns + "stop_line/";
    adsfi_proto::viz::ColorRGBA rgba;
    Rgb rgb = ColorRgb(ORANGE);
    rgba.set_a(0.8);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    LmStopLineToMarkers(header, lifetime, stop_line, stop_line_ns, rgba, ma);
  }

  for (const auto& cross_walk : local_map.cross_walks()) {
    std::string cross_walk_ns = ns + "cross_walk/";
    adsfi_proto::viz::ColorRGBA rgba;
    Rgb rgb = ColorRgb(ORANGE);
    rgba.set_a(0.8);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    LmCrossWalkToMarkers(header, lifetime, cross_walk, cross_walk_ns, rgba, ma);
  }

  for (const auto& arrow : local_map.arrows()) {
    std::string arrow_ns = ns + "arrow/";
    adsfi_proto::viz::ColorRGBA rgba;
    Rgb rgb = ColorRgb(ORANGE);
    rgba.set_a(0.8);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);
    LmArrowToMarkers(header, lifetime, arrow, arrow_ns, rgba, ma);
  }
}

void ObjToMarkers(const adsfi_proto::hz_Adsfi::AlgHeader& header,
                  const adsfi_proto::hz_Adsfi::HafTime& lifetime,
                  const hozon::perception::PerceptionObstacles& objs,
                  adsfi_proto::viz::MarkerArray* ma) {
  if (ma == nullptr) {
    return;
  }
  const std::string& ns = "perception/";
  for (const auto& object : objs.perception_obstacle()) {
    if (object.type() != hozon::perception::PerceptionObstacle::VEHICLE ||
        object.position().x() <= 0 || object.velocity().x() > 0 ||
        (PI * 3 / -4 < object.theta() && object.theta() < PI * 3 / 4)) {
      continue;
    }
    std::string obj_ns = ns + "obj/";
    adsfi_proto::viz::ColorRGBA rgba;
    Rgb rgb = ColorRgb(ORANGE);
    rgba.set_a(0.8);
    rgba.set_r(rgb.r);
    rgba.set_g(rgb.g);
    rgba.set_b(rgb.b);

    const std::string object_point_ns =
        obj_ns + "/" + std::to_string(object.track_id());

    auto* marker_point = ma->add_markers();
    marker_point->mutable_header()->CopyFrom(header);
    marker_point->mutable_lifetime()->CopyFrom(lifetime);
    marker_point->set_ns(object_point_ns);
    marker_point->set_id(0);

    marker_point->set_type(adsfi_proto::viz::MarkerType::POINTS);
    marker_point->set_action(adsfi_proto::viz::MarkerAction::ADD);
    marker_point->mutable_pose()->mutable_position()->set_x(0);
    marker_point->mutable_pose()->mutable_position()->set_y(0);
    marker_point->mutable_pose()->mutable_position()->set_z(0);
    marker_point->mutable_pose()->mutable_orientation()->set_x(0.);
    marker_point->mutable_pose()->mutable_orientation()->set_y(0.);
    marker_point->mutable_pose()->mutable_orientation()->set_z(0.);
    marker_point->mutable_pose()->mutable_orientation()->set_w(1.);
    const double point_size = 1.0;
    marker_point->mutable_scale()->set_x(point_size);
    marker_point->mutable_scale()->set_y(point_size);
    marker_point->mutable_scale()->set_z(point_size);
    marker_point->mutable_color()->CopyFrom(rgba);

    auto* mpt = marker_point->add_points();
    mpt->set_x(object.position().x());
    mpt->set_y(object.position().y());
    mpt->set_z(object.position().z());
  }
}

}  // namespace util
}  // namespace mp
}  // namespace hozon
