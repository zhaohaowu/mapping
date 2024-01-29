/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-05
 *****************************************************************************/
#include "modules/local_mapping/utils/data_convert.h"

namespace hozon {
namespace mp {
namespace lm {

void DataConvert::SetLocalization(const hozon::localization::Localization& msg,
                                  Localization* localization) {
  localization->timestamp_ = msg.header().data_stamp();
  localization->position_.x() = msg.pose_local().position().x();
  localization->position_.y() = msg.pose_local().position().y();
  localization->position_.z() = msg.pose_local().position().z();
  localization->quaternion_.w() = msg.pose_local().quaternion().w();
  localization->quaternion_.x() = msg.pose_local().quaternion().x();
  localization->quaternion_.y() = msg.pose_local().quaternion().y();
  localization->quaternion_.z() = msg.pose_local().quaternion().z();
  localization->linear_vrf_.x() = msg.pose().linear_velocity().x();
  localization->linear_vrf_.y() = msg.pose().linear_velocity().y();
  localization->linear_vrf_.z() = msg.pose().linear_velocity().z();
  localization->angular_vrf_.x() = msg.pose().angular_velocity().x();
  localization->angular_vrf_.y() = msg.pose().angular_velocity().y();
  localization->angular_vrf_.z() = msg.pose().angular_velocity().z();
}

void DataConvert::SetPerception(const hozon::perception::TransportElement& msg,
                                Perception* perception) {
  perception->timestamp_ = msg.header().data_stamp();
  SetLaneLine(msg, &perception->lane_lines_);
  SetEdgeLine(msg, &perception->edge_lines_);
  SetStopLine(msg, &perception->stop_lines_);
  SetArrow(msg, &perception->arrows_);
  SetZebraCrossing(msg, &perception->zebra_crossings_);
}

void SetIns(const hozon::localization::HafNodeInfo& msg, InsData* ins) {
  ins->position_.x() = msg.pos_gcj02().x();
  ins->position_.y() = msg.pos_gcj02().y();
  ins->position_.z() = msg.pos_gcj02().z();

  ins->quaternion_.w() = msg.quaternion().w();
  ins->quaternion_.x() = msg.quaternion().x();
  ins->quaternion_.y() = msg.quaternion().y();
  ins->quaternion_.z() = msg.quaternion().z();

  // Eigen::Vector3d p_G_V(msg->pos_gcj02().x(), msg->pos_gcj02().y(),
  //                       msg->pos_gcj02().z());
  // Eigen::Quaterniond q_G_V(msg->quaternion().w(), msg->quaternion().x(),
  //                          msg->quaternion().y(), msg->quaternion().z());
}

void DataConvert::SetLaneLine(const hozon::perception::TransportElement& msg,
                              std::vector<LaneLine>* lane_lines) {
  for (const auto& lane_line : msg.lane()) {
    LaneLine lane_line_tmp;
    lane_line_tmp.track_id_ = lane_line.track_id();
    ConvertProtoLanePos(lane_line.lanepos(), &lane_line_tmp.lanepos_);
    ConvertProtoLaneType(lane_line.lanetype(), &lane_line_tmp.lanetype_);
    ConvertProtoColor(lane_line.color(), &lane_line_tmp.color_);
    for (const auto& point : lane_line.points()) {
      lane_line_tmp.points_.emplace_back(point.x(), point.y(), point.z());
    }
    lane_line_tmp.start_point_x_ = lane_line_tmp.points_.front().x();
    lane_line_tmp.end_point_x_ = lane_line_tmp.points_.back().x();
    if (lane_line_tmp.points_.empty()) {
      continue;
    }
    std::vector<double> c(4);
    if (lane_line_tmp.points_.size() < 4) {
      continue;
    }
    CommonUtil::FitLaneLine(lane_line_tmp.points_, &c);
    lane_line_tmp.c0_ = c[0];
    lane_line_tmp.c1_ = c[1];
    lane_line_tmp.c2_ = c[2];
    lane_line_tmp.c3_ = c[3];
    lane_line_tmp.points_.clear();
    double x = lane_line_tmp.start_point_x_;
    while (x < lane_line_tmp.end_point_x_) {
      double y = lane_line_tmp.c0_ + lane_line_tmp.c1_ * x +
                 lane_line_tmp.c2_ * x * x + lane_line_tmp.c3_ * x * x * x;
      lane_line_tmp.points_.emplace_back(x, y, 0);
      x = x + 1.0;
    }
    std::sort(lane_line_tmp.points_.begin(), lane_line_tmp.points_.end(),
              [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                return a.x() < b.x();
              });
    lane_lines->emplace_back(lane_line_tmp);
  }
}

void DataConvert::SetEdgeLine(const hozon::perception::TransportElement& msg,
                              std::vector<LaneLine>* edge_lines) {
  for (const auto& edge_line : msg.road_edges()) {
    if (edge_line.points().empty()) {
      continue;
    }
    LaneLine edge_line_tmp;
    edge_line_tmp.track_id_ = edge_line.id();
    edge_line_tmp.confidence_ = edge_line.confidence();
    ConvertProtoEdgeType(edge_line.type(), &edge_line_tmp.edgetype_);
    for (const auto& point : edge_line.points()) {
      Eigen::Vector3d point_tmp = {point.x(), point.y(), point.z()};
      // HLOG_ERROR << "point_tmp: " << point_tmp;
      edge_line_tmp.points_.push_back(point_tmp);
    }
    edge_line_tmp.start_point_x_ = edge_line.points()[0].x();
    edge_line_tmp.end_point_x_ =
        edge_line.points()[edge_line.points().size() - 1].x();

    if (edge_line_tmp.points_.empty() || edge_line_tmp.end_point_x_ > 150) {
      continue;
    }
    std::vector<double> c(4);
    if (edge_line_tmp.points_.size() < 4) {
      continue;
    }
    CommonUtil::FitLaneLine(edge_line_tmp.points_, &c);
    edge_line_tmp.c0_ = c[0];
    edge_line_tmp.c1_ = c[1];
    edge_line_tmp.c2_ = c[2];
    edge_line_tmp.c3_ = c[3];
    // lane_line_tmp.points_.clear();
    double x = edge_line_tmp.start_point_x_;
    while (x < edge_line_tmp.end_point_x_) {
      double y = edge_line_tmp.c0_ + edge_line_tmp.c1_ * x +
                 edge_line_tmp.c2_ * x * x + edge_line_tmp.c3_ * x * x * x;
      edge_line_tmp.points_.emplace_back(x, y, 0);
      x = x + 1.0;
    }
    std::sort(edge_line_tmp.points_.begin(), edge_line_tmp.points_.end(),
              [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                return a.x() < b.x();
              });
    edge_lines->emplace_back(edge_line_tmp);
  }
}

void DataConvert::SetStopLine(const hozon::perception::TransportElement& msg,
                              std::vector<StopLine>* stop_lines) {
  for (const auto& stop_line : msg.stopline()) {
    if (stop_line.left_point().y() - stop_line.right_point().y() < 3 ||
        fabs(stop_line.left_point().x() - stop_line.right_point().x()) > 1) {
      continue;
    }
    StopLine stop_line_tmp;
    stop_line_tmp.confidence_ = stop_line.confidence();
    stop_line_tmp.track_id_ = stop_line.track_id();
    stop_line_tmp.left_point_ =
        Eigen::Vector3d{stop_line.left_point().x(), stop_line.left_point().y(),
                        stop_line.left_point().z()};
    stop_line_tmp.right_point_ = Eigen::Vector3d{stop_line.right_point().x(),
                                                 stop_line.right_point().y(),
                                                 stop_line.right_point().z()};
    stop_line_tmp.mid_point_.x() =
        (stop_line.left_point().x() + stop_line.right_point().x()) / 2;
    stop_line_tmp.mid_point_.y() =
        (stop_line.left_point().y() + stop_line.right_point().y()) / 2;
    stop_line_tmp.mid_point_.z() =
        (stop_line.left_point().z() + stop_line.right_point().z()) / 2;
    stop_line_tmp.length_ = CommonUtil::CalTwoPointsDis(
        stop_line_tmp.left_point_, stop_line_tmp.right_point_);
    stop_line_tmp.heading_ = CommonUtil::CalTwoPointsHeading(
        stop_line_tmp.left_point_, stop_line_tmp.right_point_);
    stop_lines->emplace_back(stop_line_tmp);
  }
}

void DataConvert::SetArrow(const hozon::perception::TransportElement& msg,
                           std::vector<Arrow>* arrows) {
  for (const auto& arrow : msg.arrow()) {
    Arrow arrow_tmp;
    arrow_tmp.track_id_ = arrow.track_id();
    arrow_tmp.confidence_ = arrow.confidence();
    arrow_tmp.points_.is_closure = arrow.points().is_closure();
    std::vector<Eigen::Vector3d> points;
    for (const auto& point : arrow.points().point()) {
      points.emplace_back(point.x(), point.y(), point.z());
    }
    if (!CommonUtil::IsConvex(points)) {
      continue;
    }
    sort(points.begin(), points.end(),
         [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
           return p1.x() > p2.x();
         });
    std::vector<Eigen::Vector3d> points_order(4);
    if (points[0].y() > points[1].y()) {
      points_order[0] = points[0];
      points_order[3] = points[1];
    } else {
      points_order[0] = points[1];
      points_order[3] = points[0];
    }
    if (points[2].y() > points[3].y()) {
      points_order[1] = points[2];
      points_order[2] = points[3];
    } else {
      points_order[1] = points[3];
      points_order[2] = points[2];
    }
    arrow_tmp.points_.points_ = points_order;
    auto dis = [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
      return std::sqrt(std::pow(p1.x() - p2.x(), 2) +
                       std::pow(p1.y() - p2.y(), 2));
    };
    arrow_tmp.length_ = (dis(points_order[0], points_order[1]) +
                         dis(points_order[2], points_order[3])) /
                        2;
    arrow_tmp.width_ = (dis(points_order[0], points_order[3]) +
                        dis(points_order[1], points_order[2])) /
                       2;
    if (arrow_tmp.length_ < arrow_tmp.width_) {
      continue;
    }
    arrow_tmp.mid_point_ = (points_order[0] + points_order[1] +
                            points_order[2] + points_order[3]) /
                           4;
    Eigen::Vector3d top_point = (points_order[0] + points_order[3]) / 2;
    Eigen::Vector3d bottom_point = (points_order[1] + points_order[2]) / 2;
    arrow_tmp.heading_ = atan2(top_point.y() - bottom_point.y(),
                               top_point.x() - bottom_point.x());
    ConvertProtoArrowType(arrow.type(), &arrow_tmp.type_);
    arrows->emplace_back(arrow_tmp);
  }
}

void DataConvert::SetZebraCrossing(
    const hozon::perception::TransportElement& msg,
    std::vector<ZebraCrossing>* zebra_crossings) {
  for (const auto& zebra_crossing : msg.zebra_crossing()) {
    ZebraCrossing zebra_crossing_tmp;
    zebra_crossing_tmp.track_id_ = zebra_crossing.track_id();
    zebra_crossing_tmp.confidence_ = zebra_crossing.confidence();
    zebra_crossing_tmp.points_.is_closure =
        zebra_crossing.points().is_closure();
    std::vector<Eigen::Vector3d> points;
    for (const auto& point : zebra_crossing.points().point()) {
      points.emplace_back(point.x(), point.y(), point.z());
    }
    if (!CommonUtil::IsConvex(points)) {
      continue;
    }
    sort(points.begin(), points.end(),
         [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
           return p1.y() > p2.y();
         });
    std::vector<Eigen::Vector3d> points_order(4);
    if (points[0].x() > points[1].x()) {
      points_order[0] = points[0];
      points_order[1] = points[1];
    } else {
      points_order[0] = points[1];
      points_order[1] = points[0];
    }
    if (points[2].x() > points[3].x()) {
      points_order[2] = points[3];
      points_order[3] = points[2];
    } else {
      points_order[2] = points[2];
      points_order[3] = points[3];
    }
    zebra_crossing_tmp.points_.points_ = points_order;
    auto dis = [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
      return std::sqrt(std::pow(p1.x() - p2.x(), 2) +
                       std::pow(p1.y() - p2.y(), 2));
    };
    zebra_crossing_tmp.length_ = (dis(points_order[0], points_order[3]) +
                                  dis(points_order[1], points_order[2])) /
                                 2;
    zebra_crossing_tmp.width_ = (dis(points_order[0], points_order[1]) +
                                 dis(points_order[2], points_order[3])) /
                                2;
    if (zebra_crossing_tmp.length_ < zebra_crossing_tmp.width_) {
      continue;
    }
    zebra_crossing_tmp.mid_point_ = (points_order[0] + points_order[1] +
                                     points_order[2] + points_order[3]) /
                                    4;
    Eigen::Vector3d top_point = (points_order[0] + points_order[3]) / 2;
    Eigen::Vector3d bottom_point = (points_order[1] + points_order[2]) / 2;
    zebra_crossing_tmp.heading_ = atan2(top_point.y() - bottom_point.y(),
                                        top_point.x() - bottom_point.x());
    zebra_crossings->emplace_back(zebra_crossing_tmp);
  }
}

void DataConvert::ConvertProtoLanePos(
    const hozon::perception::LanePositionType& raw_lanepos,
    LanePositionType* lanepos) {
  switch (raw_lanepos) {
    case hozon::perception::LanePositionType::BOLLARD_LEFT:
      *lanepos = LanePositionType::BOLLARD_LEFT;
      break;
    case hozon::perception::LanePositionType::FOURTH_LEFT:
      *lanepos = LanePositionType::FOURTH_LEFT;
      break;
    case hozon::perception::LanePositionType::THIRD_LEFT:
      *lanepos = LanePositionType::THIRD_LEFT;
      break;
    case hozon::perception::LanePositionType::ADJACENT_LEFT:
      *lanepos = LanePositionType::ADJACENT_LEFT;
      break;
    case hozon::perception::LanePositionType::EGO_LEFT:
      *lanepos = LanePositionType::EGO_LEFT;
      break;
    case hozon::perception::LanePositionType::EGO_RIGHT:
      *lanepos = LanePositionType::EGO_RIGHT;
      break;
    case hozon::perception::LanePositionType::ADJACENT_RIGHT:
      *lanepos = LanePositionType::ADJACENT_RIGHT;
      break;
    case hozon::perception::LanePositionType::THIRD_RIGHT:
      *lanepos = LanePositionType::THIRD_RIGHT;
      break;
    case hozon::perception::LanePositionType::FOURTH_RIGHT:
      *lanepos = LanePositionType::FOURTH_RIGHT;
      break;
    case hozon::perception::LanePositionType::BOLLARD_RIGHT:
      *lanepos = LanePositionType::BOLLARD_RIGHT;
      break;
    case hozon::perception::LanePositionType::OTHER:
      *lanepos = LanePositionType::OTHER;
      break;
    default:
      break;
  }
}

void DataConvert::ConvertProtoLaneType(
    const hozon::perception::LaneType& raw_lanetype, LaneType* lanetype) {
  switch (raw_lanetype) {
    case hozon::perception::LaneType::Unknown:
      *lanetype = LaneType::Unknown;
      break;
    case hozon::perception::LaneType::SolidLine:
      *lanetype = LaneType::SolidLine;
      break;
    case hozon::perception::LaneType::DashedLine:
      *lanetype = LaneType::DashedLine;
      break;
    case hozon::perception::LaneType::ShortDashedLine:
      *lanetype = LaneType::ShortDashedLine;
      break;
    case hozon::perception::LaneType::DoubleSolidLine:
      *lanetype = LaneType::DoubleSolidLine;
      break;
    case hozon::perception::LaneType::DoubleDashedLine:
      *lanetype = LaneType::DoubleDashedLine;
      break;
    case hozon::perception::LaneType::LeftSolidRightDashed:
      *lanetype = LaneType::LeftSolidRightDashed;
      break;
    case hozon::perception::LaneType::RightSolidLeftDashed:
      *lanetype = LaneType::RightSolidLeftDashed;
      break;
    case hozon::perception::LaneType::ShadedArea:
      *lanetype = LaneType::ShadedArea;
      break;
    case hozon::perception::LaneType::LaneVirtualMarking:
      *lanetype = LaneType::LaneVirtualMarking;
      break;
    case hozon::perception::LaneType::IntersectionVirualMarking:
      *lanetype = LaneType::IntersectionVirualMarking;
      break;
    case hozon::perception::LaneType::CurbVirtualMarking:
      *lanetype = LaneType::CurbVirtualMarking;
      break;
    case hozon::perception::LaneType::UnclosedRoad:
      *lanetype = LaneType::UnclosedRoad;
      break;
    case hozon::perception::LaneType::RoadVirtualLine:
      *lanetype = LaneType::RoadVirtualLine;
      break;
    case hozon::perception::LaneType::LaneChangeVirtualLine:
      *lanetype = LaneType::LaneChangeVirtualLine;
      break;
    case hozon::perception::LaneType::Other:
      *lanetype = LaneType::Other;
      break;
    default:
      break;
  }
}

void DataConvert::ConvertProtoEdgeType(
    const hozon::perception::RoadEdge::RoadEdgeType& raw_edgetype,
    EdgeType* edgetype) {
  switch (raw_edgetype) {
    case hozon::perception::RoadEdge::RoadEdgeType::
        RoadEdge_RoadEdgeType_UNKNOWN:
      *edgetype = EdgeType::UNKNOWN_EDGE;
      break;
    case hozon::perception::RoadEdge::RoadEdgeType::
        RoadEdge_RoadEdgeType_CONE_EDGE:
      *edgetype = EdgeType::CONE_EDGE;
      break;
    case hozon::perception::RoadEdge::RoadEdgeType::
        RoadEdge_RoadEdgeType_FENCE_EDGE:
      *edgetype = EdgeType::FENCE_EDGE;
      break;
    case hozon::perception::RoadEdge::RoadEdgeType::
        RoadEdge_RoadEdgeType_GROUND_EDGE:
      *edgetype = EdgeType::GROUND_EDGE;
      break;
    case hozon::perception::RoadEdge::RoadEdgeType::
        RoadEdge_RoadEdgeType_ROAD_EDGE:
      *edgetype = EdgeType::ROAD_EDGE;
      break;
    case hozon::perception::RoadEdge::RoadEdgeType::
        RoadEdge_RoadEdgeType_WATERHORSE_EDGE:
      *edgetype = EdgeType::WATERHORSE_EDGE;
      break;
    default:
      break;
  }
}

void DataConvert::ConvertProtoArrowType(
    const hozon::perception::ArrowType& raw_arrowtype, ArrowType* arrowtype) {
  switch (raw_arrowtype) {
    case hozon::perception::ArrowType::ARROWTYPE_UNKNOWN:
      *arrowtype = ArrowType::ARROWTYPE_UNKNOWN;
      break;
    case hozon::perception::ArrowType::STRAIGHT_FORWARD:
      *arrowtype = ArrowType::STRAIGHT_FORWARD;
      break;
    case hozon::perception::ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT:
      *arrowtype = ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT;
      break;
    case hozon::perception::ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT:
      *arrowtype = ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT;
      break;
    case hozon::perception::ArrowType::
        STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT:
      *arrowtype = ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT;
      break;
    case hozon::perception::ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND:
      *arrowtype = ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND;
    case hozon::perception::ArrowType::
        STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT:
      *arrowtype = ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT;
    case hozon::perception::ArrowType::TURN_LEFT:
      *arrowtype = ArrowType::TURN_LEFT;
    case hozon::perception::ArrowType::TURN_LEFT_OR_MERGE_LEFT:
      *arrowtype = ArrowType::TURN_LEFT_OR_MERGE_LEFT;
    case hozon::perception::ArrowType::TURN_LEFT_OR_TURN_AROUND:
      *arrowtype = ArrowType::TURN_LEFT_OR_TURN_AROUND;
    case hozon::perception::ArrowType::TURN_LEFT_OR_TURN_RIGHT:
      *arrowtype = ArrowType::TURN_LEFT_OR_TURN_RIGHT;
      break;
    case hozon::perception::ArrowType::TURN_RIGHT:
      *arrowtype = ArrowType::TURN_RIGHT;
      break;
    case hozon::perception::ArrowType::TURN_RIGHT_OR_MERGE_RIGHT:
      *arrowtype = ArrowType::TURN_RIGHT_OR_MERGE_RIGHT;
      break;
    case hozon::perception::ArrowType::TURN_RIGHT_OR_TURN_AROUND:
      *arrowtype = ArrowType::TURN_RIGHT_OR_TURN_AROUND;
      break;
    case hozon::perception::ArrowType::TURN_AROUND:
      *arrowtype = ArrowType::TURN_AROUND;
      break;
    case hozon::perception::ArrowType::FORBID_TURN_LEFT:
      *arrowtype = ArrowType::FORBID_TURN_LEFT;
      break;
    case hozon::perception::ArrowType::FORBID_TURN_RIGHT:
      *arrowtype = ArrowType::FORBID_TURN_RIGHT;
    case hozon::perception::ArrowType::FORBID_TURN_AROUND:
      *arrowtype = ArrowType::FORBID_TURN_AROUND;
      break;
    case hozon::perception::ArrowType::FRONT_NEAR_CROSSWALK:
      *arrowtype = ArrowType::FRONT_NEAR_CROSSWALK;
      break;
    default:
      break;
  }
}

void DataConvert::ConvertProtoColor(const hozon::perception::Color& raw_color,
                                    Color* color) {
  switch (raw_color) {
    case hozon::perception::Color::UNKNOWN:
      *color = Color::UNKNOWN;
      break;
    case hozon::perception::Color::WHITE:
      *color = Color::WHITE;
      break;
    case hozon::perception::Color::YELLOW:
      *color = Color::YELLOW;
      break;
    case hozon::perception::Color::GREEN:
      *color = Color::GREEN;
      break;
    case hozon::perception::Color::RED:
      *color = Color::RED;
      break;
    case hozon::perception::Color::BLACK:
      *color = Color::BLACK;
      break;
    default:
      break;
  }
}

void DataConvert::ConvertInnerLanePos(
    const LanePositionType& inner_lanepos,
    hozon::mapping::LanePositionType* lanepos) {
  switch (inner_lanepos) {
    case LanePositionType::BOLLARD_LEFT:
      *lanepos =
          hozon::mapping::LanePositionType::LanePositionType_BOLLARD_LEFT;
      break;
    case LanePositionType::FOURTH_LEFT:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_FOURTH_LEFT;
      break;
    case LanePositionType::THIRD_LEFT:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_THIRD_LEFT;
      break;
    case LanePositionType::ADJACENT_LEFT:
      *lanepos =
          hozon::mapping::LanePositionType::LanePositionType_ADJACENT_LEFT;
      break;
    case LanePositionType::EGO_LEFT:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_EGO_LEFT;
      break;
    case LanePositionType::EGO_RIGHT:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_EGO_RIGHT;
      break;
    case LanePositionType::ADJACENT_RIGHT:
      *lanepos =
          hozon::mapping::LanePositionType::LanePositionType_ADJACENT_RIGHT;
      break;
    case LanePositionType::THIRD_RIGHT:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_THIRD_RIGHT;
      break;
    case LanePositionType::FOURTH_RIGHT:
      *lanepos =
          hozon::mapping::LanePositionType::LanePositionType_FOURTH_RIGHT;
      break;
    case LanePositionType::BOLLARD_RIGHT:
      *lanepos =
          hozon::mapping::LanePositionType::LanePositionType_BOLLARD_RIGHT;
      break;
    case LanePositionType::OTHER:
      *lanepos = hozon::mapping::LanePositionType::LanePositionType_OTHER;
      break;
    default:
      break;
  }
}

void DataConvert::ConvertInnerLaneType(const LaneType& inner_lanetype,
                                       hozon::mapping::LaneType* lanetype) {
  switch (inner_lanetype) {
    case LaneType::Unknown:
      *lanetype = hozon::mapping::LaneType::LaneType_UNKNOWN;
      break;
    case LaneType::SolidLine:
      *lanetype = hozon::mapping::LaneType::LaneType_SOLID;
      break;
    case LaneType::DashedLine:
      *lanetype = hozon::mapping::LaneType::LaneType_DASHED;
      break;
    case LaneType::ShortDashedLine:
      *lanetype = hozon::mapping::LaneType::LaneType_SHORT_DASHED;
      break;
    case LaneType::DoubleSolidLine:
      *lanetype = hozon::mapping::LaneType::LaneType_DOUBLE_SOLID;
      break;
    case LaneType::DoubleDashedLine:
      *lanetype = hozon::mapping::LaneType::LaneType_DOUBLE_DASHED;
      break;
    case LaneType::LeftSolidRightDashed:
      *lanetype = hozon::mapping::LaneType::LaneType_LEFT_SOLID_RIGHT_DASHED;
      break;
    case LaneType::RightSolidLeftDashed:
      *lanetype = hozon::mapping::LaneType::LaneType_RIGHT_SOLID_LEFT_DASHED;
      break;
    case LaneType::ShadedArea:
      *lanetype = hozon::mapping::LaneType::LaneType_SHADED_AREA;
      break;
    case LaneType::LaneVirtualMarking:
      *lanetype = hozon::mapping::LaneType::LaneType_LANE_VIRTUAL_MARKING;
      break;
    case LaneType::IntersectionVirualMarking:
      *lanetype =
          hozon::mapping::LaneType::LaneType_INTERSECTION_VIRTUAL_MARKING;
      break;
    case LaneType::CurbVirtualMarking:
      *lanetype = hozon::mapping::LaneType::LaneType_CURB_VIRTUAL_MARKING;
      break;
    case LaneType::UnclosedRoad:
      *lanetype = hozon::mapping::LaneType::LaneType_UNCLOSED_ROAD;
      break;
    case LaneType::RoadVirtualLine:
      *lanetype = hozon::mapping::LaneType::LaneType_ROAD_VIRTUAL;
      break;
    case LaneType::LaneChangeVirtualLine:
      *lanetype = hozon::mapping::LaneType::LaneType_LANE_CHANG_VIRTUAL;
      break;
    case LaneType::Other:
      *lanetype = hozon::mapping::LaneType::LaneType_OTHER;
      break;
    default:
      break;
  }
}

// void DataConvert::ConvertInnerColor(const Color& inner_color,
//                                     hozon::mapping::Color* color) {
//   switch (inner_color) {
//     case Color::UNKNOWN:
//       *color = hozon::mapping::Color::UNKNOWN;
//       break;
//     case Color::BLACK:
//       *color = hozon::mapping::Color::BLACK;
//       break;
//     case Color::GREEN:
//       *color = hozon::mapping::Color::GREEN;
//       break;
//     case Color::RED:
//       *color = hozon::mapping::Color::RED;
//       break;
//     case Color::WHITE:
//       *color = hozon::mapping::Color::WHITE;
//       break;
//     case Color::YELLOW:
//       *color = hozon::mapping::Color::YELLOW;
//       break;
//     default:
//       break;
//   }
// }

void DataConvert::ConvertInnerMapLaneType(
    const Color& color, const bool& is_left, const LaneType& inner_lanetype,
    hozon::hdmap::LaneBoundaryType::Type* lanetype) {
  if (inner_lanetype == LaneType::DashedLine ||
      inner_lanetype == LaneType::ShortDashedLine ||
      inner_lanetype == LaneType::DoubleDashedLine ||
      (inner_lanetype == LaneType::LeftSolidRightDashed && is_left) ||
      (inner_lanetype == LaneType::RightSolidLeftDashed && !is_left)) {
    if (color == WHITE) {
      *lanetype = hozon::hdmap::LaneBoundaryType::Type::
          LaneBoundaryType_Type_DOTTED_WHITE;
    } else if (color == YELLOW) {
      *lanetype = hozon::hdmap::LaneBoundaryType::Type::
          LaneBoundaryType_Type_DOTTED_YELLOW;
    }
  } else if (inner_lanetype == LaneType::SolidLine ||
             (inner_lanetype == LaneType::LeftSolidRightDashed && !is_left) ||
             (inner_lanetype == LaneType::RightSolidLeftDashed && is_left)) {
    if (color == WHITE) {
      *lanetype = hozon::hdmap::LaneBoundaryType::Type::
          LaneBoundaryType_Type_SOLID_WHITE;
    } else if (color == YELLOW) {
      *lanetype = hozon::hdmap::LaneBoundaryType::Type::
          LaneBoundaryType_Type_SOLID_YELLOW;
    }
  } else if ((inner_lanetype == LaneType::DoubleDashedLine) &&
             color == YELLOW) {
    *lanetype = hozon::hdmap::LaneBoundaryType::Type::
        LaneBoundaryType_Type_DOUBLE_YELLOW;
  } else {
    *lanetype =
        hozon::hdmap::LaneBoundaryType::Type::LaneBoundaryType_Type_UNKNOWN;
  }
}

// void DataConvert::ConvertInnerArrowType(
//     const ArrowType& inner_arrowtype,
//     hozon::hdmap::ArrowData::Type* arrowtype) {
//   switch (inner_arrowtype) {
//     case ArrowType::ARROWTYPE_UNKNOWN:
//       *arrowtype =
//       hozon::hdmap::ArrowData::Type::ArrowData_Type_UNKNOWN_TURN; break;
//     case ArrowType::FORBID_TURN_LEFT:
//       *arrowtype =
//           hozon::hdmap::ArrowData::Type::ArrowData_Type_FORBID_LEFT_TURN;
//       break;
//     case ArrowType::FORBID_TURN_RIGHT:
//       *arrowtype =
//           hozon::hdmap::ArrowData::Type::ArrowData_Type_FORBID_RIGHT_TURN;
//       break;
//     case ArrowType::FORBID_TURN_AROUND:
//       *arrowtype =
//       hozon::hdmap::ArrowData::Type::ArrowData_Type_FORBID_U_TURN; break;
//     case ArrowType::FRONT_NEAR_CROSSWALK:
//       *arrowtype =
//           hozon::hdmap::ArrowData::Type::ArrowData_Type_FRONT_NEAR_CROSSWALK;
//       break;
//     case ArrowType::STRAIGHT_FORWARD:
//       *arrowtype = hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT;
//       break;
//     case ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT:
//       *arrowtype =
//           hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_LEFT_TURN;
//       break;
//     case ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT:
//       *arrowtype =
//           hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_RIGHT_TURN;
//       break;
//     case ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT:
//       *arrowtype = hozon::hdmap::ArrowData::Type::
//           ArrowData_Type_STRAIGHT_LEFT_RIGHT_TURN;
//       break;
//     case ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND:
//       *arrowtype =
//           hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_U_TURN;
//       break;
//     case ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT:
//       *arrowtype =
//           hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_LEFT_U_TURN;
//       break;
//     case ArrowType::TURN_LEFT:
//       *arrowtype = hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_TURN;
//       break;
//     case ArrowType::TURN_LEFT_OR_MERGE_LEFT:
//       *arrowtype =
//           hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_FRONT_TURN;
//       break;
//     case ArrowType::TURN_LEFT_OR_TURN_AROUND:
//       *arrowtype =
//       hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_U_TURN; break;
//     case ArrowType::TURN_LEFT_OR_TURN_RIGHT:
//       *arrowtype =
//           hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_RIGHT_TURN;
//       break;
//     case ArrowType::TURN_RIGHT:
//       *arrowtype =
//       hozon::hdmap::ArrowData::Type::ArrowData_Type_RIGHT_TURN; break;
//     case ArrowType::TURN_RIGHT_OR_MERGE_RIGHT:
//       *arrowtype =
//           hozon::hdmap::ArrowData::Type::ArrowData_Type_RIGHT_FRONT_TURN;
//       break;
//     case ArrowType::TURN_RIGHT_OR_TURN_AROUND:
//       *arrowtype =
//       hozon::hdmap::ArrowData::Type::ArrowData_Type_RIGHT_U_TURN; break;
//     case ArrowType::TURN_AROUND:
//       *arrowtype = hozon::hdmap::ArrowData::Type::ArrowData_Type_U_TURN;
//       break;
//     default:
//       break;
//   }
// }

}  // namespace lm
}  // namespace mp
}  // namespace hozon
