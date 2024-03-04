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

void DataConvert::SetPerceptionEnv(
    const std::shared_ptr<hozon::perception::base::FusionFrame>& msg,
    Perception* perception) {
  perception->timestamp_ = msg->header.timestamp;
  SetLaneLine(msg->scene_->lane_lines->lanelines, &perception->lane_lines_);
  SetRoadEdge(msg->scene_->road_edges->road_edges, &perception->road_edges_);
  SetStopLine(msg->scene_->stop_lines->stoplines, &perception->stop_lines_);
  SetArrow(msg->scene_->road_arrows->arrows, &perception->arrows_);
  SetZebraCrossing(msg->scene_->zebra_crossings->crosswalks,
                   &perception->zebra_crossings_);
}

void DataConvert::SetPerceptionObj(
    const std::shared_ptr<hozon::perception::base::MeasurementFrame>& msg,
    Perception* perception) {
  for (const auto& measure_obj : msg->objects_measurement_->objects_) {
    perception->objects_.push_back(*measure_obj);
  }
}

void SetIns(const hozon::localization::HafNodeInfo& msg, InsData* ins) {
  ins->position_.x() = msg.pos_gcj02().x();
  ins->position_.y() = msg.pos_gcj02().y();
  ins->position_.z() = msg.pos_gcj02().z();

  ins->quaternion_.w() = msg.quaternion().w();
  ins->quaternion_.x() = msg.quaternion().x();
  ins->quaternion_.y() = msg.quaternion().y();
  ins->quaternion_.z() = msg.quaternion().z();
}

void DataConvert::SetLaneLine(
    std::vector<hozon::perception::base::LaneLinePtr> perception_lanelines,
    std::vector<LaneLine>* lane_lines) {
  for (const auto& lane_line : perception_lanelines) {
    LaneLine lane_line_tmp;
    lane_line_tmp.track_id_ = lane_line->id;
    ConvertStructLanePos(lane_line->position, &lane_line_tmp.lanepos_);
    ConvertStructLaneType(lane_line->type, &lane_line_tmp.lanetype_);
    ConvertStructColor(lane_line->color, &lane_line_tmp.color_);
    for (const auto& laneline_point : lane_line->point_set) {
      const auto point = laneline_point.vehicle_point;
      lane_line_tmp.points_.emplace_back(point.x, point.y, point.z);
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
      double y = CommonUtil::CalCubicCurveY(lane_line_tmp, x);
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

void DataConvert::SetRoadEdge(
    std::vector<hozon::perception::base::RoadEdgePtr> perception_roadedges,
    std::vector<RoadEdge>* road_edges) {
  for (const auto& road_edge : perception_roadedges) {
    if (road_edge->point_set.empty()) {
      continue;
    }
    RoadEdge road_edge_tmp;
    road_edge_tmp.track_id_ = road_edge->id;
    road_edge_tmp.confidence_ = road_edge->confidence;
    ConvertStrutEdgeType(road_edge->type, &road_edge_tmp.edgetype_);
    for (const auto& laneline_point : road_edge->point_set) {
      const auto& point = laneline_point.vehicle_point;
      Eigen::Vector3d point_tmp = {point.x, point.y, point.z};
      road_edge_tmp.points_.push_back(point_tmp);
    }
    road_edge_tmp.start_point_x_ = road_edge_tmp.points_.front().x();
    road_edge_tmp.end_point_x_ = road_edge_tmp.points_.back().x();

    if (road_edge_tmp.points_.empty() || road_edge_tmp.end_point_x_ > 150) {
      continue;
    }
    std::vector<double> c(4);
    if (road_edge_tmp.points_.size() < 4) {
      continue;
    }
    CommonUtil::FitLaneLine(road_edge_tmp.points_, &c);
    road_edge_tmp.c0_ = c[0];
    road_edge_tmp.c1_ = c[1];
    road_edge_tmp.c2_ = c[2];
    road_edge_tmp.c3_ = c[3];
    // lane_line_tmp.points_.clear();
    double x = road_edge_tmp.start_point_x_;
    while (x < road_edge_tmp.end_point_x_) {
      double y = CommonUtil::CalCubicCurveY(road_edge_tmp, x);
      road_edge_tmp.points_.emplace_back(x, y, 0);
      x = x + 1.0;
    }
    std::sort(road_edge_tmp.points_.begin(), road_edge_tmp.points_.end(),
              [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                return a.x() < b.x();
              });
    road_edges->emplace_back(road_edge_tmp);
  }
}

void DataConvert::SetStopLine(
    std::vector<hozon::perception::base::StopLinePtr> stoplines,
    std::vector<StopLine>* stop_lines) {
  HLOG_INFO << "=======test stopline===============";
  for (const auto& stop_line : stoplines) {
    if (stop_line->point_set_3d.front().y - stop_line->point_set_3d.back().y <
            3 ||
        fabs(stop_line->point_set_3d.front().x -
             stop_line->point_set_3d.back().x) > 1) {
      continue;
    }
    StopLine stop_line_tmp;
    stop_line_tmp.confidence_ = stop_line->confidence;
    stop_line_tmp.track_id_ = stop_line->id;
    stop_line_tmp.left_point_ = Eigen::Vector3d{
        stop_line->point_set_3d.front().x, stop_line->point_set_3d.front().y,
        stop_line->point_set_3d.front().z};
    stop_line_tmp.right_point_ = Eigen::Vector3d{
        stop_line->point_set_3d.back().x, stop_line->point_set_3d.back().y,
        stop_line->point_set_3d.back().z};
    stop_line_tmp.mid_point_.x() =
        (stop_line->point_set_3d.front().x + stop_line->point_set_3d.back().x) /
        2;
    stop_line_tmp.mid_point_.y() =
        (stop_line->point_set_3d.front().y + stop_line->point_set_3d.back().y) /
        2;
    stop_line_tmp.mid_point_.z() =
        (stop_line->point_set_3d.front().z + stop_line->point_set_3d.back().z) /
        2;
    stop_line_tmp.length_ = CommonUtil::CalTwoPointsDis(
        stop_line_tmp.left_point_, stop_line_tmp.right_point_);
    stop_line_tmp.heading_ = CommonUtil::CalTwoPointsHeading(
        stop_line_tmp.left_point_, stop_line_tmp.right_point_);
    stop_lines->emplace_back(stop_line_tmp);
  }
}

void DataConvert::SetArrow(
    std::vector<hozon::perception::base::ArrowPtr> perception_arrows,
    std::vector<Arrow>* arrows) {
  for (const auto& arrow : perception_arrows) {
    Arrow arrow_tmp;
    arrow_tmp.track_id_ = arrow->id;
    arrow_tmp.confidence_ = arrow->confidence;
    arrow_tmp.points_.is_closure = false;
    std::vector<Eigen::Vector3d> points;
    for (const auto& point : arrow->point_set_3d) {
      points.emplace_back(point.x, point.y, point.z);
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
    ConvertStructArrowType(arrow->type, &arrow_tmp.type_);
    arrows->emplace_back(arrow_tmp);
  }
}

void DataConvert::SetZebraCrossing(
    std::vector<hozon::perception::base::ZebraCrossingPtr>
        perception_zebracrosswalks,
    std::vector<ZebraCrossing>* zebra_crossings) {
  for (const auto& zebra_crossing : perception_zebracrosswalks) {
    ZebraCrossing zebra_crossing_tmp;
    zebra_crossing_tmp.track_id_ = zebra_crossing->id;
    zebra_crossing_tmp.confidence_ = zebra_crossing->confidence;
    zebra_crossing_tmp.points_.is_closure = false;
    std::vector<Eigen::Vector3d> points;
    for (const auto& point : zebra_crossing->point_set_3d) {
      points.emplace_back(point.x, point.y, point.z);
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

void DataConvert::ConvertStructLanePos(
    const hozon::perception::base::LaneLinePosition& raw_lanepos,
    LanePositionType* lanepos) {
  switch (raw_lanepos) {
    case hozon::perception::base::LaneLinePosition::BOLLARD_LEFT:
      *lanepos = LanePositionType::BOLLARD_LEFT;
      break;
    case hozon::perception::base::LaneLinePosition::FOURTH_LEFT:
      *lanepos = LanePositionType::FOURTH_LEFT;
      break;
    case hozon::perception::base::LaneLinePosition::THIRD_LEFT:
      *lanepos = LanePositionType::THIRD_LEFT;
      break;
    case hozon::perception::base::LaneLinePosition::ADJACENT_LEFT:
      *lanepos = LanePositionType::ADJACENT_LEFT;
      break;
    case hozon::perception::base::LaneLinePosition::EGO_LEFT:
      *lanepos = LanePositionType::EGO_LEFT;
      break;
    case hozon::perception::base::LaneLinePosition::EGO_RIGHT:
      *lanepos = LanePositionType::EGO_RIGHT;
      break;
    case hozon::perception::base::LaneLinePosition::ADJACENT_RIGHT:
      *lanepos = LanePositionType::ADJACENT_RIGHT;
      break;
    case hozon::perception::base::LaneLinePosition::THIRD_RIGHT:
      *lanepos = LanePositionType::THIRD_RIGHT;
      break;
    case hozon::perception::base::LaneLinePosition::FOURTH_RIGHT:
      *lanepos = LanePositionType::FOURTH_RIGHT;
      break;
    case hozon::perception::base::LaneLinePosition::BOLLARD_RIGHT:
      *lanepos = LanePositionType::BOLLARD_RIGHT;
      break;
    case hozon::perception::base::LaneLinePosition::OTHER:
      *lanepos = LanePositionType::OTHER;
      break;
    default:
      break;
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

void DataConvert::ConvertStructLaneType(
    const hozon::perception::base::LaneLineType& raw_lanetype,
    LaneType* lanetype) {
  switch (raw_lanetype) {
    case hozon::perception::base::LaneLineType::Unknown:
      *lanetype = LaneType::Unknown;
      break;
    case hozon::perception::base::LaneLineType::SolidLine:
      *lanetype = LaneType::SolidLine;
      break;
    case hozon::perception::base::LaneLineType::DashedLine:
      *lanetype = LaneType::DashedLine;
      break;
    case hozon::perception::base::LaneLineType::ShortDashedLine:
      *lanetype = LaneType::ShortDashedLine;
      break;
    case hozon::perception::base::LaneLineType::DoubleSolidLine:
      *lanetype = LaneType::DoubleSolidLine;
      break;
    case hozon::perception::base::LaneLineType::DoubleDashedLine:
      *lanetype = LaneType::DoubleDashedLine;
      break;
    case hozon::perception::base::LaneLineType::LeftSolidRightDashed:
      *lanetype = LaneType::LeftSolidRightDashed;
      break;
    case hozon::perception::base::LaneLineType::RightSolidLeftDashed:
      *lanetype = LaneType::RightSolidLeftDashed;
      break;
    case hozon::perception::base::LaneLineType::FishBone:
      *lanetype = LaneType::FishBone;
      break;
    case hozon::perception::base::LaneLineType::CrossGuideLine:
      *lanetype = LaneType::CrossGuideLine;
      break;
    case hozon::perception::base::LaneLineType::Other:
      *lanetype = LaneType::Other;
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

void DataConvert::ConvertStrutEdgeType(
    const hozon::perception::base::RoadEdgeType& raw_edgetype,
    EdgeType* edgetype) {
  switch (raw_edgetype) {
    case hozon::perception::base::RoadEdgeType::UNKNOWN:
      *edgetype = EdgeType::UNKNOWN_EDGE;
      break;
    case hozon::perception::base::RoadEdgeType::CONE_EDGE:
      *edgetype = EdgeType::CONE_EDGE;
      break;
    case hozon::perception::base::RoadEdgeType::FENCE_EDGE:
      *edgetype = EdgeType::FENCE_EDGE;
      break;
    case hozon::perception::base::RoadEdgeType::GROUND_EDGE:
      *edgetype = EdgeType::GROUND_EDGE;
      break;
    case hozon::perception::base::RoadEdgeType::ROAD_EDGE:
      *edgetype = EdgeType::ROAD_EDGE;
      break;
    case hozon::perception::base::RoadEdgeType::WATERHORSE_EDGE:
      *edgetype = EdgeType::WATERHORSE_EDGE;
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

void DataConvert::ConvertStructArrowType(
    const hozon::perception::base::ArrowType& raw_arrowtype,
    ArrowType* arrowtype) {
  switch (raw_arrowtype) {
    case hozon::perception::base::ArrowType::UNKNOWN:
      *arrowtype = ArrowType::ARROWTYPE_UNKNOWN;
      break;
    case hozon::perception::base::ArrowType::STRAIGHT_FORWARD:
      *arrowtype = ArrowType::STRAIGHT_FORWARD;
      break;
    case hozon::perception::base::ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT:
      *arrowtype = ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT;
      break;
    case hozon::perception::base::ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT:
      *arrowtype = ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT;
      break;
    case hozon::perception::base::ArrowType::
        STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT:
      *arrowtype = ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT;
      break;
    case hozon::perception::base::ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND:
      *arrowtype = ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND;
    case hozon::perception::base::ArrowType::
        STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT:
      *arrowtype = ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT;
    case hozon::perception::base::ArrowType::TURN_LEFT:
      *arrowtype = ArrowType::TURN_LEFT;
    case hozon::perception::base::ArrowType::TURN_LEFT_OR_MERGE_LEFT:
      *arrowtype = ArrowType::TURN_LEFT_OR_MERGE_LEFT;
    case hozon::perception::base::ArrowType::TURN_LEFT_OR_TURN_AROUND:
      *arrowtype = ArrowType::TURN_LEFT_OR_TURN_AROUND;
    case hozon::perception::base::ArrowType::TURN_LEFT_OR_TURN_RIGHT:
      *arrowtype = ArrowType::TURN_LEFT_OR_TURN_RIGHT;
      break;
    case hozon::perception::base::ArrowType::TURN_RIGHT:
      *arrowtype = ArrowType::TURN_RIGHT;
      break;
    case hozon::perception::base::ArrowType::TURN_RIGHT_OR_MERGE_RIGHT:
      *arrowtype = ArrowType::TURN_RIGHT_OR_MERGE_RIGHT;
      break;
    case hozon::perception::base::ArrowType::TURN_RIGHT_OR_TURN_AROUND:
      *arrowtype = ArrowType::TURN_RIGHT_OR_TURN_AROUND;
      break;
    case hozon::perception::base::ArrowType::TURN_AROUND:
      *arrowtype = ArrowType::TURN_AROUND;
      break;
    case hozon::perception::base::ArrowType::FORBID_TURN_LEFT:
      *arrowtype = ArrowType::FORBID_TURN_LEFT;
      break;
    case hozon::perception::base::ArrowType::FORBID_TURN_RIGHT:
      *arrowtype = ArrowType::FORBID_TURN_RIGHT;
    case hozon::perception::base::ArrowType::FORBID_TURN_AROUND:
      *arrowtype = ArrowType::FORBID_TURN_AROUND;
      break;
    case hozon::perception::base::ArrowType::FRONT_NEAR_CROSSWALK:
      *arrowtype = ArrowType::FRONT_NEAR_CROSSWALK;
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

void DataConvert::ConvertStructColor(
    const hozon::perception::base::LaneLineColor& raw_color, Color* color) {
  switch (raw_color) {
    case hozon::perception::base::LaneLineColor::UNKNOWN:
      *color = Color::UNKNOWN;
      break;
    case hozon::perception::base::LaneLineColor::WHITE:
      *color = Color::WHITE;
      break;
    case hozon::perception::base::LaneLineColor::YELLOW:
      *color = Color::YELLOW;
      break;
    case hozon::perception::base::LaneLineColor::GREEN:
      *color = Color::GREEN;
      break;
    case hozon::perception::base::LaneLineColor::RED:
      *color = Color::RED;
      break;
    case hozon::perception::base::LaneLineColor::BLACK:
      *color = Color::BLACK;
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

void DataConvert::ConvertInnerColor(const Color& inner_color,
                                    hozon::mapping::Color* color) {
  switch (inner_color) {
    case Color::UNKNOWN:
      *color = hozon::mapping::Color::UNKNOWN;
      break;
    case Color::BLACK:
      *color = hozon::mapping::Color::BLACK;
      break;
    case Color::GREEN:
      *color = hozon::mapping::Color::GREEN;
      break;
    case Color::RED:
      *color = hozon::mapping::Color::RED;
      break;
    case Color::WHITE:
      *color = hozon::mapping::Color::WHITE;
      break;
    case Color::YELLOW:
      *color = hozon::mapping::Color::YELLOW;
      break;
    default:
      break;
  }
}

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

void DataConvert::ConvertInnerArrowType(
    const ArrowType& inner_arrowtype,
    hozon::hdmap::ArrowData::Type* arrowtype) {
  switch (inner_arrowtype) {
    case ArrowType::ARROWTYPE_UNKNOWN:
      *arrowtype = hozon::hdmap::ArrowData::Type::ArrowData_Type_UNKNOWN_TURN;
      break;
    case ArrowType::FORBID_TURN_LEFT:
      *arrowtype =
          hozon::hdmap::ArrowData::Type::ArrowData_Type_FORBID_LEFT_TURN;
      break;
    case ArrowType::FORBID_TURN_RIGHT:
      *arrowtype =
          hozon::hdmap::ArrowData::Type::ArrowData_Type_FORBID_RIGHT_TURN;
      break;
    case ArrowType::FORBID_TURN_AROUND:
      *arrowtype = hozon::hdmap::ArrowData::Type::ArrowData_Type_FORBID_U_TURN;
      break;
    case ArrowType::FRONT_NEAR_CROSSWALK:
      *arrowtype =
          hozon::hdmap::ArrowData::Type::ArrowData_Type_FRONT_NEAR_CROSSWALK;
      break;
    case ArrowType::STRAIGHT_FORWARD:
      *arrowtype = hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT;
      break;
    case ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT:
      *arrowtype =
          hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_LEFT_TURN;
      break;
    case ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT:
      *arrowtype =
          hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_RIGHT_TURN;
      break;
    case ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT:
      *arrowtype = hozon::hdmap::ArrowData::Type::
          ArrowData_Type_STRAIGHT_LEFT_RIGHT_TURN;
      break;
    case ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND:
      *arrowtype =
          hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_U_TURN;
      break;
    case ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT:
      *arrowtype =
          hozon::hdmap::ArrowData::Type::ArrowData_Type_STRAIGHT_LEFT_U_TURN;
      break;
    case ArrowType::TURN_LEFT:
      *arrowtype = hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_TURN;
      break;
    case ArrowType::TURN_LEFT_OR_MERGE_LEFT:
      *arrowtype =
          hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_FRONT_TURN;
      break;
    case ArrowType::TURN_LEFT_OR_TURN_AROUND:
      *arrowtype = hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_U_TURN;
      break;
    case ArrowType::TURN_LEFT_OR_TURN_RIGHT:
      *arrowtype =
          hozon::hdmap::ArrowData::Type::ArrowData_Type_LEFT_RIGHT_TURN;
      break;
    case ArrowType::TURN_RIGHT:
      *arrowtype = hozon::hdmap::ArrowData::Type::ArrowData_Type_RIGHT_TURN;
      break;
    case ArrowType::TURN_RIGHT_OR_MERGE_RIGHT:
      *arrowtype =
          hozon::hdmap::ArrowData::Type::ArrowData_Type_RIGHT_FRONT_TURN;
      break;
    case ArrowType::TURN_RIGHT_OR_TURN_AROUND:
      *arrowtype = hozon::hdmap::ArrowData::Type::ArrowData_Type_RIGHT_U_TURN;
      break;
    case ArrowType::TURN_AROUND:
      *arrowtype = hozon::hdmap::ArrowData::Type::ArrowData_Type_U_TURN;
      break;
    default:
      break;
  }
}

void DataConvert::ConvertMultiLaneLinesToPb(
    const std::vector<LaneLine>& lane_lines,
    const std::shared_ptr<hozon::mapping::LocalMap>& localmap) {
  for (const auto& lane_line_msg : lane_lines) {
    if (!lane_line_msg.ismature_) {
      continue;
    }
    hozon::mapping::LaneType lanetype =
        hozon::mapping::LaneType::LaneType_UNKNOWN;
    DataConvert::ConvertInnerLaneType(lane_line_msg.lanetype_, &lanetype);
    hozon::mapping::LanePositionType lanepos =
        hozon::mapping::LanePositionType::LanePositionType_OTHER;
    DataConvert::ConvertInnerLanePos(lane_line_msg.lanepos_, &lanepos);
    hozon::mapping::Color color = hozon::mapping::Color::UNKNOWN;
    DataConvert::ConvertInnerColor(lane_line_msg.color_, &color);
    auto* lane_line = localmap->add_lane_lines();
    lane_line->set_track_id(lane_line_msg.track_id_);
    lane_line->set_lanetype(lanetype);
    lane_line->set_lanepos(lanepos);
    lane_line->set_color(color);
    for (auto point_msg : lane_line_msg.fit_points_) {
      auto* point = lane_line->add_points();
      point->set_x(point_msg.x());
      point->set_y(point_msg.y());
      point->set_z(point_msg.z());
    }
  }
}

void DataConvert::ConvertMultiRoadEdgesToPb(
    const std::vector<RoadEdge>& road_edges,
    const std::shared_ptr<hozon::mapping::LocalMap>& localmap) {
  for (const auto& road_edge_msg : road_edges) {
    if (!road_edge_msg.ismature_) {
      continue;
    }
    auto* road_edge = localmap->add_edge_lines();
    road_edge->set_track_id(road_edge_msg.track_id_);
    hozon::mapping::LanePositionType lanepos =
        hozon::mapping::LanePositionType::LanePositionType_OTHER;
    DataConvert::ConvertInnerLanePos(road_edge_msg.lanepos_, &lanepos);
    road_edge->set_lanepos(lanepos);
    for (auto point_msg : road_edge_msg.fit_points_) {
      auto* point = road_edge->add_points();
      point->set_x(point_msg.x());
      point->set_y(point_msg.y());
      point->set_z(point_msg.z());
    }
  }
}

void DataConvert::ConvertMultiStopLinesToPb(
    const std::vector<StopLine>& stop_lines,
    const std::shared_ptr<hozon::mapping::LocalMap>& localmap) {
  for (const auto& stop_line_msg : stop_lines) {
    if (!stop_line_msg.ismature_) {
      continue;
    }
    auto* stop_line = localmap->add_stop_lines();
    stop_line->set_track_id(stop_line_msg.track_id_);
    Eigen::Vector3d left_point_ = {
        stop_line_msg.mid_point_.x() +
            stop_line_msg.length_ / 2.0 * cos(stop_line_msg.heading_),
        stop_line_msg.mid_point_.y() +
            stop_line_msg.length_ / 2.0 * sin(stop_line_msg.heading_),
        0};
    Eigen::Vector3d right_point_ = {
        stop_line_msg.mid_point_.x() -
            stop_line_msg.length_ / 2.0 * cos(stop_line_msg.heading_),
        stop_line_msg.mid_point_.y() -
            stop_line_msg.length_ / 2.0 * sin(stop_line_msg.heading_),
        0};
    stop_line->mutable_left_point()->set_x(left_point_.x());
    stop_line->mutable_left_point()->set_y(left_point_.y());
    stop_line->mutable_left_point()->set_z(0);
    stop_line->mutable_right_point()->set_x(right_point_.x());
    stop_line->mutable_right_point()->set_y(right_point_.y());
    stop_line->mutable_right_point()->set_z(0);
  }
}

void DataConvert::ConvertMultiArrowsToPb(
    const std::vector<Arrow>& arrows,
    const std::shared_ptr<hozon::mapping::LocalMap>& localmap) {
  for (const auto& arrow_msg : arrows) {
    if (!arrow_msg.ismature_ || arrow_msg.length_ == 0 ||
        arrow_msg.width_ == 0) {
      continue;
    }
    auto* arrow = localmap->add_arrows();
    arrow->set_track_id(arrow_msg.track_id_);
    arrow->set_heading(arrow_msg.heading_);
    Eigen::Vector3d l = {arrow_msg.length_ / 2 * cos(arrow_msg.heading_),
                         arrow_msg.length_ / 2 * sin(arrow_msg.heading_), 0};
    Eigen::Vector3d w = {-arrow_msg.width_ / 2 * sin(arrow_msg.heading_),
                         arrow_msg.width_ / 2 * cos(arrow_msg.heading_), 0};
    Eigen::Vector3d point_0 = arrow_msg.mid_point_ + l + w;
    Eigen::Vector3d point_1 = arrow_msg.mid_point_ - l + w;
    Eigen::Vector3d point_2 = arrow_msg.mid_point_ - l - w;
    Eigen::Vector3d point_3 = arrow_msg.mid_point_ + l - w;
    auto* point = arrow->mutable_points()->add_point();
    point->set_x(point_0.x());
    point->set_y(point_0.y());
    point->set_z(point_0.z());
    point = arrow->mutable_points()->add_point();
    point->set_x(point_1.x());
    point->set_y(point_1.y());
    point->set_z(point_1.z());
    point = arrow->mutable_points()->add_point();
    point->set_x(point_2.x());
    point->set_y(point_2.y());
    point->set_z(point_2.z());
    point = arrow->mutable_points()->add_point();
    point->set_x(point_3.x());
    point->set_y(point_3.y());
    point->set_z(point_3.z());
    hozon::hdmap::ArrowData::Type arrowtype =
        hozon::hdmap::ArrowData::Type::ArrowData_Type_UNKNOWN_TURN;
    DataConvert::ConvertInnerArrowType(arrow_msg.type_, &arrowtype);
    arrow->set_arrow_type(arrowtype);
  }
}

void DataConvert::ConvertMultiZebraCrossingsToPb(
    const std::vector<ZebraCrossing>& zebra_crossings,
    const std::shared_ptr<hozon::mapping::LocalMap>& localmap) {
  for (const auto& zebra_crossing_msg : zebra_crossings) {
    if (!zebra_crossing_msg.ismature_ || zebra_crossing_msg.length_ == 0 ||
        zebra_crossing_msg.width_ == 0) {
      continue;
    }
    auto* zebra_crossing = localmap->add_cross_walks();
    zebra_crossing->set_track_id(zebra_crossing_msg.track_id_);
    Eigen::Vector3d l = {
        -zebra_crossing_msg.length_ / 2 * sin(zebra_crossing_msg.heading_),
        zebra_crossing_msg.length_ / 2 * cos(zebra_crossing_msg.heading_), 0};
    Eigen::Vector3d w = {
        zebra_crossing_msg.width_ / 2 * cos(zebra_crossing_msg.heading_),
        zebra_crossing_msg.width_ / 2 * sin(zebra_crossing_msg.heading_), 0};
    Eigen::Vector3d point_0 = zebra_crossing_msg.mid_point_ + l + w;
    Eigen::Vector3d point_1 = zebra_crossing_msg.mid_point_ + l - w;
    Eigen::Vector3d point_2 = zebra_crossing_msg.mid_point_ - l - w;
    Eigen::Vector3d point_3 = zebra_crossing_msg.mid_point_ - l + w;
    auto* point = zebra_crossing->mutable_points()->add_point();
    point->set_x(point_0.x());
    point->set_y(point_0.y());
    point->set_z(point_0.z());
    point = zebra_crossing->mutable_points()->add_point();
    point->set_x(point_1.x());
    point->set_y(point_1.y());
    point->set_z(point_1.z());
    point = zebra_crossing->mutable_points()->add_point();
    point->set_x(point_2.x());
    point->set_y(point_2.y());
    point->set_z(point_2.z());
    point = zebra_crossing->mutable_points()->add_point();
    point->set_x(point_3.x());
    point->set_y(point_3.y());
    point->set_z(point_3.z());
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
