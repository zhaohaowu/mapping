/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-05
 *****************************************************************************/
#include "modules/local_mapping/ops/lane/lane_op.h"

namespace hozon {
namespace mp {
namespace lm {

void LaneOp::MatchLaneLine(const std::vector<LaneLine>& per_lane_lines,
                           const std::vector<LaneLine>& map_lane_lines,
                           std::vector<LaneLineMatchInfo>* lane_line_matches) {
  std::vector<LaneLine> map_lane_lines_tmp = map_lane_lines;
  for (const auto& per_lane_line : per_lane_lines) {
    ObjUpdateType type = ObjUpdateType::ADD_NEW;
    LaneLineMatchInfo match;
    HLOG_INFO << "per_lane_line.lanepos_ " << per_lane_line.lanepos_;
    double min_ave_dis = 0.5;
    int map_lane_index = 0;
    for (int i = 0; i < static_cast<int>(map_lane_lines_tmp.size()); i++) {
      if (map_lane_lines_tmp[i].has_matched_) {
        continue;
      }
      double sum = 0.0;
      int n = 0;
      double linear_max_x =
          per_lane_line.end_point_x_ * 0.7 + per_lane_line.start_point_x_ * 0.3;
      double curve_max_x =
          per_lane_line.end_point_x_ * 0.5 + per_lane_line.start_point_x_ * 0.5;
      for (const auto& map_point : map_lane_lines_tmp[i].points_) {
        if (per_lane_line.c2_ > 0.001
                ? map_point.x() > curve_max_x
                : map_point.x() > linear_max_x ||
                      map_point.x() < per_lane_line.start_point_x_) {
          continue;
        }
        double per_y = CommonUtil::CalCubicCurveY(per_lane_line, map_point.x());
        sum += fabs(map_point.y() - per_y);
        n++;
      }
      if (n == 0) {
        continue;
      }
      double ave_dis = sum / n;
      HLOG_INFO << "map_lane_lines_tmp[i].lanepos_ "
                << map_lane_lines_tmp[i].lanepos_;
      HLOG_INFO << "ave_dis " << ave_dis << " n " << n;
      if (ave_dis < min_ave_dis && n >= 3) {
        min_ave_dis = ave_dis;
        map_lane_index = i;
      }
    }
    if (min_ave_dis < 0.5) {
      match.per_lane_line_ = per_lane_line;
      match.map_lane_line_ = map_lane_lines_tmp[map_lane_index];
      match.update_type_ = ObjUpdateType::MERGE_OLD;
      type = ObjUpdateType::MERGE_OLD;
      map_lane_lines_tmp[map_lane_index].has_matched_ = true;
    }
    if (type == ObjUpdateType::ADD_NEW) {
      match.per_lane_line_ = per_lane_line;
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    lane_line_matches->emplace_back(match);
  }
}

void LaneOp::MatchEdgeLine(const std::vector<LaneLine>& per_edge_lines,
                           const std::vector<LaneLine>& map_edge_lines,
                           std::vector<EdgeLineMatchInfo>* edge_line_matches) {
  std::vector<LaneLine> map_edge_lines_tmp = map_edge_lines;
  for (const auto& per_edge_line : per_edge_lines) {
    ObjUpdateType type = ObjUpdateType::ADD_NEW;
    EdgeLineMatchInfo match;
    HLOG_INFO << "per_edge_line.lanepos_ " << per_edge_line.lanepos_;
    for (auto& map_edge_line : map_edge_lines_tmp) {
      if (map_edge_line.has_matched_) {
        continue;
      }
      double sum = 0.0;
      int n = 0;
      for (const auto& map_point : map_edge_line.points_) {
        if (map_point.x() > per_edge_line.end_point_x_ ||
            map_point.x() < per_edge_line.start_point_x_) {
          continue;
        }
        double per_y = per_edge_line.c0_ + per_edge_line.c1_ * map_point.x() +
                       per_edge_line.c2_ * pow(map_point.x(), 2) +
                       per_edge_line.c3_ * pow(map_point.x(), 3);
        sum += fabs(map_point.y() - per_y);
        n++;
      }
      double ave_dis = sum / n;
      HLOG_INFO << "map_edge_line.lanepos_ " << map_edge_line.lanepos_;
      HLOG_INFO << "ave_dis " << ave_dis << " n " << n;
      if (ave_dis < 0.5 && n >= 3) {
        match.per_edge_line_ = per_edge_line;
        match.map_edge_line_ = map_edge_line;
        match.update_type_ = ObjUpdateType::MERGE_OLD;
        type = ObjUpdateType::MERGE_OLD;
        map_edge_line.has_matched_ = true;
        break;
      }
    }
    if (type == ObjUpdateType::ADD_NEW) {
      match.per_edge_line_ = per_edge_line;
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    edge_line_matches->emplace_back(match);
  }
}

void LaneOp::MatchStopLine(const std::vector<StopLine>& per_stop_lines,
                           const std::vector<StopLine>& map_stop_lines,
                           std::vector<StopLineMatchInfo>* stop_line_matches) {
  std::vector<StopLine> map_stop_lines_tmp = map_stop_lines;
  for (const auto& per_stop_line : per_stop_lines) {
    if (per_stop_line.left_point_.x() < -1) {
      continue;
    }
    ObjUpdateType type = ObjUpdateType::ADD_NEW;
    StopLineMatchInfo match;
    for (auto& map_stop_line : map_stop_lines_tmp) {
      if (map_stop_line.has_matched_) {
        continue;
      }
      double per_mid_y =
          (per_stop_line.left_point_.y() + per_stop_line.right_point_.y()) / 2;
      double map_mid_y =
          (map_stop_line.left_point_.y() + map_stop_line.right_point_.y()) / 2;
      if (fabs(per_mid_y - map_mid_y) < 5) {
        match.per_stop_line_ = per_stop_line;
        match.map_stop_line_ = map_stop_line;
        match.update_type_ = ObjUpdateType::MERGE_OLD;
        type = ObjUpdateType::MERGE_OLD;
        map_stop_line.has_matched_ = true;
        break;
      }
    }
    if (type == ObjUpdateType::ADD_NEW) {
      match.per_stop_line_ = per_stop_line;
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    stop_line_matches->emplace_back(match);
  }
}

void LaneOp::MatchArrow(const std::vector<Arrow>& per_arrows,
                        const std::vector<Arrow>& map_arrows,
                        std::vector<ArrowMatchInfo>* arrow_matches) {
  std::vector<Arrow> map_arrows_tmp = map_arrows;
  for (const auto& per_arrow : per_arrows) {
    if (per_arrow.max_x < -1) {
      continue;
    }
    ObjUpdateType type = ObjUpdateType::ADD_NEW;
    ArrowMatchInfo match;
    for (auto& map_arrow : map_arrows_tmp) {
      if (map_arrow.has_matched_) {
        continue;
      }
      double map_mid_x = (map_arrow.min_x + map_arrow.max_x) / 2;
      double map_mid_y = (map_arrow.min_y + map_arrow.max_y) / 2;
      double per_mid_x = (per_arrow.min_x + per_arrow.max_x) / 2;
      double per_mid_y = (per_arrow.min_y + per_arrow.max_y) / 2;
      if (fabs(per_mid_x - map_mid_x) < 10 && fabs(per_mid_y - map_mid_y) < 2) {
        match.per_arrow_ = per_arrow;
        match.map_arrow_ = map_arrow;
        match.update_type_ = ObjUpdateType::MERGE_OLD;
        type = ObjUpdateType::MERGE_OLD;
        map_arrow.has_matched_ = true;
        break;
      }
    }
    if (type == ObjUpdateType::ADD_NEW) {
      match.per_arrow_ = per_arrow;
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    arrow_matches->emplace_back(match);
  }
}

void LaneOp::MatchZebraCrossing(
    const std::vector<ZebraCrossing>& per_zebra_crossings,
    const std::vector<ZebraCrossing>& map_zebra_crossings,
    std::vector<ZebraCrossingMatchInfo>* zebra_crossing_matches) {
  std::vector<ZebraCrossing> map_zebra_crossings_tmp = map_zebra_crossings;
  for (const auto& per_zebra_crossing : per_zebra_crossings) {
    if (per_zebra_crossing.max_x < -1) {
      continue;
    }
    ObjUpdateType type = ObjUpdateType::ADD_NEW;
    ZebraCrossingMatchInfo match;
    for (auto& map_zebra_crossing : map_zebra_crossings_tmp) {
      if (map_zebra_crossing.has_matched_) {
        continue;
      }
      double map_mid_x =
          (map_zebra_crossing.min_x + map_zebra_crossing.max_x) / 2;
      double map_mid_y =
          (map_zebra_crossing.min_y + map_zebra_crossing.max_y) / 2;
      double per_mid_x =
          (per_zebra_crossing.min_x + per_zebra_crossing.max_x) / 2;
      double per_mid_y =
          (per_zebra_crossing.min_y + per_zebra_crossing.max_y) / 2;
      if (fabs(per_mid_x - map_mid_x) < 10) {
        match.per_zebra_crossing_ = per_zebra_crossing;
        match.map_zebra_crossing_ = map_zebra_crossing;
        match.update_type_ = ObjUpdateType::MERGE_OLD;
        type = ObjUpdateType::MERGE_OLD;
        map_zebra_crossing.has_matched_ = true;
        break;
      }
    }
    if (type == ObjUpdateType::ADD_NEW) {
      match.per_zebra_crossing_ = per_zebra_crossing;
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    zebra_crossing_matches->emplace_back(match);
  }
}

ConstDrDataPtr LaneOp::GetDrPoseForTime(double timestamp) {
  auto& local_data = LocalDataSingleton::GetInstance();

  // HLOG_INFO << "find timestamp: " << ara::log::Setprecision(20) << timestamp;

  ConstDrDataPtr before = nullptr;
  ConstDrDataPtr after = nullptr;
  local_data.dr_buffer().get_messages_around(timestamp, before, after);

  if (before == nullptr && after == nullptr) {
    HLOG_ERROR << "GetDrPoseForTime is null: " << timestamp;
    return nullptr;
  }

  if (before == nullptr) {
    DrDataPtr dr_ptr = std::make_shared<DrData>();

    double delta_t = timestamp - after->timestamp;
    dr_ptr->timestamp = timestamp;
    dr_ptr->pose =
        after->pose + after->quaternion * (after->local_vel * delta_t);
    dr_ptr->quaternion = after->quaternion;
    Eigen::Vector3d delta_ang = after->local_omg * delta_t;
    if (delta_ang.norm() > 1e-12) {
      dr_ptr->quaternion =
          dr_ptr->quaternion * Eigen::Quaterniond(Eigen::AngleAxisd(
                                   delta_ang.norm(), delta_ang.normalized()));
      dr_ptr->quaternion = dr_ptr->quaternion.normalized();
    }

    dr_ptr->local_vel = after->local_vel;
    dr_ptr->local_omg = after->local_omg;
    dr_ptr->local_acc = after->local_acc;
    dr_ptr->gear = after->gear;

    return dr_ptr;
  }

  if (before->timestamp == timestamp && after->timestamp == timestamp) {
    return before;
  }

  // before time == after time is not happend for this if
  if (after->timestamp <= before->timestamp) {
    HLOG_ERROR << "GetDrPoseForTime: after->timestamp <= before->timestamp: "
               << after->timestamp << " < " << before->timestamp;
    return nullptr;
  }

  double ratio =
      (timestamp - before->timestamp) / (after->timestamp - before->timestamp);
  auto dr_pose_state =
      std::make_shared<DrData>(before->Interpolate(ratio, *after, timestamp));
  return dr_pose_state;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
