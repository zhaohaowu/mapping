/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： utils.cc
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#include "modules/map_fusion_02/modules/lane/road_topo_builder/utils.h"

#include <cmath>

namespace hozon {
namespace mp {
namespace mf {

float TopoUtils::LaneDist(const Lane::Ptr& lane_in_curr,
                          const Lane::Ptr& lane_in_next) {
  size_t sizet = lane_in_curr->center_line_pts.size();
  Eigen::Vector3f lane_in_next_norm(0.0, 0.0, 0.0);
  size_t next_lane_cp_idx = 1;
  while (next_lane_cp_idx < lane_in_next->center_line_pts.size() &&
         lane_in_next_norm.norm() < 2.0) {
    lane_in_next_norm = lane_in_next->center_line_pts[next_lane_cp_idx].pt -
                        lane_in_next->center_line_pts[0].pt;
    next_lane_cp_idx++;
  }
  if (lane_in_next_norm.norm() > 2.0) {
    lane_in_next_norm = lane_in_next_norm.normalized();
    Eigen::Vector3f lane_curr_next_vec =
        lane_in_curr->center_line_pts[sizet - 1].pt -
        lane_in_next->center_line_pts[0].pt;
    float dis = (lane_curr_next_vec.cross(lane_in_next_norm)).norm();
    return dis;
  }
  return 10.0;
}

float TopoUtils::CalculateDistPt(const Lane::Ptr& lane_in_next,
                                 const Lane::Ptr& lane_in_curr, size_t sizet) {
  return pow(lane_in_next->center_line_pts[0].pt.y() -
                 lane_in_curr->center_line_pts[sizet - 1].pt.y(),
             2) +
         pow(lane_in_next->center_line_pts[0].pt.x() -
                 lane_in_curr->center_line_pts[sizet - 1].pt.x(),
             2);
}

float TopoUtils::CalculatePoint2CenterLine(const Lane::Ptr& lane_in_next,
                                           const Lane::Ptr& lane_in_curr) {
  return abs(lane_in_next->center_line_pts[0].pt.y() -
             (lane_in_curr->center_line_param[0] +
              lane_in_curr->center_line_param[1] *
                  lane_in_next->center_line_pts[0].pt.x())) /
         sqrt(1 + pow(lane_in_curr->center_line_param[1], 2));
}

float TopoUtils::Calculate2CenterlineAngle(const Lane::Ptr& lane_in_next,
                                           const Lane::Ptr& lane_in_curr,
                                           size_t sizet) {
  return atan((lane_in_next->center_line_pts[0].pt.y() -
               lane_in_curr->center_line_pts[sizet - 1].pt.y()) /
              (lane_in_next->center_line_pts[0].pt.x() -
               lane_in_curr->center_line_pts[sizet - 1].pt.x())) -
         atan(lane_in_curr->center_line_param[1]);
}

double TopoUtils::CalcLaneLength(const Lane::Ptr& lane) {
  if (lane == nullptr || lane->center_line_pts.size() < 2) {
    return 0;
  }
  double len = 0;
  for (int i = 0; i < static_cast<int>(lane->center_line_pts.size()) - 1; ++i) {
    Eigen::Vector3f p0(lane->center_line_pts.at(i).pt.x(),
                       lane->center_line_pts.at(i).pt.y(), 0);
    Eigen::Vector3f p1(lane->center_line_pts.at(i + 1).pt.x(),
                       lane->center_line_pts.at(i + 1).pt.y(), 0);
    len += Dist(p0, p1);
  }
  return len;
}

float TopoUtils::PointToLineDis(const LineSegment& line, float line_front_x,
                                float line_front_y) {
  if (line.pts.empty()) {
    return 0.0;
  }
  if (line.pts.size() == 1) {
    return std::hypot(line.pts[0].pt.x() - line_front_x,
                      line.pts[0].pt.y() - line_front_y);
  }
  int index_left = 0;
  if (index_left < static_cast<int>(line.pts.size()) - 1) {
    int index_right = index_left + 1;
    while (index_right < static_cast<int>(line.pts.size()) &&
           line.pts[index_right].pt.x() - line.pts[index_left].pt.x() < 0.1) {
      index_right++;
    }
    if (index_right < static_cast<int>(line.pts.size())) {
      const auto& p_left = line.pts[index_left].pt;
      const auto& p_right = line.pts[index_right].pt;
      float k = (p_right.y() - p_left.y()) / (p_right.x() - p_left.x());
      float b = p_right.y() - k * p_right.x();
      return abs(line_front_x * k + b - line_front_y) / sqrt(1 + k * k);
    }
  }
  return 0.0;
}

void TopoUtils::FitCenterLine(Lane::Ptr lane) {
  std::vector<hozon::common::math::Vec2d> left_pts;
  std::vector<hozon::common::math::Vec2d> right_pts;
  left_pts.reserve(lane->left_boundary->pts.size());
  right_pts.reserve(lane->right_boundary->pts.size());
  for (const auto& pt : lane->left_boundary->pts) {
    left_pts.emplace_back(pt.pt.x(), pt.pt.y());
  }
  for (const auto& pt : lane->right_boundary->pts) {
    right_pts.emplace_back(pt.pt.x(), pt.pt.y());
  }
  std::vector<hozon::common::math::Vec2d> cent_pts;
  math::GenerateCenterPoint(left_pts, right_pts, &cent_pts);
  for (const auto& pt : cent_pts) {
    Point cpt(PointType::RAW, static_cast<float>(pt.x()),
              static_cast<float>(pt.y()), 0);
    lane->center_line_pts.emplace_back(cpt);
  }
  if (lane->center_line_pts.size() > 3) {
    lane->center_line_param = math::FitLaneline(lane->center_line_pts);
    lane->center_line_param_front =
        math::FitLanelinefront(lane->center_line_pts);
  }
}

bool TopoUtils::LineIdConsistant(const LineSegment::Ptr& line, Id id) {
  if (line->id == id) {
    return true;
  }
  for (const auto& detele_id : line->deteled_ids) {
    if (detele_id == id) {
      return true;
    }
  }
  return false;
}

bool TopoUtils::NeedToConnect(const Lane::Ptr& lane_in_curr,
                              const Lane::Ptr& lane_in_next) {
  if (lane_in_curr->left_boundary->id == lane_in_next->right_boundary->id ||
      lane_in_curr->right_boundary->id == lane_in_next->left_boundary->id) {
    return false;
  }

  if (!IsAccessLane(lane_in_curr, lane_in_next)) {
    return false;
  }

  size_t sizet = lane_in_curr->center_line_pts.size();
  float dis_pt = TopoUtils::CalculateDistPt(lane_in_next, lane_in_curr, sizet);
  float angel_thresh{0.0};
  float dis_thresh{0.0};
  if (dis_pt < 8 || (lane_in_curr->lanepos_id == lane_in_next->lanepos_id &&
                     lane_in_curr->lanepos_id != "99_99")) {
    // HLOG_ERROR << "lane_in_curr=" << lane_in_curr->str_id_with_group
    //            << "   lane_in_next" << lane_in_next->str_id_with_group
    //            << "  dis_pt = " << dis_pt
    //            << " lane_in_curr->lanepos_id =" << lane_in_curr->lanepos_id
    //            << "  lane_in_next->lanepos_id = " <<
    //            lane_in_next->lanepos_id;
    return true;
  } else if (!lane_in_curr->center_line_param.empty() &&
             !lane_in_next->center_line_pts.empty()) {
    if (dis_pt > 10000) {
      // 差的太远不计算
      return false;
    }
    if (dis_pt > 100) {
      angel_thresh = 10;
      dis_thresh = 3;
    } else {
      angel_thresh = 25;
      dis_thresh = 1.8;
    }
    float dis =
        TopoUtils::CalculatePoint2CenterLine(lane_in_next, lane_in_curr);
    float angle =
        TopoUtils::Calculate2CenterlineAngle(lane_in_next, lane_in_curr, sizet);
    // HLOG_INFO << "angle = " << angle * 180 / pi_;
    // HLOG_INFO << "lane angle lane_in_next->center_line_pts[0].pt  "
    //           << lane_in_next->center_line_pts[0].pt.y() << "   "
    //           << lane_in_next->center_line_pts[0].pt.x();
    // HLOG_INFO << "lane_in_curr->center_line_pts[sizet - 1].pt  "
    //           << lane_in_curr->center_line_pts[sizet - 1].pt.y() << "   "
    //           << lane_in_curr->center_line_pts[sizet - 1].pt.x()
    //           << "  angle is "
    //           << atan((lane_in_next->center_line_pts[0].pt.y() -
    //                    lane_in_curr->center_line_pts[sizet - 1].pt.y()) /
    //                   (lane_in_next->center_line_pts[0].pt.x() -
    //                    lane_in_curr->center_line_pts[sizet - 1].pt.x())) *
    //                  180 / pi_;
    // HLOG_INFO << "lane_in_curr->center_line_param[1] is"
    //           << atan(lane_in_curr->center_line_param[1]) * 180 / pi_;
    // HLOG_INFO << "lane_in_curr=" << lane_in_curr->str_id_with_group
    //           << "   lane_in_next" << lane_in_next->str_id_with_group
    //           << "  dis2l = " << dis << "  dis thresh = " << dis_thresh
    //           << " angle = " << abs(angle) * 180 / pi_
    //           << "  angel_thresh = " << angel_thresh;
    if ((dis < dis_thresh && abs(angle) * 180 / M_PI < angel_thresh)) {
      // HLOG_INFO << "lane_in_curr=" << lane_in_curr->str_id_with_group
      //           << "   lane_in_next" << lane_in_next->str_id_with_group
      //           << "  dis2l = " << dis << "  dis thresh = " << dis_thresh
      //           << " angle = " << angle << "  angel_thresh = " <<
      //           angel_thresh;
      return true;
    }
  }
  // HLOG_INFO << "lane_in_curr=" << lane_in_curr->str_id_with_group;
  return false;
}

bool TopoUtils::NeedToConnect(
    Group::Ptr curr_group, Group::Ptr next_group, int i, int j,
    std::map<int, std::vector<int>>* curr_group_next_lane,
    std::map<int, std::vector<int>>* next_group_prev_lane) {
  auto& lane_in_curr = curr_group->lanes[i];
  auto& lane_in_next = next_group->lanes[j];
  if (!IsAccessLane(lane_in_curr, lane_in_next)) {
    return false;
  }
  size_t sizet = lane_in_curr->center_line_pts.size();
  float dis_pt = CalculateDistPt(lane_in_next, lane_in_curr, sizet);
  float angel_thresh{0.0};
  float dis_thresh{0.0};
  if (dis_pt < 8 || (lane_in_curr->lanepos_id == lane_in_next->lanepos_id &&
                     lane_in_curr->lanepos_id != "99_99")) {
    if (curr_group_next_lane->find(i) != curr_group_next_lane->end()) {
      curr_group_next_lane->at(i).emplace_back(j);
    } else {
      curr_group_next_lane->insert({i, {j}});
    }
    if (next_group_prev_lane->find(j) != next_group_prev_lane->end()) {
      next_group_prev_lane->at(j).emplace_back(i);
    } else {
      next_group_prev_lane->insert({j, {i}});
    }
    // HLOG_ERROR << "lane_in_curr=" << lane_in_curr->str_id_with_group
    //            << "   lane_in_next" << lane_in_next->str_id_with_group
    //            << "  dis_pt = " << dis_pt
    //            << " lane_in_curr->lanepos_id =" << lane_in_curr->lanepos_id
    //            << "  lane_in_next->lanepos_id = " <<
    //            lane_in_next->lanepos_id;

    return true;
  } else if (!lane_in_curr->center_line_param.empty() &&
             !lane_in_next->center_line_pts.empty()) {
    if (dis_pt > 10000) {
      // 差的太远不计算
      return false;
    }
    if (dis_pt > 100) {
      angel_thresh = 10;
      dis_thresh = 3;
    } else {
      angel_thresh = 25;
      dis_thresh = 1.8;
    }
    float dis = CalculatePoint2CenterLine(lane_in_next, lane_in_curr);
    float angle = Calculate2CenterlineAngle(lane_in_next, lane_in_curr, sizet);
    // HLOG_ERROR << "angle = "<<angle*180/pi_;
    if ((dis < dis_thresh && abs(angle) * 180 / M_PI < angel_thresh)) {
      //! TBD:
      //! 这里直接把后一个lane中心线的第一个点加到前一个lane中心线的末尾，
      //! 后续需要考虑某些异常情况，比如后一个lane中心线的第一个点在前一个lane中心线最后
      //! 一个点的后方，这样直连就导致整个中心线往后折返了；以及还要考虑横向偏移较大时不平
      //! 滑的问题
      if (curr_group_next_lane->find(i) != curr_group_next_lane->end()) {
        curr_group_next_lane->at(i).emplace_back(j);
      } else {
        curr_group_next_lane->insert({i, {j}});
      }
      if (next_group_prev_lane->find(j) != next_group_prev_lane->end()) {
        next_group_prev_lane->at(j).emplace_back(i);
      } else {
        next_group_prev_lane->insert({j, {i}});
      }
      // HLOG_ERROR << "lane_in_curr=" << lane_in_curr->str_id_with_group
      //            << "   lane_in_next" << lane_in_next->str_id_with_group
      //            << "  dis2l = " << dis << "  dis thresh = " << dis_thresh
      //            << " angle = " << angle << "  angel_thresh = " <<
      //            angel_thresh;

      return true;
    }
  }
  return false;
}

bool TopoUtils::IsAccessLane(const Lane::Ptr& lane_in_curr,
                             const Lane::Ptr& lane_in_next) {
  if (lane_in_curr->center_line_pts.empty() ||
      lane_in_next->center_line_pts.empty()) {
    return false;
  }

  // 从lane_in_next车道的中心线上找出一条大于2米长度的向量出来(从起点位置开始)。
  Eigen::Vector3f lane_in_next_norm(0.0, 0.0, 0.0);
  for (size_t next_lane_cp_idx = 1;
       next_lane_cp_idx < lane_in_next->center_line_pts.size();
       ++next_lane_cp_idx) {
    lane_in_next_norm = lane_in_next->center_line_pts[next_lane_cp_idx].pt -
                        lane_in_next->center_line_pts[0].pt;
    if (lane_in_next_norm.norm() >= 2.0) {
      break;
    }
  }

  if (lane_in_next_norm.norm() <= 2.0) {
    return false;
  }

  size_t sizet = lane_in_curr->center_line_pts.size();
  Eigen::Vector3f lane_curr2next_vec =
      lane_in_curr->center_line_pts[sizet - 1].pt -
      lane_in_next->center_line_pts[0].pt;
  lane_in_next_norm = lane_in_next_norm.normalized();
  // 判断两车道中心线的横向距离是否大于2.0, 如果小于2米，认为是可通行的。
  if ((lane_in_next_norm.cross(lane_curr2next_vec)).norm() <= 2.0) {
    return true;
  }

  // 判断curr点是否在next_lane的右侧
  bool is_right = (lane_in_next_norm.y() * lane_curr2next_vec.x() -
                   lane_in_next_norm.x() * lane_curr2next_vec.y()) > 0;

  if (is_right &&
      (lane_in_next->right_boundary->type == LineType::LaneType_SOLID ||
       lane_in_next->right_boundary->type == LineType::LaneType_DOUBLE_SOLID ||
       lane_in_next->right_boundary->type ==
           LineType::LaneType_LEFT_SOLID_RIGHT_DASHED)) {
    HLOG_DEBUG << "NOT ACCESS RIGHT!";
    return false;
  }
  if (!is_right &&
      (lane_in_next->left_boundary->type == LineType::LaneType_SOLID ||
       lane_in_next->left_boundary->type == LineType::LaneType_DOUBLE_SOLID ||
       lane_in_next->left_boundary->type ==
           LineType::LaneType_RIGHT_SOLID_LEFT_DASHED)) {
    HLOG_DEBUG << "NOT ACCESS LEFT!";
    return false;
  }

  return true;
}

bool TopoUtils::IsRightLane(const Group::Ptr& next_group, int cur_lane_index,
                            int right_lane_inex) {
  for (const auto& right_lane_name :
       next_group->lanes[cur_lane_index]->right_lane_str_id_with_group) {
    if (right_lane_name ==
        next_group->lanes[right_lane_inex]->str_id_with_group) {
      return true;
      break;
    }
  }
  return false;
}

bool TopoUtils::IsLeftLane(const Group::Ptr& next_group, int cur_lane_index,
                           int left_lane_index) {
  for (const auto& left_lane_name :
       next_group->lanes[cur_lane_index]->left_lane_str_id_with_group) {
    if (left_lane_name ==
        next_group->lanes[left_lane_index]->str_id_with_group) {
      return true;
    }
  }
  return false;
}

// 判断车道是否收缩，即宽度越来越小
bool TopoUtils::IsShrinkLane(const Lane::Ptr& lane, float min_lane_width) {
  // 为空或点太少，默认非收缩
  if (lane == nullptr || lane->left_boundary->pts.size() < 3 ||
      lane->right_boundary->pts.size() < 3) {
    return false;
  }
  const auto& left = lane->left_boundary->pts;
  const auto& right = lane->right_boundary->pts;
  const auto& left_back_pt = left.back().pt;
  const auto& left_mid_pt = left.at(left.size() / 2).pt;

  const auto& right_back_v_pt0 =
      right.at(static_cast<int>(right.size()) - 2).pt;
  const auto& right_back_v_pt1 =
      right.at(static_cast<int>(right.size()) - 1).pt;

  size_t right_mid_idx = right.size() / 2;
  const auto& right_mid_v_pt0 = right.at(right_mid_idx - 1).pt;
  const auto& right_mid_v_pt1 = right.at(right_mid_idx).pt;

  Eigen::Vector2f lfbt(left_back_pt.x(), left_back_pt.y());
  Eigen::Vector2f lmpt(left_mid_pt.x(), left_mid_pt.y());
  Eigen::Vector2f rbvpt0(right_back_v_pt0.x(), right_back_v_pt0.y());
  Eigen::Vector2f rbvpt1(right_back_v_pt1.x(), right_back_v_pt1.y());
  Eigen::Vector2f rmvpt0(right_mid_v_pt0.x(), right_mid_v_pt0.y());
  Eigen::Vector2f rmvpt1(right_mid_v_pt1.x(), right_mid_v_pt1.y());

  auto left_back_dist = PointToVectorDist(rbvpt0, rbvpt1, lfbt);
  auto left_mid_dist = PointToVectorDist(rmvpt0, rmvpt1, lmpt);
  auto left_diff_dist = left_mid_dist - left_back_dist;

  return (left_diff_dist > 0 &&
          std::abs(left_diff_dist) > kShrinkDiffThreshold &&
          left_back_dist < min_lane_width + 0.5);
}

bool TopoUtils::IsBoundaryValid(const LineSegment& line) {
  if (line.pts.size() < 3) {
    return true;
  }
  Eigen::Vector3f x1, x2, x3;
  x1 = line.pts[0].pt;
  x2 = line.pts[1].pt;
  int index2 = 1;
  // HLOG_ERROR << "X1=" << x1.x() << "  " << x1.y();
  while ((x2 - x1).norm() < 0.2 &&
         index2 < static_cast<int>(line.pts.size()) - 1) {
    index2++;
    x2 = line.pts[index2].pt;
  }
  // HLOG_ERROR << "x2=" << x2.x() << "  " << x2.y();
  if (index2 > line.pts.size() - 2) {
    return true;
  }
  int index3 = index2 + 1;
  x3 = line.pts[index3].pt;
  while ((x2 - x3).norm() < 0.2 &&
         index3 < static_cast<int>(line.pts.size()) - 1) {
    index3++;
    x3 = line.pts[index3].pt;
  }
  // HLOG_ERROR << "x3=" << x3.x() << "  " << x3.y();
  if (index3 > static_cast<int>(line.pts.size()) - 1) {
    return true;
  }
  Eigen::Vector3f norm_x12 = (x1 - x2).normalized();
  Eigen::Vector3f norm_x32 = (x3 - x2).normalized();

  return (abs(norm_x12.dot(norm_x32)) >= 0.5);
}

bool TopoUtils::IsIntersect(const Lane::Ptr& line1, const Lane::Ptr& line2) {
  if (line1->center_line_pts.size() < 2 || line2->center_line_pts.size() < 2) {
    return false;
  }
  Eigen::Vector3d l1_s(line1->center_line_pts[0].pt.x(),
                       line1->center_line_pts[0].pt.y(), 0.0);
  Eigen::Vector3d l1_e(line1->center_line_pts.back().pt.x(),
                       line1->center_line_pts.back().pt.y(), 0.0);
  Eigen::Vector3d l2_s(line2->center_line_pts[0].pt.x(),
                       line2->center_line_pts[0].pt.y(), 0.0);
  Eigen::Vector3d l2_e(line2->center_line_pts.back().pt.x(),
                       line2->center_line_pts.back().pt.y(), 0.0);
  if (!line1->center_line_param.empty() && !line2->center_line_param.empty()) {
    // 往前延伸10米
    Eigen::Vector3d l1_forward_norm(1.0, line1->center_line_param[1], 0.0);
    Eigen::Vector3d l2_forward_norm(1.0, line2->center_line_param[1], 0.0);
    l1_forward_norm.normalized();
    l2_forward_norm.normalized();
    // HLOG_ERROR << "l1_forward_norm.Y = " << l1_forward_norm.y()
    //            << "  l2_forward_norm.Y = " << l2_forward_norm.y();
    l1_e.x() = l1_e.x() + l1_forward_norm.x() * 10;
    l1_e.y() = l1_e.y() + l1_forward_norm.y() * 10;
    l2_e.x() = l2_e.x() + l2_forward_norm.x() * 10;
    l2_e.y() = l2_e.y() + l2_forward_norm.y() * 10;
  }
  // HLOG_ERROR << " l1_s = " << l1_s.x() << "  " << l1_s.y()
  //            << "  l1_e = " << l1_e.x() << "  " << l1_e.y();
  // HLOG_ERROR << "l2_s = " << l2_s.x() << "  " << l2_s.y()
  //            << "   l2_e = " << l2_e.x() << "  " << l2_e.y();
  // 快速排除不可能相交的线
  if ((l1_s.x() > l1_e.x() ? l1_s.x() : l1_e.x()) <
          (l2_s.x() < l2_e.x() ? l2_s.x() : l2_e.x()) ||
      (l2_s.x() > l2_e.x() ? l2_s.x() : l2_e.x()) <
          (l1_s.x() < l1_e.x() ? l1_s.x() : l1_e.x()) ||
      (l1_s.y() > l1_e.y() ? l1_s.y() : l1_e.y()) <
          (l2_s.y() < l2_e.y() ? l2_s.y() : l2_e.y()) ||
      (l2_s.y() > l2_e.y() ? l2_s.y() : l2_e.y()) <
          (l1_s.y() < l1_e.y() ? l1_s.y() : l1_e.y())) {
    return false;
  }
  // 叉乘判断是否相交, 叉乘的正负代表逆时针顺时针即可判断方向 AB.cross(AP)
  if (((l1_e - l2_s).cross(l2_e - l2_s)).dot((l1_s - l2_s).cross(l2_e - l2_s)) >
          0 ||
      ((l2_e - l1_s).cross(l1_e - l1_s)).dot((l2_s - l1_s).cross(l1_e - l1_s)) >
          0) {
    return false;
  }
  return true;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
