/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhu Kaiying <zhukaiying@sensetime.com>
 */

#include "semantic_mm/matching/matcher/matcher_bbox.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <set>

#include "semantic_mm/common/hungarian.hpp"
#include "semantic_mm/common/math_tools.hpp"

namespace senseAD {
namespace localization {
namespace smm {
adLocStatus_t MatcherBBox::IOUMatch(
    const double iou_thre, bool use_hungarian,
    std::unordered_map<id_t, id_t>* matching_indices) const {
  std::unordered_map<id_t, std::unordered_map<id_t, double>> query_ref_costs;
  for (const auto& query_item : query_bboxes_) {
    const auto& query_id = query_item.first;
    const auto& query_bbox = query_item.second;
    std::unordered_map<id_t, double> to_ref_costs;
    for (const auto& ref_item : ref_bboxes_) {
      const auto& ref_id = ref_item.first;
      const auto& ref_bbox = ref_item.second;
      double iou = BoundingBoxIOU(query_bbox, ref_bbox);
      if (iou > iou_thre) {
        to_ref_costs.insert(std::make_pair(ref_id, 1.0 - iou));
      }
    }
    if (!to_ref_costs.empty()) {
      query_ref_costs.insert(std::make_pair(query_id, std::move(to_ref_costs)));
    }
  }

  if (use_hungarian) {
    CalBipartiteGraphMatchingPairs(query_ref_costs, 1.0 - iou_thre, 1.0,
                                   matching_indices);
  } else {
    CalNearestMatchingPairs(query_ref_costs, 1.0 - iou_thre, matching_indices);
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatcherBBox::EpipolarIOUMatch(
    const double min_depth, const double max_depth, const double iou_thre,
    const double search_line_extend, bool use_hungarian,
    std::unordered_map<id_t, id_t>* matching_indices) const {
  if (!cam_model_) return LOC_NULL_PTR;

  std::unordered_map<id_t, std::unordered_map<id_t, double>> query_ref_costs;
  for (const auto& query_item : query_bboxes_) {
    const auto& query_id = query_item.first;
    const auto& query_bbox = query_item.second;
    const auto query_pose = query_poses_.find(query_id);
    if (query_pose == query_poses_.end()) continue;
    std::unordered_map<id_t, double> to_ref_costs;
    for (const auto& ref_item : ref_bboxes_) {
      const auto& ref_id = ref_item.first;
      const auto& ref_bbox = ref_item.second;
      const auto ref_pose = ref_poses_.find(ref_id);
      if (ref_pose == ref_poses_.end()) continue;
      SE3d T_ref_query = ref_pose->second.inverse() * query_pose->second;

      double match_iou = -1.0;
      Eigen::Matrix3d R_ref_query = T_ref_query.so3().matrix();
      double yaw =
          std::atan2(-R_ref_query(2, 0),
                     std::sqrt(1 - R_ref_query(2, 0) * R_ref_query(2, 0)));
      if (T_ref_query.translation().norm() < 30 && std::fabs(yaw) < 0.5) {
        // calculate frame to frame iou, no projection
        match_iou = BoundingBoxIOU(query_bbox, ref_bbox);
        if (match_iou > 0.9 && match_iou > iou_thre) {
          // no need to calculate with other trackers
          to_ref_costs.insert(std::make_pair(ref_id, 1.0 - match_iou));
          break;
        }
      }

      double min_search_depth = min_depth;
      double max_search_depth = max_depth;
      auto iter = ref_bbox_3d_centers_.find(ref_id);
      if (iter != ref_bbox_3d_centers_.end() && iter->second.z > 0.0) {
        Point3D_t center3d = T_ref_query.inverse() * iter->second;
        if (center3d.z > 0.0) {
          min_search_depth = std::max(2.0, center3d.z - 30.0);
          max_search_depth = center3d.z + 30.0;
        }
      }
      Point2D_t lt_pt(query_bbox.center.x - query_bbox.width / 2.0,
                      query_bbox.center.y - query_bbox.length / 2.0);
      Point2D_t rb_pt(query_bbox.center.x + query_bbox.width / 2.0,
                      query_bbox.center.y + query_bbox.length / 2.0);
      Point3D_t lt_view_ray;
      cam_model_->Pixel2Camera(lt_pt, &lt_view_ray);
      Point3D_t rb_view_ray;
      cam_model_->Pixel2Camera(rb_pt, &rb_view_ray);
      Point3D_t lt_proj1 = T_ref_query * (lt_view_ray * min_search_depth);
      Point3D_t lt_proj2 = T_ref_query * (lt_view_ray * max_search_depth);
      Point3D_t rb_proj1 = T_ref_query * (rb_view_ray * min_search_depth);
      Point3D_t rb_proj2 = T_ref_query * (rb_view_ray * max_search_depth);
      Point2D_t lt_search_start_pt, lt_search_end_pt, rb_search_start_pt,
          rb_search_end_pt;
      cam_model_->Camera2Pixel(lt_proj1, &lt_search_start_pt);
      cam_model_->Camera2Pixel(lt_proj2, &lt_search_end_pt);
      cam_model_->Camera2Pixel(rb_proj1, &rb_search_start_pt);
      cam_model_->Camera2Pixel(rb_proj2, &rb_search_end_pt);

      // make sure start point is in left
      if (lt_search_start_pt.x > lt_search_end_pt.x) {
        Point2D_t temp = lt_search_end_pt;
        lt_search_end_pt = lt_search_start_pt;
        lt_search_start_pt = temp;
      }
      if (rb_search_start_pt.x > rb_search_end_pt.x) {
        Point2D_t temp = rb_search_end_pt;
        rb_search_end_pt = rb_search_start_pt;
        rb_search_start_pt = temp;
      }
      BoundingBox2D start_bbox;
      start_bbox.center =
          Point2D_t(0.5 * (rb_search_start_pt.x + lt_search_start_pt.x),
                    0.5 * (rb_search_start_pt.y + lt_search_start_pt.y));
      start_bbox.width = std::fabs(rb_search_start_pt.x - lt_search_start_pt.x);
      start_bbox.length = rb_search_start_pt.y - lt_search_start_pt.y;
      BoundingBox2D end_bbox;
      end_bbox.center =
          Point2D_t(0.5 * (rb_search_end_pt.x + lt_search_end_pt.x),
                    0.5 * (rb_search_end_pt.y + lt_search_end_pt.y));
      end_bbox.width = std::fabs(rb_search_end_pt.x - lt_search_end_pt.x);
      end_bbox.length = rb_search_end_pt.y - lt_search_end_pt.y;

      if (ref_bbox.center.x < start_bbox.center.x - search_line_extend ||
          ref_bbox.center.x > end_bbox.center.x + search_line_extend) {
        if (match_iou < 0.0) continue;
      } else {
        // calculate projection iou
        BoundingBox2D interp_bbox;
        if (lt_search_end_pt.x - lt_search_start_pt.x < 3 ||
            rb_search_end_pt.x - rb_search_start_pt.x < 3) {
          // object may in the center of camera view
          interp_bbox.center = ref_bbox.center;
          interp_bbox.width = std::max(start_bbox.width, end_bbox.width);
          interp_bbox.length = std::max(start_bbox.length, end_bbox.length);
        } else {
          if (ref_bbox.center.x < start_bbox.center.x) {
            double factor = (ref_bbox.center.x - end_bbox.center.x) /
                            (start_bbox.center.x - end_bbox.center.x);
            BBoxInterp(end_bbox, start_bbox, factor, &interp_bbox);
          } else {
            double factor = (ref_bbox.center.x - start_bbox.center.x) /
                            (end_bbox.center.x - start_bbox.center.x);
            BBoxInterp(start_bbox, end_bbox, factor, &interp_bbox);
          }
        }
        double iou = BoundingBoxIOU(interp_bbox, ref_bbox);
        match_iou = std::max(match_iou, iou);
      }

      if (match_iou < iou_thre) continue;
      if (CheckMoveFlow(query_id, ref_id) != LOC_SUCCESS) continue;
      to_ref_costs.insert(std::make_pair(ref_id, 1.0 - match_iou));
    }
    if (!to_ref_costs.empty()) {
      query_ref_costs.insert(std::make_pair(query_id, std::move(to_ref_costs)));
    }
  }

  if (use_hungarian) {
    CalBipartiteGraphMatchingPairs(query_ref_costs, 1.0 - iou_thre, 1.0,
                                   matching_indices);
  } else {
    CalNearestMatchingPairs(query_ref_costs, 1.0 - iou_thre, matching_indices);
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatcherBBox::CalNearestMatchingPairs(
    const std::unordered_map<id_t, std::unordered_map<id_t, double>>&
        query_ref_cost,
    const double cost_thre,
    std::unordered_map<id_t, id_t>* matching_indices) const {
  if (!matching_indices) return LOC_NULL_PTR;

  std::vector<std::pair<id_t, id_t>> query_ref_indices;
  for (const auto& item : query_ref_cost) {
    const auto& query_id = item.first;
    id_t min_cost_id = 0;
    double min_cost = std::numeric_limits<double>::max();
    for (const auto& to_ref_iou : item.second) {
      if (to_ref_iou.second < min_cost) {
        min_cost = to_ref_iou.second;
        min_cost_id = to_ref_iou.first;
      }
    }
    if (min_cost > cost_thre) continue;
    query_ref_indices.emplace_back(query_id, min_cost_id);
  }

  // reverse check, make sure injective matching(one-to-one)
  std::unordered_map<id_t, std::map<double, id_t>> reverse_distances;
  for (const auto& pair : query_ref_indices) {
    const id_t& ref_id = pair.second;
    const id_t& query_id = pair.first;
    double cost = query_ref_cost.at(query_id).at(ref_id);
    auto& to_query_iou = reverse_distances[ref_id];
    to_query_iou.insert({cost, query_id});
  }
  for (const auto& item : reverse_distances) {
    const id_t& query_id = item.second.begin()->second;
    const id_t& ref_id = item.first;
    matching_indices->insert(std::make_pair(query_id, ref_id));
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatcherBBox::CalBipartiteGraphMatchingPairs(
    const std::unordered_map<id_t, std::unordered_map<id_t, double>>&
        query_ref_costs,
    const double cost_thre, const double max_cost,
    std::unordered_map<id_t, id_t>* matching_indices) const {
  if (!matching_indices) return LOC_NULL_PTR;

  std::set<id_t> used_ref_ids;
  for (const auto& item : query_ref_costs) {
    for (const auto& to_ref_cost : item.second) {
      if (to_ref_cost.second > cost_thre) continue;
      used_ref_ids.insert(to_ref_cost.first);
    }
  }
  std::vector<id_t> ref_id_in_table;
  ref_id_in_table.reserve(used_ref_ids.size());
  ref_id_in_table.assign(used_ref_ids.begin(), used_ref_ids.end());

  std::vector<std::vector<double>> cost_table;
  std::vector<id_t> query_id_in_table;
  for (const auto& item : query_ref_costs) {
    const auto& query_id = item.first;
    const auto& to_ref_costs = item.second;
    std::vector<double> costs;
    for (const auto& ref_id : ref_id_in_table) {
      const auto iter = to_ref_costs.find(ref_id);
      if (iter == to_ref_costs.end() || iter->second > cost_thre) {
        costs.emplace_back(max_cost);
      } else {
        costs.emplace_back(iter->second);
      }
    }
    query_id_in_table.emplace_back(query_id);
    cost_table.emplace_back(costs);
  }

  HungarianOptimizer<double> optimizer;
  optimizer.SetCosts(cost_table);
  std::vector<std::pair<size_t, size_t>> assignments;
  if (optimizer.Solve(&assignments)) {
    for (const auto& item : assignments) {
      if (item.first >= query_id_in_table.size() ||
          item.second >= ref_id_in_table.size()) {
        continue;
      }
      matching_indices->insert(std::make_pair(query_id_in_table[item.first],
                                              ref_id_in_table[item.second]));
    }
    return LOC_SUCCESS;
  } else {
    return LOC_LOCALIZATION_ERROR;
  }
}

adLocStatus_t MatcherBBox::CheckMoveFlow(id_t query_id, id_t ref_id) const {
  auto query_bbox_iter = query_bboxes_.find(query_id);
  if (query_bbox_iter == query_bboxes_.end()) {
    return LOC_INVALID;
  }
  auto ref_bbox_iter = ref_bboxes_.find(ref_id);
  if (ref_bbox_iter == ref_bboxes_.end()) {
    return LOC_INVALID;
  }

  auto iter = ref_bbox_move_flow_.find(ref_id);
  if (iter != ref_bbox_move_flow_.end()) {
    Point2D_t predict_move_flow = iter->second;
    Point2D_t matched_move_flow =
        query_bbox_iter->second.center - ref_bbox_iter->second.center;

    if (std::fabs(predict_move_flow.x) < 1e-8 &&
        std::fabs(predict_move_flow.y) < 1e-8) {
      // invalid predict, do not check;
      return LOC_SUCCESS;
    } else {
      Point2D_t flow_diff = matched_move_flow - predict_move_flow;

      if (std::fabs(flow_diff.x) > 50) {
        return LOC_INVALID;
      } else {
        return LOC_SUCCESS;
      }
    }
  }
  return LOC_SUCCESS;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
