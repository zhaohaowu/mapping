/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhu Kaiying <zhukaiying@sensetime.com>
 */

#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "semantic_mm/common/camera_model.hpp"
#include "semantic_mm/matching/matcher/matcher_base.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class MatcherBBox {
 public:
  DEFINE_SMART_PTR(MatcherBBox)

  MatcherBBox() = default;
  ~MatcherBBox() = default;

  void SetCameraModel(const CameraModel::Ptr& cam_model) {
    cam_model_ = cam_model;
  }

  void SetQueryBox(const std::unordered_map<id_t, BoundingBox2D>& bboxes) {
    query_bboxes_ = bboxes;
  }

  void SetReferenceBox(const std::unordered_map<id_t, BoundingBox2D>& bboxes) {
    ref_bboxes_ = bboxes;
  }

  void SetQueryPose(const std::unordered_map<id_t, SE3d>& poses) {
    query_poses_ = poses;
  }

  void SetReferencePose(const std::unordered_map<id_t, SE3d>& poses) {
    ref_poses_ = poses;
  }

  void SetRefBBox3DCenter(
      const std::unordered_map<id_t, Point3D_t>& bbox_3d_centers) {
    ref_bbox_3d_centers_ = bbox_3d_centers;
  }

  void SetRefBBoxMoveFlow(
      const std::unordered_map<id_t, Point2D_t>& bbox_move_flow) {
    ref_bbox_move_flow_ = bbox_move_flow;
  }

  // @brief: matching based on box to box IOU
  adLocStatus_t IOUMatch(
      const double iou_thre, bool use_hungarian,
      std::unordered_map<id_t, id_t>* matching_indices) const;

  // @brief: matching based on box to projected box IOU
  adLocStatus_t EpipolarIOUMatch(
      const double min_depth, const double max_depth, const double iou_thre,
      const double search_line_extend, bool use_hungarian,
      std::unordered_map<id_t, id_t>* matching_indices) const;

 private:
  // @brief: get nearest matching pairs
  adLocStatus_t CalNearestMatchingPairs(
      const std::unordered_map<id_t, std::unordered_map<id_t, double>>&
          query_ref_cost,
      const double cost_thre,
      std::unordered_map<id_t, id_t>* matching_pairs) const;

  // @brief: get bipartite graph matching pairs
  adLocStatus_t CalBipartiteGraphMatchingPairs(
      const std::unordered_map<id_t, std::unordered_map<id_t, double>>&
          query_ref_cost,
      const double cost_thre, const double max_cost,
      std::unordered_map<id_t, id_t>* matching_pairs) const;

  adLocStatus_t CheckMoveFlow(id_t query_id, id_t ref_id) const;

 private:
  CameraModel::Ptr cam_model_;

  std::unordered_map<id_t, BoundingBox2D> query_bboxes_;
  std::unordered_map<id_t, BoundingBox2D> ref_bboxes_;
  std::unordered_map<id_t, Point3D_t> ref_bbox_3d_centers_;
  std::unordered_map<id_t, Point2D_t> ref_bbox_move_flow_;
  std::unordered_map<id_t, SE3d> query_poses_;
  std::unordered_map<id_t, SE3d> ref_poses_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
