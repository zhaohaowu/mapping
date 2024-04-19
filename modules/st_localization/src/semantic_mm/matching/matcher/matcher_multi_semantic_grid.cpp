/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#include "semantic_mm/matching/matcher/matcher_multi_semantic_grid.hpp"

#include <algorithm>
#include <limits>

#include "common/utility.hpp"
#include "semantic_mm/base/lane_line.hpp"
#include "semantic_mm/base/pole.hpp"
#include "semantic_mm/base/traffic_sign.hpp"
#include "semantic_mm/common/configure.hpp"
#include "semantic_mm/common/math_tools.hpp"
#include "semantic_mm/frame/frame.hpp"
#include "semantic_mm/frame/frame_package.hpp"
#include "semantic_mm/map/map_manager.hpp"
#include "semantic_mm/matching/matching_manager.hpp"
#include "semantic_mm/tracking/tracking_manager.hpp"

namespace senseAD {
namespace localization {
namespace smm {

MatcherMultiSemanticGrid::MatcherMultiSemanticGrid() {
  grid_idx_converter_.reset(
      new VoxelIndexConverter(MatchParam::grid_size, MatchParam::grid_size));

  search_ys_.clear();
  step_size_ = MatchParam::grid_size / MatchParam::search_scale;
  lat_steps_ = std::round(MatchParam::max_search_lateral / step_size_);
  for (int i = -lat_steps_; i <= lat_steps_; ++i) {
    double y = i * step_size_;
    search_ys_.emplace_back(y);
  }

  // lane level smm reloc heading cov
  lane_level_heading_std_ = 2.0 * M_PI / 180.0;
}

void MatcherMultiSemanticGrid::InitMatch() {
  // reset map grids
  map_grids_.clear();
  max_index_.x_idx = std::numeric_limits<int>::min();
  max_index_.y_idx = std::numeric_limits<int>::min();
  min_index_.x_idx = std::numeric_limits<int>::max();
  min_index_.y_idx = std::numeric_limits<int>::max();
  max_map_line_heading_ = -M_PI;
  min_map_line_heading_ = M_PI;
  map_type_nums_.clear();
  map_sign_points_.clear();
  map_pole_points_.clear();

  // reset percept data
  percept_points_in_bv_.clear();
  downsampled_percept_points_in_bv_.clear();
  percept_line_types_.clear();
  vehicle_to_percept_line_dists_.clear();
  percept_type_nums_.clear();
  matched_type_nums_.clear();
  percept_frame_sources_.clear();
  percept_left_rs_id_ = -1;
  percept_right_rs_id_ = -1;
  percept_line_ave_heading_ = 0;
  percept_left_line_id_ = -1;
  percept_right_line_id_ = -1;

  // reset distance db
  map_dist_db_ = std::make_shared<ConsistencyDistDB>();
  percept_dist_db_ = std::make_shared<ConsistencyDistDB>();

  // search space related
  // search_ys_.clear(); // lateral search space is fixed
  heading_samples_.clear();
  heading_idx_for_each_y_.clear();
  multi_heading_percept_points_.clear();

  // search related
  possible_match_pairs_vec_.clear();
  last_possible_match_pairs_vec_.clear();
  map_line_type_centroids_.clear();
  percept_map_dist_db_ = std::make_shared<PerceptPointsToMapLineDist>();

  // search result related
  lateral_scores_predict_.clear();
  lateral_scores_measurement_.clear();
  thresholded_lateral_scores_.clear();
  lateral_peaks_.clear();

  // final result
  inlier_points_.clear();
  match_samples_.clear();
  d_heading_ = 0;
  d_lateral_ = 0;
  lateral_std_ = 0;
  heading_std_ = 0;
  multi_frame_match_.clear();
}

adLocStatus_t MatcherMultiSemanticGrid::ProcessGridMatch(
    const std::shared_ptr<FramePackage>& frame_package,
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager) {
  InitMatch();

  // 1. precondition check
  auto status = PreConditionCheck(tracking_manager, map_manager);
  if (status != LOC_SUCCESS) return status;

  // 2. preprocess percept/map and score lateral space
  status = LateralSpaceScoring(tracking_manager, map_manager);
  if (status != LOC_SUCCESS) return status;

  // 3. histogram filtering
  status = HistogramFiltering(frame_package);
  if (status != LOC_SUCCESS) return status;

  // 4. postcondition check
  if (MatchParam::enable_dist_consist_check) {
    status = DistanceConsistencyCheck(tracking_manager, map_manager);
    if (status != LOC_SUCCESS) return status;
  }

  status = PostConditionCheck();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "post condition check failed!";
    return status;
  }

  GetFinalMatchIndex();

  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::ProcessGridMatchRoadLevelAccuracy() {
  // regenerate match sample from lateral peaks
  if (lateral_peaks_.empty()) return LOC_LOCALIZATION_ERROR;
  if (match_samples_.empty()) {
    auto status = GetMatchSamples();
    if (status != LOC_SUCCESS) {
      LC_LDEBUG(MATCHING) << "smm reloc process road level failed";
      return status;
    }
  }
  GetFinalMatchIndex();

  // estimate roughly lateral/heading uncertainty
  double min_lateral = d_lateral_, max_lateral = d_lateral_;
  for (auto match : match_samples_) {
    auto sample = match.pose_sample;
    max_lateral = std::max(sample.y, max_lateral);
    min_lateral = std::min(sample.y, min_lateral);
  }

  map_road_heading_ = std::max(map_road_heading_, 1.0 * M_PI / 180.0);
  map_road_heading_ =
      std::min(map_road_heading_, 2.0 * MatchParam::max_map_heading);

  map_road_width_ = std::max(map_road_width_, 10.0);
  map_road_width_ = std::min(map_road_width_, 50.0);

  double factor_lat = std::fabs(max_lateral - min_lateral) /
                      (2.0 * MatchParam::max_search_lateral);
  lateral_std_ =
      0.5 * (1.0 + factor_lat) * map_road_width_ + 0.3 * std::fabs(d_lateral_);
  heading_std_ = map_road_heading_ + 0.3 * std::fabs(d_heading_);

  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::ProcessRelocTrackerMatch(
    const std::shared_ptr<FramePackage>& frame_package,
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager) {
  InitMatch();

  // 1. precondition check
  auto status = PreConditionCheck(tracking_manager, map_manager);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "smm reloc precondition check failed!";
    return status;
  }

  // 2. calculate likelihood
  status = LateralSpaceScoringWithParallelCheck(tracking_manager, map_manager,
                                                frame_package);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING)
        << "smm reloc lateral space score with parallel check failed!";
    return status;
  }

  // 3. histogram filter update
  status = HistogramFilteringNew(frame_package);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "smm reloc histogram filter failed!";
    return status;
  }

  // 4. estimate smm reloc pose cov
  status = EstimateRelocPoseCov();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "smm reloc estimate reloc pose cov failed!";
    return status;
  }
  // for visualization
  frame_package->SetMatchedLanelineData(inlier_points_);

  GetFinalMatchIndex();
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::LateralSpaceScoring(
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager) {
  auto status = EstimatePerceptLanelineHeading(tracking_manager);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "estimate laneline param failed!";
    return status;
  }

  status = SamplePerceptionPoints(tracking_manager);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "sample perception points failed!";
    return status;
  }

  status = ConstructMapGrids(map_manager);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "construct map grids failed!";
    return status;
  }

  status = GenerateSearchPose();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "generate search poses failed!";
    return status;
  }

  status = ScoreAllPoses();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "score all poses failed!";
    return status;
  }
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::LateralSpaceScoringWithParallelCheck(
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager,
    const std::shared_ptr<FramePackage>& frame_package) {
  // downsample all percept line points
  auto status = SamplePerceptionPointsNew(tracking_manager);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "sample perception points failed!";
    return status;
  }

  // get line type of valid percept lines
  status = RemoveUnstablePerceptLines();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCING) << "remove unstable perception lines failed!";
    return status;
  }

  // polyfit the nearest left and right percept line
  FindTheNearestPerceptLine2Vehicle();
  status = EstimatePerceptLanelineParams(tracking_manager);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "estimate percept laneline param failed!";
    return status;
  }
  SetPerceptPolyfitLineParam(frame_package, tracking_manager);

  // get line type and centroid of valid map lines
  status = CalculateMapLineTypeCentorid(map_manager);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "Calculate Map Line Type Centorid failed!";
    return status;
  }

  // polyfit valid map lines
  status = EstimateMapLaneLineParams(map_manager);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "estimate map laneline param failed!";
    return status;
  }
  SetMapPolyfitLineParam(frame_package);

  // get possible match pairs with line type consistency
  GetPossibleMatchPairsWithLineType();

  // calculate search heading for each lateral search position
  status = GenerateSearchPoseWithParallelCheck(frame_package);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "generate search poses failed!";
    return status;
  }

  // get likelihood distrib of histogram filter
  status = ScoreAllPosesNew(frame_package);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "score all poses failed!";
    return status;
  }
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::HistogramFiltering(
    const std::shared_ptr<FramePackage>& frame_package) {
  auto status = FilterPrediction(frame_package);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "filter prediction failed!";
    return status;
  }
  status = FilterUpdate();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "filter update failed!";
    return status;
  }

  status = FilterConvergenceCheck();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "filter not converged yet!";
    return status;
  }

  status = GetMatchSamples();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "get match samples failed!";
    return status;
  }
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::HistogramFilteringNew(
    const std::shared_ptr<FramePackage>& frame_package) {
  auto status = FilterPrediction(frame_package);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "filter prediction failed!";
    return status;
  }

  status = FilterUpdateNew();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "filter update failed!";
    return status;
  }
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::PreConditionCheck(
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager) {
  // statistic all perception type number
  PerceptionStatistic(tracking_manager);
  MapStatistic(map_manager);
  int roadside_num = percept_type_nums_[MatchSemanticType::RoadSideLine];
  int dashed_line_num = percept_type_nums_[MatchSemanticType::DashedLine];
  int solid_line_num = percept_type_nums_[MatchSemanticType::SolidLine];
  int sign_num = percept_type_nums_[MatchSemanticType::Sign];
  int pole_num = percept_type_nums_[MatchSemanticType::Pole];

  int line_num = roadside_num + dashed_line_num + solid_line_num;
  int total_elem_num = line_num + sign_num + pole_num;

  // total 3 semantic type, 5 match semantic type
  int semantic_type_num = 0;
  if (line_num) ++semantic_type_num;
  if (sign_num) ++semantic_type_num;
  if (pole_num) ++semantic_type_num;

  int match_semantic_type_num = 0;
  for (auto& item : percept_type_nums_) {
    if (item.second) ++match_semantic_type_num;
  }

  // 3 line match type
  int line_match_type_num = 0;
  if (dashed_line_num) ++line_match_type_num;
  if (solid_line_num) ++line_match_type_num;
  if (roadside_num) ++line_match_type_num;

  bool map_rs_valid = false, percept_rs_valid = false;
  double inconsist_dist;
  GetRoadsideDistanceCoarse(tracking_manager, map_manager, &percept_rs_valid,
                            &map_rs_valid, &inconsist_dist);

  // condition 1: pole exists, and there are enough match type/element number
  if (pole_num != 0 && match_semantic_type_num >= 3 && total_elem_num >= 5)
    return LOC_SUCCESS;

  // condition 2: there are reltively enough type/element number
  if (line_match_type_num >= 2 && total_elem_num >= 3) {
    // TODO(xxx): pattern match to reject ambiguity in early stage
    return LOC_SUCCESS;
  }

  // condition 3: percept roadside width check
  if (line_match_type_num >= 1 && total_elem_num >= 2) {
    // must have both roadside and pass consistency check
    if (map_rs_valid && percept_rs_valid &&
        inconsist_dist < MatchParam::max_prematch_rs_inconsist) {
      return LOC_SUCCESS;
    } else {
      LC_LDEBUG(MATCHING) << "percept/map roadside width inconsistent!";
    }
  }

  LC_LDEBUG(MATCHING) << "no precondition satisfied!";
  return LOC_LOCALIZATION_ERROR;
}

void MatcherMultiSemanticGrid::PerceptionStatistic(
    const std::shared_ptr<TrackingManager>& tracking_manager) {
  percept_type_nums_.clear();
  // laneline
  auto multi_frame_fused_lanelines = tracking_manager->GetFusedLaneLines();
  for (const auto& frame : multi_frame_fused_lanelines) {
    auto cam_name = frame.first;
    if (MatchParam::disable_fov30_lanelines &&
        cam_name.find("30") != std::string::npos)
      continue;
    for (const auto& item : frame.second) {
      if (item.second->processed_bv_points.empty()) continue;
      if (item.second->line_type == LineType::Curb) {
        ++percept_type_nums_[MatchSemanticType::RoadSideLine];
      } else if (IsDashedLine(item.second->line_style)) {
        ++percept_type_nums_[MatchSemanticType::DashedLine];
      } else {
        ++percept_type_nums_[MatchSemanticType::SolidLine];
      }
      percept_frame_sources_.insert({item.first, cam_name});
    }
  }
  // sign
  auto multi_frame_fused_signs = tracking_manager->GetFusedSigns();
  for (const auto& frame : multi_frame_fused_signs) {
    auto cam_name = frame.first;
    if (MatchParam::disable_fov30_lanelines &&
        cam_name.find("30") != std::string::npos)
      continue;
    percept_type_nums_[MatchSemanticType::Sign] += frame.second.size();
    for (const auto& item : frame.second) {
      percept_frame_sources_.insert({item.first, cam_name});
    }
  }
  // pole
  auto multi_frame_fused_poles = tracking_manager->GetFusedPoles();
  for (const auto& frame : multi_frame_fused_poles) {
    auto cam_name = frame.first;
    if (MatchParam::disable_fov30_lanelines &&
        cam_name.find("30") != std::string::npos)
      continue;
    percept_type_nums_[MatchSemanticType::Pole] += frame.second.size();
    for (const auto& item : frame.second) {
      percept_frame_sources_.insert({item.first, cam_name});
    }
  }
}

void MatcherMultiSemanticGrid::MapStatistic(
    const std::shared_ptr<MapManager>& map_manager) {
  map_type_nums_.clear();

  // laneline
  for (const auto& item : map_manager->GetLocalLaneLines()) {
    const auto& line_segment = item.second->GetLineSegments().front();
    if (line_segment->GetPoints().size() < 2) continue;
    if (IsRoadSideLine(line_segment->GetLineType())) {
      ++map_type_nums_[MatchSemanticType::RoadSideLine];
    } else if (IsDashedLine(line_segment->GetLineStyle())) {
      ++map_type_nums_[MatchSemanticType::DashedLine];
    } else {
      ++map_type_nums_[MatchSemanticType::SolidLine];
    }
  }
  // sign
  map_type_nums_[MatchSemanticType::Sign] =
      map_manager->GetLocalTrafficSigns().size();

  // pole
  map_type_nums_[MatchSemanticType::Pole] = map_manager->GetLocalPoles().size();
}

void MatcherMultiSemanticGrid::GetRoadsideDistanceCoarse(
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager,
    bool* percept_roadside_valid, bool* map_roadside_valid,
    double* inconsist_distance) {
  // for perception
  double percept_left_rs_dist = -1e10, percept_right_rs_dist = 1e10;
  for (const auto& frame : tracking_manager->GetFusedLaneLines()) {
    auto cam_name = frame.first;
    if (MatchParam::disable_fov30_lanelines &&
        cam_name.find("30") != std::string::npos)
      continue;
    for (const auto& item : frame.second) {
      const auto& points = item.second->processed_bv_points;
      if (points.empty()) continue;
      double y_mean = 0;
      for (const auto& pt : points) y_mean += pt.y;
      y_mean /= points.size();
      // get roadside id
      if (item.second->line_type != LineType::Curb) continue;
      if (y_mean > 0 && y_mean > percept_left_rs_dist) {
        percept_left_rs_dist = y_mean;
        percept_left_rs_id_ = item.first;
      }
      if (y_mean < 0 && y_mean < percept_right_rs_dist) {
        percept_right_rs_dist = y_mean;
        percept_right_rs_id_ = item.first;
      }
    }
  }
  bool percept_found = percept_left_rs_id_ != -1 &&
                       percept_right_rs_id_ != -1 &&
                       percept_left_rs_id_ != percept_right_rs_id_;

  // for map
  double map_left_rs_dist = -1e10, map_right_rs_dist = 1e10;
  id_t map_left_rs_id = -1, map_right_rs_id = -1;
  for (const auto& item : map_manager->GetLocalLaneLines()) {
    const auto& line_segment = item.second->GetLineSegments().front();
    const auto& points = line_segment->GetPoints();
    if (points.empty()) continue;
    if (!IsRoadSideLine(line_segment->GetLineType())) continue;
    double y_mean = 0;
    for (const auto& pt : points) y_mean += pt.y;
    y_mean /= points.size();
    if (y_mean > 0 && y_mean > map_left_rs_dist) {
      map_left_rs_dist = y_mean;
      map_left_rs_id = item.first;
    }
    if (y_mean < 0 && y_mean < map_right_rs_dist) {
      map_right_rs_dist = y_mean;
      map_right_rs_id = item.first;
    }
  }
  bool map_found = map_left_rs_id != -1 && map_right_rs_id != -1 &&
                   map_left_rs_id != map_right_rs_id;

  double p_rs_dist = percept_left_rs_dist - percept_right_rs_dist;
  double m_rs_dist = map_left_rs_dist - map_right_rs_dist;
  double inconsist_dist = std::fabs(p_rs_dist - m_rs_dist);

  *percept_roadside_valid = percept_found;
  *map_roadside_valid = map_found;
  *inconsist_distance = inconsist_dist;
}

adLocStatus_t MatcherMultiSemanticGrid::GenerateSearchPose() {
  // get map line points along lateral search line(x = 0)
  std::vector<MapPoint> map_grid_points;
  for (auto grid : map_grids_) {
    if (grid.first.x_idx != 0) continue;
    for (const auto& map_elemment : grid.second.map_elements) {
      if (!IsLaneLine(map_elemment.first)) continue;
      for (const auto& pt : map_elemment.second) {
        map_grid_points.emplace_back(pt);
        max_map_line_heading_ =
            std::max(max_map_line_heading_, pt.local_heading);
        min_map_line_heading_ =
            std::min(min_map_line_heading_, pt.local_heading);
      }
    }
  }
  map_road_heading_ = std::fabs(
      NormalizeAngleDiff(max_map_line_heading_ - min_map_line_heading_));
  if (map_grid_points.size() < 2) return LOC_LOCALIZATION_ERROR;

  // sort map grids in lateral direction
  std::sort(map_grid_points.begin(), map_grid_points.end(),
            [](const MapPoint& a, const MapPoint& b) {
              return a.point.y < b.point.y;
            });

  // get left and right map line point at each search position
  std::vector<std::pair<int, int>> solutions;
  solutions.reserve(search_ys_.size());
  for (const auto& y : search_ys_) {
    int left = 0, right = static_cast<int>(map_grid_points.size() - 1);
    if (y < map_grid_points.front().point.y) {
      solutions.emplace_back(left, -1);
      continue;
    }
    if (y > map_grid_points.back().point.y) {
      solutions.emplace_back(-1, right);
      continue;
    }
    Point2D_t search_pt(0, y);
    Point2D_t direct =
        map_grid_points.back().point - map_grid_points.front().point;
    direct = direct / direct.Norm();
    double pt_proj = (search_pt - map_grid_points.front().point).Dot(direct);
    while (right - left > 1) {
      int mid = left + (right - left) / 2;
      double proj = (map_grid_points[mid].point - map_grid_points.front().point)
                        .Dot(direct);
      if (proj < pt_proj) {
        left = mid;
      } else {
        right = mid;
      }
    }
    solutions.emplace_back(left, right);
  }

  // align heading btw percept and map at each search position
  std::vector<int> sample_heading_idx_for_each_y;
  std::vector<double> heading_samples;
  multi_heading_percept_points_.clear();
  int last_left = -1, last_right = -1;
  for (const auto& id_pair : solutions) {
    int left = id_pair.first, right = id_pair.second;
    if (left == -1 && right == -1) {
      LC_LDEBUG(MATCHING) << "why both side idx empty?";
      return LOC_LOCALIZATION_ERROR;
    }
    // same neighbour lines as last iteration, save computation
    if (left == last_left && right == last_right) {
      int last_idx = static_cast<int>(multi_heading_percept_points_.size()) - 1;
      sample_heading_idx_for_each_y.emplace_back(last_idx);
      continue;
    }
    double map_heading = 0;
    double cnt = 0;
    if (left != -1) {
      map_heading += map_grid_points[left].local_heading;
      ++cnt;
    }
    if (right != -1) {
      map_heading += map_grid_points[right].local_heading;
      ++cnt;
    }
    map_heading /= cnt;
    double sample_heading =
        NormalizeAngleDiff(map_heading - percept_line_ave_heading_);

    // limit aligned heading
    if (std::fabs(sample_heading) > MatchParam::max_align_heading) {
      double sign = sample_heading / std::fabs(sample_heading);
      sample_heading = sign * MatchParam::max_align_heading;
    }

    // rotate percept points
    Eigen::Matrix2d rotation;
    double cosh = std::cos(sample_heading), sinh = std::sin(sample_heading);
    rotation << cosh, -sinh, sinh, cosh;
    std::unordered_map<id_t, std::vector<PerceptPoint>> rotated_percept_pts =
        percept_points_in_bv_;
    for (auto& item : rotated_percept_pts) {
      for (auto& pt : item.second) pt.point = rotation * pt.point;
    }
    multi_heading_percept_points_.emplace_back(rotated_percept_pts);
    heading_samples.emplace_back(sample_heading);

    // generate heading index
    int heading_idx =
        static_cast<int>(multi_heading_percept_points_.size()) - 1;
    sample_heading_idx_for_each_y.emplace_back(heading_idx);
    last_left = left;
    last_right = right;
  }
  heading_idx_for_each_y_ = std::move(sample_heading_idx_for_each_y);
  heading_samples_ = std::move(heading_samples);
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::GenerateSearchPoseWithParallelCheck(
    const std::shared_ptr<FramePackage>& frame_package) {
  int search_size = search_ys_.size();
  std::vector<std::pair<double, bool>> search_heading_valid_array;
  std::vector<int> sample_heading_idx_for_each_y;
  std::vector<double> heading_samples;
  search_heading_valid_array.reserve(search_size);
  sample_heading_idx_for_each_y.reserve(search_size);
  heading_samples.reserve(search_size);

  const auto& map_line_params = frame_package->GetMapLinePolyfitParam();
  if (map_line_params.empty()) {
    LC_LDEBUG(MATCHING) << "no valid map polyfit line";
    return LOC_LOCALIZATION_ERROR;
  }

  // get percept valid
  Eigen::Vector4d percept_left_line_param, percept_right_line_param;
  bool percept_line_valid = frame_package->GetPerceptPolyfitParam(
      &percept_left_line_param, &percept_right_line_param);

  int last_left = -1, last_right = -1;
  for (const auto search_y : search_ys_) {
    // Determine the map lane line ids on the left and right sides of
    // search_y
    auto pair = BinarySearchTwoNearestMapLinesId(search_y, map_line_params);
    int left = pair.first, right = pair.second;

    if (left == -1 && right == -1) {
      LC_LDEBUG(MATCHING) << "search y's map left line and right line = 0";
      return LOC_LOCALIZATION_ERROR;
    }
    // check last is equal to cur
    if (last_left == left && last_right == right) {
      int last_idx = static_cast<int>(heading_samples.size()) - 1;
      sample_heading_idx_for_each_y.emplace_back(last_idx);
      search_heading_valid_array.emplace_back(
          search_y, search_heading_valid_array.back().second);
      continue;
    }
    // check last is parallel to cur
    if (last_right == left) {
      if (last_left != -1 && right != -1 &&
          CheckTwoPolyFitLineParallel(map_line_params[last_left],
                                      map_line_params[right])) {
        int last_idx = static_cast<int>(heading_samples.size()) - 1;
        sample_heading_idx_for_each_y.emplace_back(last_idx);
        search_heading_valid_array.emplace_back(
            search_y, search_heading_valid_array.back().second);

        last_left = left;
        last_right = right;
        continue;
      }
    }
    // check map line valid
    bool search_map_line_valid = true;
    if (left != -1 && right != -1) {
      search_map_line_valid = CheckTwoPolyFitLineParallel(
          map_line_params[left], map_line_params[right]);
    }
    // calculate sample heading
    double sample_heading = 0;
    bool search_heading_valid = search_map_line_valid && percept_line_valid;
    if (search_heading_valid) {
      sample_heading = SolveYawByOptimPoint2LineDist(
          percept_left_line_param, percept_right_line_param, map_line_params,
          left, right);

      sample_heading =
          std::min(std::max(sample_heading, -MatchParam::max_align_heading),
                   MatchParam::max_align_heading);
    }
    // check sample heading
    if (search_heading_valid) {
      if (left != -1 && percept_right_line_id_ != -1) {
        const auto& percept_points =
            downsampled_percept_points_in_bv_.at(percept_right_line_id_);
        search_heading_valid = CheckPerceptPointsMapLineParallel(
            percept_points, map_line_params[left],
            Sophus::SE2<double>(sample_heading, Eigen::Vector2d::Zero()));
      } else if (right != -1 && percept_left_line_id_ != -1) {
        const auto& percept_points =
            downsampled_percept_points_in_bv_.at(percept_left_line_id_);
        search_heading_valid = CheckPerceptPointsMapLineParallel(
            percept_points, map_line_params[right],
            Sophus::SE2<double>(sample_heading, Eigen::Vector2d::Zero()));
      }
    }
    if (!search_heading_valid) sample_heading = 0;

    // calculate percept-to-map dist db
    ConstructPerceptionMapDistDB(sample_heading);

    // store related data
    heading_samples.emplace_back(sample_heading);
    search_heading_valid_array.emplace_back(search_y, search_heading_valid);

    int last_idx = static_cast<int>(heading_samples.size()) - 1;
    sample_heading_idx_for_each_y.emplace_back(last_idx);

    last_left = left;
    last_right = right;
  }

  heading_idx_for_each_y_ = std::move(sample_heading_idx_for_each_y);
  heading_samples_ = std::move(heading_samples);
  frame_package->SetSearchHeadingValidArray(search_heading_valid_array);
  return LOC_SUCCESS;
}

double MatcherMultiSemanticGrid::SolveYawByOptimPoint2LineDist(
    const Eigen::Vector4d& percept_left_line_param,
    const Eigen::Vector4d& percept_right_line_param,
    const std::vector<Eigen::Vector4d>& map_all_line_params, int left,
    int right) {
  std::vector<std::vector<Point2DWithCov>> percept_points;
  std::vector<Eigen::Vector4d> percept_line_params;
  std::vector<Eigen::Vector4d> map_line_params;
  auto PreparePerceptDataForSolveYaw =
      [this, &percept_line_params, &map_line_params, &percept_points](
          id_t percept_line_id, const Eigen::Vector4d& percept_line_param,
          const Eigen::Vector4d& map_line_param) {
        const auto& percept_line_points =
            downsampled_percept_points_in_bv_.at(percept_line_id);
        percept_line_params.emplace_back(percept_line_param);
        map_line_params.emplace_back(map_line_param);
        percept_points.emplace_back(percept_line_points);
      };

  // ensure that corresponding percept and map line polyfit parameters are
  // both valid
  if (left != -1 && percept_poly_line_params_.count(percept_right_line_id_)) {
    PreparePerceptDataForSolveYaw(percept_right_line_id_,
                                  percept_right_line_param,
                                  map_all_line_params[left]);
  }
  if (right != -1 && percept_poly_line_params_.count(percept_left_line_id_)) {
    PreparePerceptDataForSolveYaw(percept_left_line_id_,
                                  percept_left_line_param,
                                  map_all_line_params[right]);
  }
  double optimize_heading = 0;
  bool valid = OptimizeYawByPerceptPointsMapLine(
      percept_points, percept_line_params, map_line_params, &optimize_heading);
  if (!valid) {
    LC_LDEBUG(MATCHING)
        << "smm reloc optimize yaw by percept points to map line failed";
  }
  return optimize_heading;
}

double MatcherMultiSemanticGrid::SolveYByOptimPoint2LineDist(double y_init) {
  const Sophus::SE2<double> transform(d_heading_, Eigen::Vector2d(0, y_init));
  double y_mean = 0, w_sum = 0, y_var = 0;
  std::vector<double> y_diffs, weights;
  const auto& match_sample = match_samples_.front();
  for (const auto& item : match_sample.match->GetAllSemanticMatch()) {
    const auto& semantic_type = item.first;
    const auto& match_pairs = item.second;
    if (semantic_type != SemanticType::LaneLine) continue;
    for (const auto& match_pair : match_pairs) {
      const auto& percept_id = match_pair.first;
      const auto& map_id = match_pair.second;
      if (percept_id != percept_left_line_id_ &&
          percept_id != percept_right_line_id_)
        continue;
      if (!downsampled_percept_points_in_bv_.count(percept_id)) {
        LC_LDEBUG(debug) << "Can not find percept id: " << percept_id
                         << " in solve lateral position, impossible!";
        continue;
      }
      if (!map_poly_line_params_.count(map_id)) {
        LC_LDEBUG(debug) << "Can not find map id: " << map_id
                         << " in solve lateral position, impossible!";
        continue;
      }
      const auto& percept_points =
          downsampled_percept_points_in_bv_.at(percept_id);
      const auto& map_line_param = map_poly_line_params_.at(map_id).line_param;
      for (const auto& percept_point : percept_points) {
        Point2D_t transformed_point;
        transformed_point = transform * percept_point.point;
        double map_y = FourDegreePolyFunc(map_line_param, transformed_point.x);
        double y_diff = transformed_point.y - map_y;
        if (std::fabs(y_diff) > step_size_) continue;
        double weight =
            1.0 / std::max(percept_point.cov.diagonal().norm(), 1e-4);
        y_diffs.emplace_back(y_diff);
        weights.emplace_back(weight);
        inlier_points_.emplace_back(
            Point3D_t(percept_point.point.x, percept_point.point.y, 0));
        w_sum += weight;
      }
    }
  }
  for (auto& weight : weights) weight /= w_sum;
  for (int i = 0; i < y_diffs.size(); i++) {
    y_mean += weights[i] * y_diffs[i];
  }
  for (int i = 0; i < y_diffs.size(); i++) {
    y_var += weights[i] * (y_diffs[i] - y_mean) * (y_diffs[i] - y_mean);
  }
  lane_level_lateral_std_ = std::sqrt(y_var);
  return y_init - y_mean;
}

adLocStatus_t MatcherMultiSemanticGrid::ConstructMapGrids(
    const std::shared_ptr<MapManager>& map_manager) {
  auto UpdateGrid = [this](const VoxelIndex& index,
                           const MatchSemanticType& grid_type,
                           const MapPoint& map_pt) {
    auto& grid = map_grids_[index];
    auto& element_ids = grid.map_elements[grid_type];
    element_ids.emplace_back(map_pt);
  };

  map_grids_.clear();
  // lane lines
  const LaneLine::PtrUMap& map_lanelines = map_manager->GetLocalLaneLines();
  for (const auto& item : map_lanelines) {
    const auto& line_segment = item.second->GetLineSegments().front();
    id_t id = item.first;
    const auto& ll_pts = line_segment->GetPoints();
    if (ll_pts.size() < 2) continue;

    // estimate local heading
    std::vector<double> local_headings;
    double heading_x0 = 0;
    double dist_x0 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < ll_pts.size() - 1; ++i) {
      Point3D_t pi = ll_pts[i], pj = ll_pts[i + 1];
      if (pi.x > pj.x) {
        pi = ll_pts[i + 1];
        pj = ll_pts[i];
      }
      double heading_line =
          std::atan2(pj.y - pi.y, std::max(pj.x - pi.x, 1e-8));
      local_headings.emplace_back(heading_line);
      if (std::fabs(pi.x) < dist_x0) {
        dist_x0 = std::fabs(pi.x);
        heading_x0 = heading_line;
      }
    }
    local_headings.emplace_back(local_headings.back());
    // reject map lines with large heading w.r.t. vehicle
    if (std::fabs(heading_x0) > MatchParam::max_map_heading) continue;

    // get line match type
    MatchSemanticType grid_type = MatchSemanticType::SolidLine;
    if (IsRoadSideLine(line_segment->GetLineType())) {
      grid_type = MatchSemanticType::RoadSideLine;
    } else if (IsDashedLine(line_segment->GetLineStyle())) {
      grid_type = MatchSemanticType::DashedLine;
    }

    // upsample map points into continuous grid index
    std::unordered_set<VoxelIndex, VoxelIndexHash> index_added;
    for (size_t i = 0; i < ll_pts.size(); ++i) {
      Point2D_t pt(ll_pts[i].x, ll_pts[i].y);
      auto index = grid_idx_converter_->PointInnerToVoxelIndex(pt);
      if (index_added.count(index)) continue;
      MapPoint map_pt(id, pt, local_headings[i]);
      UpdateGrid(index, grid_type, map_pt);
      max_index_.x_idx = std::max(max_index_.x_idx, index.x_idx);
      max_index_.y_idx = std::max(max_index_.y_idx, index.y_idx);
      min_index_.x_idx = std::min(min_index_.x_idx, index.x_idx);
      min_index_.y_idx = std::min(min_index_.y_idx, index.y_idx);
      index_added.insert(index);
      if (i == 0) continue;

      // upsample points if map points distance is larger than grid size
      Point2D_t pt_last(ll_pts[i - 1].x, ll_pts[i - 1].y);
      Point2D_t vec = pt - pt_last;
      double scale_x = 1.0, scale_y = 1.0;
      int sample_num;
      if (std::fabs(vec.x) > std::fabs(vec.y)) {
        sample_num =
            static_cast<int>(std::fabs(vec.x) / MatchParam::grid_size + 0.5);
        scale_y = vec.y / vec.x;
      } else {
        sample_num =
            static_cast<int>(std::fabs(vec.y) / MatchParam::grid_size + 0.5);
        scale_x = vec.x / vec.y;
      }
      double heading_last = local_headings[i - 1];
      for (int j = 1; j <= sample_num; ++j) {
        double delta = j * MatchParam::grid_size;
        double sample_x = pt_last.x + scale_x * delta;
        double sample_y = pt_last.y + scale_y * delta;
        Point2D_t pt_sample(sample_x, sample_y);
        auto sample_index =
            grid_idx_converter_->PointInnerToVoxelIndex(pt_sample);
        if (!index_added.count(sample_index)) {
          MapPoint sample_map_pt(id, pt_sample, heading_last);
          UpdateGrid(sample_index, grid_type, sample_map_pt);
          index_added.insert(sample_index);
        }
      }
    }
  }
  map_road_width_ =
      std::fabs((max_index_.y_idx - min_index_.y_idx) * MatchParam::grid_size);
  if (map_grids_.empty()) return LOC_LOCALIZATION_ERROR;

  // signs
  const auto& map_signs = map_manager->GetLocalTrafficSigns();
  for (const auto& item : map_signs) {
    id_t id = item.first;
    auto center = item.second->GetCenter();
    Point2D_t pt(center.x, center.y);
    map_sign_points_.emplace_back(MapPoint(id, pt));
  }

  // poles
  const auto& map_poles = map_manager->GetLocalPoles();
  for (const auto& item : map_poles) {
    id_t id = item.first;
    auto center = item.second->GetBottomPoint();
    Point2D_t pt(center.x, center.y);
    map_pole_points_.emplace_back(MapPoint(id, pt));
  }

  // construct map distance dababase
  for (const auto& item : map_lanelines) {
    auto points = item.second->GetLineSegments().front()->GetPoints();
    map_dist_db_->AddElement(item.first, SemanticType::LaneLine, points);
  }
  for (const auto& item : map_signs) {
    map_dist_db_->AddElement(item.first, SemanticType::TrafficSign,
                             {item.second->GetCenter()});
  }
  for (const auto& item : map_poles) {
    map_dist_db_->AddElement(item.first, SemanticType::TrafficSign,
                             {item.second->GetBottomPoint()});
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::EstimatePerceptLanelineHeading(
    const std::shared_ptr<TrackingManager>& tracking_manager) {
  struct PolyLineParam {
    bool valid;
    Eigen::VectorXd line_param;
    double heading;
    double confidence;
  };
  std::unordered_map<id_t, PolyLineParam> pecept_poly_line_params;

  // laneline process
  const auto& multi_frame_fused_lanelines =
      tracking_manager->GetFusedLaneLines();
  for (const auto& cam_ll_pair : multi_frame_fused_lanelines) {
    auto cam_name = cam_ll_pair.first;
    if (MatchParam::disable_fov30_lanelines &&
        cam_name.find("30") != std::string::npos)
      continue;
    const auto& fused_lanelines = cam_ll_pair.second;
    for (const auto& item : fused_lanelines) {
      const auto& points = item.second->processed_bv_points;
      if (points.empty()) {
        LC_LDEBUG(MATCHING)
            << "why percept line " << item.first << " is empty?";
        continue;
      }
      std::vector<double> weights;
      weights.reserve(item.second->processed_bv_point_covs.size());
      double w_sum = 0;
      for (auto cov : item.second->processed_bv_point_covs) {
        double cov_norm = std::sqrt(cov.diagonal().norm());
        double w = 1.0 / std::max(cov_norm, 1e-4);
        weights.emplace_back(w);
        w_sum += w;
      }
      for (auto& w : weights) w /= w_sum;

      Eigen::VectorXd line_param(4, 1);
      double mean_residual = 0, max_residual = 0;
      double line_length = (points.front() - points.back()).Norm();
      bool has_fit = false;
      int order = 0;
      if (points.back().x < 0 || points.front().x > 2 || line_length < 5.0 ||
          points.size() < 10) {
        // line too short or far away from vehicle, straght line fitting
        order = 1;
        Eigen::VectorXd straght_line_param;
        has_fit = PolyLineLeastSquareFitting(points, weights, order,
                                             &straght_line_param,
                                             &mean_residual, &max_residual);
        if (has_fit) {
          line_param.setZero();
          line_param[2] = straght_line_param[0];
          line_param[3] = straght_line_param[1];
        }
      } else {
        // try cubic poly line fitting
        order = 3;
        has_fit = PolyLineLeastSquareFitting(
            points, weights, order, &line_param, &mean_residual, &max_residual);
        // downgrade to straght line fitting
        if (has_fit && (mean_residual > 0.2 && max_residual > 0.5)) {
          order = 1;
          Eigen::VectorXd straght_line_param;
          has_fit = PolyLineLeastSquareFitting(points, weights, order,
                                               &straght_line_param,
                                               &mean_residual, &max_residual);
          if (has_fit) {
            line_param.setZero();
            line_param[2] = straght_line_param[0];
            line_param[3] = straght_line_param[1];
          }
        }
      }

      PolyLineParam polyline_param;
      polyline_param.valid = has_fit;
      if (!has_fit) {
        polyline_param.confidence = 0;
        LC_LDEBUG(MATCHING) << "percept line " << item.first
                            << " has invalid line fitting param";
      } else {
        double fit_confidence = std::exp(-mean_residual / 0.1);
        double len_confidence = std::min(10.0, line_length) / 10.0;
        double num_confidence =
            std::min(10.0, static_cast<double>(points.size())) / 10.0;
        polyline_param.line_param = line_param;
        polyline_param.confidence =
            fit_confidence * len_confidence * num_confidence;
        // line heading at x = 0
        double tanh = line_param[2];
        double heading_line = std::atan2(tanh, 1.0);
        polyline_param.heading = heading_line;
      }
      pecept_poly_line_params.insert({item.first, polyline_param});
    }
  }
  // check fitting quality, corse check
  int good_line_cnt = 0;
  for (const auto& param : pecept_poly_line_params) {
    if (param.second.confidence > 0.3) ++good_line_cnt;
  }
  if (good_line_cnt == 0) {
    LC_LDEBUG(MATCHING) << "percept line quality bad!";
    return LOC_LOCALIZATION_ERROR;
  }

  // get average percept line headings
  double left_line_distance = 100, right_line_distance = -100;
  id_t invalid = std::numeric_limits<id_t>::min();
  id_t left_id = invalid, right_id = invalid;
  for (const auto& pecept_line_param : pecept_poly_line_params) {
    if (!pecept_line_param.second.valid) continue;
    id_t id = pecept_line_param.first;
    double dist = pecept_line_param.second.line_param[3];
    if (dist > 0 && dist < left_line_distance) {
      left_line_distance = dist;
      left_id = id;
    }
    if (dist < 0 && dist > right_line_distance) {
      right_line_distance = dist;
      right_id = id;
    }
  }

  percept_line_ave_heading_ = 0;
  double sum_confidence = 0;
  int sum_cnt = 0;
  for (const auto& pecept_line_param : pecept_poly_line_params) {
    const auto& param = pecept_line_param.second;
    if (!param.valid) continue;
    if (std::fabs(param.heading) > MatchParam::max_percept_heading) continue;
    id_t id = pecept_line_param.first;
    // increase weight for lines close to vehcle
    double weight = 1.0;
    if (id == left_id || id == right_id) weight = 5.0;
    percept_line_ave_heading_ += weight * param.confidence * param.heading;
    sum_confidence += weight * param.confidence;
    ++sum_cnt;
  }
  if (sum_cnt == 0) return LOC_LOCALIZATION_ERROR;
  percept_line_ave_heading_ /= sum_confidence;
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::EstimatePerceptLanelineParams(
    const std::shared_ptr<TrackingManager>& tracking_manager) {
  auto PolyfitPerceptLine = [this](const id_t& percept_id) {
    if (!downsampled_percept_points_in_bv_.count(percept_id)) {
      LC_LDEBUG(MATCHING) << "Can not find percept id: " << percept_id
                          << ", in estimate percept line params, impossible!";
      return;
    }
    const auto& percept_points =
        downsampled_percept_points_in_bv_.at(percept_id);
    std::vector<Point3D_t> percept_pts_3d;
    std::vector<double> weights;
    percept_pts_3d.reserve(percept_points.size());
    weights.reserve(percept_points.size());
    double w_sum = 0;
    for (int i = 0; i < percept_points.size(); i++) {
      double cov_norm = percept_points[i].cov.diagonal().norm();
      double w = 1.0 / std::max(cov_norm, 1e-4);
      weights.emplace_back(w);
      percept_pts_3d.emplace_back(
          Point3D_t(percept_points[i].point.x, percept_points[i].point.y, 0));
      w_sum += w;
    }
    for (auto& w : weights) w /= w_sum;

    Eigen::Vector4d line_param;
    bool has_fit =
        AdaptedPolyfitLineWithLength(percept_pts_3d, weights, &line_param);

    PolyLineParam polyline_param;
    polyline_param.valid = has_fit;
    polyline_param.line_param = line_param;
    if (has_fit) percept_poly_line_params_.insert({percept_id, polyline_param});
  };

  percept_poly_line_params_.clear();

  if (percept_left_line_id_ != -1) {
    PolyfitPerceptLine(percept_left_line_id_);
  }
  if (percept_right_line_id_ != -1) {
    PolyfitPerceptLine(percept_right_line_id_);
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::EstimateMapLaneLineParams(
    const std::shared_ptr<MapManager>& map_manager) {
  map_poly_line_params_.clear();
  const auto& map_lanelines = map_manager->GetLocalLaneLines();
  if (map_lanelines.empty()) {
    return LOC_LOCALIZATION_ERROR;
  }

  int map_line_type_centroids_size = map_line_type_centroids_.size();
  std::vector<std::pair<id_t, LineTypeCentroid>> map_line_type_centroids;
  map_line_type_centroids.reserve(map_line_type_centroids_size);
  std::vector<bool> map_line_fit_success(map_line_type_centroids_size, false);

  for (int j = 0; j < map_line_type_centroids_size; j++) {
    id_t id = map_line_type_centroids_[j].first;
    if (map_lanelines.find(id) == map_lanelines.end()) {
      LC_LDEBUG(debug) << "why can not find map id: " << id
                       << "in estimate map laneline params";
      continue;
    }

    const auto& points =
        map_lanelines.at(id)->GetLineSegments().front()->GetPoints();

    std::vector<Point3D_t> inlier_points;
    inlier_points.reserve(points.size());
    for (int i = 0; i < points.size(); i++) {
      if (points[i].x < -10. || points[i].x > 20.) continue;
      inlier_points.emplace_back(points[i]);
    }
    std::vector<double> inlier_weights;
    inlier_weights.resize(inlier_points.size(), 1.0 / inlier_points.size());

    Eigen::Vector4d line_param;
    bool has_fit = AdaptedPolyfitLineWithLength(inlier_points, inlier_weights,
                                                &line_param);

    PolyLineParam polyline_param;
    polyline_param.valid = has_fit;
    polyline_param.line_param = line_param;
    polyline_param.confidence = 1.0;

    if (has_fit) {
      map_line_fit_success[j] = true;
      map_poly_line_params_.insert({id, polyline_param});
    }
  }

  if (map_poly_line_params_.empty()) {
    return LOC_LOCALIZATION_ERROR;
  }
  // remove polyfit failed map lines
  for (int j = 0; j < map_line_type_centroids_.size(); j++) {
    if (!map_line_fit_success[j]) continue;
    map_line_type_centroids.emplace_back(map_line_type_centroids_[j]);
  }

  map_line_type_centroids_ = map_line_type_centroids;
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::SetPerceptPolyfitLineParam(
    const std::shared_ptr<FramePackage>& frame_package,
    const std::shared_ptr<TrackingManager>& tracking_manager) {
  bool percept_line_valid = false;
  Eigen::Vector4d left_line_param, right_line_param;
  left_line_param.setZero();
  right_line_param.setZero();

  if (percept_poly_line_params_.count(percept_left_line_id_)) {
    left_line_param =
        percept_poly_line_params_.at(percept_left_line_id_).line_param;
    percept_line_valid = true;
    LC_LDEBUG(debug) << "percept left line polyfit success!, percept id: "
                     << percept_left_line_id_;
  }
  if (percept_poly_line_params_.count(percept_right_line_id_)) {
    right_line_param =
        percept_poly_line_params_.at(percept_right_line_id_).line_param;
    percept_line_valid = true;
    LC_LDEBUG(debug) << "percept right line polyfit success!, percept id: "
                     << percept_right_line_id_;
  }

  frame_package->SetPerceptPolyfitParam(left_line_param, right_line_param,
                                        percept_line_valid);

  return LOC_SUCCESS;
}

void MatcherMultiSemanticGrid::SetMapPolyfitLineParam(
    const std::shared_ptr<FramePackage>& frame_package) {
  int param_size = 4;
  std::vector<Eigen::Vector4d> map_line_params;
  for (const auto& item : map_poly_line_params_) {
    if (!item.second.valid) continue;
    const auto& map_line_param = item.second.line_param;
    map_line_params.emplace_back(map_line_param);
    max_map_line_heading_ = std::max(
        std::atan2(map_line_param[param_size - 2], 1.0), max_map_line_heading_);
    min_map_line_heading_ = std::min(
        std::atan2(map_line_param[param_size - 2], 1.0), min_map_line_heading_);
  }
  std::sort(map_line_params.begin(), map_line_params.end(),
            [](const Eigen::Vector4d& a, const Eigen::Vector4d& b) {
              int n = a.size();
              return a[n - 1] < b[n - 1];
            });
  frame_package->SetMapLinePolyfitParam(map_line_params);

  map_road_heading_ = std::fabs(
      NormalizeAngleDiff(max_map_line_heading_ - min_map_line_heading_));
  map_road_width_ = map_line_params.back()[param_size - 1] -
                    map_line_params.front()[param_size - 1];
}

adLocStatus_t MatcherMultiSemanticGrid::SamplePerceptionPoints(
    const std::shared_ptr<TrackingManager>& tracking_manager) {
  // NOTE: tracking results in map-free mode should have globle unique id
  // sample laneline points
  const auto& multi_frame_fused_lanelines =
      tracking_manager->GetFusedLaneLines();
  for (const auto& cam_ll_pair : multi_frame_fused_lanelines) {
    auto cam_name = cam_ll_pair.first;
    if (MatchParam::disable_fov30_lanelines &&
        cam_name.find("30") != std::string::npos)
      continue;
    const auto& fused_lanelines = cam_ll_pair.second;
    for (const auto& item : fused_lanelines) {
      id_t tracker_id = item.first;
      if (percept_points_in_bv_.count(tracker_id)) {
        LC_LDEBUG(MATCHING) << "why already has sampled perception points?";
        continue;
      }
      auto& percept_pts = percept_points_in_bv_[tracker_id];

      MatchSemanticType grid_line_type = MatchSemanticType::SolidLine;
      if (item.second->line_type == LineType::Curb) {
        grid_line_type = MatchSemanticType::RoadSideLine;
      } else if (IsDashedLine(item.second->line_style)) {
        grid_line_type = MatchSemanticType::DashedLine;
      }

      // sample perception points
      const auto& points = item.second->processed_bv_points;
      const auto& covs = item.second->processed_bv_point_covs;
      std::vector<int> sample_idxs =
          VoxelDownSample2D(points, covs, MatchParam::line_sample_size);
      std::sort(sample_idxs.begin(), sample_idxs.end(),
                [&points](int i, int j) { return points[i].x < points[j].x; });
      percept_pts.reserve(sample_idxs.size());
      for (const auto& i : sample_idxs) {
        PerceptPoint pt(Point2D_t(points[i].x, points[i].y), covs[i],
                        grid_line_type);
        percept_pts.emplace_back(pt);
      }
    }
  }

  // assign weight to sampled laneline points
  double w_max = 0.0;
  for (auto& item : percept_points_in_bv_) {
    for (auto& pt : item.second) {
      double w = 1.0 / (std::max(pt.cov.diagonal().norm(), 1e-4));
      pt.weight = w;
      w_max = std::max(w_max, w);
    }
  }
  for (auto& item : percept_points_in_bv_) {
    for (auto& pt : item.second) {
      pt.weight /= w_max;
    }
  }

  // get traffic signs
  const auto& multi_frame_fused_signs = tracking_manager->GetFusedSigns();
  for (const auto& cam_ts_pair : multi_frame_fused_signs) {
    const auto& cam_name = cam_ts_pair.first;
    if (MatchParam::disable_fov30_lanelines &&
        cam_name.find("30") != std::string::npos)
      continue;

    const auto& fused_signs = cam_ts_pair.second;
    for (const auto& item : fused_signs) {
      id_t tracker_id = item.first;
      if (percept_points_in_bv_.count(tracker_id)) {
        LC_LDEBUG(MATCHING) << "why already has sampled perception points?";
        continue;
      }
      auto& percept_pts = percept_points_in_bv_[tracker_id];
      auto processed_center = item.second->processed_center;
      // TODO(fy): get cov from tracker
      PerceptPoint pt(Point2D_t(processed_center.x, processed_center.y),
                      Eigen::Matrix2d::Identity(), MatchSemanticType::Sign,
                      item.second->observ_cnt);
      percept_pts.emplace_back(pt);
    }
  }

  // get poles
  const auto& multi_frame_fused_poles = tracking_manager->GetFusedPoles();
  for (const auto& cam_p_pair : multi_frame_fused_poles) {
    const auto& cam_name = cam_p_pair.first;
    if (MatchParam::disable_fov30_lanelines &&
        cam_name.find("30") != std::string::npos)
      continue;

    const auto& fused_poles = cam_p_pair.second;
    for (const auto& item : fused_poles) {
      id_t tracker_id = item.first;
      if (percept_points_in_bv_.count(tracker_id)) {
        LC_LDEBUG(MATCHING) << "why already has sampled perception points?";
        continue;
      }
      auto& percept_pts = percept_points_in_bv_[tracker_id];
      auto processed_center = item.second->processed_center;
      PerceptPoint pt(Point2D_t(processed_center.x, processed_center.y),
                      Eigen::Matrix2d::Identity(), MatchSemanticType::Pole,
                      item.second->observ_cnt);
      percept_pts.emplace_back(pt);
    }
  }

  // construct percept distance database
  for (const auto& item : percept_points_in_bv_) {
    id_t id = item.first;
    SemanticType semantic_type =
        MatchTypeToSemanticType(item.second.front().type);
    std::vector<Point3D_t> points;
    points.reserve(item.second.size());
    for (const auto& percept_pt : item.second) {
      points.emplace_back(percept_pt.point.x, percept_pt.point.y, 0);
    }
    if (points.empty()) continue;
    percept_dist_db_->AddElement(id, semantic_type, points);
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::SamplePerceptionPointsNew(
    const std::shared_ptr<TrackingManager>& tracking_manager) {
  const auto& multi_frame_fused_lanelines =
      tracking_manager->GetFusedLaneLines();
  for (const auto& frame : multi_frame_fused_lanelines) {
    auto cam_name = frame.first;
    if (MatchParam::disable_fov30_lanelines &&
        cam_name.find("30") != std::string::npos)
      continue;
    for (const auto& item : frame.second) {
      const auto& percept_id = item.first;
      if (downsampled_percept_points_in_bv_.count(percept_id)) {
        LC_LDEBUG(debug) << "why downsampled percept points has already "
                            "exist? percept_id: "
                         << percept_id;
        continue;
      }
      const auto& points = item.second->processed_bv_points;
      const auto& covs = item.second->processed_bv_point_covs;
      if (points.empty()) continue;

      // downsample perception points
      auto& downsampled_percept_pts =
          downsampled_percept_points_in_bv_[percept_id];
      std::vector<int> sample_idxs = VoxelDownSample2D(
          points, covs, MatchParam::line_sample_size, -10, 20);
      if (sample_idxs.empty()) {
        downsampled_percept_points_in_bv_.erase(percept_id);
        continue;
      }

      std::sort(sample_idxs.begin(), sample_idxs.end(),
                [&points](int i, int j) { return points[i].x < points[j].x; });
      downsampled_percept_pts.reserve(sample_idxs.size());
      for (const auto& i : sample_idxs) {
        Point2DWithCov percept_point(Point2D_t(points[i].x, points[i].y),
                                     covs[i]);
        downsampled_percept_pts.emplace_back(percept_point);
      }

      // save line types
      MatchSemanticType line_type;
      if (item.second->line_type == LineType::Curb) {
        line_type = MatchSemanticType::RoadSideLine;
      } else if (IsDashedLine(item.second->line_style)) {
        line_type = MatchSemanticType::DashedLine;
      } else {
        line_type = MatchSemanticType::SolidLine;
      }
      percept_line_types_.insert({percept_id, line_type});
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::RemoveUnstablePerceptLines() {
  for (const auto& item : downsampled_percept_points_in_bv_) {
    const auto& percept_id = item.first;
    const auto& percept_points = item.second;
    if (percept_points.empty()) continue;

    std::vector<Point3D_t> percept_pts_3d;
    percept_pts_3d.reserve(percept_points.size());
    for (const auto point2d_with_cov : percept_points) {
      percept_pts_3d.emplace_back(
          Point3D_t(point2d_with_cov.point.x, point2d_with_cov.point.y, 0));
    }

    Point3D_t origin(0, 0, 0);
    bool valid = false;
    double vehicle_to_percept_line_dist =
        Point2LineDistance2D(origin, percept_pts_3d, &valid, true);

    if (valid) {
      vehicle_to_percept_line_dists_.insert(
          {percept_id, vehicle_to_percept_line_dist});
    } else {
      // vehicle is out range of percept line, this line may be unstable
      percept_line_types_.erase(percept_id);
    }
  }
  if (percept_line_types_.empty()) {
    LC_LDEBUG(MATCHING) << "valid percept line types is empty!";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

void MatcherMultiSemanticGrid::FindTheNearestPerceptLine2Vehicle() {
  std::vector<std::pair<id_t, double>> vehicle_to_percept_line_dists_vec;
  for (const auto& item : vehicle_to_percept_line_dists_) {
    vehicle_to_percept_line_dists_vec.emplace_back(item);
  }

  std::sort(vehicle_to_percept_line_dists_vec.begin(),
            vehicle_to_percept_line_dists_vec.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });

  if (vehicle_to_percept_line_dists_vec.front().second >= 0) {
    percept_right_line_id_ = vehicle_to_percept_line_dists_vec.front().first;
    percept_left_line_id_ = -1;
  } else if (vehicle_to_percept_line_dists_vec.back().second <= 0) {
    percept_left_line_id_ = vehicle_to_percept_line_dists_vec.back().first;
    percept_right_line_id_ = -1;
  } else {
    int l = 0, r = vehicle_to_percept_line_dists_vec.size() - 1;
    while (r > l + 1) {
      int mid = l + (r - l) / 2;
      double dist = vehicle_to_percept_line_dists_vec[mid].second;
      if (dist < 0) {
        l = mid;
      } else {
        r = mid;
      }
    }
    percept_left_line_id_ = vehicle_to_percept_line_dists_vec[l].first;
    percept_right_line_id_ = vehicle_to_percept_line_dists_vec[r].first;
  }
}

adLocStatus_t MatcherMultiSemanticGrid::CalculateMapLineTypeCentorid(
    const std::shared_ptr<MapManager>& map_manager) {
  const auto& map_lanelines = map_manager->GetLocalLaneLines();
  if (map_lanelines.empty()) {
    LC_LDEBUG(MATCHING)
        << "calculate map linetype centroid local map laneline empty";
    return LOC_LOCALIZATION_ERROR;
  }
  for (const auto& item : map_lanelines) {
    const auto& line_segment = item.second->GetLineSegments().front();
    const auto& points = line_segment->GetPoints();
    if (points.size() < 2) continue;

    LineTypeCentroid line_type_centroid;
    if (IsRoadSideLine(line_segment->GetLineType())) {
      line_type_centroid.line_type = MatchSemanticType::RoadSideLine;
    } else if (IsDashedLine(line_segment->GetLineStyle())) {
      line_type_centroid.line_type = MatchSemanticType::DashedLine;
    } else {
      line_type_centroid.line_type = MatchSemanticType::SolidLine;
    }

    Point2D_t line_centroid(0, 0);
    double w_sum = 0;
    for (int i = 0; i < points.size(); i++) {
      if (points[i].x < -10. || points[i].x > 20.) continue;
      line_centroid.x += points[i].x;
      line_centroid.y += points[i].y;
      w_sum++;
    }
    if (w_sum == 0) continue;
    line_centroid.x /= w_sum;
    line_centroid.y /= w_sum;
    line_type_centroid.line_centroid = line_centroid;
    map_line_type_centroids_.emplace_back(item.first, line_type_centroid);
  }
  int n = map_line_type_centroids_.size();
  if (n == 0) {
    LC_LDEBUG(MATCHING) << "calculate map linetype centroid no valid lines";
    return LOC_LOCALIZATION_ERROR;
  }

  std::sort(map_line_type_centroids_.begin(), map_line_type_centroids_.end(),
            [](const auto& a, const auto& b) {
              return a.second.line_centroid.y < b.second.line_centroid.y;
            });

  // remove too near map lines to reduce calculation time
  std::vector<std::pair<id_t, LineTypeCentroid>> map_line_type_centroids;
  map_line_type_centroids.reserve(n);

  int i = 0, j = 1;
  map_line_type_centroids.emplace_back(map_line_type_centroids_[0]);
  while (j < n) {
    while (j < n &&
           map_line_type_centroids_[j].second.line_centroid.y -
                   map_line_type_centroids_[i].second.line_centroid.y <
               1.0 &&
           map_line_type_centroids_[j].second.line_type ==
               map_line_type_centroids_[i].second.line_type) {
      j++;
    }
    if (j < n) {
      map_line_type_centroids.emplace_back(map_line_type_centroids_[j]);
      i = j;
      j++;
    }
  }
  map_line_type_centroids_ = map_line_type_centroids;
  return LOC_SUCCESS;
}

void MatcherMultiSemanticGrid::ConstructPerceptionMapDistDB(
    double optimize_heading) {
  for (const auto& match_pairs : possible_match_pairs_vec_) {
    if (match_pairs.empty()) continue;
    for (const auto& match_pair : match_pairs) {
      const auto& percept_id = match_pair.first;
      const auto& map_id = match_pair.second;
      if (!downsampled_percept_points_in_bv_.count(percept_id)) {
        LC_LDEBUG(debug) << "Can not find percept id: " << percept_id
                         << " in construct percept map dist db!, impossible!";
        continue;
      }
      if (!map_poly_line_params_.count(map_id)) {
        LC_LDEBUG(debug) << "Can not find map id: " << map_id
                         << " in construct percept map dist db!, impossible!";
        continue;
      }
      const auto& percept_points =
          downsampled_percept_points_in_bv_.at(percept_id);
      const auto& map_line_param = map_poly_line_params_.at(map_id).line_param;
      double percept2map_dist =
          percept_map_dist_db_->GetDistbtwPerceptPointsMapLine(
              percept_id, percept_points, map_id, map_line_param,
              optimize_heading);
      if (percept2map_dist == 1e9) {
        LC_LDEBUG(debug) << "percept_id: " << percept_id
                         << ", to map_id: " << map_id
                         << " dist is 1e9, impossible!";
      }
    }
  }
}

void MatcherMultiSemanticGrid::RemoveImpossibleMatchPairsWithLateralInformation(
    double heading, double search_y, double search_width,
    std::vector<MatchPairVec>* possible_match_pairs_vec) {
  auto CheckMatchPairValid = [](double min_dist, double max_dist) {
    if (min_dist * max_dist <= 0) {
      return true;
    } else {
      double match_threshold = 0.6;
      return std::min(std::fabs(min_dist), std::fabs(max_dist)) <
             match_threshold;
    }
  };

  possible_match_pairs_vec->clear();
  for (const auto& match_pairs : possible_match_pairs_vec_) {
    MatchPairVec final_possible_match_pairs;
    for (const auto& match_pair : match_pairs) {
      const auto& percept_id = match_pair.first;
      const auto& map_id = match_pair.second;
      if (!downsampled_percept_points_in_bv_.count(percept_id)) {
        LC_LDEBUG(debug) << "Can not find percept id: " << percept_id
                         << " in remove impossible matchpairs with "
                            "lateral information!, impossible!";
        continue;
      }
      if (!map_poly_line_params_.count(map_id)) {
        LC_LDEBUG(debug) << "Can not find map id: " << map_id
                         << " in remove impossible matchpairs with "
                            "lateral information!, impossible!";
        continue;
      }
      const auto& percept_points =
          downsampled_percept_points_in_bv_.at(percept_id);
      const auto& map_line_param = map_poly_line_params_.at(map_id).line_param;
      double min_dist = percept_map_dist_db_->GetDistbtwPerceptPointsMapLine(
          percept_id, percept_points, map_id, map_line_param, heading,
          search_y);
      double max_dist = min_dist + search_width;
      if (!CheckMatchPairValid(min_dist, max_dist)) {
        continue;
      }
      final_possible_match_pairs.emplace_back(match_pair);
    }
    if (!final_possible_match_pairs.empty()) {
      possible_match_pairs_vec->emplace_back(final_possible_match_pairs);
    }
  }
}

void MatcherMultiSemanticGrid::GetFinalPossibleMatchPairsAtSearchY(
    double heading, double search_y,
    const std::vector<Eigen::Vector4d>& map_line_params,
    std::vector<MatchPairVec>* final_possible_match_pairs_vec, int* last_left,
    int* last_right) {
  auto pair = BinarySearchTwoNearestMapLinesId(search_y, map_line_params);
  int left = pair.first, right = pair.second;
  if (left == *last_left && right == *last_right) {
    *final_possible_match_pairs_vec = last_possible_match_pairs_vec_;
  } else {
    int param_size = 4;
    double search_width =
        right != -1 ? map_line_params[right][param_size - 1] - search_y : 4.0;
    RemoveImpossibleMatchPairsWithLateralInformation(
        heading, search_y, search_width, final_possible_match_pairs_vec);
  }
  *last_left = left;
  *last_right = right;
  last_possible_match_pairs_vec_ = *final_possible_match_pairs_vec;
}

adLocStatus_t MatcherMultiSemanticGrid::ScoreAllPoses() {
  if (search_ys_.size() != heading_idx_for_each_y_.size()) {
    LC_LDEBUG(MATCHING) << "heading idx and search position doesn't match!";
    return LOC_LOCALIZATION_ERROR;
  }
  if (heading_samples_.size() != multi_heading_percept_points_.size()) {
    LC_LDEBUG(MATCHING)
        << "heading sample and rotated percept points doesn't match!";
    return LOC_LOCALIZATION_ERROR;
  }

  lateral_scores_measurement_.clear();
  double min_score = std::numeric_limits<double>::max();
  for (size_t i = 0; i < search_ys_.size(); ++i) {
    double y = search_ys_[i];
    int h_idx = heading_idx_for_each_y_[i];
    double heading = heading_samples_[h_idx];
    double score = ScorePose(y, h_idx);

    PoseScore pose_score;
    pose_score.y = y;
    pose_score.heading_idx = h_idx;
    pose_score.heading = heading;
    pose_score.score = score;
    lateral_scores_measurement_.emplace_back(pose_score);
    min_score = std::min(min_score, score);
  }

  // normalization
  double total_score = 0;
  for (auto& item : lateral_scores_measurement_) {
    item.score -= min_score;  // remove negative
    total_score += item.score;
  }
  if (total_score < 1e-8) {
    LC_LDEBUG(MATCHING) << "no valid score";
    return LOC_LOCALIZATION_ERROR;
  }
  for (auto& item : lateral_scores_measurement_) {
    item.score /= total_score;
  }
  return LOC_SUCCESS;
}

void MatcherMultiSemanticGrid::GetPossibleMatchPairsWithLineType() {
  possible_match_pairs_vec_.clear();
  MatchPairVec possible_match_pairs;
  for (const auto& percept_item : percept_line_types_) {
    const auto& percept_id = percept_item.first;
    const auto& percept_line_type = percept_item.second;
    for (const auto& map_item : map_line_type_centroids_) {
      const auto& map_id = map_item.first;
      const auto& map_line_type = map_item.second.line_type;
      if (percept_line_type != map_line_type) continue;
      possible_match_pairs.emplace_back(percept_id, map_id);
    }
  }
  possible_match_pairs_vec_.emplace_back(possible_match_pairs);
}

adLocStatus_t MatcherMultiSemanticGrid::ScoreAllPosesNew(
    const std::shared_ptr<FramePackage>& frame_package) {
  if (search_ys_.size() != heading_idx_for_each_y_.size()) {
    LC_LDEBUG(MATCHING) << "heading idx and search position doesn't match!";
    return LOC_LOCALIZATION_ERROR;
  }
  lateral_scores_measurement_.clear();

  const auto& map_line_params = frame_package->GetMapLinePolyfitParam();
  double sigma = step_size_;
  int last_left = -1, last_right = -1;
  for (int i = 0; i < search_ys_.size(); ++i) {
    double search_y = search_ys_[i];
    int h_idx = heading_idx_for_each_y_[i];
    double optimize_heading = heading_samples_[h_idx];

    // get final possible match_pairs vec at search y
    std::vector<MatchPairVec> final_possible_match_pairs_vec;
    GetFinalPossibleMatchPairsAtSearchY(
        optimize_heading, search_y, map_line_params,
        &final_possible_match_pairs_vec, &last_left, &last_right);

    // calculate score without one-to-one match relationship
    double score = 0;
    for (int i = 0; i < final_possible_match_pairs_vec.size(); i++) {
      const auto& match_pairs = final_possible_match_pairs_vec[i];
      if (match_pairs.empty()) continue;
      for (const auto& match_pair : match_pairs) {
        const auto& percept_id = match_pair.first;
        const auto& map_id = match_pair.second;
        if (!downsampled_percept_points_in_bv_.count(percept_id)) {
          LC_LDEBUG(debug) << "Can not find percept id: " << percept_id
                           << " in score all poses!, impossible!";
          continue;
        }
        if (!map_poly_line_params_.count(map_id)) {
          LC_LDEBUG(debug) << "Can not find map id: " << map_id
                           << " in score all poses!, impossible!";
          continue;
        }
        const auto& percept_points =
            downsampled_percept_points_in_bv_.at(percept_id);
        const auto& map_line_param =
            map_poly_line_params_.at(map_id).line_param;
        double dist = percept_map_dist_db_->GetDistbtwPerceptPointsMapLine(
            percept_id, percept_points, map_id, map_line_param,
            optimize_heading, search_y);
        score += std::exp(-(dist * dist) / (sigma * sigma));
      }
    }

    PoseScore pose_score;
    pose_score.y = search_y;
    pose_score.heading_idx = h_idx;
    pose_score.heading = optimize_heading;
    pose_score.score = score;
    lateral_scores_measurement_.emplace_back(pose_score);
  }

  double total_score = 0;
  for (auto& item : lateral_scores_measurement_) {
    total_score += item.score;
  }
  if (total_score < 1e-8) {
    LC_LDEBUG(MATCHING) << "likelihood distri is invalid!, and histogram "
                           "filter needs to be reset!";
    lateral_scores_.clear();
    return LOC_LOCALIZATION_ERROR;
  }
  for (auto& item : lateral_scores_measurement_) {
    item.score /= total_score;
  }
  return LOC_SUCCESS;
}

double MatcherMultiSemanticGrid::ScorePose(
    double y, int heading_idx,
    std::unordered_map<id_t, std::vector<id_t>>* hit_info) {
  if (heading_idx < 0 || heading_idx >= multi_heading_percept_points_.size()) {
    LC_LDEBUG(MATCHING) << "why beyond heading idx?";
    return 0.0;
  }

  static auto DTScore = [](double dist, double sigma) {
    return std::exp(-0.5 * dist * dist / (sigma * sigma));
  };

  double score = 0.0;
  std::unordered_map<id_t, std::vector<id_t>> hit_info_record;
  const auto& percept_points = multi_heading_percept_points_[heading_idx];
  for (const auto& percept_elem : percept_points) {
    if (hit_info != nullptr)
      hit_info_record[percept_elem.first].reserve(percept_elem.second.size());

    double score_for_element = 0;
    // iterate through all points of a single peception element
    for (const auto& pt : percept_elem.second) {
      Point2D_t percept_pt(pt.point.x, pt.point.y + y);
      MatchSemanticType percept_type = pt.type;
      id_t hit_map_id = -1;  // -1 is not map id
      double score_for_pt = 0;

      // point level scoring
      if (IsLaneLine(percept_type)) {
        auto hit_index =
            grid_idx_converter_->PointInnerToVoxelIndex(percept_pt);
        auto local_idx_pattern = LocalIndexPattern(hit_index, percept_type);
        for (const auto& grid_idx : local_idx_pattern) {
          auto grid_iter = map_grids_.find(grid_idx);
          if (grid_iter == map_grids_.end()) continue;  // no grid hit
          const auto& map_elements = grid_iter->second.map_elements;
          auto grid_type_iter = map_elements.find(percept_type);
          if (grid_type_iter == map_elements.end())
            continue;  // no same type find
          if (grid_type_iter->second.empty()) continue;

          const auto& map_pt = grid_type_iter->second.front().point;
          double dt_score =
              DTScore(percept_pt.y - map_pt.y, MatchParam::grid_size);
          // TODO(fy): weights are not used yet, as proning to overfit
          // to single line! need better cov/weight estimation
          score_for_pt = dt_score * 1.0;
          hit_map_id = grid_type_iter->second.front().id;
          break;
        }
      } else if (percept_type == MatchSemanticType::Sign) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& map_pt : map_sign_points_) {
          double p2p_dist = (map_pt.point - percept_pt).Norm();
          if (p2p_dist < MatchParam::p2m_sign_thres && p2p_dist < min_dist) {
            min_dist = p2p_dist;
            hit_map_id = map_pt.id;
          }
        }
        if (hit_map_id != -1) {
          double dt_score = DTScore(min_dist, MatchParam::p2m_sign_thres);
          // reduce weight of sign
          score_for_pt = 0.3 * dt_score * pt.observ_cnt;
        }
      } else if (percept_type == MatchSemanticType::Pole) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& map_pt : map_pole_points_) {
          double p2p_dist = (map_pt.point - percept_pt).Norm();
          if (p2p_dist < MatchParam::p2m_pole_thres && p2p_dist < min_dist) {
            min_dist = p2p_dist;
            hit_map_id = map_pt.id;
          }
        }
        if (hit_map_id != -1) {
          double dt_score = DTScore(min_dist, MatchParam::p2m_pole_thres);
          // increase weight of pole
          score_for_pt = 1.5 * dt_score * pt.observ_cnt;
        }
      }

      score_for_element += score_for_pt;
      if (hit_info != nullptr)
        hit_info_record[percept_elem.first].emplace_back(hit_map_id);
    }
    // TODO(fy): how to normalize for each element/semantic?
    score += score_for_element;
  }

  if (hit_info != nullptr) *hit_info = std::move(hit_info_record);
  return score;
}

std::vector<VoxelIndex> MatcherMultiSemanticGrid::LocalIndexPattern(
    const VoxelIndex& hit_index, MatchSemanticType type) {
  std::vector<VoxelIndex> pattern;
  if (IsLaneLine(type)) {
    int lat_extend = MatchParam::lat_dt_extend;
    pattern.reserve(2 * lat_extend + 1);
    pattern.emplace_back(hit_index);
    for (int j = -lat_extend; j <= lat_extend; ++j) {
      if (j == 0) continue;
      VoxelIndex index(hit_index.x_idx, hit_index.y_idx + j);
      pattern.emplace_back(index);
    }
  } else {
    int long_extend = 2, lat_extend = 1;
    pattern.reserve((2 * lat_extend + 1) * (2 * long_extend + 1));
    for (int i = -long_extend; i <= long_extend; ++i) {
      for (int j = -lat_extend; j <= lat_extend; ++j) {
        VoxelIndex index(hit_index.x_idx + i, hit_index.y_idx + j);
        pattern.emplace_back(index);
      }
    }
  }
  return pattern;
}

adLocStatus_t MatcherMultiSemanticGrid::FilterPrediction(
    const std::shared_ptr<FramePackage>& frame_package) {
  // check if need reset filter
  auto main_frame = frame_package->GetFrames().front();
  OdomState curr_odom_state = main_frame->GetOdomState();
  NavState curr_nav_state = main_frame->GetNavState();
  uint64_t max_delta_timestamp = 5e8;  // 500ms

  bool nav_status_invalid =
      curr_nav_state.timestamp == 0 || nav_state_.timestamp == 0;
  bool dr_timestamp_disorder =
      curr_odom_state.timestamp <= odom_state_.timestamp;
  bool dr_timestamp_large_gap =
      curr_odom_state.timestamp - odom_state_.timestamp > max_delta_timestamp;

  if (lateral_scores_.empty() || nav_status_invalid || dr_timestamp_disorder ||
      dr_timestamp_large_gap) {
    std::string reason;
    if (lateral_scores_.empty()) reason = "posterior distri empty | ";
    if (nav_status_invalid) reason += " nav_status_invalid | ";
    if (dr_timestamp_disorder) reason += " dr_timestamp_disorder";
    if (dr_timestamp_large_gap)
      reason += " dr_timestamp_large_gap " +
                std::to_string(curr_odom_state.timestamp) + " " +
                std::to_string(odom_state_.timestamp);
    LC_LDEBUG(MATCHING) << "reset histogram filter! reason: " << reason;
    odom_state_ = curr_odom_state;
    nav_state_ = curr_nav_state;
    has_prediction_ = false;
    lateral_scores_predict_.clear();
    return LOC_SUCCESS;
  }

  // grid center prediction
  SE3d delta_nav_state = nav_state_.pose.inverse() * curr_nav_state.pose;
  SE3d delta_odom_state = odom_state_.pose.inverse() * curr_odom_state.pose;
  SE3d delta_pose = delta_nav_state.inverse() * delta_odom_state;
  double y_delta = delta_pose.translation()(1);
  double heading_delta =
      delta_pose.so3().unit_quaternion().toRotationMatrix().eulerAngles(2, 1,
                                                                        0)(0);

  lateral_scores_predict_ = std::move(lateral_scores_);
  for (auto& item : lateral_scores_predict_) {
    double heading_prior = item.heading + heading_delta;
    double y_prior = std::cos(item.heading) * y_delta + item.y;
    item.heading = heading_prior;
    item.y = y_prior;
  }

  // align to current search grids
  int max_step_num = search_ys_.size();
  std::vector<int> search_y_idx(max_step_num, -1);
  for (auto i = 0; i < lateral_scores_predict_.size(); ++i) {
    // quantize lateral position
    auto& item = lateral_scores_predict_[i];
    int step = std::round(item.y / step_size_);
    item.y = step * step_size_;
    // buffer index
    int y_idx = lat_steps_ + step;
    if (y_idx < 0 || y_idx >= max_step_num) continue;
    search_y_idx[y_idx] = i;
  }

  std::vector<PoseScore> temp_lat_scores;
  temp_lat_scores.resize(max_step_num);
  for (int i = 0; i < max_step_num; ++i) {
    PoseScore predict_pose_socre;
    if (search_y_idx[i] == -1) {
      predict_pose_socre.score = 0;
      predict_pose_socre.y = (i - lat_steps_) * step_size_;
      predict_pose_socre.heading = 0;  // not used
    } else {
      predict_pose_socre = lateral_scores_predict_[search_y_idx[i]];
    }
    temp_lat_scores[i] = predict_pose_socre;
  }
  lateral_scores_predict_ = temp_lat_scores;

  // grid uncertainty prediction, consider only lateral dimension
  // assume that uncertainty correlates with odom distance
  double sigma = 0.01 * delta_odom_state.translation().norm();
  double lower_bound = MatchParam::min_hf_predict_sigma;
  sigma = std::max(sigma, lower_bound);  // limit lower bound
  double sigma2 = sigma * sigma;
  double total_score = 0;
  for (int i = 0; i < max_step_num; ++i) {
    double pre_score = 0;
    const auto& item = temp_lat_scores[i];
    for (int j = 0; j < max_step_num; ++j) {
      const auto& item_other = temp_lat_scores[j];
      double dist = item.y - item_other.y;
      double exp = std::exp(-0.5 * dist * dist / sigma2);
      pre_score += exp * item_other.score;
    }
    lateral_scores_predict_[i].score = pre_score;
    total_score += pre_score;
  }

  // normalization
  if (total_score < 1e-8) {
    LC_LDEBUG(MATCHING) << "reset histogram filter! reason: "
                        << "no vaild predict score";
    odom_state_ = curr_odom_state;
    nav_state_ = curr_nav_state;
    has_prediction_ = false;
    lateral_scores_predict_.clear();
    return LOC_SUCCESS;
  }
  for (auto& item : lateral_scores_predict_) {
    item.score /= total_score;
  }

  odom_state_ = curr_odom_state;
  nav_state_ = curr_nav_state;
  has_prediction_ = true;
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::FilterUpdate() {
  static auto KLDivergence = [](const std::vector<PoseScore>& p,
                                const std::vector<PoseScore>& q,
                                double* kl_divergence) {
    if (p.size() != q.size()) return false;
    double result = 0;
    for (size_t i = 0; i < p.size(); ++i) {
      double pi = p[i].score, qi = q[i].score;
      if (pi != 0) result += pi * std::log(pi / qi);
    }
    *kl_divergence = std::max(0.0, result);
    return true;
  };

  if (!has_prediction_) {
    LC_LDEBUG(MATCHING)
        << "no prediction, using measurement as posterior directly!";
    lateral_scores_ = lateral_scores_measurement_;
    return LOC_SUCCESS;
  }

  // evaluate kl divergence btw meansure and predict distribution
  double kl_divergence = 0.0;
  if (!KLDivergence(lateral_scores_measurement_, lateral_scores_predict_,
                    &kl_divergence)) {
    LC_LDEBUG(MATCHING)
        << "get kl divergence btw meansurement and prediction failed!";
    return LOC_LOCALIZATION_ERROR;
  }
  LC_LDEBUG(MATCHING) << "kl_divergence " << kl_divergence;
  if (kl_divergence < 1e-8) {
    LC_LDEBUG(MATCHING) << "kl divergence close to 0, using measurement as "
                           "posterior directly!";
    lateral_scores_ = lateral_scores_measurement_;
    return LOC_SUCCESS;
  }

  lateral_scores_ = lateral_scores_measurement_;
  // add bias to reduce prediction weight
  double kl_inv = 1.0 / (kl_divergence + MatchParam::kl_divergence_bias);
  double total_score = 0.0;
  for (size_t i = 0; i < lateral_scores_measurement_.size(); ++i) {
    double p_measure = lateral_scores_measurement_[i].score;
    double p_predict = lateral_scores_predict_[i].score;
    double p_update = p_measure * std::pow(p_predict, kl_inv);
    lateral_scores_[i].score = p_update;
    total_score += p_update;

    if (MatchParam::enable_hf_heading_fusion) {
      double yaw_meas = lateral_scores_measurement_[i].heading;
      double yaw_pred = lateral_scores_predict_[i].heading;
      lateral_scores_[i].heading =
          (p_measure * yaw_meas + p_predict * yaw_pred) /
          (p_measure + p_predict);
    }
  }
  // normalization
  if (total_score < 1e-8) {
    LC_LDEBUG(MATCHING) << "no vaild score";
    return LOC_LOCALIZATION_ERROR;
  }
  for (auto& item : lateral_scores_) {
    item.score /= total_score;
  }
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::FilterUpdateNew() {
  static auto KLDivergence = [](const std::vector<PoseScore>& p,
                                const std::vector<PoseScore>& q,
                                double* kl_divergence) {
    if (p.size() != q.size()) return false;
    double prob_sum_p = 0, prob_sum_q = 0;
    for (int i = 0; i < p.size(); ++i) {
      prob_sum_p += p[i].score;
      prob_sum_q += q[i].score;
    }
    if (prob_sum_p < 1e-8 || prob_sum_q < 1e-8) return false;
    double result = 0;
    for (int i = 0; i < p.size(); ++i) {
      double pi = p[i].score, qi = q[i].score;
      double diff = std::log((pi + 0.01) / (qi + 0.01));
      diff = diff > 0 ? diff : -diff;
      result += pi * diff;
    }
    result /= prob_sum_p;
    result += 1.0;
    *kl_divergence = result;
    return true;
  };

  if (!has_prediction_) {
    LC_LDEBUG(MATCHING)
        << "no prediction, using measurement as posterior directly!";
    lateral_scores_ = lateral_scores_measurement_;
    return LOC_SUCCESS;
  }

  double kl_divergence = 0.0;
  if (!KLDivergence(lateral_scores_measurement_, lateral_scores_predict_,
                    &kl_divergence)) {
    LC_LDEBUG(MATCHING) << "get kl divergence btw likelihood and "
                           "prediction failed, and histogram filter needs "
                           "to be reset!";
    lateral_scores_.clear();
    return LOC_LOCALIZATION_ERROR;
  }
  LC_LDEBUG(MATCHING) << "kl_divergence " << kl_divergence;

  lateral_scores_ = lateral_scores_measurement_;
  double kl_inv = 1.0 / kl_divergence;
  double total_score = 0.0;
  for (size_t i = 0; i < lateral_scores_measurement_.size(); ++i) {
    double p_measure = lateral_scores_measurement_[i].score;
    double p_predict = lateral_scores_predict_[i].score;
    double p_update = p_measure * std::pow(p_predict, kl_inv);
    lateral_scores_[i].score = p_update;
    total_score += p_update;
  }
  // normalization
  if (total_score < 1e-8) {
    LC_LDEBUG(MATCHING) << "posterior distri is invalid, and histogram "
                           "filter needs to be reset!";
    lateral_scores_.clear();
    return LOC_LOCALIZATION_ERROR;
  }
  for (auto& item : lateral_scores_) {
    item.score /= total_score;
  }
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::FilterConvergenceCheck() {
  // detect peak in posterior distribution
  auto status = HistogramPeakDetection();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "histogram peak detection failed!";
    converge_travel_dist_ = 0.0;
    odom_state_last_converge_.timestamp = 0;
    return status;
  }
  std::sort(
      lateral_peaks_.begin(), lateral_peaks_.end(),
      [](const PoseScore& a, const PoseScore& b) { return a.score > b.score; });
  d_lateral_ = lateral_peaks_.front().y;
  d_heading_ = lateral_peaks_.front().heading;

  // check if filter converged to single peak
  if (lateral_peaks_.size() > 1) {
    LC_LDEBUG(MATCHING) << "too many peaks, ambiguous matching possible!";
    converge_travel_dist_ = 0.0;
    odom_state_last_converge_.timestamp = 0;
    return LOC_LOCALIZATION_ERROR;
  }

  // check converge timegap
  uint64_t timegap =
      odom_state_.timestamp - odom_state_last_converge_.timestamp;
  if (timegap >= MatchParam::max_converge_timegap) {
    converge_travel_dist_ = 0.0;
    odom_state_last_converge_ = odom_state_;
    LC_LDEBUG(MATCHING) << "start to accumulate converge distance...";
    return LOC_LOCALIZATION_ERROR;
  }

  // check error state
  if (!lateral_scores_predict_.empty()) {
    PoseScore max_prior;
    for (size_t i = 0; i < lateral_scores_predict_.size(); ++i) {
      if (lateral_scores_predict_[i].score > max_prior.score)
        max_prior = lateral_scores_predict_[i];
    }
    double err_state_lateral = d_lateral_ - max_prior.y;
    double err_state_heading = d_heading_ - max_prior.heading;
    LC_LDEBUG(MATCHING) << "histogram filter error state lateral/heading(deg): "
                        << err_state_lateral << "|"
                        << err_state_heading * 180.0 / M_PI;
    if (std::fabs(err_state_lateral) > 0.6) {
      LC_LDEBUG(MATCHING) << "large error state, filter is diverged!";
      converge_travel_dist_ = 0.0;
      odom_state_last_converge_.timestamp = 0;
      return LOC_LOCALIZATION_ERROR;
    }
  }

  // check converge distance
  SE3d delta_pose = odom_state_last_converge_.pose.inverse() * odom_state_.pose;
  converge_travel_dist_ += delta_pose.translation().norm();
  if (converge_travel_dist_ < MatchParam::min_converge_dist) {
    odom_state_last_converge_ = odom_state_;
    LC_LDEBUG(MATCHING) << "converge dist " << converge_travel_dist_
                        << ", filter converging...";
    return LOC_LOCALIZATION_ERROR;
  }

  LC_LDEBUG(MATCHING) << "converge dist " << converge_travel_dist_
                      << ", filter converged!";
  odom_state_last_converge_ = odom_state_;
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::HistogramPeakDetection() {
  // calculate sample score threshold
  if (lateral_scores_.empty()) return LOC_INVALID;
  std::vector<PoseScore> lateral_scores_tmp = lateral_scores_;
  std::sort(
      lateral_scores_tmp.begin(), lateral_scores_tmp.end(),
      [](const PoseScore& a, const PoseScore& b) { return a.score > b.score; });
  double score_thres = 0, max_gap = 0;
  size_t n = lateral_scores_tmp.size();
  for (size_t i = 1; i < n; ++i) {
    double score_prev = lateral_scores_tmp[i - 1].score;
    double score_curr = lateral_scores_tmp[i].score;
    double gap = score_prev - score_curr;
    if (gap > max_gap) {
      max_gap = gap;
      score_thres = score_curr;
    }
  }
  double score_median = n % 2 == 0 ? 0.5 * (lateral_scores_tmp[n / 2].score +
                                            lateral_scores_tmp[n / 2 - 1].score)
                                   : lateral_scores_tmp[n / 2].score;
  double score_max = lateral_scores_tmp.front().score;
  double upper_bound = score_max * MatchParam::max_peak_scale;
  score_thres = std::max(score_thres, score_median);  // limit lower bound
  score_thres = std::min(score_thres, upper_bound);   // limit upper bound

  // thresholding good samples
  thresholded_lateral_scores_.clear();
  for (const auto& item : lateral_scores_tmp) {
    if (item.score > score_thres)
      thresholded_lateral_scores_.emplace_back(item);
  }
  if (thresholded_lateral_scores_.empty()) return LOC_LOCALIZATION_ERROR;

  // clustering according to lateral position
  lateral_peaks_.clear();
  std::sort(thresholded_lateral_scores_.begin(),
            thresholded_lateral_scores_.end(),
            [](const PoseScore& a, const PoseScore& b) { return a.y > b.y; });
  std::vector<std::pair<int, int>> cluster_ids;
  int split_i = 0;
  for (auto j = 1; j < thresholded_lateral_scores_.size(); ++j) {
    double delta_y =
        thresholded_lateral_scores_[j - 1].y - thresholded_lateral_scores_[j].y;
    if (delta_y < MatchParam::grid_size / MatchParam::search_scale * 1.5)
      continue;
    cluster_ids.emplace_back(split_i, j - 1);
    split_i = j;
  }
  cluster_ids.emplace_back(split_i, thresholded_lateral_scores_.size() - 1);
  // get center of each cluster
  for (const auto& pair : cluster_ids) {
    if (pair.first > pair.second) continue;
    double cluster_width = thresholded_lateral_scores_[pair.first].y -
                           thresholded_lateral_scores_[pair.second].y;
    // cluster too wide
    if (cluster_width > MatchParam::max_cluster_width) continue;

    double y_mean = 0, score_sum = 0, best_score = 0;
    int best_heading_idx = 0;
    for (int i = pair.first; i <= pair.second; ++i) {
      double score = thresholded_lateral_scores_[i].score;
      if (score > best_score) {
        best_score = score;
        best_heading_idx = thresholded_lateral_scores_[i].heading_idx;
      }
      y_mean += score * thresholded_lateral_scores_[i].y;
      score_sum += score;
    }
    y_mean /= score_sum;

    PoseScore pose_score;
    pose_score.y = y_mean;
    pose_score.heading_idx = best_heading_idx;
    pose_score.heading = heading_samples_[best_heading_idx];
    pose_score.score = best_score;
    lateral_peaks_.emplace_back(pose_score);
  }
  if (lateral_peaks_.empty()) return LOC_LOCALIZATION_ERROR;
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::GetMatchSamples() {
  // transform histogram peaks into match results
  match_samples_.clear();
  for (const auto& sample : lateral_peaks_) {
    MatchScore match_sample;
    match_sample.match = std::make_shared<MatchIndex>();
    match_sample.pose_sample = sample;
    // get match info
    int hit_pt_cnt = 0, all_pt_cnt = 0;
    std::unordered_map<id_t, std::vector<id_t>> hit_info;
    ScorePose(match_sample.pose_sample.y, match_sample.pose_sample.heading_idx,
              &hit_info);
    if (hit_info.empty()) return LOC_LOCALIZATION_ERROR;

    // convert hit info to match index
    for (const auto& item : hit_info) {
      id_t percept_id = item.first;
      if (item.second.empty()) continue;
      all_pt_cnt += item.second.size();

      if (!percept_points_in_bv_.count(percept_id)) continue;
      MatchSemanticType match_type =
          percept_points_in_bv_.at(percept_id).front().type;
      SemanticType semantic_type = MatchTypeToSemanticType(match_type);

      // get match map ids, 1-to-n possible
      std::unordered_set<id_t> map_ids;  // remove redundant
      for (const auto& id : item.second) {
        if (id == -1) continue;
        ++hit_pt_cnt;
        map_ids.insert(id);
      }
      for (const auto& map_id : map_ids) {
        match_sample.match->AddSemanticMatch(
            semantic_type, std::make_pair(percept_id, map_id));
      }
    }
    if (all_pt_cnt == 0) {
      LC_LDEBUG(MATCHING) << "why sampled percept points empty?";
      return LOC_LOCALIZATION_ERROR;
    }
    match_sample.hit_ratio =
        static_cast<double>(hit_pt_cnt) / static_cast<double>(all_pt_cnt);
    match_samples_.emplace_back(match_sample);
  }

  if (match_samples_.empty()) return LOC_LOCALIZATION_ERROR;
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::GetMatchSamplesNew() {
  match_samples_.clear();
  for (const auto& sample : lateral_peaks_) {
    MatchScore match_sample;
    match_sample.pose_sample = sample;
    match_sample.match = std::make_shared<MatchIndex>();

    // select the closest match pairs as best match
    Sophus::SE2<double> T_map_percept(sample.heading,
                                      Eigen::Vector2d(0, sample.y));
    int hit_pt_cnt = 0, all_pt_cnt = 0;
    std::unordered_set<id_t> matched_map_ids;
    for (const auto& percept_item : percept_line_types_) {
      // binary search for map id which matched percept id
      const auto& percept_id = percept_item.first;
      if (!downsampled_percept_points_in_bv_.count(percept_id)) {
        LC_LDEBUG(debug) << "Can not find percept id: " << percept_id
                         << " in Get Match Samples New!, impossible!";
        continue;
      }
      const auto& percept_points =
          downsampled_percept_points_in_bv_.at(percept_id);
      all_pt_cnt += percept_points.size();
      std::vector<std::pair<MatchPair, double>> percept_map_dist_vec;
      for (const auto& match_pairs : possible_match_pairs_vec_) {
        if (match_pairs.empty()) continue;
        for (const auto& match_pair : match_pairs) {
          if (match_pair.first != percept_id ||
              matched_map_ids.count(match_pair.second))
            continue;
          const auto map_id = match_pair.second;
          if (!map_poly_line_params_.count(map_id)) {
            LC_LDEBUG(debug) << "Can not find map id: " << map_id
                             << " in Get Match Samples New!, impossible!";
            continue;
          }
          const auto& map_line_param =
              map_poly_line_params_.at(map_id).line_param;
          double dist = percept_map_dist_db_->GetDistbtwPerceptPointsMapLine(
              percept_id, percept_points, map_id, map_line_param,
              sample.heading, sample.y);
          percept_map_dist_vec.emplace_back(match_pair, dist);
        }
      }
      if (percept_map_dist_vec.empty()) continue;

      id_t matched_map_id = -1;
      double matched_dist = std::numeric_limits<double>::max();
      for (const auto& item : percept_map_dist_vec) {
        if (std::fabs(item.second) < matched_dist) {
          matched_map_id = item.first.second;
          matched_dist = std::fabs(item.second);
        }
      }

      if (matched_dist <= step_size_) {
        // percept2map dist < search step, match success!
        match_sample.match->AddSemanticMatch(SemanticType::LaneLine,
                                             {percept_id, matched_map_id});
        matched_map_ids.insert(matched_map_id);

        const auto& map_line_param = map_poly_line_params_.at(matched_map_id);
        for (const auto& point : percept_points) {
          Point2D_t transformed_point = T_map_percept * point.point;
          double map_y = FourDegreePolyFunc(map_line_param.line_param,
                                            transformed_point.x);
          if (std::fabs(map_y - transformed_point.y) <= step_size_)
            hit_pt_cnt++;
        }
      }
    }

    double hit_ratio = static_cast<double>(hit_pt_cnt) / all_pt_cnt;
    match_sample.hit_ratio = hit_ratio;
    if (hit_ratio < 0.25) continue;
    match_samples_.emplace_back(match_sample);
  }

  if (match_samples_.empty()) return LOC_LOCALIZATION_ERROR;
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::EstimateRelocPoseCov() {
  auto CheckMatchSampleAmbiguity = [this](const MatchScore& match_sample) {
    // Check whether the matching line type is single
    std::unordered_map<MatchSemanticType, int, EnumClassHash> matched_type_nums;
    std::vector<double> vehicle_to_matched_percept_line_dists;
    for (const auto& item : match_sample.match->GetAllSemanticMatch()) {
      const auto& semantic_type = item.first;
      const auto& match_pairs = item.second;
      if (semantic_type != SemanticType::LaneLine) continue;
      for (const auto& match_pair : match_pairs) {
        const auto& percept_id = match_pair.first;
        if (!percept_line_types_.count(percept_id)) {
          LC_LDEBUG(MATCHING) << "Can not find percept id: " << percept_id
                              << "in estimate reloc pose cov, impossible!";
          continue;
        }
        const auto& match_semantic_type = percept_line_types_.at(percept_id);
        ++matched_type_nums[match_semantic_type];

        if (!vehicle_to_percept_line_dists_.count(percept_id)) {
          LC_LDEBUG(MATCHING) << "Can not find percept id: " << percept_id
                              << "in estimate reloc pose cov, impossible!";
          continue;
        }
        const double dist = vehicle_to_percept_line_dists_.at(percept_id);
        vehicle_to_matched_percept_line_dists.emplace_back(dist);
      }
    }

    int match_rs_num = matched_type_nums[MatchSemanticType::RoadSideLine];
    int match_sl_num = matched_type_nums[MatchSemanticType::SolidLine];
    int match_dl_num = matched_type_nums[MatchSemanticType::DashedLine];

    int match_line_style_num = 0;
    if (match_rs_num) ++match_line_style_num;
    if (match_sl_num) ++match_line_style_num;
    if (match_dl_num) ++match_line_style_num;
    if (match_line_style_num <= 1) {
      LC_LDEBUG(debug) << "smm reloc single match line type!";
      return true;
    }

    // Check whether all matching line are on the same side
    std::sort(vehicle_to_matched_percept_line_dists.begin(),
              vehicle_to_matched_percept_line_dists.end(),
              [](const auto& a, const auto& b) { return a < b; });
    if (vehicle_to_matched_percept_line_dists.front() > 0 ||
        vehicle_to_matched_percept_line_dists.back() < 0) {
      LC_LDEBUG(debug) << "smm reloc all match lines are on the same side!";
      return true;
    }
    return false;
  };

  auto status = HistogramPeakDetection();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "histogram peak detection failed!";
    converge_travel_dist_ = 0.0;
    odom_state_last_converge_.timestamp = 0;
    return status;
  }

  std::sort(
      lateral_peaks_.begin(), lateral_peaks_.end(),
      [](const PoseScore& a, const PoseScore& b) { return a.score > b.score; });

  status = GetMatchSamplesNew();
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(MATCHING) << "get match samples new failed";
    converge_travel_dist_ = 0.0;
    odom_state_last_converge_.timestamp = 0;
    return status;
  }

  // estimate reloc pose roughly
  const auto& match_sample = match_samples_.front();
  double y_init = match_sample.pose_sample.y;
  d_heading_ = match_sample.pose_sample.heading;
  d_lateral_ = SolveYByOptimPoint2LineDist(y_init);

  if (match_samples_.size() > 1) {
    LC_LDEBUG(MATCHING)
        << "smm reloc may have multiple match pairs, road-level cov";
    EstimateRoadLevelRelocCov();
    converge_travel_dist_ = 0.0;
    odom_state_last_converge_.timestamp = 0;
    return LOC_SUCCESS;
  }

  if (CheckMatchSampleAmbiguity(match_sample)) {
    LC_LDEBUG(MATCHING)
        << "smm reloc match sample is ambiguous, road-level cov";
    EstimateRoadLevelRelocCov();
    converge_travel_dist_ = 0.0;
    odom_state_last_converge_.timestamp = 0;
    return LOC_SUCCESS;
  }

  float64_t dt = (static_cast<float64_t>(odom_state_.timestamp) -
                  static_cast<float64_t>(odom_state_last_converge_.timestamp)) /
                 1e9;
  if (dt > 1.0) {
    LC_LDEBUG(MATCHING)
        << "smm reloc histogram filter begin to converge, road-level cov";
    EstimateRoadLevelRelocCov();
    converge_travel_dist_ = 0.0;
    odom_state_last_converge_ = odom_state_;
    return LOC_SUCCESS;
  }

  converge_travel_dist_ +=
      (odom_state_last_converge_.pose.inverse() * odom_state_.pose)
          .translation()
          .norm();
  odom_state_last_converge_ = odom_state_;
  if (converge_travel_dist_ < MatchParam::min_converge_dist) {
    LC_LDEBUG(MATCHING)
        << "smm reloc histogram filter single peak converge mile is "
        << converge_travel_dist_ << ", road-level cov";
    EstimateRoadLevelRelocCov();
  } else {
    LC_LDEBUG(MATCHING)
        << "smm reloc histogram filter single peak converge mile is "
        << converge_travel_dist_ << ", lane-level cov";
    lateral_std_ = lane_level_lateral_std_;
    heading_std_ = lane_level_heading_std_;
  }

  return LOC_SUCCESS;
}

void MatcherMultiSemanticGrid::EstimateRoadLevelRelocCov() {
  double min_lateral = d_lateral_, max_lateral = d_lateral_;
  for (auto match : match_samples_) {
    auto sample = match.pose_sample;
    max_lateral = std::max(sample.y, max_lateral);
    min_lateral = std::min(sample.y, min_lateral);
  }

  map_road_heading_ = std::max(map_road_heading_, 1.0 * M_PI / 180.0);
  map_road_heading_ =
      std::min(map_road_heading_, 2.0 * MatchParam::max_map_heading);

  map_road_width_ = std::max(map_road_width_, 10.0);
  map_road_width_ = std::min(map_road_width_, 50.0);

  double factor_lat = std::fabs(max_lateral - min_lateral) /
                      (2.0 * MatchParam::max_search_lateral);
  lateral_std_ =
      0.5 * (1.0 + factor_lat) * map_road_width_ + 0.3 * std::fabs(d_lateral_);
  heading_std_ = map_road_heading_ + 0.3 * std::fabs(d_heading_);
}

adLocStatus_t MatcherMultiSemanticGrid::DistanceConsistencyCheck(
    const std::shared_ptr<TrackingManager>& tracking_manager,
    const std::shared_ptr<MapManager>& map_manager) {
  // get only involved percept/map ids
  std::unordered_map<id_t, SemanticType> map_ids_involved;
  std::unordered_map<id_t, SemanticType> percept_ids_involved;
  for (const auto& sample : match_samples_) {
    auto match_pairs = sample.match->GetAllSemanticMatch();
    for (const auto& item : match_pairs) {
      SemanticType semantic_type = item.first;
      for (const auto& pair : item.second) {
        percept_ids_involved.insert({pair.first, semantic_type});
        map_ids_involved.insert({pair.second, semantic_type});
      }
    }
  }

  // score all matches
  std::unordered_map<SemanticType, double, EnumClassHash> sigma_p2m;
  sigma_p2m[SemanticType::LaneLine] = MatchParam::p2m_sigma_line;
  sigma_p2m[SemanticType::TrafficSign] = MatchParam::p2m_sigma_sign;
  sigma_p2m[SemanticType::Pole] = MatchParam::p2m_sigma_pole;

  for (auto& match_sample : match_samples_) {
    MatchPairVec match_pair = match_sample.match->GetAllMatchInOneVec();
    double y = match_sample.pose_sample.y;
    double heading = match_sample.pose_sample.heading;
    // SE3d T_map_percept(SO3d(0, 0, heading),
    //                           Eigen::Vector3d(0, y, 0)); old Sophus
    SE3d T_map_percept(SO3d::exp(Eigen::Vector3d(0, 0, heading)),
                       Eigen::Vector3d(0, y, 0));

    // evaluate percept to map distance
    double p2m_score = 0, p2m_dist = 0;
    int p2m_cnt = 0;
    for (auto match_i : match_pair) {
      id_t p_i = match_i.first, m_i = match_i.second;
      SemanticType percept_type = percept_ids_involved.at(p_i);
      double dist = 0;
      if (!map_dist_db_->GetDistBetweenElements(m_i, p_i, percept_dist_db_,
                                                T_map_percept, &dist))
        continue;
      dist = std::fabs(dist);
      double sigma2 = std::pow(sigma_p2m.at(percept_type), 2.0);
      p2m_score += std::exp(-dist * dist / sigma2);
      p2m_dist += dist;
      ++p2m_cnt;
      LC_LDEBUG(MATCHING) << static_cast<int>(percept_type)
                          << " match pairs p_i->m_i " << p_i << " -> " << m_i
                          << ", dist " << dist;
    }
    if (p2m_cnt == 0) {
      p2m_dist = 1e10;
      p2m_score = 0;
    } else {
      p2m_dist /= p2m_cnt;
      p2m_score /= p2m_cnt;
    }

    // evaluate match to match consistency
    double consist_score = 0, consist_dist = 0;
    int consist_cnt = 0;
    for (size_t i = 0; i < match_pair.size(); ++i) {
      id_t p_i = match_pair[i].first, m_i = match_pair[i].second;
      if (!percept_ids_involved.count(p_i)) continue;
      SemanticType percept_type_i = percept_ids_involved.at(p_i);

      for (size_t j = i + 1; j < match_pair.size(); ++j) {
        id_t p_j = match_pair[j].first, m_j = match_pair[j].second;
        if (!percept_ids_involved.count(p_j)) continue;
        SemanticType percept_type_j = percept_ids_involved.at(p_j);

        double dist_p_ij = 0, dist_m_ij = 0;
        if (!percept_dist_db_->GetDistBetweenElements(p_i, p_j, &dist_p_ij))
          continue;
        if (!map_dist_db_->GetDistBetweenElements(m_i, m_j, &dist_m_ij))
          continue;
        double dist = std::fabs(dist_p_ij - dist_m_ij);
        double sigma2 = std::pow(sigma_p2m.at(percept_type_i), 2.0) +
                        std::pow(sigma_p2m.at(percept_type_j), 2.0);
        consist_score += std::exp(-dist * dist / sigma2);
        consist_dist += dist;
        ++consist_cnt;
        LC_LDEBUG(MATCHING) << "p_i " << p_i << " p_j " << p_j << ", dist_p_ij "
                            << dist_p_ij << " / m_i " << m_i << " m_j " << m_j
                            << ", dist_m_ij " << dist_m_ij;
      }
    }
    if (consist_cnt == 0) {
      consist_dist = 1e10;
      consist_score = 0;
    } else {
      consist_dist /= consist_cnt;
      consist_score /= consist_cnt;
    }

    match_sample.match_score =
        match_sample.hit_ratio * (MatchParam::weight_p2m * p2m_score +
                                  MatchParam::weight_consist * consist_score);
    match_sample.consistency_distance_score = consist_score;
    match_sample.percept2map_distance_score = p2m_score;
    match_sample.consistency_distance = consist_dist;
    match_sample.percept2map_distance = p2m_dist;

    LC_LDEBUG(MATCHING) << "match score " << match_sample.match_score
                        << " hit ratio " << match_sample.hit_ratio
                        << ", p2m/consist dist " << p2m_dist << "/"
                        << consist_dist << ", p2m/consist score " << p2m_score
                        << "/" << consist_score;
  }

  // get best match
  std::sort(match_samples_.begin(), match_samples_.end(),
            [](const MatchScore& a, const MatchScore& b) {
              return a.match_score > b.match_score;
            });
  d_lateral_ = match_samples_.front().pose_sample.y;
  d_heading_ = heading_samples_[match_samples_.front().pose_sample.heading_idx];

  // check match score
  if (match_samples_.front().match_score < MatchParam::min_match_score) {
    LC_LDEBUG(MATCHING) << "distance consistency check failed!";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

adLocStatus_t MatcherMultiSemanticGrid::PostConditionCheck() {
  // matched perception element number statistic
  MatchIndex::Ptr match_index = match_samples_.front().match;
  if (match_index == nullptr) return LOC_LOCALIZATION_ERROR;

  MatchPairVec match_pairs = match_index->GetAllMatchInOneVec();
  if (match_pairs.empty()) return LOC_LOCALIZATION_ERROR;

  matched_type_nums_.clear();
  std::vector<std::pair<id_t, id_t>> rs_matches;
  for (const auto& pair : match_pairs) {
    id_t p_id = pair.first;
    auto iter = percept_points_in_bv_.find(p_id);
    if (iter == percept_points_in_bv_.end()) continue;
    MatchSemanticType match_type = iter->second.front().type;
    matched_type_nums_[match_type].insert(p_id);
    if (match_type == MatchSemanticType::RoadSideLine) {
      rs_matches.emplace_back(pair);
    }
    LC_LDEBUG(MATCHING) << "match pair: " << p_id << " -> " << pair.second;
  }

  // get matched percept element number
  int match_rs_num = matched_type_nums_[MatchSemanticType::RoadSideLine].size();
  int match_sl_num = matched_type_nums_[MatchSemanticType::SolidLine].size();
  int match_dl_num = matched_type_nums_[MatchSemanticType::DashedLine].size();
  int match_pole_num = matched_type_nums_[MatchSemanticType::Pole].size();
  int match_sign_num = matched_type_nums_[MatchSemanticType::Sign].size();

  // get matched line style num
  int match_line_style_num = 0;
  if (match_rs_num) ++match_line_style_num;
  if (match_sl_num) ++match_line_style_num;
  if (match_dl_num) ++match_line_style_num;

  // get total match elem num
  int total_match_elem_num = match_rs_num + match_sl_num + match_dl_num +
                             match_pole_num + match_sign_num;

  // get percept element number
  int percept_rs_num = percept_type_nums_[MatchSemanticType::RoadSideLine];
  int percept_ll_num = percept_type_nums_[MatchSemanticType::DashedLine] +
                       percept_type_nums_[MatchSemanticType::SolidLine];
  int percept_line_num = percept_rs_num + percept_ll_num;
  LC_LDEBUG(MATCHING) << "m_rs_n " << match_rs_num << "/"
                      << "m_sl_n " << match_sl_num << "/"
                      << "m_dl_n " << match_dl_num << "/"
                      << "m_pole_n " << match_pole_num << "/"
                      << "m_sign_n " << match_sign_num;
  LC_LDEBUG(MATCHING) << "best reloc match with hit ratio "
                      << match_samples_.front().hit_ratio;

  // check match percept pt ratio
  if (match_samples_.front().hit_ratio < 0.5) {
    LC_LDEBUG(MATCHING) << "hit ratio too low";
    return LOC_LOCALIZATION_ERROR;
  }

  // consider check with matched pole/sign
  if (match_pole_num != 0 && match_line_style_num >= 2 &&
      total_match_elem_num >= 5)
    return LOC_SUCCESS;

  // check if has 2 widest roadsides match
  LC_LDEBUG(MATCHING) << "percept_left_rs_id_ " << percept_left_rs_id_;
  LC_LDEBUG(MATCHING) << "percept_right_rs_id_ " << percept_right_rs_id_;
  bool two_roadside_matched = false;
  for (size_t i = 0; i < rs_matches.size(); ++i) {
    if (two_roadside_matched) break;
    id_t rs_i = rs_matches[i].first, rs_map_i = rs_matches[i].second;
    for (size_t j = i + 1; j < rs_matches.size(); ++j) {
      id_t rs_j = rs_matches[j].first, rs_map_j = rs_matches[j].second;
      bool widest_roadside_pair =
          (rs_i == percept_left_rs_id_ && rs_j == percept_right_rs_id_) ||
          (rs_i == percept_right_rs_id_ && rs_j == percept_left_rs_id_);
      if (!widest_roadside_pair) continue;
      double percept_rs_dist = 0, map_rs_dist = 0;
      if (!percept_dist_db_->GetDistBetweenElements(rs_i, rs_j,
                                                    &percept_rs_dist))
        continue;
      if (!map_dist_db_->GetDistBetweenElements(rs_map_i, rs_map_j,
                                                &map_rs_dist))
        continue;
      LC_LDEBUG(MATCHING) << "percept roadside " << rs_i << "/" << rs_j
                          << " dist " << percept_rs_dist;
      LC_LDEBUG(MATCHING) << "map roadside " << rs_map_i << "/" << rs_map_j
                          << " dist " << map_rs_dist;
      if (std::fabs(percept_rs_dist - map_rs_dist) <
          MatchParam::max_match_rs_inconsist) {
        two_roadside_matched = true;
        break;
      }
    }
  }

  // check matched line number
  // 1. at least 3 line match, 1 roadside match
  int match_line_num = match_rs_num + match_sl_num + match_dl_num;
  if ((percept_line_num > 2 && match_line_num <= 2) || match_rs_num == 0) {
    LC_LDEBUG(MATCHING) << "too few line matched!";
    return LOC_LOCALIZATION_ERROR;
  }

  // 2. if 2 roadside matched, max unmatched line num is 2
  int unmatch_line_num = percept_line_num - match_line_num;
  if (two_roadside_matched && unmatch_line_num > 2) {
    LC_LDEBUG(MATCHING)
        << "2 roadside matched, but still has too many unmatched lines!";
    return LOC_LOCALIZATION_ERROR;
  }

  // 3. if only 1 roadside match
  if (!two_roadside_matched) {
    // matched lines should have different style
    int map_sl_num = map_type_nums_[MatchSemanticType::SolidLine];
    int map_dl_num = map_type_nums_[MatchSemanticType::DashedLine];
    int map_rs_num = map_type_nums_[MatchSemanticType::RoadSideLine];
    if ((match_sl_num == 0 && map_sl_num != 0) ||
        (match_dl_num == 0 && map_dl_num != 0)) {
      LC_LDEBUG(MATCHING)
          << "only single line style, match ambiguous possible!";
      return LOC_LOCALIZATION_ERROR;
    }
    // max unmatched line num is 1
    if (unmatch_line_num > 1) {
      LC_LDEBUG(MATCHING) << "too many unmatched lines!";
      return LOC_LOCALIZATION_ERROR;
    }
    // other roadside should be matched
    int unmatch_percept_rs_num = percept_rs_num - match_rs_num;
    int unmatch_map_rs_num = map_rs_num - match_rs_num;
    if (unmatch_percept_rs_num != 0 && unmatch_map_rs_num != 0) {
      LC_LDEBUG(MATCHING) << "unmached roadside is not allowed!";
      return LOC_LOCALIZATION_ERROR;
    }
  }
  return LOC_SUCCESS;
}

void MatcherMultiSemanticGrid::GetFinalMatchIndex() {
  // all check pass, split to multi camera match result
  MatchIndex::Ptr match_index = match_samples_.front().match;
  if (match_index == nullptr) return;
  auto multi_semantic_match_pairs = match_index->GetAllSemanticMatch();
  for (const auto& item : multi_semantic_match_pairs) {
    SemanticType semantic_type = item.first;
    const MatchPairVec& match_pairs = item.second;
    for (const auto& pair : match_pairs) {
      id_t percept_id = pair.first;
      id_t map_id = pair.second;
      LC_LDEBUG(debug) << "smm reloc final match pairs, percept_id: "
                       << percept_id << ", map_id: " << map_id;
      if (!percept_frame_sources_.count(percept_id)) continue;
      auto cam_name = percept_frame_sources_.at(percept_id);
      if (!multi_frame_match_.count(cam_name)) {
        multi_frame_match_[cam_name] = std::make_shared<MatchIndex>();
      }
      multi_frame_match_[cam_name]->AddSemanticMatch(semantic_type, pair);
    }
  }
}

std::vector<std::vector<std::pair<double, double>>>
MatcherMultiSemanticGrid::GetRelocalDistributions() const {
  std::vector<std::vector<std::pair<double, double>>> data;

  std::vector<std::pair<double, double>> prior;
  for (const auto& item : lateral_scores_predict_) {
    prior.emplace_back(item.y, item.score);
  }
  data.emplace_back(std::move(prior));

  std::vector<std::pair<double, double>> likelihood;
  for (const auto& item : lateral_scores_measurement_) {
    likelihood.emplace_back(item.y, item.score);
  }
  data.emplace_back(std::move(likelihood));

  std::vector<std::pair<double, double>> posterior;
  for (const auto& item : lateral_scores_) {
    posterior.emplace_back(item.y, item.score);
  }
  data.emplace_back(std::move(posterior));

  std::vector<std::pair<double, double>> peaks;
  for (const auto& item : lateral_peaks_) {
    peaks.emplace_back(item.y, item.score);
  }
  data.emplace_back(std::move(peaks));

  return data;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
