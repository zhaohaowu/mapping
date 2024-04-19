/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "localization/common/log.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "semantic_mm/common/match_index.hpp"
#include "semantic_mm/common/math_tools.hpp"
#include "semantic_mm/common/voxel_index.hpp"
#include "semantic_mm/matching/matcher/consistency_dist_db.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class FramePackage;
class MapManager;
class TrackingManager;

struct MatchParam {
  // search and map grid size related
  static constexpr double grid_size = 1.2;
  static constexpr double search_scale = 2.0;
  static constexpr double max_search_lateral = 7.8;
  static constexpr int lat_dt_extend = 1;
  // heading alignment related
  static constexpr double max_percept_heading = 45.0 * M_PI / 180.0;
  static constexpr double max_map_heading = 30.0 * M_PI / 180.0;
  static constexpr double max_align_heading = 3.0 * M_PI / 180.0;
  // percept laneline point sample dist
  static constexpr double line_sample_size = 4.0;
  // histogram filter related
  static constexpr double min_hf_predict_sigma = 0.5;
  static constexpr double kl_divergence_bias = 2.0;
  static constexpr uint64_t max_converge_timegap = 1e9;
  static constexpr double min_converge_dist = 10.0;
  static constexpr double max_peak_scale = 0.5;
  static constexpr double max_cluster_width = 5.0;
  // pole/sign map match related
  static constexpr double p2m_sign_thres = 1.0;
  static constexpr double p2m_pole_thres = 2.5;
  // distance consistency related
  static constexpr double p2m_sigma_line = 0.3;
  static constexpr double p2m_sigma_sign = 0.6;
  static constexpr double p2m_sigma_pole = 0.4;
  static constexpr double weight_p2m = 0.7;
  static constexpr double weight_consist = 0.3;
  static constexpr double min_match_score = 0.25;
  // roadside check related
  static constexpr double max_prematch_rs_inconsist = 3.0;
  static constexpr double max_match_rs_inconsist = 2.0;
  // others
  static constexpr bool disable_fov30_lanelines = true;
  static constexpr bool enable_hf_heading_fusion = false;
  static constexpr bool enable_dist_consist_check = false;
};

// define semantic type for grid match
enum class MatchSemanticType {
  Unknown = 0,
  DashedLine = 1,
  SolidLine = 2,
  RoadSideLine = 3,
  Sign = 4,
  Pole = 5,
  Other = 255,
};

static inline SemanticType MatchTypeToSemanticType(MatchSemanticType type) {
  if (type == MatchSemanticType::DashedLine ||
      type == MatchSemanticType::SolidLine ||
      type == MatchSemanticType::RoadSideLine)
    return SemanticType::LaneLine;
  else if (type == MatchSemanticType::Sign)
    return SemanticType::TrafficSign;
  else if (type == MatchSemanticType::Pole)
    return SemanticType::Pole;
  else
    return SemanticType::NONE;
}

static inline bool IsLaneLine(MatchSemanticType type) {
  return MatchTypeToSemanticType(type) == SemanticType::LaneLine;
}

class MatcherMultiSemanticGrid {
 public:
  DEFINE_SMART_PTR(MatcherMultiSemanticGrid)

  MatcherMultiSemanticGrid();
  ~MatcherMultiSemanticGrid() = default;

  // @brief: lane-level relocalization matching core
  adLocStatus_t ProcessGridMatch(
      const std::shared_ptr<FramePackage>& frame_package,
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager);

  // @brief: road-level relocalization if lane-level reloc fail
  adLocStatus_t ProcessGridMatchRoadLevelAccuracy();

  // @brief: new version smm relocalization without grid
  adLocStatus_t ProcessRelocTrackerMatch(
      const std::shared_ptr<FramePackage>& frame_package,
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager);

  // @brief: get initial pose correcton
  NavState GetPoseCorrection() const {
    NavState nav_state;
    // nav_state.pose = SE3d(SO3d(0, 0, d_heading_),
    //                              Eigen::Vector3d(0, d_lateral_, 0)); old
    //                              Sophus
    nav_state.pose = SE3d(SO3d::exp(Eigen::Vector3d(0, 0, d_heading_)),
                          Eigen::Vector3d(0, d_lateral_, 0));
    nav_state.pose_cov.setIdentity();
    nav_state.pose_cov *= 10000.0;
    nav_state.pose_cov(1, 1) = lateral_std_ * lateral_std_;
    nav_state.pose_cov(5, 5) = heading_std_ * heading_std_;
    return nav_state;
  }

  // @brief: get match pair index
  std::unordered_map<std::string, MatchIndex::Ptr> GetMatchIndex() const {
    return multi_frame_match_;
  }

  // @brief: get histogram distributions for visualization
  std::vector<std::vector<std::pair<double, double>>> GetRelocalDistributions()
      const;

 private:
  ///////////// common for two versions reloc ///////////////////////////
  // @brief: init for matching
  void InitMatch();

  // @brief: check if semantic type/element enough for matching
  adLocStatus_t PreConditionCheck(
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager);

  // @brief: semantic type/element statistic
  void PerceptionStatistic(
      const std::shared_ptr<TrackingManager>& tracking_manager);

  // @brief: semantic type/element statistic
  void MapStatistic(const std::shared_ptr<MapManager>& map_manager);

  // @brief: get map/perception roadside width coarsely
  void GetRoadsideDistanceCoarse(
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager,
      bool* percept_roadside_valid, bool* map_roadside_vaild,
      double* inconsist_distance);

  // @brief: get final match index
  void GetFinalMatchIndex();

  // @brief: filter prediction using posterior of last step
  adLocStatus_t FilterPrediction(
      const std::shared_ptr<FramePackage>& frame_package);

  // @brief: detect posterior histogram peaks
  adLocStatus_t HistogramPeakDetection();

  //////////////////////// old version reloc related ////////////////////////

  //////////////////////// lateral search ////////////////////////
  // @brief: lateral scoring core
  adLocStatus_t LateralSpaceScoring(
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager);

  // @brief: estimate percept laneline polyfit params for heading alignment to
  // map
  adLocStatus_t EstimatePerceptLanelineHeading(
      const std::shared_ptr<TrackingManager>& tracking_manager);

  // @brief: sample perception points
  adLocStatus_t SamplePerceptionPoints(
      const std::shared_ptr<TrackingManager>& tracking_manager);

  // @brief: construct map grids for point-level matching and scoring
  adLocStatus_t ConstructMapGrids(
      const std::shared_ptr<MapManager>& map_manager);

  // @brief: generate search poses, including y and heading
  adLocStatus_t GenerateSearchPose();

  // @brief: score all poses
  adLocStatus_t ScoreAllPoses();

  // @brief: score at a certain pose and get hit info
  double ScorePose(
      double y, int heading_idx,
      std::unordered_map<id_t, std::vector<id_t>>* hit_info = nullptr);

  //////////////////////// histogram filtering ////////////////////////
  // @brief: histogram filtering core
  adLocStatus_t HistogramFiltering(
      const std::shared_ptr<FramePackage>& frame_package);

  // @brief: filter update with measurement
  adLocStatus_t FilterUpdate();

  // @brief: check if filter has converged inspace
  adLocStatus_t FilterConvergenceCheck();

  // @brief: get match pair index at each peak
  adLocStatus_t GetMatchSamples();

  //////////////////////// post-condition check ////////////////////////
  // @brief: check if match satisfies distance consistency
  adLocStatus_t DistanceConsistencyCheck(
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager);

  // @brief: check if enough semantic types/elements matched with map
  adLocStatus_t PostConditionCheck();

  //////////////////////// new version reloc related ////////////////////////

  //////////////////////// find nearest percept line //////////////
  // @breif: downsample percept lines
  adLocStatus_t SamplePerceptionPointsNew(
      const std::shared_ptr<TrackingManager>& tracking_manager);

  // @breif: remove percept lines which are far away from vehicle
  adLocStatus_t RemoveUnstablePerceptLines();

  // @brief: get the nearest left and right percept line
  void FindTheNearestPerceptLine2Vehicle();

  // @breif: estimate the nearest percept line params
  adLocStatus_t EstimatePerceptLanelineParams(
      const std::shared_ptr<TrackingManager>& tracking_manager);

  // @breif: estimate map lanline polyfit params
  adLocStatus_t EstimateMapLaneLineParams(
      const std::shared_ptr<MapManager>& map_manager);

  adLocStatus_t CalculateMapLineTypeCentorid(
      const std::shared_ptr<MapManager>& map_manager);

  //////////////////////// lateral search ////////////////////////
  // @brief: lateral scoring with parallel check core
  adLocStatus_t LateralSpaceScoringWithParallelCheck(
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager,
      const std::shared_ptr<FramePackage>& frame_package);

  // @breif: generate serach heading for each search y
  adLocStatus_t GenerateSearchPoseWithParallelCheck(
      const std::shared_ptr<FramePackage>& frame_package);

  // @brief: construct percept-to-map dist db
  void ConstructPerceptionMapDistDB(double optimize_heading);

  // @brief: get each search pose's score according to percept-to-map dist db
  adLocStatus_t ScoreAllPosesNew(
      const std::shared_ptr<FramePackage>& frame_package);

  //////////////////////// calculate search heading ///////////////////
  // @brief: solve heading by optimize percept point to map line lateral dist
  double SolveYawByOptimPoint2LineDist(
      const Eigen::Vector4d& percept_left_line_param,
      const Eigen::Vector4d& percept_right_line_param,
      const std::vector<Eigen::Vector4d>& map_all_line_params, int left,
      int right);

  // @brief: solve y by optimize percept point to map line lateral dist
  double SolveYByOptimPoint2LineDist(double y_init);

  //////////////////////// histogram filter ////////////////////////
  // @breif: new version histogram filter
  adLocStatus_t HistogramFilteringNew(
      const std::shared_ptr<FramePackage>& frame_package);

  // @brief: new version filter update
  adLocStatus_t FilterUpdateNew();

  //////////////////////// match pairs ////////////////////////
  // @brief: get possible match pairs with match semantic type consistency
  void GetPossibleMatchPairsWithLineType();

  // @brief: remove lateral inconsistent match pairs from possible match pairs
  void RemoveImpossibleMatchPairsWithLateralInformation(
      double heading, double search_y, double search_width,
      std::vector<MatchPairVec>* possible_match_pairs_vec);

  // @brief: get final possible match pairs
  void GetFinalPossibleMatchPairsAtSearchY(
      double heading, double search_y,
      const std::vector<Eigen::Vector4d>& map_line_params,
      std::vector<MatchPairVec>* final_possible_match_pairs_vec, int* last_left,
      int* last_right);

  //////////////////////// estimate smm reloc pose and cov //////
  // @brief; new version estimate reloc pose and cov
  adLocStatus_t EstimateRelocPoseCov();

  // @brief: new version estimate road-level reloc cov
  void EstimateRoadLevelRelocCov();

  // @brief: new version get match pairs at each peak
  adLocStatus_t GetMatchSamplesNew();

  ////////////////////// for frame_package interface /////////////
  // @brief: set percept polyfit line param
  adLocStatus_t SetPerceptPolyfitLineParam(
      const std::shared_ptr<FramePackage>& frame_package,
      const std::shared_ptr<TrackingManager>& tracking_manager);

  // @brief: set map polyfit line param
  void SetMapPolyfitLineParam(
      const std::shared_ptr<FramePackage>& frame_package);

 private:
  // @brief: get local index pattern
  std::vector<VoxelIndex> LocalIndexPattern(const VoxelIndex& hit_index,
                                            MatchSemanticType type);

 private:
  // data struct definition for internal use
  struct PerceptPoint {
    PerceptPoint(const Point2D_t& pt, const Eigen::Matrix2d& cov,
                 MatchSemanticType type)
        : point(pt), cov(cov), type(type) {}
    PerceptPoint(const Point2D_t& pt, const Eigen::Matrix2d& cov,
                 MatchSemanticType type, int obs_cnt)
        : point(pt), cov(cov), type(type), observ_cnt(obs_cnt) {}
    Point2D_t point;
    Eigen::Matrix2d cov;
    MatchSemanticType type;
    int observ_cnt{1};
    double weight;
  };

  struct MapPoint {
    MapPoint(id_t id, const Point2D_t& pt) : id(id), point(pt) {}
    MapPoint(id_t id, const Point2D_t& pt, double heading)
        : id(id), point(pt), local_heading(heading) {}
    id_t id;
    Point2D_t point;
    double local_heading{0};
  };

  struct Grid {
    std::unordered_map<MatchSemanticType, std::list<MapPoint>, EnumClassHash>
        map_elements;
  };

  struct PoseScore {
    double y;
    double heading;
    int heading_idx{-1};
    double score{0};
  };

  struct PolyLineParam {
    bool valid;
    Eigen::Vector4d line_param;
    double confidence;
  };

  struct LineTypeCentroid {
    MatchSemanticType line_type;
    Point2D_t line_centroid;
  };

  struct MatchScore {
    MatchIndex::Ptr match;
    PoseScore pose_sample;
    double hit_ratio;
    double match_score;
    double percept2map_distance_score;
    double consistency_distance_score;
    double percept2map_distance;
    double consistency_distance;
  };

 private:
  // odomstate related
  OdomState odom_state_;
  OdomState odom_state_last_converge_;
  NavState nav_state_;

  // map grid related
  VoxelIndexConverter::Ptr grid_idx_converter_;
  std::unordered_map<VoxelIndex, Grid, VoxelIndexHash> map_grids_;
  VoxelIndex max_index_, min_index_;
  double max_map_line_heading_, min_map_line_heading_;
  double map_road_heading_, map_road_width_;
  std::unordered_map<MatchSemanticType, size_t, EnumClassHash> map_type_nums_;
  std::vector<MapPoint> map_sign_points_;
  std::vector<MapPoint> map_pole_points_;

  // map line related
  std::unordered_map<id_t, PolyLineParam> map_poly_line_params_;

  // perception related
  std::unordered_map<id_t, PolyLineParam> percept_poly_line_params_;
  std::unordered_map<id_t, std::vector<PerceptPoint>> percept_points_in_bv_;
  std::unordered_map<id_t, std::vector<Point2DWithCov>>
      downsampled_percept_points_in_bv_;
  std::unordered_map<MatchSemanticType, size_t, EnumClassHash>
      percept_type_nums_;
  std::unordered_map<MatchSemanticType, std::unordered_set<id_t>, EnumClassHash>
      matched_type_nums_;
  std::unordered_map<id_t, std::string> percept_frame_sources_;
  std::unordered_map<id_t, double> vehicle_to_percept_line_dists_;
  id_t percept_left_rs_id_ = -1;
  id_t percept_right_rs_id_ = -1;
  id_t percept_left_line_id_ = -1;
  id_t percept_right_line_id_ = -1;
  double percept_line_ave_heading_{0};

  // distance db related
  std::shared_ptr<ConsistencyDistDB> map_dist_db_;
  std::shared_ptr<ConsistencyDistDB> percept_dist_db_;

  // search space related
  double step_size_;
  int lat_steps_;
  std::vector<double> search_ys_;
  std::vector<double> heading_samples_;
  std::vector<int> heading_idx_for_each_y_;
  std::vector<std::unordered_map<id_t, std::vector<PerceptPoint>>>
      multi_heading_percept_points_;

  // possible match pair related
  std::vector<MatchPairVec> possible_match_pairs_vec_;
  std::vector<MatchPairVec> last_possible_match_pairs_vec_;
  std::unordered_map<id_t, MatchSemanticType> percept_line_types_;
  std::vector<std::pair<id_t, LineTypeCentroid>> map_line_type_centroids_;
  std::shared_ptr<PerceptPointsToMapLineDist> percept_map_dist_db_;

  // search score related
  bool has_prediction_{false};
  std::vector<PoseScore> lateral_scores_predict_;
  std::vector<PoseScore> lateral_scores_measurement_;
  std::vector<PoseScore> lateral_scores_;
  std::vector<PoseScore> thresholded_lateral_scores_;
  std::vector<PoseScore> lateral_peaks_;

  // smm reloc match inlier points
  std::vector<Point3D_t> inlier_points_;

  // estimate reloc pose cov related
  double converge_travel_dist_{0};
  double lane_level_lateral_std_{0};
  double lane_level_heading_std_{0};

  // final relocalization result
  double d_heading_{0};
  double d_lateral_{0};
  double lateral_std_{0};
  double heading_std_{0};
  std::vector<MatchScore> match_samples_;
  std::unordered_map<std::string, MatchIndex::Ptr> multi_frame_match_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
