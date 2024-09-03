/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： reloc.hpp
 *   author     ： zhaohaowu/nihongjie
 *   date       ： 2024.04
 ******************************************************************************/

#pragma once

#include <cmath>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Transform.h"
#include "base/utils/log.h"
#include "depend/proto/localization/localization.pb.h"
#include "modules/location/fusion_center/lib/defines.h"
#include "modules/location/pose_estimation/lib/reloc/base.hpp"

namespace hozon {
namespace mp {
namespace loc {
namespace pe {
enum LaneType {
  UNKNOWN = 0,      // 未知
  SOLID_LINE = 1,   // 单实线
  DASHED_LINE = 2,  // 单虚线
  Road_Edge = 3     // 路沿
};

struct LaneLine {
  LaneType lane_type;
  std::vector<Eigen::Vector3d> points;
};

struct MappingManager {
  std::unordered_map<int, LaneLine> lane_lines;
};
struct TrackingManager {
  double timestamp;
  std::unordered_map<int, LaneLine> lane_lines;
};

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
  static constexpr double max_converge_timegap = 1;
  static constexpr double min_converge_dist = 5.0;
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

static inline SemanticType MatchTypeToSemanticType(LaneType type) {
  if (type == LaneType::SOLID_LINE || type == LaneType::DASHED_LINE ||
      type == LaneType::Road_Edge) {
    return SemanticType::LaneLine;
  }
  // if (type == MatchSemanticType::Sign) {
  //   return SemanticType::TrafficSign;
  // }
  // if (type == MatchSemanticType::Pole) {
  //   return SemanticType::Pole;
  // }
  return SemanticType::NONE;
}

static inline bool IsLaneLine(LaneType type) {
  return MatchTypeToSemanticType(type) == SemanticType::LaneLine;
}

// class holds the voxel index in 2-dimonsions
class VoxelIndex {  // NOLINT
 public:
  VoxelIndex() = default;
  VoxelIndex(int x, int y) : x_idx(x), y_idx(y) {}
  VoxelIndex(const VoxelIndex& rhs)  // NOLINT
      : x_idx(rhs.x_idx), y_idx(rhs.y_idx) {}
  ~VoxelIndex() = default;

  bool operator==(const VoxelIndex& rhs) const {
    return (x_idx == rhs.x_idx) && (y_idx == rhs.y_idx);
  }

 public:
  int x_idx;  // x-direction index // NOLINT
  int y_idx;  // y-direction index // NOLINT
};

// class holds the hash struct of voxel index
class VoxelIndexHash {
 public:
  size_t operator()(const VoxelIndex& idx) const {
    size_t seed = 0;
    hash_combine(&seed, idx.x_idx);
    hash_combine(&seed, idx.y_idx);
    return seed;
  }
  template <class T>
  void hash_combine(size_t* seed, const T& v) const {
    std::hash<T> hasher;
    *seed ^= hasher(v) + 0x9e3779b9 + (*seed << 6) + (*seed >> 2);
  }
};

// class holds the converter of voxel index with point3d
class VoxelIndexConverter {
 public:
  DEFINE_SMART_PTR(VoxelIndexConverter)

 public:
  explicit VoxelIndexConverter(double res) : res_x_(res), res_y_(res) {}
  VoxelIndexConverter(double res_x, double res_y)
      : res_x_(res_x), res_y_(res_y) {}
  ~VoxelIndexConverter() = default;

  // @brief: converter methods
  VoxelIndex PointInnerToVoxelIndex(const Eigen::Vector2d& point) const {
    return {Index(point.x(), res_x_), Index(point.y(), res_y_)};
  }
  VoxelIndex PointEigenToVoxelIndex(const Eigen::Vector2d& vec) const {
    return {Index(vec(0), res_x_), Index(vec(1), res_y_)};
  }

  Eigen::Vector2d VoxelIndexToPointInner(const VoxelIndex& index) const {
    return {Center(index.x_idx, res_x_), Center(index.y_idx, res_y_)};
  }
  Eigen::Vector2d VoxelIndexToPointEigen(const VoxelIndex& index) const {
    return {Center(index.x_idx, res_x_), Center(index.y_idx, res_y_)};
  }

 private:
  static inline int Index(double d, double res) {
    double base = d + 0.5 * res;
    return base < 0 ? static_cast<int>((base / res) - 1)
                    : static_cast<int>(base / res);
  }
  static inline double Center(int idx, double res) { return (idx * res); }

 private:
  double res_x_;  // voxel x resolution
  double res_y_;  // voxel y resolution
};

class Reloc {
 public:
  DEFINE_SMART_PTR(Reloc)

  Reloc();
  ~Reloc() = default;

  void ResetStep(const double& max_search_lateral);

  // @brief: lane-level relocalization matching core
  bool ProcData(
      const std::shared_ptr<::hozon::localization::Localization>& localization,
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MappingManager>& map_manager);
  void RvizFunc(const std::shared_ptr<TrackingManager>& tracking_manager);

  void GetRelocPose(Eigen::Affine3d* const T_w_v) {
    if (T_w_v == nullptr) {
      HLOG_ERROR << "why nullptr";
      return;
    }
    *T_w_v = T_w_v_;
  }

  // @brief: get initial pose correcton
  NavState GetPoseCorrection() const {
    NavState nav_state;

    nav_state.pose =
        Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(0, 0, d_heading_)),
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
  bool PreConditionCheck(
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MappingManager>& map_manager);

  // @brief: semantic type/element statistic
  void PerceptionStatistic(
      const std::shared_ptr<TrackingManager>& tracking_manager);

  // @brief: semantic type/element statistic
  void MapStatistic(const std::shared_ptr<MappingManager>& map_manager);

  // @brief: get map/perception roadside width coarsely
  void GetRoadsideDistanceCoarse(
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MappingManager>& map_manager,
      bool* percept_roadside_valid, bool* map_roadside_vaild,
      double* inconsist_distance);

  // @brief: get final match index
  void GetFinalMatchIndex();

  // @brief: filter prediction using posterior of last step
  bool FilterPrediction(
      const std::shared_ptr<::hozon::localization::Localization>& localization);

  // @brief: detect posterior histogram peaks
  bool HistogramPeakDetection();

  //////////////////////// old version reloc related ////////////////////////

  //////////////////////// lateral search ////////////////////////
  // @brief: lateral scoring core
  bool LateralSpaceScoring(
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MappingManager>& map_manager);

  // @brief: estimate percept laneline polyfit params for heading alignment to
  // map
  bool EstimatePerceptLanelineHeading(
      const std::shared_ptr<TrackingManager>& tracking_manager);

  // @brief: construct map grids for point-level matching and scoring
  bool ConstructMapGrids(const std::shared_ptr<MappingManager>& map_manager);

  // @brief: generate search poses, including y and heading
  bool GenerateSearchPose();

  // @brief: score all poses
  bool ScoreAllPoses(const std::shared_ptr<TrackingManager>& tracking_manager);

  // @brief: score at a certain pose and get hit info
  double ScorePose(
      double y, double heading,
      const std::shared_ptr<TrackingManager>& tracking_manager,
      std::unordered_map<int, std::vector<int>>* hit_info = nullptr);

  //////////////////////// histogram filtering ////////////////////////
  // @brief: histogram filtering core
  bool HistogramFiltering(
      const std::shared_ptr<::hozon::localization::Localization>& localization);

  // @brief: filter update with measurement
  bool FilterUpdate();

  // @brief: check if filter has converged inspace
  bool FilterConvergenceCheck();

  // @brief: get match pair index at each peak
  bool GetMatchSamples(
      const std::shared_ptr<TrackingManager>& tracking_manager);

  //////////////////////// post-condition check ////////////////////////

  // @brief: check if enough semantic types/elements matched with map
  bool PostConditionCheck(
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MappingManager>& map_manager);

 private:
  // @brief: get local index pattern
  std::vector<VoxelIndex> LocalIndexPattern(const VoxelIndex& hit_index,
                                            LaneType type);
  static Sophus::SE3d Node2SE3(const hozon::mp::loc::fc::Node& node);
  static Sophus::SE3d Node2SE3(
      const std::shared_ptr<hozon::mp::loc::fc::Node>& node);
  void LocalizationToNode(
      const ::hozon::localization::Localization& localization,
      hozon::mp::loc::fc::Node* global_pose, hozon::mp::loc::fc::Node* dr_pose);
  template <typename T>
  void Point3dToVector3d(const T& enu, Eigen::Vector3d* vector);
  template <typename T>
  double FindGoodLane(const std::shared_ptr<T>& manager, bool* is_found,
                      int* left_rs_id, int* right_rs_id);
  void ComputeRelocPose(
      const std::shared_ptr<::hozon::localization::Localization>& localization);

 private:
  struct Grid {
    std::unordered_map<LaneType, std::list<MapPoint>, EnumClassHash>
        map_elements;
  };

  struct PoseScore {
    double y;
    double heading;
    double score;
  };

  struct PolyLineParam {
    bool valid;
    Eigen::Vector4d line_param;
    double confidence;
  };

  struct LineTypeCentroid {  // NOLINT
    MatchSemanticType line_type;
    Eigen::Vector2d line_centroid;
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

  struct ConsistencyDistDB {
    DEFINE_SMART_PTR(ConsistencyDistDB)

    void AddElement(int id, SemanticType type,
                    const std::vector<Eigen::Vector3d>& points) {
      if (element_points_.count(id) != 0U) {
        HLOG_ERROR << "element " << id << "already exists!";
        return;
      }
      element_points_.insert(std::make_pair(id, std::make_pair(type, points)));
    }

    bool GetDistBetweenElements(int id1, int id2, double* dist) {
      if ((element_points_.count(id1) == 0U) ||
          (element_points_.count(id2) == 0U)) {
        return false;
      }
      const auto& element1 = element_points_.at(id1);
      const auto& element2 = element_points_.at(id2);
      SemanticType type1 = element1.first;
      SemanticType type2 = element2.first;
      const auto& points1 = element1.second;
      const auto& points2 = element2.second;
      bool signed_dist =
          type1 == SemanticType::LaneLine || type2 == SemanticType::LaneLine;

      // 1. find evaluated distance btw id1 and id2
      MatchPair pair = std::make_pair(id1, id2);
      if (consistency_dists_.count(pair) != 0U) {
        *dist = consistency_dists_.at(pair);
        return std::fabs(*dist) < 1e9;
      }
      // 2. reverted id pair means negative of signed distance
      pair = std::make_pair(id2, id1);
      if (consistency_dists_.count(pair) != 0U) {
        *dist = consistency_dists_.at(pair);
        if (signed_dist) {
          *dist *= -1;
        }
        return std::fabs(*dist) < 1e9;
      }

      // 3. no distance found for (id1, id2) or (id2, id1), try to evaluate
      double distance = 0;
      int valid_num = 0;
      bool p2p_distance = type1 != SemanticType::LaneLine &&
                          type2 != SemanticType::LaneLine &&
                          points1.size() == 1 && points2.size() == 1;
      if (p2p_distance) {
        // unsigned point to point distance
        distance = (points1[0] - points2[0]).norm();
        valid_num = 1;
      } else if (points1.size() <= points2.size()) {
        distance = Line2LineDistance2D(points1, points2, &valid_num, true);
      } else if (points1.size() > points2.size()) {
        distance =
            -1.0 * Line2LineDistance2D(points2, points1, &valid_num, true);
      }
      if (valid_num == 0) {
        distance = 1e10;
      }
      HLOG_DEBUG << id1 << "-" << id2 << " dist " << distance;
      consistency_dists_.insert({std::make_pair(id1, id2), distance});
      *dist = distance;
      return valid_num != 0;
    }

    bool GetDistBetweenElements(int id, int id_other,
                                const ConsistencyDistDB::Ptr& other_db,
                                const Sophus::SE3d& T_this_other,
                                double* dist) const {
      auto iter = element_points_.find(id);
      if (iter == element_points_.end()) {
        return false;
      }
      auto type = iter->second.first;
      const auto& points = iter->second.second;

      auto iter_other = other_db->element_points_.find(id_other);
      if (iter_other == other_db->element_points_.end()) {
        return false;
      }
      auto type_other = iter_other->second.first;
      const auto& points_other = iter_other->second.second;
      std::vector<Eigen::Vector3d> points_other_transed =
          TransformPoints(points_other, T_this_other);

      double distance = 0;
      int valid_num = 0;
      bool p2p_distance = type != SemanticType::LaneLine &&
                          type_other != SemanticType::LaneLine &&
                          points.size() == 1 && points_other.size() == 1;
      if (p2p_distance) {
        distance = (points[0] - points_other_transed[0]).norm();
        valid_num = 1;
      } else {
        distance =
            Line2LineDistance2D(points_other_transed, points, &valid_num);
      }
      if (valid_num == 0) {
        distance = 1e10;
      }
      *dist = distance;
      return true;
    }

   private:
    // element points with id and semantic type
    std::unordered_map<int,
                       std::pair<SemanticType, std::vector<Eigen::Vector3d>>>
        element_points_;
    // distance btw element pairs
    std::unordered_map<MatchPair, double, MatchPairHash> consistency_dists_;
  };

 private:
  // odomstate related
  std::shared_ptr<::hozon::localization::Localization>
      odom_state_last_converge_ = nullptr;
  std::shared_ptr<::hozon::localization::Localization> last_localization_node_ =
      nullptr;

  // map grid related
  VoxelIndexConverter::Ptr grid_idx_converter_;
  std::unordered_map<VoxelIndex, Grid, VoxelIndexHash> map_grids_;
  std::unordered_map<MatchSemanticType, size_t, EnumClassHash> map_type_nums_;
  std::vector<MapPoint> map_sign_points_;
  std::vector<MapPoint> map_pole_points_;

  // map line related
  std::unordered_map<int, PolyLineParam> map_poly_line_params_;

  // perception related
  std::unordered_map<int, PolyLineParam> percept_poly_line_params_;
  std::unordered_map<LaneType, size_t, EnumClassHash> percept_type_nums_;
  std::unordered_map<LaneType, std::unordered_set<int>, EnumClassHash>
      matched_type_nums_;
  std::unordered_map<int, std::string> percept_frame_sources_;
  std::unordered_map<int, double> vehicle_to_percept_line_dists_;
  int percept_left_rs_id_ = -1;
  int percept_right_rs_id_ = -1;
  int map_left_rs_id_ = -1;
  int map_right_rs_id_ = -1;
  int percept_left_line_id_ = -1;
  int percept_right_line_id_ = -1;
  double percept_line_ave_heading_{0};

  // distance db related
  std::shared_ptr<ConsistencyDistDB> map_dist_db_;
  std::shared_ptr<ConsistencyDistDB> percept_dist_db_;

  // search space related
  double step_size_;
  int lat_steps_;
  std::vector<double> search_ys_;
  std::vector<std::pair<double, double>> search_poses_;

  // possible match pair related
  std::vector<MatchPairVec> possible_match_pairs_vec_;
  std::vector<MatchPairVec> last_possible_match_pairs_vec_;
  std::unordered_map<int, MatchSemanticType> percept_line_types_;
  std::vector<std::pair<int, LineTypeCentroid>> map_line_type_centroids_;
  std::shared_ptr<PerceptPointsToMapLineDist> percept_map_dist_db_;

  // search score related
  bool has_prediction_{false};
  std::vector<PoseScore> scores_predict_;
  std::vector<PoseScore> scores_measurement_;
  std::vector<PoseScore> last_scores_posterior_;
  std::vector<PoseScore> thresholded_lateral_scores_;
  std::vector<PoseScore> lateral_peaks_;

  // smm reloc match inlier points
  std::vector<Eigen::Vector3d> inlier_points_;

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
  Eigen::Affine3d T_w_v_ = Eigen::Affine3d(Eigen::Matrix4d::Identity());
};

inline std::vector<int> VoxelDownSample2D(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<Eigen::Matrix2d>& covs, double sample_dist,
    double min_x = -100, double max_x = 100, double min_y = -100,
    double max_y = 100) {
  std::unordered_map<VoxelIndex, int, VoxelIndexHash> voxels;
  VoxelIndexConverter voxel_converter(sample_dist);
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    const auto& pt = points[i];
    if (pt.x() < min_x || pt.x() > max_x || pt.y() < min_y || pt.y() > max_y) {
      continue;
    }
    auto index =
        voxel_converter.PointInnerToVoxelIndex(Eigen::Vector2d(pt.x(), pt.y()));
    auto iter = voxels.find(index);
    if (iter == voxels.end()) {
      voxels.insert({index, i});
    } else {
      // int i_ori = iter->second;
      // double cov_original = covs[i_ori].diagonal().norm();
      // double cov_other = covs[i].diagonal().norm();
      // if (cov_other < cov_original) {
      //   iter->second = i;
      // }
    }
  }
  std::vector<int> sampled_idxs;
  sampled_idxs.reserve(voxels.size());
  for (const auto& item : voxels) {
    sampled_idxs.emplace_back(item.second);
  }
  return sampled_idxs;
}
}  // namespace pe
}  // namespace loc
}  // namespace mp
}  // namespace hozon
