/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： detect_cut_pt.h
 *   author     ： mahaijun
 *   date       ： 2024.05
 ******************************************************************************/

#pragma once

#include <stdlib.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>
#include <Sophus/so3.hpp>

#include "modules/map_fusion_02/common/common_data.h"
#include "modules/map_fusion_02/modules/lane/road_builder/ctp_util.h"
#include "modules/map_fusion_02/modules/lane/road_builder/cut_point.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

enum class JunctionPtType : std::uint8_t {
  UNKNOWN = 0,
  SPLIT = 1,
  MERGE = 2,
  Multi2Single = 3,
  Single2Multi = 4,
  V_Shaped = 5,
  V_Shaped_Inv = 6,  // 倒V字形状
  BROKEN = 7,        // 断路
  Broken_Start = 8
};

// only indicates lane merge point or split point
struct JunctionPoint {
  Point2D img_pt;

  // 当前车体BEV坐标系, 只有2d图像的分合流点使用
  Point2D bv_pt;
  Point3D world_pt;

  // split, merge, V shape ...
  JunctionPtType point_type = JunctionPtType::UNKNOWN;

  id_t source_line_id = 0;

  // for: split, merge , V/Y型
  id_t target_line_id = 0;

  // line id 后续都统一用 source line id 和 target line id
  // 切分点都用type 去区分, 所以以下的变量 后续要删掉

  // perception line track id
  id_t source_merge_track_id = 0;
  id_t target_merge_track_id = 0;

  // source_spit_line is the son line
  id_t source_split_track_id = 0;
  id_t target_split_track_id = 0;
};

class LaneLine {
 public:
  DEFINE_PTR(LaneLine)
  LaneLine() = default;
  ~LaneLine() = default;

  LaneLine(const LaneLine& line);
  const std::vector<Point3D>& GetPoints() const { return points_; }
  void SetPoints(const std::vector<Point3D>& points) { points_ = points; }
  id_t GetId() const { return id_; }
  void SetId(const id_t id) { id_ = id; }
  void SetJunctionPoints(const std::vector<JunctionPoint>& pts) {
    junction_points_ = pts;
  }

 private:
  id_t id_;
  std::vector<Point3D> points_;
  std::vector<JunctionPoint> junction_points_;
};

using LinePointsPair = std::pair<LaneLine::Ptr, std::vector<Point3D>>;

// for junc pt detect
struct DetectPointInfo {
  // 1:split 2:merge 3:start 4:end 5:V 6:V_inverse 7:broken
  size_t point_type = 0;

  Point3D world_pt;

  id_t source_line_id = 0;
  id_t target_line_id = 0;

  std::vector<id_t> involve_line_ids;

  // 切分线 direction
  Eigen::Vector3d cut_line_dir;

  std::vector<Sophus::SE3d> ref_poses;

  // detect times
  size_t detect_times = 0;

  // not detected again times
  size_t no_again_detect_times = 0;
};

struct CategoryInfo {
  std::pair<size_t, size_t> path_region;

  std::vector<LaneLine::Ptr> lines;

  std::vector<DetectPointInfo> detect_point_infos;

  DetectPointInfo main_point_info;  // Broken end

  // 0: continue, 1: left, 2: right
  size_t to_next_type;
};

class DetectCutPt {
 public:
  DetectCutPt() = default;
  ~DetectCutPt() = default;

  SMStatus DetectCutPoint(const std::shared_ptr<std::vector<KinePosePtr>>& path,
                          const Sophus::SE3d& Twv,
                          const std::vector<LaneLine::Ptr>& input_lines,
                          const KinePosePtr& curr_pose,
                          std::vector<CutPoint>* cutpoints);

 private:
  bool DetectSplitMergePt(const std::vector<LinePointsPair>& input_lines,
                          const Sophus::SE3d& Twv,
                          std::vector<DetectPointInfo>* new_detect_infos);

  bool JudgeSource(const std::vector<Point3D>& points1,
                   const std::vector<Point3D>& points2, const Point3D& junc_pt,
                   bool is_split);

  bool DetectSplitPt(const std::vector<Point3D>& pts1,
                     const std::vector<Point3D>& pts2, Point3D* split_pt);

  bool DetectMergePt(const std::vector<Point3D>& pts1,
                     const std::vector<Point3D>& pts2, Point3D* merge_pt);

  bool UnifiedUpdate(const std::vector<DetectPointInfo>& new_detect_infos,
                     const Sophus::SE3d& Twv);

  void GroupCutInfos(const std::vector<DetectPointInfo>& infos,
                     std::vector<CutPoint>* cutpoints);

  void RefineCutInfos(const Sophus::SE3d& Twv,
                      std::vector<CutPoint>* cutpoints);

  bool DetectSingle2MultiPt(
      const std::vector<std::pair<LaneLine::Ptr, std::vector<Point3D>>>&
          front_lines,
      const std::map<id_t, bool>& pts_behind, std::map<id_t, Point3D>* res);

  bool DetectMulti2SinglePt(
      const std::vector<std::pair<LaneLine::Ptr, std::vector<Point3D>>>&
          front_lines,
      std::map<id_t, Point3D>* res);

  bool FindInfo(const std::vector<DetectPointInfo>& store_infos,
                const DetectPointInfo& info, id_t* index);

  SMStatus SortFromLeft2Right(
      std::vector<std::pair<LaneLine::Ptr, std::vector<Point3D>>>* front_lines);

  bool IsConsistentOfTwoLines(const std::vector<Point3D>& points1,
                              const std::vector<Point3D>& points2);

  bool DoCompleteBrokenCategory(
      const std::vector<LaneLine::Ptr>& lines,
      std::vector<std::vector<LaneLine::Ptr>>* all_classified);

  bool IsGoodLaneTwoLine(const std::vector<Point3D>& segment1,
                         const std::vector<Point3D>& segment2,
                         bool start_detect_mode);

  bool CreateFurtherCategory(
      const std::vector<LinePointsPair>& lines_vehicle_pts, bool detect_mode,
      std::vector<std::vector<LaneLine::Ptr>>* all_classified);

  bool CutWholePath(const std::vector<Sophus::SE3d>& path,
                    std::vector<std::pair<size_t, size_t>>* sub_paths);

  bool GetCategorySubPosePath(
      const std::vector<std::pair<size_t, size_t>>& sub_paths,
      const std::vector<LaneLine::Ptr>& lines,
      std::vector<std::pair<size_t, size_t>>* target_path);

  bool SortSubCategories(
      const Sophus::SE3d& ref_pose,
      const std::vector<LinePointsPair>& input_lines,
      const std::vector<std::vector<LaneLine::Ptr>>& start_cates,
      const std::vector<std::vector<LaneLine::Ptr>>& end_cates,
      std::vector<DetectPointInfo>* infos);

  void GenerateCutPoint(id_t id, id_t main_line_id, id_t target_line_id,
                        const Point3D& point, const CutPointType& type,
                        const Eigen::Vector3d& cutline_dir,
                        const std::vector<id_t>& line_ids,
                        const std::vector<Sophus::SE3d>& pose_path,
                        CutPoint* cut_point_out);

  bool DetectCategories(const std::vector<LinePointsPair>& lines_vehicle_pts,
                        const Sophus::SE3d& pose,
                        std::vector<DetectPointInfo>* detect_point_infos);

  bool DetectEachSubCategories(std::vector<CategoryInfo>* cate_infos);

  bool FindPosePathRegion(
      const std::vector<std::pair<size_t, size_t>>& sub_paths,
      const std::vector<std::vector<LaneLine::Ptr>>& categories_lines,
      std::vector<CategoryInfo>* category_infos);

  bool RefineCateDetectInfos(const std::vector<CategoryInfo>& cate_infos,
                             std::vector<DetectPointInfo>* point_infos,
                             bool use_ref_pose);

  SMStatus ComputeCategories(const std::vector<LaneLine::Ptr>& input_lines,
                             std::vector<CategoryInfo>* cate_infos);

  void FindBrokenStart(
      const std::vector<std::vector<LaneLine::Ptr>>& categories);

  double LinePointDistPath(const Point3D& line_p);

  void TransBrokenStart(const Sophus::SE3d& Twv,
                        std::vector<CutPoint>* cutpoints);

  void TransCtpToVehicle(const Sophus::SE3d& Twv,
                         std::vector<CutPoint>* cutpoints);

  void TransLinesToVehicle(const Sophus::SE3d& Twv,
                           const std::vector<LaneLine::Ptr>& input_lines);

  Eigen::Vector3d GetLineDir(const std::vector<Point3D>& line_points,
                             const int& id, const bool& forward);

  void ReCalculateCutDir(std::vector<CutPoint>* cutpoints);

  void ProcessPath(const std::shared_ptr<std::vector<KinePosePtr>>& path);

  bool CalLineAngle(const Point3D& point1, const Point3D& point2,
                    double& angle1);  // NOLINT
  bool CalBrokenAngle(
      const std::vector<std::vector<LaneLine::Ptr>>& linesvec_broken,  // NOLINT
      std::vector<std::vector<std::pair<double, LaneLine::Ptr>>>&
          linesvec_sort,  // NOLINT
      const bool ret);
  bool CanAddToCluster(
      const std::vector<std::pair<double, LaneLine::Ptr>>& cluster,
      const double element, const double maxdifference);
  bool ClusterData(const std::vector<std::pair<double, LaneLine::Ptr>>& data,
                   const double maxdifference,
                   std::vector<std::vector<std::pair<double, LaneLine::Ptr>>>&
                       clusters);  // NOLINT
  void CalBrokenAveangle(
      const std::vector<std::vector<std::pair<double, LaneLine::Ptr>>>&
          clusters,
      std::vector<std::pair<int, int>>& aveanglevec);  // NOLINT
  void SkipSinglePoint(
      std::vector<std::vector<LaneLine::Ptr>>& linevec,  // NOLINT
      const double& angle, const bool ret);

  template <typename T>
  inline std::string GetElementsIdVec(const std::vector<T>& vec);

  template <typename T>
  inline std::string GetVecIdStr(const std::vector<T>& vec);

 private:
  std::vector<DetectPointInfo> store_detect_infos_;

  double angle_ = 0;
  std::map<id_t, bool> have_pts_behind_veh_;

  std::vector<Sophus::SE3d> whole_pose_path_;

  std::unordered_map<id_t, LaneLine::Ptr> LaneLineTable_;

  std::vector<LaneLine::Ptr> lines_local_;

  std::vector<LaneLine::Ptr> lines_vehicle_;

  std::vector<CutPoint> broken_front_ctps_;

  std::vector<Pose> path_;

  id_t start_index_ = 0;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
