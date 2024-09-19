/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： detect_cut_pt.cc
 *   author     ： mahaijun
 *   date       ： 2024.05
 ******************************************************************************/

#include "modules/map_fusion_02/modules/lane/road_builder/detect_cut_pt.h"

#include <algorithm>
#include <cfloat>
#include <climits>
#include <deque>
#include <limits>
#include <stack>
#include <unordered_map>

#include "base/utils/log.h"

namespace hozon {
namespace mp {
namespace mf {

/*
input:
  path: local系
  input_lines: local系
  Twv: 当前pose到local系
  curr_pose: 当前pose
output：
  cutpoints： vehicle系

*/
SMStatus DetectCutPt::DetectCutPoint(
    const std::shared_ptr<std::vector<KinePosePtr>>& path,
    const Sophus::SE3d& Twv, const std::vector<LaneLine::Ptr>& input_lines,
    const KinePosePtr& curr_pose, std::vector<CutPoint>* cutpoints) {
  HLOG_INFO << "DetectCutPoint!!!!!!";

  if (input_lines.empty()) {
    HLOG_WARN << "input lines is empty";
    return SMStatus::ERROR;
  }

  lines_local_ = input_lines;

  ProcessPath(path);

  whole_pose_path_ = std::vector<Sophus::SE3d>({Twv});

  // delete invalid
  for (auto it = lines_local_.begin(); it != lines_local_.end();) {
    const auto& line = (*it);
    if (line == nullptr || line->GetPoints().size() <= 2) {
      HLOG_ERROR << " have invalid line";
      it = lines_local_.erase(it);
    } else {
      ++it;
    }
  }

  if (lines_local_.size() < 2) {
    HLOG_WARN << "input lines size < 2, no need run whole detect module ";
    return SMStatus::SUCCESS;
  }

  LaneLineTable_.clear();
  for (const auto& line : lines_local_) {
    LaneLineTable_.emplace(line->GetId(), line);
  }

  HLOG_DEBUG << "input valid lines id: " << GetElementsIdVec(lines_local_);

  // compute category infos
  std::vector<CategoryInfo> category_infos;
  if (ComputeCategories(lines_local_, &category_infos) != SMStatus::SUCCESS) {
    HLOG_ERROR << "Compute Categories error ";
  }

  // refine category point infos
  std::vector<DetectPointInfo> new_detect_infos;
  RefineCateDetectInfos(category_infos, &new_detect_infos, true);

  // update pts of line
  if (UnifiedUpdate(new_detect_infos, Twv) != true) {
    HLOG_ERROR << "unified update detect info error";
  }

  // convert lines detect info to CutPoint struct
  GroupCutInfos(store_detect_infos_, cutpoints);

  TransBrokenStart(Twv, cutpoints);
  cutpoints->insert(cutpoints->end(), broken_front_ctps_.begin(),
                    broken_front_ctps_.end());

  TransCtpToVehicle(Twv, cutpoints);

  TransLinesToVehicle(Twv, lines_local_);

  ReCalculateCutDir(cutpoints);

  // debug
  {
    HLOG_INFO << "cut points output info : ";
    for (auto& ctp : *cutpoints) {
      HLOG_INFO << "id: " << ctp.GetId()
                << " type: " << static_cast<char>(ctp.GetType())
                << " point: " << ctp.GetPoint().x << ", " << ctp.GetPoint().y
                << " stamp: " << ctp.GetStamp();
    }
  }

  return SMStatus::SUCCESS;
}

SMStatus DetectCutPt::ComputeCategories(
    const std::vector<LaneLine::Ptr>& input_lines,
    std::vector<CategoryInfo>* cate_infos) {
  if (whole_pose_path_.empty() || input_lines.empty() ||
      cate_infos == nullptr) {
    HLOG_ERROR << "pose path or input lines empty";
    return SMStatus::ERROR;
  }

  // 根据broken 递归分类,得到所有分类结果
  std::vector<std::vector<LaneLine::Ptr>> categories;
  if (DoCompleteBrokenCategory(input_lines, &categories) != true) {
    HLOG_ERROR << "do category error";
    return SMStatus::ERROR;
  }
  HLOG_DEBUG << "categories size: " << categories.size();

  FindBrokenStart(categories);

  std::vector<std::pair<size_t, size_t>> sub_paths;
  if (CutWholePath(whole_pose_path_, &sub_paths) !=
      true) {  // 只有一个位姿，相当于没有切分
    HLOG_ERROR << "cut whole pose path error";
    return SMStatus::ERROR;
  }

  // 对每个分类, 找到首尾path index, and sorted
  FindPosePathRegion(sub_paths, categories, cate_infos);

  if (DetectEachSubCategories(cate_infos) != true) {
    HLOG_ERROR << "detect each sub categories error";
  }

  {
    // debug log
    HLOG_DEBUG << "final sorted categories info size:" << cate_infos->size();
    for (size_t i = 0; i < cate_infos->size(); ++i) {
      const auto& cate_info = cate_infos->at(i);
      const auto& path_region = cate_info.path_region;
      const auto& lines = cate_info.lines;
      HLOG_DEBUG << "\t id: " << i << ",start end index: " << path_region.first
                 << ", " << path_region.second
                 << ", include lines: " << GetElementsIdVec(lines)
                 << ", have sub cutpoints:"
                 << cate_info.detect_point_infos.size();
      for (const auto& pt_info : cate_info.detect_point_infos) {
        HLOG_DEBUG << " \t cutpt type: " << pt_info.point_type
                   << ", on main line id: " << pt_info.source_line_id
                   << ", involve line: "
                   << GetVecIdStr(pt_info.involve_line_ids);
      }
    }
  }
  return SMStatus::SUCCESS;
}

// 寻找path_中与line_p点最近的一个pose，并返回pose的timestamp
double DetectCutPt::LinePointDistPath(const Point3D& line_p) {
  HLOG_DEBUG << "line_p: " << line_p.x << ", " << line_p.y;
  double nearest_time = 0.0;
  double min_dist = DBL_MAX;
  for (auto& pose : path_) {
    double dist = std::sqrt(std::pow((pose.pos.x() - line_p.x), 2) +
                            std::pow((pose.pos.y() - line_p.y), 2));
    // HLOG_DEBUG << "dist: " << dist;
    if (dist < min_dist) {
      nearest_time = pose.stamp;
      min_dist = dist;
    }
  }
  HLOG_DEBUG << "min_dist: " << min_dist;
  return nearest_time;
}

void DetectCutPt::FindBrokenStart(
    const std::vector<std::vector<LaneLine::Ptr>>& categories) {
  if (path_.empty()) {
    return;
  }
  //  从line中找到broken起始点
  if (!broken_front_ctps_.empty()) {
    broken_front_ctps_.clear();
  }
  auto tmpcategories = categories;
  SkipSinglePoint(tmpcategories, 3, false);
  for (auto& cat : tmpcategories) {
    CutPoint broken_fro;
    Point3D min_p;
    id_t min_id = INT_MAX;
    double min_tt = DBL_MAX;
    double t_dist = 0;
    for (auto& line : cat) {
      // local系
      std::vector<Point3D> l_ps = line->GetPoints();
      HLOG_DEBUG << "l_ps[0]: " << l_ps.front().x << ", " << l_ps.front().y;
      t_dist = LinePointDistPath(l_ps.front());
      HLOG_DEBUG << "t_dist: " << t_dist;
      if (t_dist < min_tt) {
        min_p = l_ps.front();
        min_id = line->GetId();
        min_tt = t_dist;
        HLOG_DEBUG << "min_tt: " << min_tt;
      }
    }
    HLOG_DEBUG << "min_p: " << min_p.x << ", " << min_p.y;
    broken_fro.SetId(start_index_);
    broken_fro.SetPoint(min_p);
    broken_fro.SetType(CutPointType::Broken_Start);
    broken_fro.SetStamp(t_dist);
    broken_fro.SetMainLineId(min_id);
    broken_front_ctps_.push_back(broken_fro);
    start_index_++;
  }
}

void DetectCutPt::TransBrokenStart(const Sophus::SE3d& Twv,
                                   std::vector<CutPoint>* cutpoints) {
  // 计算broken起始点位姿转换
  Eigen::Vector3d dir = Twv.so3().matrix().col(0);
  Eigen::Vector3d un_dir = dir.normalized();

  // 切分线朝车头方向的左侧
  Eigen::Matrix3d R =
      Eigen::AngleAxisd(90.0 * M_PI / 180.0, Eigen::Vector3d(0, 0, 1))
          .toRotationMatrix();

  // 切分线垂直于车体行驶方向的方向向量
  Eigen::Vector3d car_dir_ver = R * un_dir;
  Eigen::Vector3d extend_dir = car_dir_ver.normalized();
  for (auto& p : broken_front_ctps_) {
    // 切分线上的点
    Eigen::Vector3d cut_point(p.GetPoint().x, p.GetPoint().y, p.GetPoint().z);
    const double extend_dist = 40.0;
    Eigen::Vector3d start = -extend_dir * extend_dist + cut_point;
    Eigen::Vector3d end = extend_dir * extend_dist + cut_point;

    std::vector<Point3D> cutline_pts = {Point3D(start[0], start[1], start[2]),
                                        Point3D(end[0], end[1], end[2])};
    p.SetExtrePoint(cutline_pts);
  }
}

void DetectCutPt::TransCtpToVehicle(const Sophus::SE3d& Twv,
                                    std::vector<CutPoint>* cutpoints) {
  std::vector<CutPoint> back_ctps, front_ctps;
  // convert cut points from world to vehicle coordinate
  for (auto& ctp : *cutpoints) {
    auto p_local = ctp.GetPoint();
    ctp.SetPoint(Twv.inverse() * ctp.GetPoint());

    if (ctp.GetPoint().x < 0.0) {
      ctp.SetStamp(LinePointDistPath(p_local));
      back_ctps.push_back(ctp);
    } else {
      ctp.SetStamp(0);
      front_ctps.push_back(ctp);
    }
  }
  cutpoints->clear();

  std::sort(
      back_ctps.begin(), back_ctps.end(),
      [](const auto& a, const auto& b) { return a.GetStamp() < b.GetStamp(); });
  std::sort(front_ctps.begin(), front_ctps.end(),
            [](const auto& a, const auto& b) {
              return a.GetPoint().x < b.GetPoint().x;
            });

  // for (int i = 0; i < static_cast<int>(back_ctps.size() - 1); i++) {
  //   auto ctp1 = back_ctps.at(i);
  //   auto ctp2 = back_ctps.at(i + 1);
  //   HLOG_DEBUG << "ctp1 type: " << static_cast<char>(ctp1.GetType());
  //   HLOG_DEBUG << "ctp2 type: " << static_cast<char>(ctp2.GetType());

  //   auto c_d = ctp2.GetPoint() - ctp1.GetPoint();
  //   auto c_sd = ctp2.GetStamp() - ctp1.GetStamp();

  //   HLOG_DEBUG << "ctp1 point: " << ctp1.GetPoint().x << ", "
  //              << ctp1.GetPoint().y;
  //   HLOG_DEBUG << "ctp2 point: " << ctp2.GetPoint().x << ", "
  //              << ctp2.GetPoint().y;
  //   HLOG_DEBUG << "dist: " << c_d.Norm2D();
  //   HLOG_DEBUG << "c_sd: " << c_sd;

  //   if (c_d.Norm2D() < 10.0 || c_sd < 1.0) {
  //     HLOG_DEBUG << "rmv ctp2!!!!!";
  //     back_ctps.erase(back_ctps.begin() + i + 1);
  //   }
  // }
  cutpoints->insert(cutpoints->end(), back_ctps.begin(), back_ctps.end());
  cutpoints->insert(cutpoints->end(), front_ctps.begin(), front_ctps.end());

  for (auto& ctp : *cutpoints) {
    std::vector<Point3D> extrPts = ctp.GetExtrePoint();
    HLOG_DEBUG << "befor  extrPts: " << extrPts.at(0).x << ", "
               << extrPts.at(0).y << ", " << extrPts.at(1).x << ", "
               << extrPts.at(1).y;
    extrPts.at(0) = Twv.inverse() * extrPts.at(0);
    extrPts.at(1) = Twv.inverse() * extrPts.at(1);
    HLOG_DEBUG << "after  extrPts: " << extrPts.at(0).x << ", "
               << extrPts.at(0).y << ", " << extrPts.at(1).x << ", "
               << extrPts.at(1).y;
    ctp.SetExtrePoint(extrPts);
  }
}

void DetectCutPt::TransLinesToVehicle(
    const Sophus::SE3d& Twv, const std::vector<LaneLine::Ptr>& input_lines) {
  if (input_lines.empty()) {
    HLOG_WARN << "lines_local is empty";
    return;
  }

  lines_vehicle_ = input_lines;

  for (auto& l : lines_vehicle_) {
    std::vector<Point3D> points = l->GetPoints();
    for (auto& p : points) {
      p = Twv.inverse() * p;
    }
    l->SetPoints(points);
  }
}

Eigen::Vector3d DetectCutPt::GetLineDir(const std::vector<Point3D>& line_points,
                                        const int& id, const bool& forward) {
  Eigen::Vector3d dir(0.0, 0.0, 0.0);
  int l_size = static_cast<int>(line_points.size());
  if (l_size < 2) {
    return dir;
  }
  const Point3D p = line_points[id];
  Point3D neiber_p(0.0, 0.0, 0.0);
  HLOG_DEBUG << "id: " << id;
  if (forward) {
    for (int i = id + 1; i < l_size - 1; i++) {
      HLOG_DEBUG << " i: " << i << " p: " << p.x << " " << p.y
                 << " l_points[i]: " << line_points[i].x << " "
                 << line_points[i].y
                 << " dist: " << (line_points[i] - p).Norm2D();
      if ((line_points[i] - p).Norm2D() > 5.0) {
        neiber_p = line_points[i];
        HLOG_DEBUG << "find neiber: " << neiber_p.x << " " << neiber_p.y;
        dir = Eigen::Vector3d(neiber_p.x - p.x, neiber_p.y - p.y,
                              neiber_p.z - p.z);
        break;
      } else {
        continue;
      }
    }
    // 如果所有点的距离都小于5，dir没有赋值，则取最远点
    if (dir.norm() < 1e-6) {
      dir = Eigen::Vector3d(line_points.back().x - p.x,
                            line_points.back().y - p.y,
                            line_points.back().z - p.z);
    }
  } else {
    for (int i = id - 1; i > 0; i--) {
      HLOG_DEBUG << " i: " << i << " p: " << p.x << " " << p.y
                 << " l_points[i]: " << line_points[i].x << " "
                 << line_points[i].y
                 << " dist: " << (line_points[i] - p).Norm2D();
      if ((line_points[i] - p).Norm2D() > 5.0) {
        neiber_p = line_points[i];
        HLOG_DEBUG << "find neiber: " << neiber_p.x << " " << neiber_p.y;
        dir = Eigen::Vector3d(p.x - neiber_p.x, p.y - neiber_p.y,
                              p.z - neiber_p.z);
        break;
      } else {
        continue;
      }
    }
    if (dir.norm() < 1e-6) {
      dir = Eigen::Vector3d(p.x - line_points.front().x,
                            p.y - line_points.front().y,
                            p.z - line_points.front().z);
    }
  }
  return dir;
}

// 重新计算切分线方向
void DetectCutPt::ReCalculateCutDir(std::vector<CutPoint>* cutpoints) {
  for (auto& ctp : *cutpoints) {
    int idx_l = 0, idx = 0, neiber_idx = 0;
    const Point3D p = ctp.GetPoint();
    auto ctp_type = ctp.GetType();
    HLOG_DEBUG << "p: " << p.x << ", " << p.y
               << " ctp_type: " << static_cast<char>(ctp_type);
    // 不处理车前的broken点
    // if (p.x > 0 && ctp_type == CutPointType::Broken) {
    //   continue;
    // }
    // 不处理车前的点
    // if (p.x > 0) {
    //   continue;
    // }

    if (ctp_type == CutPointType::Merge || ctp_type == CutPointType::Split) {
      idx_l = static_cast<int>(ctp.GetTargetLineId());
    } else {
      idx_l = static_cast<int>(ctp.GetMainLineId());
    }

    HLOG_DEBUG << "idx_l: " << idx_l;
    std::vector<Point3D> l_points;
    for (auto l : lines_vehicle_) {
      if (idx_l == l->GetId()) {
        l_points = l->GetPoints();
      }
    }

    int l_size = l_points.size();
    HLOG_DEBUG << "l_size: " << l_size;
    if (l_size < 2) {
      continue;
    }

    Eigen::Vector3d dir(0.0, 0.0, 0.0);
    // split 和 merge 点对应的计算切线方向点在target
    // line上，找距离最近的点对应的索引 i_pt
    if (ctp_type == CutPointType::Merge || ctp_type == CutPointType::Split) {
      HLOG_DEBUG << "split or merge";
      double d_min = DBL_MAX;
      int i_pt = -1;
      for (int i = 0; i < l_size - 1; i++) {
        auto l_pt = l_points[i];
        if ((l_pt - p).Norm2D() < d_min) {
          d_min = (l_pt - p).Norm2D();
          i_pt = i;
        } else {
          continue;
        }
      }
      dir = GetLineDir(l_points, i_pt, true);
    } else {  // 其他类型点
      auto ret_p = std::find(l_points.begin(), l_points.end(), p);
      if (ret_p != l_points.end()) {
        idx = ret_p - l_points.begin();
        HLOG_DEBUG << "other idx: " << idx;
      } else {
        HLOG_DEBUG << "not find, search nearest point";
        double temp_min = DBL_MAX;
        int t_mi = -1;
        for (int i = 0; i < l_size - 1; i++) {
          auto l_pt = l_points[i];
          if ((l_pt - p).Norm2D() < temp_min) {
            temp_min = (l_pt - p).Norm2D();
            t_mi = i;
          }
        }
        idx = t_mi;
      }
      HLOG_DEBUG << "l_point idx: " << idx << " p: " << l_points[idx].x << " "
                 << l_points[idx].y;

      // 少变多从前往后找
      if (ctp_type == CutPointType::Single2Multi) {
        HLOG_DEBUG << "Single2Multi";
        dir = GetLineDir(l_points, idx, true);
      } else if (ctp_type == CutPointType::Multi2Single) {
        // 多变少是最后一个点，需要从后往前找
        HLOG_DEBUG << "Multi2Single";
        dir = GetLineDir(l_points, idx, false);
      } else if (ctp_type == CutPointType::Broken_Start) {
        // Broken start 从前往后
        HLOG_DEBUG << "broken start";
        dir = GetLineDir(l_points, idx, true);
      } else if (ctp_type == CutPointType::Broken) {
        // Broken end 从后往前
        HLOG_DEBUG << "broken end";
        dir = GetLineDir(l_points, idx, false);
      }
    }
    if (dir != Eigen::Vector3d(0.0, 0.0, 0.0)) {
      Eigen::Vector3d dir_n = dir.normalized();
      HLOG_DEBUG << "dir: " << dir.x() << " " << dir.y() << " " << dir.z();
      HLOG_DEBUG << "dir_n: " << dir_n.x() << " " << dir_n.y() << " "
                 << dir_n.z();

      Eigen::Matrix3d R =
          Eigen::AngleAxisd(90.0 * M_PI / 180.0, Eigen::Vector3d(0, 0, 1))
              .toRotationMatrix();
      Eigen::Vector3d extend_dir = (R * dir_n).normalized();

      Eigen::Vector3d cut_point(p.x, p.y, p.z);
      const double extend_dist = 40.0;
      Eigen::Vector3d start = -extend_dir * extend_dist + cut_point;
      Eigen::Vector3d end = extend_dir * extend_dist + cut_point;

      std::vector<Point3D> cutline_pts = {Point3D(start[0], start[1], start[2]),
                                          Point3D(end[0], end[1], end[2])};

      ctp.SetParams(extend_dir);
      ctp.SetExtrePoint(cutline_pts);
    }
  }
}

void DetectCutPt::ProcessPath(
    const std::shared_ptr<std::vector<KinePosePtr>>& path) {
  // local系
  for (auto& p : *path) {
    path_.push_back(*p);
  }
}

float64_t TestDistPoint2LineDirection(const Point3D& point,
                                      const Point3D& line_point,
                                      const Eigen::Vector3d& line_dir) {
  Point3D line_direction = Point3D(line_dir[0], line_dir[1], line_dir[2]);
  if (line_direction.Norm() < 1e-8) {
    return (point - line_point).Norm();
  }
  double dist = std::sqrt(line_direction.CrossSqr(point - line_point)) /
                line_direction.Norm();
  return std::fabs(dist);
}

bool DetectCutPt::RefineCateDetectInfos(
    const std::vector<CategoryInfo>& cate_infos,
    std::vector<DetectPointInfo>* point_infos_out, bool use_ref_pose) {
  // group cut point out
  for (size_t i = 0; i < cate_infos.size(); ++i) {
    const auto& cate_info = cate_infos[i];
    std::pair<size_t, size_t> path_region = cate_info.path_region;

    const auto& ref_pose = whole_pose_path_.at(path_region.second);
    Eigen::Vector3d ref_pose_trans = ref_pose.translation();
    Point3D ref_pose_position =
        Point3D(ref_pose_trans[0], ref_pose_trans[1], ref_pose_trans[2]);
    std::vector<DetectPointInfo> point_infos;
    point_infos.assign(cate_info.detect_point_infos.begin(),
                       cate_info.detect_point_infos.end());

    point_infos.push_back(cate_info.main_point_info);

    // same category have too cloest pt
    std::reverse(point_infos.begin(), point_infos.end());
    if (point_infos.size() >= 2) {
      for (auto it = point_infos.begin() + 1; it != point_infos.end();) {
        auto pre_it = it - 1;

        double dist = TestDistPoint2LineDirection(
            it->world_pt, pre_it->world_pt, pre_it->cut_line_dir);

        HLOG_DEBUG << "\t\t pre to current dist: " << dist;

        if (dist < 5.0) {
          if (pre_it == point_infos.begin()) {
            // 优先 keep the first pt
            it = point_infos.erase(it);
            continue;
          }

          Point3D pos;
          if (use_ref_pose) {
            // use the latest Twv position
            pos = ref_pose_position;
          } else {
            // use the first pt postion (closest end junction)
            point_infos.front().world_pt;
          }
          double dist1 = TestDistPoint2LineDirection(pos, pre_it->world_pt,
                                                     pre_it->cut_line_dir);
          double dist2 =
              TestDistPoint2LineDirection(pos, it->world_pt, it->cut_line_dir);
          if (dist1 < dist2) {
            it = point_infos.erase(it);
          } else {
            it = point_infos.erase(pre_it);
            ++it;
          }
        } else {
          ++it;
        }
      }
    }
    HLOG_DEBUG << "\t after delete all cutpoints size:" << point_infos.size();
    for (const auto& pt_info : point_infos) {
      HLOG_DEBUG << " \t cutpt type: " << pt_info.point_type
                 << ", on main line id: " << pt_info.source_line_id
                 << ", involve line: " << GetVecIdStr(pt_info.involve_line_ids);
    }
    if (!point_infos.empty()) {
      // reverse again
      std::reverse(point_infos.begin(), point_infos.end());
      point_infos_out->insert((*point_infos_out).end(), point_infos.begin(),
                              point_infos.end());
    }
  }
  return true;
}

bool DetectCutPt::FindInfo(const std::vector<DetectPointInfo>& store_infos,
                           const DetectPointInfo& info, id_t* index) {
  // find info function
  auto iter = std::find_if(
      store_infos.begin(), store_infos.end(), [&info](const auto& store_info) {
        if (info.point_type == 1 || info.point_type == 2 ||
            info.point_type == 5 || info.point_type == 6) {
          // 分合流
          if (info.point_type == store_info.point_type) {
            // 一种type的点只有一个
            if ((info.source_line_id == store_info.source_line_id &&
                 info.target_line_id == store_info.target_line_id) ||
                (info.source_line_id == store_info.target_line_id &&
                 info.target_line_id == store_info.source_line_id)) {
              return true;
            }
          }

        } else if (info.point_type == 3 || info.point_type == 4 ||
                   info.point_type == 7) {
          // 少变多,多变少,broken
          if (info.source_line_id == store_info.source_line_id &&
              info.point_type == store_info.point_type) {
            return true;
          }
        }
        return false;
      });
  if (iter == store_infos.end()) {
    return false;
  }
  *index = std::distance(store_infos.begin(), iter);
  return true;
}

bool DetectCutPt::UnifiedUpdate(
    const std::vector<DetectPointInfo>& new_detect_infos,
    const Sophus::SE3d& Twv) {
  const Sophus::SE3d Tvw = Twv.inverse();

  // 多变少，少变多, broken 需要每次重新计算, 删除旧的
  for (auto iter = store_detect_infos_.begin();
       iter != store_detect_infos_.end();) {
    auto info = *iter;
    if (info.point_type == 1 || info.point_type == 2 || info.point_type == 3 ||
        info.point_type == 4 || info.point_type == 7) {
      iter = store_detect_infos_.erase(iter);
    } else {
      ++iter;
    }
  }

  HLOG_DEBUG << " new detect info size: " << new_detect_infos.size();

  // add the new infos to store
  for (size_t i = 0; i < new_detect_infos.size(); ++i) {
    auto info = new_detect_infos[i];
    if (info.point_type == 0) {
      HLOG_ERROR << "the new detect info point type error";
      continue;
    }
    Point3D pt_v = Tvw * info.world_pt;
    Eigen::Vector3d pt_v_eigen = Eigen::Vector3d(pt_v.x, pt_v.y, pt_v.z);
    HLOG_DEBUG << " \t cut point in line :" << info.source_line_id
               << ", target is " << info.target_line_id
               << ", type : " << static_cast<int>(info.point_type);

    id_t index = 0;
    // 每次进来store_detect_infos_ 的size为0
    if (FindInfo(store_detect_infos_, info, &index) == true) {
      // front vehicle pt
      if ((Tvw * info.world_pt).x < 0) {
        continue;
      }
      HLOG_DEBUG << "find info, index is: " << index;
      // update
      store_detect_infos_.at(index).source_line_id = info.source_line_id;
      store_detect_infos_.at(index).target_line_id = info.target_line_id;
      store_detect_infos_.at(index).world_pt = info.world_pt;
      store_detect_infos_.at(index).detect_times++;
    } else {
      HLOG_DEBUG << "not find, need add ";
      info.detect_times = 1;
      store_detect_infos_.push_back(std::move(info));
    }
  }

  // delete the infos if line is invaild   应该是不会删的
  for (auto iter = store_detect_infos_.begin();
       iter != store_detect_infos_.end();) {
    auto info = *iter;
    if (info.point_type == 1 || info.point_type == 2 || info.point_type == 5 ||
        info.point_type == 6) {
      if ((LaneLineTable_[info.source_line_id] == nullptr) ||
          (LaneLineTable_[info.target_line_id] == nullptr)) {
        iter = store_detect_infos_.erase(iter);
      } else {
        ++iter;
      }
    } else {
      if (LaneLineTable_[info.source_line_id] == nullptr) {
        iter = store_detect_infos_.erase(iter);
      } else {
        ++iter;
      }
    }
  }

  // add the final infos to map line ptr
  std::map<id_t, std::vector<JunctionPoint>> line_juncpts_map;
  for (const auto& info : store_detect_infos_) {
    JunctionPoint junction_point;

    if (info.point_type == 1) {
      junction_point.point_type = JunctionPtType::SPLIT;
    } else if (info.point_type == 2) {
      junction_point.point_type = JunctionPtType::MERGE;
    } else if (info.point_type == 3) {
      junction_point.point_type = JunctionPtType::Multi2Single;
    } else if (info.point_type == 4) {
      junction_point.point_type = JunctionPtType::Single2Multi;
    } else if (info.point_type == 5) {
      junction_point.point_type = JunctionPtType::V_Shaped;
    } else if (info.point_type == 6) {
      junction_point.point_type = JunctionPtType::V_Shaped_Inv;
    } else if (info.point_type == 7) {
      junction_point.point_type = JunctionPtType::BROKEN;
    } else if (info.point_type == 8) {
      junction_point.point_type = JunctionPtType::Broken_Start;
    } else {
      HLOG_ERROR << "detect info point type error:" << info.point_type;
      continue;
    }
    junction_point.source_line_id = info.source_line_id;
    junction_point.target_line_id = info.target_line_id;
    junction_point.world_pt = info.world_pt;

    if (line_juncpts_map.count(info.source_line_id) == 0) {
      // init
      line_juncpts_map[info.source_line_id] = {junction_point};
    } else {
      // add
      line_juncpts_map[info.source_line_id].push_back(junction_point);
    }
  }

  // clear 现有的
  std::vector<LaneLine::Ptr> map_lines;
  map_lines.reserve(LaneLineTable_.size());
  for (const auto& item : LaneLineTable_) {
    map_lines.push_back(item.second);
  }

  for (auto line : map_lines) {
    if (line == nullptr) {
      continue;
    }
    line->SetJunctionPoints(std::vector<JunctionPoint>());
  }
  for (const auto& line_juncpts : line_juncpts_map) {
    auto line = LaneLineTable_[line_juncpts.first];
    if (line == nullptr) {
      continue;
    }
    // set line junction pts
    line->SetJunctionPoints(line_juncpts.second);
  }
  return true;
}

void DetectCutPt::GenerateCutPoint(id_t id, id_t main_line_id,
                                   id_t target_line_id, const Point3D& point,
                                   const CutPointType& type,
                                   const Eigen::Vector3d& cutline_dir,
                                   const std::vector<id_t>& line_ids,
                                   const std::vector<Sophus::SE3d>& pose_path,
                                   CutPoint* cut_point_out) {
  Eigen::Vector3d extend_dir = cutline_dir.normalized();

  // 切分线上的点
  Eigen::Vector3d cut_point(point.x, point.y, point.z);
  const double extend_dist = 40.0;
  Eigen::Vector3d start = -extend_dir * extend_dist + cut_point;
  Eigen::Vector3d end = extend_dir * extend_dist + cut_point;

  std::vector<Point3D> cutline_pts = {Point3D(start[0], start[1], start[2]),
                                      Point3D(end[0], end[1], end[2])};

  CutPoint cut_pt;
  cut_pt.SetId(id);  // set id after sorted
  cut_pt.SetMainLineId(main_line_id);
  cut_pt.SetTargetLineId(target_line_id);
  cut_pt.SetPoint(point);
  cut_pt.SetType(type);
  cut_pt.SetParams(extend_dir);
  cut_pt.SetExtrePoint(cutline_pts);
  cut_pt.SetSubPosePath(pose_path);
  cut_pt.SetLineIds(line_ids);
  (*cut_point_out) = std::move(cut_pt);
}

void DetectCutPt::GroupCutInfos(const std::vector<DetectPointInfo>& infos,
                                std::vector<CutPoint>* cutpoints) {
  // size_t start_index = 1;
  for (const auto& info : infos) {
    CutPoint cut_pt;
    CutPointType cut_pt_type;
    if (info.point_type == 1) {
      cut_pt_type = CutPointType::Split;
    } else if (info.point_type == 2) {
      cut_pt_type = CutPointType::Merge;
    } else if (info.point_type == 3) {
      // start, is single 2 multi
      cut_pt_type = CutPointType::Single2Multi;
    } else if (info.point_type == 4) {
      cut_pt_type = CutPointType::Multi2Single;
    } else if (info.point_type == 5) {
      cut_pt_type = CutPointType::V_Shaped;
    } else if (info.point_type == 6) {
      cut_pt_type = CutPointType::V_Shaped_Inv;
    } else if (info.point_type == 7) {
      cut_pt_type = CutPointType::Broken;
    } else {
      HLOG_ERROR << " input cut type may error, info type: " << info.point_type;
      continue;
    }

    GenerateCutPoint(start_index_, info.source_line_id, info.target_line_id,
                     info.world_pt, cut_pt_type, info.cut_line_dir,
                     info.involve_line_ids, info.ref_poses, &cut_pt);
    cutpoints->emplace_back(std::move(cut_pt));

    start_index_++;
  }
}

bool DetectCutPt::DoCompleteBrokenCategory(
    const std::vector<LaneLine::Ptr>& lines,
    std::vector<std::vector<LaneLine::Ptr>>* all_classified) {
  if (lines.size() <= 1) {
    // whole lines size at least 2
    HLOG_ERROR << "input lines too less to category, size: " << lines.size();
    return false;
  }

  std::stack<std::vector<LaneLine::Ptr>> data_stack;
  data_stack.push(lines);

  size_t loop_cnt = 0;
  while (!data_stack.empty()) {
    const std::vector<LaneLine::Ptr> loop_input_lines = data_stack.top();
    data_stack.pop();

    if (loop_input_lines.size() == 0) {
      continue;
    }
    HLOG_DEBUG << "this loop lines : " << GetElementsIdVec(loop_input_lines);

    std::vector<LaneLine::Ptr> input_lines = loop_input_lines;
    std::vector<LaneLine::Ptr> classified_lines, other_lines;
    for (auto it = input_lines.begin(); it != input_lines.end() - 1;) {
      const auto& line1 = (*it);
      if (line1 == nullptr || line1->GetPoints().size() <= 2) {
        ++it;
        continue;
      }
      HLOG_DEBUG << " select line " << line1->GetId();
      HLOG_DEBUG << " select line " << line1->GetType();
      HLOG_DEBUG << " select line " << line1->GetPoints().size();
      HLOG_DEBUG << " start point " << line1->GetPoints().front().x << ", "
                 << line1->GetPoints().front().y;
      HLOG_DEBUG << " end point " << line1->GetPoints().back().x << ", "
                 << line1->GetPoints().back().y;
      classified_lines = std::vector<LaneLine::Ptr>({line1});
      other_lines.clear();

      for (auto next_it = it + 1; next_it != input_lines.end();) {
        const auto& line2 = (*next_it);
        HLOG_DEBUG << " select line2 " << line2->GetId();
        HLOG_DEBUG << " select line2 " << line2->GetType();
        HLOG_DEBUG << " select line2 " << line2->GetPoints().size();
        HLOG_DEBUG << " start point " << line2->GetPoints().front().x << ", "
                   << line2->GetPoints().front().y;
        HLOG_DEBUG << " end point " << line2->GetPoints().back().x << ", "
                   << line2->GetPoints().back().y;
        if (line2 == nullptr || line2->GetPoints().size() <= 2) {
          ++next_it;
          continue;
        }

        if (IsConsistentOfTwoLines(line1->GetPoints(), line2->GetPoints()) ==
            true) {
          classified_lines.push_back(line2);  // 同类

          HLOG_DEBUG << "\tline " << line1->GetId() << ", consist with "
                     << line2->GetId();
        } else {
          HLOG_DEBUG << "\tline " << line1->GetId() << ", not consist with "
                     << line2->GetId();
          other_lines.push_back(line2);
        }
        ++next_it;
      }
      break;
    }
    if (other_lines.empty() && input_lines.size() == 1) {
      // 只剩最后一根了, 说明此次loop线都是同一类
      classified_lines = loop_input_lines;
    }

    for (auto it = other_lines.begin(); it != other_lines.end();) {
      bool add_status = false;
      for (const auto& classified_line : classified_lines) {
        if (IsConsistentOfTwoLines((*it)->GetPoints(),
                                   classified_line->GetPoints()) == true) {
          // add
          classified_lines.push_back(*it);
          add_status = true;
          break;
        }
      }
      if (add_status) {
        // delete
        it = other_lines.erase(it);
      } else {
        ++it;
      }
    }

    if (!classified_lines.empty()) {
      HLOG_DEBUG << "\t classified lines: "
                 << GetElementsIdVec(classified_lines);
      all_classified->push_back(classified_lines);
    }

    if (other_lines.size() >= 1) {
      data_stack.push(other_lines);
    }

    loop_cnt++;
    if (loop_cnt > 10) {
      HLOG_ERROR << "have broken road detect over 10 times, may error";
      return false;
    }
  }

  {
    // debug log
    HLOG_DEBUG << "all categories size: " << all_classified->size();
    for (size_t i = 0; i < all_classified->size(); ++i) {
      HLOG_DEBUG << "\t cate: " << i << ", include lines: "
                 << GetElementsIdVec(all_classified->at(i));
    }
  }
  return true;
}

bool DetectCutPt::CutWholePath(
    const std::vector<Sophus::SE3d>& path,
    std::vector<std::pair<size_t, size_t>>* sub_paths) {
  // 对整个轨迹,进行一段一段切分
  // 如果上一段方向 与 本次方向, 反向,则切分
  if (path.empty()) {
    return false;
  }
  std::vector<std::pair<size_t, size_t>> index_pair_vec;
  if (path.size() <= 2) {
    index_pair_vec.push_back(std::make_pair(0, path.size() - 1));
  } else {
    const float angle_thr = 5.0;  // angle
    const float dist_thr = 5.0;

    for (size_t i = 0; i < path.size() - 1;) {
      size_t end_index = i + 1;
      bool status = false;
      for (size_t j = i + 1; j < path.size() - 1; ++j) {
        Eigen::Vector3d p1 = path[j - 1].translation();
        Eigen::Vector3d p2 = path[j].translation();
        Eigen::Vector3d p3 = path[j + 1].translation();
        if ((p3 - p1).norm() * (p2 - p1).norm() < 1e-6) {
          continue;
        }
        double angle = std::acos((p3 - p1).dot(p2 - p1) /
                                 ((p3 - p1).norm() * (p2 - p1).norm())) *
                       180 / M_PI;
        if (angle < angle_thr &&
            std::fabs((p3 - p2).norm() - (p2 - p1).norm()) < dist_thr) {
          continue;
        }
        status = true;  // found
        end_index = j;
        break;
      }

      if (status == true) {
        index_pair_vec.push_back(std::make_pair(i, end_index));
        i = end_index;
      } else {
        index_pair_vec.push_back(std::make_pair(i, path.size() - 1));
        break;
      }
    }
  }
  (*sub_paths) = index_pair_vec;

  {
    // debug log
    HLOG_DEBUG << "sub_paths size:" << sub_paths->size();
    for (size_t i = 0; i < sub_paths->size(); ++i) {
      HLOG_DEBUG << "\t id: " << i
                 << ",start end index: " << sub_paths->at(i).first << ", "
                 << sub_paths->at(i).second;
    }
  }
  return true;
}

bool DetectCutPt::FindPosePathRegion(
    const std::vector<std::pair<size_t, size_t>>& sub_paths,
    const std::vector<std::vector<LaneLine::Ptr>>& categories_lines,
    std::vector<CategoryInfo>* category_infos) {
  for (size_t i = 0; i < categories_lines.size(); ++i) {
    const auto& cate_lines = categories_lines[i];
    if (cate_lines.empty()) {
      HLOG_ERROR << "category lines empty";
      continue;
    }
    if (whole_pose_path_.size() == 1) {  //! 只进这里
      CategoryInfo cate_info;
      cate_info.path_region = std::make_pair(0, 0);
      cate_info.lines = cate_lines;
      category_infos->emplace_back(std::move(cate_info));
      continue;
    }
  }
  return true;
}

bool DetectCutPt::DetectEachSubCategories(
    std::vector<CategoryInfo>* cate_infos) {
  for (size_t i = 0; i < cate_infos->size(); ++i) {
    const auto& sub_pose_region = cate_infos->at(i).path_region;
    const auto& sub_category_lines = cate_infos->at(i).lines;

    const auto& pose = whole_pose_path_.at(sub_pose_region.second);

    // need convert to vehicle coord for detect
    const auto& cate_Tvw = pose.inverse();
    std::vector<LinePointsPair> lines_vehicle_pts;
    for (const auto& line : sub_category_lines) {
      if (line == nullptr || line->GetPoints().size() < 2) {
        continue;
      }
      // 又转到车体系了
      std::vector<Point3D> vehicle_pts;
      std::transform(line->GetPoints().begin(), line->GetPoints().end(),
                     std::back_inserter(vehicle_pts),
                     [cate_Tvw](const Point3D& pt) {
                       Point3D tmp = cate_Tvw * pt;
                       tmp.z = 0;
                       return tmp;
                     });
      lines_vehicle_pts.push_back(std::make_pair(line, vehicle_pts));
    }
    HLOG_DEBUG << " current cate id: " << i;

    std::vector<DetectPointInfo> point_infos;
    if (DetectCategories(lines_vehicle_pts, pose, &point_infos) != true) {
      HLOG_ERROR << "detect categories cut points error";
      continue;
    }

    // is sorted already
    cate_infos->at(i).detect_point_infos = point_infos;
  }
  if (cate_infos->size() >= 1) {
    // 根据尾部pose的方向向量, create broken cutpoint
    for (size_t i = 0; i < cate_infos->size(); ++i) {
      const auto& cate_info = cate_infos->at(i);
      std::pair<size_t, size_t> path_region = cate_info.path_region;
      const auto& cate_lines = cate_info.lines;
      if (path_region.second < 0 ||
          path_region.second > whole_pose_path_.size() - 1) {
        HLOG_ERROR << "invalid regin value: " << path_region.second;
        return false;
      }
      const auto& ref_pose = whole_pose_path_.at(path_region.second);
      Eigen::Vector3d dir = ref_pose.so3().matrix().col(0);

      Eigen::Vector3d un_dir = dir.normalized();
      auto ref_dir = Point3D(un_dir[0], un_dir[1], un_dir[2]);

      // 对cate lines 进行排序
      auto input_lines = cate_lines;
      auto Compare = [&](const auto& line1, const auto& line2) {
        Point3D p1 = line1->GetPoints().back();
        Point3D p2 = line2->GetPoints().back();
        return p1.Dot(ref_dir) < p2.Dot(ref_dir);
      };
      std::sort(input_lines.begin(), input_lines.end(), Compare);

      // 尾部切分点 (最靠前的线最后一个点)
      std::vector<std::vector<LaneLine::Ptr>> tmpvecLines;
      tmpvecLines.push_back(input_lines);
      SkipSinglePoint(tmpvecLines, 3, true);
      Point3D pt = tmpvecLines.front().back()->GetPoints().back();

      // 待切分线的lines id
      std::vector<id_t> line_ids;
      for (const auto& line : tmpvecLines.front()) {
        if (line == nullptr) continue;
        line_ids.push_back(line->GetId());
      }

      // 切分线朝车头方向的左侧
      Eigen::Matrix3d R =
          Eigen::AngleAxisd(90.0 * M_PI / 180.0, Eigen::Vector3d(0, 0, 1))
              .toRotationMatrix();

      // 切分线垂直于车体行驶方向的方向向量
      Eigen::Vector3d car_dir_ver = R * un_dir;

      // set sub pose path
      std::vector<Sophus::SE3d> sub_path_poses(
          whole_pose_path_.begin() + path_region.first,
          whole_pose_path_.begin() + path_region.second + 1);

      DetectPointInfo detect_info;
      detect_info.world_pt = pt;
      detect_info.point_type = 7;  // broken point
      detect_info.source_line_id = tmpvecLines.front().back()->GetId();
      detect_info.involve_line_ids = line_ids;
      detect_info.ref_poses = sub_path_poses;
      detect_info.cut_line_dir = car_dir_ver;

      cate_infos->at(i).main_point_info = std::move(detect_info);
    }
  }

  return true;
}

bool DetectCutPt::CalBrokenAngle(
    const std::vector<std::vector<LaneLine::Ptr>>& linesvec_broken,  // NOLINT
    std::vector<std::vector<std::pair<double, LaneLine::Ptr>>>& linesvec_sort,
    const bool ret) {
  if (linesvec_broken.empty()) {
    return false;
  }
  linesvec_sort.resize(linesvec_broken.size());
  for (size_t i = 0; i < linesvec_broken.size(); i++) {
    if (linesvec_broken[i].empty()) {
      return false;
    }
    std::vector<std::pair<double, LaneLine::Ptr>> linevec_sort;
    linevec_sort.resize(linesvec_broken[i].size());
    double angle_vec = 0.0;
    for (size_t j = 0; j < linesvec_broken[i].size(); j++) {
      LaneLine::Ptr line = linesvec_broken[i][j];
      if (line->GetPoints().size() < 2) {
        return false;
      }
      double angle = 0.0;
      Point3D direction_v1;
      Point3D vec_dir(1, 0, 0);
      const int angle_value = 5;
      // 如果end优先执行
      if (ret) {
        if (line->GetPoints().size() > angle_value) {
          direction_v1 = line->GetPoints().back() -
                         *std::prev(line->GetPoints().end(), angle_value);
        } else {
          direction_v1 = line->GetPoints().back() - line->GetPoints().front();
        }
      } else {
        if (line->GetPoints().size() > angle_value) {
          direction_v1 =
              line->GetPoints()[angle_value] - line->GetPoints().front();
        } else {
          direction_v1 = line->GetPoints().back() - line->GetPoints().front();
        }
      }
      if (CalLineAngle(direction_v1, vec_dir, angle)) {
        linevec_sort[j] = std::make_pair(angle, line);
      } else {
        linevec_sort[j] = std::make_pair(-DBL_MAX, line);
      }
    }
    linesvec_sort[i] = linevec_sort;
  }
  return true;
}

bool DetectCutPt::CalLineAngle(const Point3D& point1, const Point3D& point2,
                               double& angle1) {
  if ((0 == point1.Norm()) || (0 == point2.Norm())) {
    return false;
  }
  double value_cos = const_cast<Point3D&>(point1).Dot(point2) /
                     (point1.Norm() * point2.Norm());
  if (std::fabs(value_cos) > 1.0) {
    return false;
  }
  angle1 = std::acos(value_cos) * 180.0 / M_PI;
  return true;
}

bool DetectCutPt::CanAddToCluster(
    const std::vector<std::pair<double, LaneLine::Ptr>>& cluster,
    const double element, const double maxdifference) {
  for (const auto& value : cluster) {
    if (std::fabs(value.first - element) > maxdifference) {
      return false;
    }
  }
  return true;
}

bool DetectCutPt::ClusterData(
    const std::vector<std::pair<double, LaneLine::Ptr>>& data,
    const double maxdifference,
    std::vector<std::vector<std::pair<double, LaneLine::Ptr>>>& clusters) {
  if (data.size() < 2) return false;
  for (const auto element : data) {
    bool added = false;
    // 试将元素加入已有的聚类,当满足多个簇时，优先聚类在前面的簇中
    for (auto& cluster : clusters) {
      if (CanAddToCluster(cluster, element.first, maxdifference)) {
        cluster.emplace_back(element);
        added = true;
        break;
      }
    }
    // 如果元素未能加入任何已有的聚类，则创建一个新的聚类
    if (!added) {
      clusters.push_back(
          std::vector<std::pair<double, LaneLine::Ptr>>{element});
    }
  }

  return true;
}
void DetectCutPt::CalBrokenAveangle(
    const std::vector<std::vector<std::pair<double, LaneLine::Ptr>>>& clusters,
    std::vector<std::pair<int, int>>& aveanglevec) {
  for (size_t i = 0; i < clusters.size(); i++) {
    int aveangle = 0;
    aveangle = static_cast<int>(clusters[i].size());
    // 索引从1开始
    aveanglevec.emplace_back(std::make_pair(i + 1, aveangle));
  }
  std::sort(aveanglevec.begin(), aveanglevec.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
              return a.second > b.second;
            });
}
void DetectCutPt::SkipSinglePoint(
    std::vector<std::vector<LaneLine::Ptr>>& linevec,  // NOLINT
    const double& angle, const bool ret) {
  std::vector<std::vector<std::pair<double, LaneLine::Ptr>>> linesvec_sort;
  // 获取broken角度信息linesvec_sort
  bool angle_mask = CalBrokenAngle(linevec, linesvec_sort, ret);
  if (angle_mask) {
    for (size_t i = 0; i < linesvec_sort.size(); i++) {
      // deal with single broken-lines
      std::vector<std::vector<std::pair<double, LaneLine::Ptr>>> clusters;
      std::vector<std::pair<int, int>> aveanglevec;
      ClusterData(linesvec_sort[i], angle,
                  clusters);  // 聚类
      if (!clusters.empty()) {
        // 统计聚类元素的avgsize
        CalBrokenAveangle(clusters, aveanglevec);
        // 单个broken删除离散点
        if (aveanglevec.size() > 1) {
          std::unordered_set<int> delete_ids;
          for (auto it = aveanglevec.begin() + 1; it != aveanglevec.end();
               it++) {
            int index = -1;
            index = it->first;
            if (index > 0 && index <= clusters.size()) {
              for (size_t m = 0; m < clusters[index - 1].size(); m++) {
                delete_ids.insert(clusters[index - 1][m].second->GetId());
              }
            }
          }

          // 删除匹配ID的LaneLine
          linevec[i].erase(
              std::remove_if(linevec[i].begin(), linevec[i].end(),
                             [&delete_ids](const LaneLine::Ptr& line) {
                               return delete_ids.count(line->GetId()) > 0;
                             }),
              linevec[i].end());
        }
      }
    }
  }
}
bool DetectCutPt::DetectCategories(
    const std::vector<LinePointsPair>& lines_vehicle_pts,
    const Sophus::SE3d& pose,
    std::vector<DetectPointInfo>* detect_point_infos) {
  std::vector<DetectPointInfo> split_merge_infos;

  // detect split, merge

  if (DetectSplitMergePt(lines_vehicle_pts, pose, &split_merge_infos) != true) {
    HLOG_ERROR << "Detect new Split Merge Pt error";
  }

  // start point
  std::vector<std::vector<LaneLine::Ptr>> start_cates;
  CreateFurtherCategory(lines_vehicle_pts, true, &start_cates);

  // end point
  std::vector<std::vector<LaneLine::Ptr>> end_cates;
  CreateFurtherCategory(lines_vehicle_pts, false, &end_cates);

  {
    HLOG_DEBUG << "\t start_cates size: " << start_cates.size();
    for (const auto& cate : start_cates) {
      HLOG_DEBUG << "\t\t cate: " << GetElementsIdVec(cate);
    }
    HLOG_DEBUG << "\t end_cates size: " << end_cates.size();
    for (const auto& cate : end_cates) {
      HLOG_DEBUG << "\t\t cate: " << GetElementsIdVec(cate);
    }
  }

  std::vector<DetectPointInfo> potential_infos;
  if (start_cates.size() >= 2 || end_cates.size() >= 2) {
    // get start end cut point
    if (SortSubCategories(pose, lines_vehicle_pts, start_cates, end_cates,
                          &potential_infos) != true) {
      HLOG_ERROR << "this sub category sort error";
    }
  }

  std::vector<DetectPointInfo> final_infos;
  final_infos.assign(split_merge_infos.begin(), split_merge_infos.end());

  if (!potential_infos.empty()) {
    // delete same type pt, keep the split and merge detect pt
    for (size_t i = 0; i < split_merge_infos.size(); ++i) {
      const auto& info = split_merge_infos[i];
      for (auto iter = potential_infos.begin();
           iter != potential_infos.end();) {
        const auto& pote_info = *iter;
        if ((info.source_line_id == pote_info.source_line_id) &&
            ((info.point_type == 1 && pote_info.point_type == 3) ||
             (info.point_type == 2 && pote_info.point_type == 4) ||
             (info.point_type == 5 && pote_info.point_type == 3) ||
             (info.point_type == 6 && pote_info.point_type == 4))) {
          HLOG_INFO << "erase pote_info";
          iter = potential_infos.erase(iter);
          break;
        } else {
          ++iter;
        }
      }
    }
    final_infos.insert(final_infos.end(), potential_infos.begin(),
                       potential_infos.end());
  }

  if (final_infos.empty()) {
    return true;
  }

  Eigen::Vector3d dir = pose.so3().matrix().col(0);
  Eigen::Vector3d un_dir = dir.normalized();
  Eigen::Matrix3d R =
      Eigen::AngleAxisd(90.0 * M_PI / 180.0, Eigen::Vector3d(0, 0, 1))
          .toRotationMatrix();

  for (size_t i = 0; i < final_infos.size(); ++i) {
    final_infos[i].ref_poses = std::vector<Sophus::SE3d>({});
    final_infos[i].cut_line_dir = R * un_dir;
  }

  if (final_infos.size() > 1) {
    // 切分线朝车头方向的左侧
    // Eigen::Matrix3d R =
    //     Eigen::AngleAxisd(90.0 * M_PI / 180.0, Eigen::Vector3d(0, 0, 1))
    //         .toRotationMatrix();
    Eigen::Vector3d dir = pose.so3().matrix().col(0);
    Eigen::Vector3d un_dir = dir.normalized();
    // sort cut points
    auto ref_dir = Point3D(un_dir[0], un_dir[1], un_dir[2]);
    std::sort(final_infos.begin(), final_infos.end(),
              [ref_dir](const auto& item1, const auto& item2) {
                Point3D pt1 = item1.world_pt;
                Point3D pt2 = item2.world_pt;
                return pt1.Dot(ref_dir) < pt2.Dot(ref_dir);
              });
  }
  (*detect_point_infos) = final_infos;

  HLOG_DEBUG << "this cate detect point info size: "
             << detect_point_infos->size();

  return true;
}

bool DetectCutPt::DetectSplitMergePt(
    const std::vector<LinePointsPair>& input_lines, const Sophus::SE3d& Twv,
    std::vector<DetectPointInfo>* new_detect_infos) {
  if (input_lines.size() < 2) {
    HLOG_ERROR << "use to detect lines too less, size: " << input_lines.size();
    return false;
  }
  // 涉及到的lines id
  std::vector<id_t> line_ids;
  for (const auto& item : input_lines) {
    const auto& line = item.first;
    if (line == nullptr) continue;
    line_ids.push_back(line->GetId());
  }
  HLOG_DEBUG << "detect split merge lines id: " << GetVecIdStr(line_ids);
  // detect
  for (size_t i = 0; i < input_lines.size() - 1; ++i) {
    for (size_t j = i + 1; j < input_lines.size(); ++j) {
      const auto& line1 = input_lines[i].first;
      const auto& line2 = input_lines[j].first;
      const auto& point1 = input_lines[i].second;
      const auto& point2 = input_lines[j].second;
      if (point1.size() <= 2 || point2.size() <= 2) {
        continue;
      }

      // TODO(qin): need simplify when modifing math_tools
      double mean_d = 0, min_d = 0;
      int overlap_status = 0;
      auto status =
          DistBetweenTwoLine(point1, point2, &mean_d, &min_d, &overlap_status);
      if (overlap_status != 0) {
        // no overlap
        continue;
      }

      // detect split pt
      HLOG_DEBUG << " -> detect split point " << line1->GetId() << ","
                 << line2->GetId();

      Point3D split_pt;
      if (DetectSplitPt(point1, point2, &split_pt) == true ||
          DetectSplitPt(point2, point1, &split_pt) == true) {
        // create new info
        DetectPointInfo detect_info;
        if (line1->GetType() == LineType::LaneType_SOLID &&
            line2->GetType() == LineType::LaneType_SOLID) {
          detect_info.point_type = 5;
        } else {
          detect_info.point_type = 1;
        }
        if (JudgeSource(line1->GetPoints(), line2->GetPoints(), Twv * split_pt,
                        true) == true) {
          detect_info.source_line_id = line1->GetId();
          detect_info.target_line_id = line2->GetId();
          // need use origin line pt
          detect_info.world_pt = line1->GetPoints().front();  // front pt
          detect_info.involve_line_ids = line_ids;
        } else {
          detect_info.source_line_id = line2->GetId();
          detect_info.target_line_id = line1->GetId();
          detect_info.world_pt = line2->GetPoints().front();
          detect_info.involve_line_ids = line_ids;
        }
        new_detect_infos->push_back(std::move(detect_info));
      }

      // detect merge pt
      HLOG_DEBUG << " -> detect merge point " << line1->GetId() << ","
                 << line2->GetId();

      Point3D merge_pt;
      if (DetectMergePt(point1, point2, &merge_pt) == true ||
          DetectMergePt(point2, point1, &merge_pt) == true) {
        HLOG_DEBUG << "type: " << static_cast<int>(line1->GetType()) << ","
                   << static_cast<int>(line2->GetType());
        // create new info
        DetectPointInfo detect_info;
        detect_info.world_pt = Twv * merge_pt;  // proj pt
        if (line1->GetType() == LineType::LaneType_SOLID &&
            line2->GetType() == LineType::LaneType_SOLID) {
          detect_info.point_type = 6;
        } else {
          detect_info.point_type = 2;
        }

        if (JudgeSource(point1, point2, merge_pt, false) == true) {
          detect_info.source_line_id = line1->GetId();
          detect_info.target_line_id = line2->GetId();
        } else {
          detect_info.source_line_id = line2->GetId();
          detect_info.target_line_id = line1->GetId();
        }
        detect_info.involve_line_ids = line_ids;
        new_detect_infos->push_back(std::move(detect_info));
      }
    }
  }
  return true;
}

bool DetectCutPt::CreateFurtherCategory(
    const std::vector<LinePointsPair>& lines_vehicle_pts, bool detect_mode,
    std::vector<std::vector<LaneLine::Ptr>>* all_classified) {
  if (lines_vehicle_pts.size() <= 1) {
    // whole lines size at least 2
    HLOG_ERROR << "input lines too less to category, size: "
               << lines_vehicle_pts.size();
    return false;
  }

  std::stack<std::vector<LinePointsPair>> data_stack;
  data_stack.push(lines_vehicle_pts);

  size_t loop_cnt = 0;

  while (!data_stack.empty()) {
    const std::vector<LinePointsPair> loop_input_lines = data_stack.top();
    data_stack.pop();

    if (loop_input_lines.size() == 0) {
      continue;
    }

    std::vector<LinePointsPair> input_lines = loop_input_lines;
    std::vector<LinePointsPair> classified_lines, other_lines;
    for (auto it = input_lines.begin(); it != input_lines.end() - 1;) {
      const auto& line1 = (*it).first;
      const auto& points1 = (*it).second;
      if (line1 == nullptr || line1->GetPoints().size() <= 2 ||
          points1.size() <= 2) {
        ++it;
        continue;
      }
      classified_lines = std::vector<LinePointsPair>({(*it)});
      other_lines.clear();

      for (auto next_it = it + 1; next_it != input_lines.end();) {
        const auto& line2 = (*next_it).first;
        const auto& points2 = (*next_it).second;
        if (line2 == nullptr || line2->GetPoints().size() <= 2 ||
            points2.size() <= 2) {
          ++next_it;
          continue;
        }
        if (IsGoodLaneTwoLine(points1, points2, detect_mode) == true) {
          classified_lines.push_back((*next_it));  // 同类

          HLOG_DEBUG << "\tline " << line1->GetId() << ", is good lane with "
                     << line2->GetId();
        } else {
          HLOG_DEBUG << "\tline " << line1->GetId()
                     << ", not good lane with // " << line2->GetId();
          other_lines.push_back((*next_it));
        }
        ++next_it;
      }
      if (other_lines.empty()) {
        // for line1, all is in one section, delete line1
        it = input_lines.erase(it);
      } else {
        // 存在异类
        break;
      }
    }
    if (other_lines.empty() && input_lines.size() == 1) {
      // 只剩最后一根了, 说明此次loop线都是同一类
      classified_lines = loop_input_lines;
    }
    for (const auto& other_line : other_lines) {
      for (auto it = classified_lines.begin(); it != classified_lines.end();) {
        if (IsGoodLaneTwoLine(other_line.second, it->second, detect_mode) ==
            true) {
          // delete
          it = classified_lines.erase(it);
        } else {
          ++it;
        }
      }
    }

    if (!classified_lines.empty()) {
      std::vector<LaneLine::Ptr> lines;
      std::transform(
          classified_lines.begin(), classified_lines.end(),
          std::back_inserter(lines),
          [](const LinePointsPair& line_points) { return line_points.first; });
      all_classified->push_back(lines);
    }

    if (other_lines.size() >= 1) {
      data_stack.push(other_lines);
    }

    loop_cnt++;
    if (loop_cnt > 10) {
      HLOG_ERROR << "have good lane detect over 10 times, may error";
      return false;
    }
  }
  return true;
}

bool DetectCutPt::SortSubCategories(
    const Sophus::SE3d& ref_pose,
    const std::vector<LinePointsPair>& input_lines,
    const std::vector<std::vector<LaneLine::Ptr>>& start_cates,
    const std::vector<std::vector<LaneLine::Ptr>>& end_cates,
    std::vector<DetectPointInfo>* infos) {
  // 涉及到的lines id
  std::vector<id_t> line_ids;
  for (const auto& item : input_lines) {
    const auto& line = item.first;
    if (line == nullptr) {
      continue;
    }
    line_ids.push_back(line->GetId());
  }

  Eigen::Vector3d un_dir = (ref_pose.so3().matrix().col(0)).normalized();
  auto ref_dir = Point3D(un_dir[0], un_dir[1], un_dir[2]);

  auto ComputeCutPoints =
      [&](const std::vector<std::vector<LaneLine::Ptr>>& input_cates,
          std::vector<DetectPointInfo>* new_infos, const bool start) -> bool {
    if (input_cates.size() < 2) {
      return true;
    }
    auto Compare = [&](const auto& line1, const auto& line2) {
      Point3D p1, p2;
      if (start) {
        p1 = line1->GetPoints().front();
        p2 = line2->GetPoints().front();
      } else {
        p1 = line1->GetPoints().back();
        p2 = line2->GetPoints().back();
      }
      return p1.Dot(ref_dir) < p2.Dot(ref_dir);
    };
    std::map<double, std::vector<LaneLine::Ptr>> sorted_cates;
    for (size_t i = 0; i < input_cates.size(); ++i) {
      auto lines = input_cates[i];
      // 类中线与线排序
      std::sort(lines.begin(), lines.end(), Compare);

      Point3D use_point;
      if (start) {
        use_point = lines.front()->GetPoints().front();
      } else {
        use_point = lines.back()->GetPoints().back();
      }

      double proj_val = use_point.Dot(ref_dir);
      if (sorted_cates.count(proj_val) == 0) {
        sorted_cates.insert(std::make_pair(proj_val, lines));
      } else {
        HLOG_ERROR << "have same proj value, may error";
        sorted_cates[proj_val] = lines;
      }
    }
    if (sorted_cates.size() < 2) {
      HLOG_ERROR << "after sort sorted cates size < 2 error,"
                 << sorted_cates.size();
      return false;
    }
    std::vector<LaneLine::Ptr> sorted_lines;
    for (auto it = sorted_cates.begin(); it != sorted_cates.end(); ++it) {
      if (start) {
        sorted_lines.push_back(it->second.front());
      } else {
        sorted_lines.push_back(it->second.back());
      }
    }

    if (start) {
      for (size_t i = 1; i < sorted_lines.size(); ++i) {
        const auto& line = sorted_lines.at(i);
        if (line == nullptr || line->GetPoints().size() < 2) {
          continue;
        }

        DetectPointInfo detect_info;
        detect_info.world_pt = line->GetPoints().front();  // 取start
        detect_info.point_type = 3;                        // sigle to multi
        detect_info.source_line_id = line->GetId();
        detect_info.involve_line_ids = line_ids;
        new_infos->push_back(std::move(detect_info));
      }
    } else {
      for (size_t i = 0; i < sorted_lines.size() - 1; ++i) {
        const auto& line = sorted_lines.at(i);
        if (line == nullptr || line->GetPoints().size() < 2) {
          continue;
        }
        DetectPointInfo detect_info;
        detect_info.world_pt = line->GetPoints().back();  // 取end
        detect_info.point_type = 4;                       // multi to sigle
        detect_info.source_line_id = line->GetId();
        detect_info.involve_line_ids = line_ids;
        new_infos->push_back(std::move(detect_info));
      }
    }
    return true;
  };

  if (ComputeCutPoints(start_cates, infos, true) != true) {
    return false;
  }
  if (ComputeCutPoints(end_cates, infos, false) != true) {
    return false;
  }
  return true;
}

bool DetectCutPt::IsGoodLaneTwoLine(const std::vector<Point3D>& segment1,
                                    const std::vector<Point3D>& segment2,
                                    bool start_detect_mode) {
  float overlap_value;
  if (segment1.size() < 2 || segment2.size() < 2) {
    HLOG_ERROR << "exist input segment pts size too less ";
    return false;
  }
  Point3D back_pt = segment1.back();
  Point3D front_pt = segment1.front();
  Point3D direction_v = back_pt - front_pt;
  if (direction_v.Dot(segment2.front() - front_pt) > 0) {
    front_pt = segment2.front();
  }
  if (direction_v.Dot(segment2.back() - back_pt) < 0) {
    back_pt = segment2.back();
  }
  Point3D new_direction = back_pt - front_pt;
  float overlap = direction_v.Dot(new_direction);
  if (overlap <= 0) {
    overlap_value = 0.0;
  } else {
    float hd_length = direction_v.Norm2D();
    HLOG_DEBUG << "back_x: " << back_pt.x << " front_x: " << front_pt.x
               << " hd_length: " << hd_length;
    overlap_value = overlap / direction_v.Norm2D();
  }
  if (overlap_value < 1e-3) {
    return false;  // two line no overlap
  }
  if ((segment1.back() - segment1.front()).Norm2D() < 1e-3 ||
      (segment2.back() - segment2.front()).Norm2D() < 1e-3) {
    HLOG_ERROR << "input line segment start end pt too close, not support, "
               << (segment1.back() - segment1.front()).Norm2D();
    return false;
  }

  size_t index;
  int pos_flag;
  Point3D candi_pt1, candi_pt2;
  double dist1, dist2;
  if (start_detect_mode) {
    if (GetProjectPoint(segment1, segment2.front(), &candi_pt1, &index,
                        &pos_flag) != SMStatus::SUCCESS) {
      return false;
    }
    if (GetProjectPoint(segment2, segment1.front(), &candi_pt2, &index,
                        &pos_flag) != SMStatus::SUCCESS) {
      return false;
    }
    dist1 = (segment1.front() - candi_pt1).Norm2D();
    dist2 = (segment2.front() - candi_pt2).Norm2D();
  } else {
    if (GetProjectPoint(segment1, segment2.back(), &candi_pt1, &index,
                        &pos_flag) != SMStatus::SUCCESS) {
      return false;
    }
    if (GetProjectPoint(segment2, segment1.back(), &candi_pt2, &index,
                        &pos_flag) != SMStatus::SUCCESS) {
      return false;
    }
    dist1 = (segment1.back() - candi_pt1).Norm2D();
    dist2 = (segment2.back() - candi_pt2).Norm2D();
  }
  const double dist_thr = 10.0;
  if (dist1 < dist_thr && dist2 < dist_thr) {
    return true;
  }
  return false;
}

bool DetectCutPt::JudgeSource(const std::vector<Point3D>& points1,
                              const std::vector<Point3D>& points2,
                              const Point3D& junc_pt, bool is_split) {
  bool pts1_is_source = true;
  Point3D first_pt1, first_pt2;
  if (is_split) {
    first_pt1 = points1.front();
    first_pt2 = points2.front();
  } else {
    first_pt1 = points1.back();
    first_pt2 = points2.back();
  }
  double dist = (junc_pt - first_pt1).Norm2D() - (junc_pt - first_pt2).Norm2D();
  if (std::fabs(dist) <= 2.0) {
    double heading1, heading2;
    {
      Point3D start_pt;
      if (GetFootPointInLine(Point3D(0, 0, 0), points1, &start_pt) !=
          SMStatus::SUCCESS) {
        start_pt = first_pt1;
      }
      heading1 =
          atan2(std::fabs(junc_pt.y - start_pt.y), junc_pt.x - start_pt.x);
    }
    {
      Point3D start_pt;
      if (GetFootPointInLine(Point3D(0, 0, 0), points2, &start_pt) !=
          SMStatus::SUCCESS) {
        start_pt = first_pt2;
      }
      heading2 =
          atan2(std::fabs(junc_pt.y - start_pt.y), junc_pt.x - start_pt.x);
    }
    if (heading1 > heading2) {
      pts1_is_source = true;
    } else {
      pts1_is_source = false;
    }

  } else if (dist > 2.0) {
    pts1_is_source = false;
  } else if (dist < -2.0) {
    pts1_is_source = true;
  }
  return pts1_is_source;
}

bool DetectCutPt::DetectSplitPt(const std::vector<Point3D>& pts1,
                                const std::vector<Point3D>& pts2,
                                Point3D* split_pt) {
  if (pts1.size() <= 2 || pts2.size() <= 2) {
    HLOG_ERROR << "wrong points size";
    return false;
  }
  // 1. get proj_pt and calculate first_pt distance
  // Judge condition1: first_pt distance < th1
  Point3D candi_pt;
  size_t index;
  int pos_flag;
  if (GetProjectPoint(pts2, pts1.front(), &candi_pt, &index, &pos_flag) !=
      SMStatus::SUCCESS) {
    return false;
  }
  // if (pos_flag == -1 || pos_flag == 1) {
  //     // skip below or over line
  //     return false;
  // }
  double front_dist = (pts1.front() - candi_pt).Norm2D();
  const double split_dist_th1 = 1.5;
  HLOG_DEBUG << " front_dist : " << front_dist;
  if (front_dist > split_dist_th1) {
    return false;
  }

  // 2. calculate average distance
  // 从算8个点改成算较短的一条车道线的中点到另一条车道线的最近距离
  double mindis_cur = 0;
  if (pts1[pts1.size() - 1].x <= pts2[pts2.size() - 1].x) {
    size_t compare_cnt = pts1.size() / 2 - 1;  // pts1 size一定大于2
    Point3D query_pt = pts1.at(compare_cnt);
    mindis_cur = FindMinDist(query_pt, pts2, index, pts2.size() - 1);
  } else {
    size_t compare_cnt =
        index + (pts2.size() - 1 - index) / 2;  // pts2 size一定大于2
    Point3D query_pt = pts2.at(compare_cnt);
    mindis_cur = FindMinDist(query_pt, pts1, 0, pts1.size() - 1);
  }
  HLOG_DEBUG << "mindist: " << mindis_cur;
  if (mindis_cur >= 5.0 || mindis_cur <= front_dist) {
    return false;
  }
  const double split_dist_th2 = 0.5;
  if (mindis_cur - front_dist > split_dist_th2) {
    (*split_pt) = pts1.front();
  } else {
    return false;
  }
  return true;
}

bool DetectCutPt::DetectMergePt(const std::vector<Point3D>& pts1,
                                const std::vector<Point3D>& pts2,
                                Point3D* merge_pt) {
  if (pts1.size() <= 2 || pts2.size() <= 2) {
    HLOG_ERROR << "wrong points size";
    return false;
  }
  // 1. get proj_pt and calculate first_pt distance
  // Judge condition1: first_pt distance < th1
  Point3D candi_pt;
  size_t index;
  int pos_flag;
  if (GetProjectPoint(pts2, pts1.back(), &candi_pt, &index, &pos_flag) !=
      SMStatus::SUCCESS) {
    return false;
  }
  // if (pos_flag == -1 || pos_flag == 1) {
  //     // skip below or over line
  //     return false;
  // }
  double end_dist = (pts1.back() - candi_pt).Norm2D();
  const double merge_dist_th1 = 1.5;
  HLOG_DEBUG << " end_dist : " << end_dist;
  if (end_dist > merge_dist_th1) {
    return false;
  }

  // 2. calculate average distance
  double mindis_cur = 0;
  if (pts1[0].x >= pts2[0].x) {
    size_t compare_cnt = pts1.size() / 2 - 1;  // pts1 size一定大于2
    Point3D query_pt = pts1.at(compare_cnt);
    mindis_cur = FindMinDist(query_pt, pts2, 0, index - 1);
  } else {
    size_t compare_cnt = index / 2;  // pts2 size一定大于2
    Point3D query_pt = pts2.at(compare_cnt);
    mindis_cur = FindMinDist(query_pt, pts1, 0, pts1.size() - 1);
  }
  HLOG_DEBUG << "mindist: " << mindis_cur;
  if (mindis_cur >= 5.0 || mindis_cur <= end_dist) {
    return false;
  }

  const double merge_dist_th2 = 0.5;
  if (mindis_cur - end_dist > merge_dist_th2) {
    (*merge_pt) = pts1.back();
  } else {
    return false;
  }
  return true;
}

bool DetectCutPt::GetCategorySubPosePath(
    const std::vector<std::pair<size_t, size_t>>& sub_paths,
    const std::vector<LaneLine::Ptr>& lines,
    std::vector<std::pair<size_t, size_t>>* target_path) {
  if (sub_paths.empty()) {
    HLOG_ERROR << " sub paths is empty";
    return false;
  }
  // 截取一段合适的轨迹(方向基本一致,距离不太远)
  std::map<size_t, std::vector<Point3D>> sub_paths_map;
  size_t path_cnt = 0;
  for (const auto& sub_path : sub_paths) {
    // 获取区间点 [start_index, end_index]
    std::vector<Sophus::SE3d> sub_path_poses(
        whole_pose_path_.begin() + sub_path.first,
        whole_pose_path_.begin() + sub_path.second + 1);

    std::vector<Point3D> points;
    std::transform(sub_path_poses.begin(), sub_path_poses.end(),
                   std::back_inserter(points), [&](const auto& pose) {
                     return Point3D(pose.translation()[0],
                                    pose.translation()[1],
                                    pose.translation()[2]);
                   });
    sub_paths_map.insert(std::make_pair(path_cnt, points));
    path_cnt++;
  }
  std::vector<size_t> good_path_indexs;
  for (const auto& line : lines) {
    if (line == nullptr || line->GetPoints().size() < 2) {
      continue;
    }
    for (const auto& it : sub_paths_map) {
      size_t index = it.first;
      const auto& path_points = it.second;
      if (std::find(good_path_indexs.begin(), good_path_indexs.end(), index) !=
          good_path_indexs.end()) {
        continue;
      }

      // Todo: dapat pos 只有一个或两个的情况
      if (IsConsistentOfTwoLines(line->GetPoints(), path_points)) {
        // find new good path
        good_path_indexs.push_back(index);
        break;
      }
    }
  }
  if (good_path_indexs.empty()) {
    HLOG_ERROR << " not found good path, input path size:" << sub_paths.size();
    // 可以选择距离最近, 方向基本一致的
    return false;
  }

  std::sort(good_path_indexs.begin(), good_path_indexs.end());

  // get best pose path (暂且粗略的选择距离最长的一段pose)
  size_t best_index = good_path_indexs.front();
  if (good_path_indexs.size() >= 2) {
    double max_dist = -1.0;
    for (size_t i = 0; i < good_path_indexs.size(); ++i) {
      const auto& pts = sub_paths_map.at(good_path_indexs[i]);

      double dist = (pts.back() - pts.front()).Norm();
      if (dist > max_dist) {
        max_dist = dist;
        best_index = good_path_indexs[i];
      }
    }
  }
  target_path->push_back(sub_paths.at(best_index));

  return true;
}

bool CompareTwoCategories(const std::vector<Sophus::SE3d>& path,
                          const CategoryInfo& cate_info1,
                          const CategoryInfo& cate_info2) {
  if (path.empty()) {
    HLOG_ERROR << "input path size is empty ";
    return false;
  }
  std::pair<size_t, size_t> path_region1 = cate_info1.path_region;
  std::pair<size_t, size_t> path_region2 = cate_info2.path_region;

  if ((path_region1.second < 0 || path_region1.second > path.size() - 1) ||
      (path_region2.second < 0 || path_region2.second > path.size() - 1)) {
    HLOG_ERROR << " exist invalid index, region1 end index: "
               << path_region1.second
               << ", region2 end index: " << path_region2.second;
    return false;
  }

  if (path_region1.second < path_region2.second) {
    return true;
  } else if (path_region1.second > path_region2.second) {
    return false;
  }

  auto lines1 = cate_info1.lines;
  auto lines2 = cate_info2.lines;

  // 尾部 index 相等
  const auto& ref_pose = path.at(path_region1.second);
  Eigen::Vector3d dir = ref_pose.so3().matrix().col(0);

  Eigen::Vector3d un_dir = dir.normalized();
  auto ref_dir = Point3D(un_dir[0], un_dir[1], un_dir[2]);

  auto Compare = [&](const auto& line1, const auto& line2) {
    Point3D p1 = line1->GetPoints().back();
    Point3D p2 = line2->GetPoints().back();
    return p1.Dot(ref_dir) < p2.Dot(ref_dir);
  };

  // sort lines1, lines2
  std::sort(lines1.begin(), lines1.end(), Compare);
  std::sort(lines2.begin(), lines2.end(), Compare);

  // 选sort之后的最后一根线的末尾点
  Point3D p1 = lines1.back()->GetPoints().back();
  Point3D p2 = lines2.back()->GetPoints().back();
  return p1.Dot(ref_dir) < p2.Dot(ref_dir);
}

bool DetectCutPt::IsConsistentOfTwoLines(const std::vector<Point3D>& points1,
                                         const std::vector<Point3D>& points2) {
  if (points1.size() < 2 || points2.size() < 2) {
    return false;
  }

  // 两条线是否一致: 朝向,overlap,距离
  Point3D direction_1 = points1.back() - points1.front();
  Point3D direction_2 = points2.back() - points2.front();
  HLOG_DEBUG << "points1.back: " << points1.back().x << ", "
             << points1.back().y;
  HLOG_DEBUG << "points1.front: " << points1.front().x << ", "
             << points1.front().y;
  HLOG_DEBUG << "points2.back: " << points2.back().x << ", "
             << points2.back().y;
  HLOG_DEBUG << "points2.front: " << points2.front().x << ", "
             << points2.front().y;

  if (direction_1.Dot(direction_2) < 0) {  // 反向
    HLOG_DEBUG << "direction_1.Dot(direction_2) < 0 ";
    return false;
  }

  if (direction_1.Norm() * direction_2.Norm() < 1e-6) {  // 垂直
    HLOG_DEBUG << "direction_1.Norm() * direction_2.Norm() < 1e-6 ";

    return false;
  }
  double angle = std::acos(direction_1.Dot(direction_2) /
                           (direction_1.Norm() * direction_2.Norm())) *
                 180.0 / M_PI;
  HLOG_DEBUG << "angle: " << angle;
  // if (angle > 30.0) {
  // !考虑弯道，暂时的方法
  if (angle > 45.0) {
    return false;
  }
  if (!IsOverLapTwoLine(points1, points2)) {  // 计算两条线的重合长度
    HLOG_DEBUG << "IsOverLapTwoLine FALSE ";
    return false;
  }

  double mean_d = 0, min_d = 0;
  int overlap_status = 0;
  auto status =
      DistBetweenTwoLine(points1, points2, &mean_d, &min_d, &overlap_status);
  if (status != SMStatus::SUCCESS) {
    HLOG_ERROR << "DistBetweenTwoLine fail";
    return false;
  }
  if (overlap_status != 0) {
    HLOG_DEBUG << "no overlap";
    // no overlap
    return false;
  }
  HLOG_DEBUG << "mean_d: " << mean_d << ", min_d: " << min_d
             << ", overlap_status: " << overlap_status;
  const double dist_thr = 50.0;
  if (mean_d > dist_thr && min_d > dist_thr) {
    HLOG_DEBUG << "two line dist over:" << dist_thr << ", mean dist: " << mean_d
               << ", min_d: " << min_d;
    return false;
  }
  return true;
}

template <typename T>
inline std::string DetectCutPt::GetElementsIdVec(const std::vector<T>& vec) {
  std::ostringstream os;
  for (const auto& item : vec) {
    if (item == nullptr) {
      os << " [NULL]";
    } else {
      os << " " << item->GetId();
    }
  }
  return os.str();
}

template <typename T>
inline std::string DetectCutPt::GetVecIdStr(const std::vector<T>& vec) {
  std::ostringstream os;
  for (const auto& item : vec) {
    os << " " << item;
  }
  return os.str();
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
