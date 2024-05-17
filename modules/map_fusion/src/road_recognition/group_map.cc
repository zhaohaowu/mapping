/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： group_map.cc
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#include "map_fusion/road_recognition/group_map.h"
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "base/utils/log.h"
#include "map_fusion/fusion_common/calc_util.h"
#include "map_fusion/fusion_common/element_map.h"

namespace hozon {
namespace mp {
namespace mf {
namespace gm {

using ProtoBoundType = hozon::hdmap::LaneBoundaryType;
using hozon::common::math::Vec2d;

struct LaneWithNextLanes {
  Lane::Ptr lane = nullptr;
  std::vector<Lane::Ptr> next_lanes;
};

bool GroupMap::Build(const std::shared_ptr<std::vector<KinePose::Ptr>>& path,
                     const KinePose::Ptr& curr_pose,
                     const em::ElementMap::Ptr& ele_map, IsCross is_cross) {
  if (path == nullptr || curr_pose == nullptr || ele_map == nullptr) {
    HLOG_ERROR << "input nullptr";
    return false;
  }

  // - - - - - + - - - -              +-----
  // |   |   | . |                         |
  // - - - - - + - - - -                   |
  // |   |   | . |                         |
  // - - - - - + - - - -                   |
  // |   |   | . |                       Group
  // - - - - - + - - - -                   |
  //  \  |   | . |                         |
  // - - - - - + - - - -                   |
  //    \|   | . |                         |
  // - - - - - + - - - -              +-----
  //     |   | . |                         |
  // - - - - - + - - - -                   |
  //     |   | . |                       Group
  // - - - - - + - - - -                   |
  //     |   | . |                         |
  // - - - - - + - - - -              +-----
  //     |   | . |\                        |
  // - - - - - + - - - -                   |
  //     |   | ^ |  \                      |
  // - - - - - + - - - -                   |
  //     |   | . |   |                   Group
  // - - - - - + - - - -                   |
  //     |   | . |   |                     |
  // - - - - - + - - - -                   |
  //     |   | . |   |    GroupSegment     |
  // - - - - - + - - - -   SliceLine  +-----
  // 0.
  // 先对所有车道线的点进行等间距采样，间隔比path的点间距要小，比如path间距2m，这里车道线按1m间距采样；
  // 这里采样的目的是，防止原始车道线的点间距太大，导致下面的GroupSegment切分不到点；
  // 1. 生成所有GroupSegment：
  // 遍历path里每个pose，相邻两个pose组成一个GroupSegment；将每个pose的vehicle系下自车原点(0,
  // 0)、 左边点(0, 1)、右边点(0,
  // -1)根据当时的pose转到local系下；然后再根据curr_pose再转到当前 车体系下；
  // 2. 按顺序遍历每个GroupSegment，将所有线传递给每个GroupSegment用于构建：
  // 遍历每根线的每一个点，判断这个点是否在GroupSegment.start_slice的左侧，如果左侧就丢掉，继续下一个点；
  // 然后判断点是否在GroupSegment.end_slice的左侧或其上，如果是，说明此点在start_slice与end_slice之间，
  // 就把这个点加到LineSegment；
  // 遍历每个LineSegment，计算LineSegment的中点，然后计算中点到向量(start_slice.po,
  // end_slice.po)的距离dist_to_path，
  // 并且左边为正，右边为负，对所有LineSegment按dist_to_path从大到小重新排序，即将所有LineSegment从左到右排序了；
  // 遍历排序后的LineSegment，计算每相邻两个LineSegment.center的间距，如果超过一定阈值且小于一定阈值，那就认为可以组成
  // 一个LaneSegment，并把左右LineSegment分别填充到LaneSegment的左右边界；并且将左右LineSegment的id转为字符串并拼接，
  // 作为LaneSegment的str_id（比如"12_13"）；
  // 将GroupSegment里的所有LaneSegment的str_id拼接（比如"12_13|13_14|14_15"），作为GroupSegment的str_id；
  // 3. 聚合GroupSegment为Group：
  // 遍历所有GroupSegment，对于第一个GroupSegment，直接创建一个Group，并且将第一个GroupSegment直接加入到Group中，
  // 并且Group的str_id也赋值为这个GroupSegment的str_id；
  // 对于后面每个GroupSegment，如果str_id与最后一个Group的str_id相同，就把自己加入到最后一个Group；如果不相同就
  // 新创建一个Group，将自己加到这个Group中；直到遍历完所有GroupSegment，这样就得到所有Group；
  // 4. 遍历每个Group，把Group中GroupSegment聚合成多个Lane：
  // 遍历此Group中所有GroupSegment，对于第一个GroupSegment，其中每一个LaneSegment都创建出一个Lane，将LaneSegment的左右边界
  // 直接塞到Lane的左右边界，并且Lane的str_id直接使用LaneSegment的str_id，
  // str_id_with_group使用Group.str_id和LaneSegment.str_id进行拼接（比如"12_13|13_14|14_15:12_13"）；
  // 对于后面每个GroupSegment，按索引将每个LaneSegment的左右边界塞到对应Lane的左右边界；这样所有纵向的LaneSegment就聚合成单独的Lane了；
  // 聚合完Lane后，遍历所有Lane，填充每个Lane的left_lane_str_id_with_group和right_lane_str_id_with_group；
  // 对每个Lane，计算出中心线center_line_pts；
  // 5. 遍历每个Group，关联Lane的前后继：
  // 对相邻的两个Group，如果前面Group中的某个Lane的str_id在后面Group中存在，就分别设置它们的prev_lane_str_id_with_group
  // 和next_lane_str_id_with_group；
  // 6. TBD：分合流时的前后继，比如：
  // case 1:
  // 分流:  1->3, 1->4, 2->4, 2->5
  // 或合流: 3->1, 4->1, 4->2, 5->2
  // |   |   |   |
  // | 3 | 4 | 5 |
  // |   |   |   |
  // - - - - - - -
  //   |   |   |
  //   | 1 | 2 |
  //   |   |   |
  //
  // case 2:
  // 分流:  1->3, 1->4, 2->5
  // 或合流: 3->1, 4->1, 5->2
  // |   |   |   |
  // |   |   |   |
  //  \3 | 4 | 5 |
  //   \ |   |   |
  //     - - - - -
  //     |   |   |
  //     | 1 | 2 |
  //     |   |   |
  is_cross_ = is_cross;
  HLOG_INFO << "IS_CROSS" << is_cross_.cross_after_lane_ << "  "
            << is_cross.cross_before_lane_ << "  " << is_cross.is_crossing_;
  curr_pose_ = curr_pose;
  std::deque<Line::Ptr> lines;
  // lane_line_interp_dist可以设为-1，当前上游点间隔已经是1m，这里不用插值出更细的点
  RetrieveBoundaries(ele_map, conf_.lane_line_interp_dist, &lines);
  UpdatePathInCurrPose(*path, *curr_pose);
  BuildGroupSegments(path, curr_pose, &lines, &group_segments_, ele_map);
  BuildGroups(ele_map->map_info.stamp, group_segments_, &groups_);
  return true;
}

// 从原始ElementMap里提取出车道线：
// 1.将ElementMap里点按原顺序保存到队列；
// 2.interp_dist为插值间距，当>0时按此间距对车道线点进行插值；
// 3.计算每根线的末端平均heading和平均点间距，用于后面预测.
void GroupMap::RetrieveBoundaries(const em::ElementMap::Ptr& ele_map,
                                  float interp_dist,
                                  std::deque<Line::Ptr>* lines) {
  if (ele_map == nullptr || lines == nullptr) {
    return;
  }
  zebra_.clear();
  arrow_.clear();
  stopline_.clear();
  overlaps_.clear();
  bool need_interp = (interp_dist > 0);

  for (const auto& bound_pair : ele_map->boundaries) {
    const auto& bound = bound_pair.second;
    if (bound == nullptr) {
      HLOG_ERROR << "found nullptr boundary";
      continue;
    }
    if (bound->nodes.empty()) {
      continue;
    }
    auto line = std::make_shared<Line>();
    line->id = bound->id;
    line->type = bound->linetype;
    line->color = bound->color;
    line->isego = bound->is_ego;
    line->lanepos = bound->lanepos;
    em::Point last_raw(0, 0, 0);
    for (const auto& node : bound->nodes) {
      if (node == nullptr) {
        HLOG_ERROR << "found nullptr node";
        continue;
      }
      const auto& curr_raw = node->point;
      if (need_interp && !line->pts.empty()) {
        // 方向向量
        em::Point n = curr_raw - last_raw;
        float dist = n.norm();
        if (dist > interp_dist) {
          n.normalize();
          int interp_cnt = static_cast<int>(dist / interp_dist);
          for (int i = 0; i != interp_cnt; ++i) {
            em::Point interp_pt = last_raw + (i + 1) * n;
            Point pt(gm::INTERPOLATED, interp_pt.x(), interp_pt.y(),
                     interp_pt.z());
            line->pts.emplace_back(pt);
          }
        }
      }

      Point pt(gm::RAW, curr_raw.x(), curr_raw.y(), curr_raw.z());
      line->pts.emplace_back(pt);
      last_raw = curr_raw;
    }

    if (line->pts.empty()) {
      continue;
    }

    // 计算末端平均heading、平均间距
    const int kAvgPtCount = 20;
    int avg_n = std::min(static_cast<int>(line->pts.size()), kAvgPtCount);
    if (avg_n < 2) {
      line->mean_end_interval = 0;
      line->mean_end_heading = 0;
      line->mean_end_heading_std_dev = 0;
    } else {
      std::vector<Point> last_n_pts(line->pts.end() - avg_n, line->pts.end());
      // 计算前后两点连成的向量与x的夹角theta的均值
      std::vector<double> thetas(last_n_pts.size() - 1, 0);
      double mean_theta = 0.;
      double mean_interval = 0.;
      for (size_t i = 0; i != last_n_pts.size() - 1; ++i) {
        const Eigen::Vector2f pa = last_n_pts.at(i).pt.head<2>();
        const Eigen::Vector2f pb = last_n_pts.at(i + 1).pt.head<2>();
        Eigen::Vector2f v = pb - pa;
        double theta = 0;
        if (std::abs(v.x()) > 1e-2) {
          theta = atan2(v.y(), v.x());  // atan2计算出的角度范围是[-pi, pi]
        }
        thetas[i] = theta;
        mean_theta += theta;
        mean_interval += v.norm();
      }

      mean_interval /= static_cast<double>(thetas.size());
      mean_theta /= static_cast<double>(thetas.size());
      double square_sum =
          std::inner_product(thetas.begin(), thetas.end(), thetas.begin(), 0.0);
      double std_theta =
          std::sqrt(square_sum / static_cast<double>(thetas.size()) -
                    mean_theta * mean_theta);
      line->mean_end_interval = mean_interval;
      line->mean_end_heading = mean_theta;
      line->mean_end_heading_std_dev = std_theta;
    }

    lines->emplace_back(line);
  }

  for (const auto& arrow : ele_map->arrows) {
    Arrow arw;
    arw.id = arrow.second->id;
    arw.type = arrow.second->type;
    arw.polygon = arrow.second->polygon;
    arrow_[arw.id] = std::make_shared<Arrow>(arw);
  }

  for (const auto& stopline : ele_map->stop_lines) {
    Stpl stpl;
    stpl.id = stopline.second->id;
    stpl.points = stopline.second->points;
    stopline_[stpl.id] = std::make_shared<Stpl>(stpl);
  }

  for (const auto& zebra : ele_map->cross_walks) {
    Zebra zbr;
    zbr.id = zebra.second->id;
    zbr.polygon = zebra.second->polygon;
    zebra_[zbr.id] = std::make_shared<Zebra>(zbr);
  }
}

// 按轨迹方向生成所有GroupSegments
void GroupMap::BuildGroupSegments(
    const std::shared_ptr<std::vector<KinePose::Ptr>>& path,
    const KinePose::Ptr& curr_pose, std::deque<Line::Ptr>* lines,
    std::vector<GroupSegment::Ptr>* group_segments,
    const em::ElementMap::Ptr& ele_map) {
  if (path == nullptr || curr_pose == nullptr || lines == nullptr ||
      group_segments == nullptr) {
    return;
  }

  CreateGroupSegFromPath(*path, *curr_pose, group_segments);
  SplitPtsToGroupSeg(lines, group_segments);
  GenLaneSegInGroupSeg(group_segments);
  EgoLineTrajectory(group_segments, ele_map);
}

void GroupMap::FitLaneline(const em::ElementMap::Ptr& ele_map, int id_1,
                           int id_2, int near_line) {
  predict_line_params_.clear();
  int size1_s = -1, size2_s = -1, size1_num = 0, size2_num = 0;
  for (const auto& node : ele_map->boundaries[id_1]->nodes) {
    if (node->point.x() > -15 && node->point.x() < 15) {
      if (size1_s == -1) {
        size1_s = node->id;
      }
      size1_num++;
    } else if (node->point.x() > 15) {
      break;
    }
  }
  for (const auto& node : ele_map->boundaries[id_2]->nodes) {
    if (node->point.x() > -15 && node->point.x() < 15) {
      if (size2_s == -1) {
        size2_s = node->id;
      }
      size2_num++;
    } else if (node->point.x() > 15) {
      break;
    }
  }
  //! TBD: 用三次来拟合线会更准确，目前只是二次的，待改进
  if (size1_num < 8 && size2_num < 8) {
    ego_line_exist_ = false;
    return;
  }
  if (id_1 == near_line && size1_num > 4) {
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(size1_num, 4);
    Eigen::VectorXd x(size1_num);
    Eigen::VectorXd b(size1_num);
    for (int i = 0; i < size1_num; i++) {
      double xi = ele_map->boundary_nodes[size1_s + i]->point.x();
      double yi = ele_map->boundary_nodes[size1_s + i]->point.y();
      A(i, 0) = 1.0;
      A(i, 1) = xi;
      A(i, 2) = xi * xi;
      A(i, 3) = xi * xi * xi;
      b[i] = yi;
    }
    x = (A.transpose() * A).inverse() * A.transpose() * b;
    predict_line_params_ = {x[0], x[1], x[2], x[3]};
  } else {
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(size2_num, 4);
    Eigen::VectorXd x(size2_num);
    Eigen::VectorXd b(size2_num);
    for (int i = 0; i < size2_num; i++) {
      double xi = ele_map->boundary_nodes[size2_s + i]->point.x();
      double yi = ele_map->boundary_nodes[size2_s + i]->point.y();
      A(i, 0) = 1.0;
      A(i, 1) = xi;
      A(i, 2) = xi * xi;
      A(i, 3) = xi * xi * xi;
      b[i] = yi;
    }
    x = (A.transpose() * A).inverse() * A.transpose() * b;
    predict_line_params_ = {x[0], x[1], x[2], x[3]};
  }
  // HLOG_ERROR << "predict_line_params_ = " << predict_line_params_[0] << "  "
  //            << predict_line_params_[1] << "  " << predict_line_params_[2]
  //            << "  " << predict_line_params_[3];
}

void GroupMap::EgoLineTrajectory(std::vector<GroupSegment::Ptr>* grp_segment,
                                 const em::ElementMap::Ptr& ele_map) {
  int line1_id = -200, line2_id = -200, near_line = -200;
  for (auto& seg : *grp_segment) {
    int flag = 0;
    if (seg->end_slice.po.x() > -5 && seg->start_slice.po.x() < 5) {
      if (seg->line_segments.size() > 1) {
        for (unsigned int i = 0; i < seg->line_segments.size() - 1; ++i) {
          if (seg->line_segments[i]->dist_to_path *
                  seg->line_segments[i + 1]->dist_to_path <
              0.1) {
            line1_id = seg->line_segments[i]->id;
            line2_id = seg->line_segments[i + 1]->id;
            near_line = (abs(seg->line_segments[i]->dist_to_path) <
                         abs(seg->line_segments[i + 1]->dist_to_path))
                            ? line1_id
                            : line2_id;
            // HLOG_ERROR << "line1_id = " << line1_id
            //            << "  line2_id = " << line2_id;
            flag = 1;
            break;
          }
        }
      }
    }
    if (flag) {
      break;
    }
  }
  if (line1_id == -200 && line2_id == -200) {
    ego_line_exist_ = false;
  } else {
    ego_line_exist_ = true;
    FitLaneline(ele_map, line1_id, line2_id, near_line);
  }
}

// 将所有GroupSegments聚合成一个个Group
void GroupMap::BuildGroups(double stamp,
                           std::vector<GroupSegment::Ptr> group_segments,
                           std::vector<Group::Ptr>* groups) {
  if (group_segments.empty() || groups == nullptr) {
    return;
  }
  UniteGroupSegmentsToGroups(stamp, group_segments, groups);
  GenLanesInGroups(groups, stamp);
}

// 按轨迹方向计算出所有切分线，每两根相邻切分线组成一个GroupSegment
void GroupMap::CreateGroupSegFromPath(
    const std::vector<KinePose::Ptr>& path, const KinePose& curr_pose,
    std::vector<GroupSegment::Ptr>* segments) {
  if (segments == nullptr) {
    HLOG_ERROR << "input nullptr";
    return;
  }
  // 将path里一个个pose转换成分割线SliceLine
  std::vector<SliceLine> slice_lines;
  Pose pose_local_to_veh = curr_pose.Inverse();
  for (const auto& p : path) {
    if (p == nullptr) {
      HLOG_ERROR << "found nullptr pose in path";
      continue;
    }
    em::Point pl_veh(0, 1, 0);
    pl_veh *= conf_.half_slice_length;
    em::Point pr_veh(0, -1, 0);
    pr_veh *= conf_.half_slice_length;
    em::Point pl_local = p->TransformPoint(pl_veh);
    em::Point pr_local = p->TransformPoint(pr_veh);
    em::Point po_local = p->pos;

    em::Point pl_curr_veh = pose_local_to_veh.TransformPoint(pl_local);
    em::Point pr_curr_veh = pose_local_to_veh.TransformPoint(pr_local);
    em::Point po_curr_veh = pose_local_to_veh.TransformPoint(po_local);
    SliceLine slice;
    slice.po = po_curr_veh;
    slice.pl = pl_curr_veh;
    slice.pr = pr_curr_veh;
    slice_lines.emplace_back(slice);
  }

  if (slice_lines.size() < 2) {
    HLOG_ERROR << "no enough slice lines: " << slice_lines.size();
    return;
  }

  // 从SliceLines里创建出GroupSegments
  const auto slice_num = slice_lines.size();
  segments->clear();
  segments->reserve(slice_num);
  for (size_t i = 0; i != slice_num - 1; ++i) {
    auto seg = std::make_shared<GroupSegment>();
    seg->start_slice = slice_lines.at(i);
    seg->end_slice = slice_lines.at(i + 1);
    seg->str_id.clear();
    segments->emplace_back(seg);
  }
}

// 将所有车道线点切分到所属GroupSegment：对每根线的点从front到back，判断是否在GroupSegment
// 范围内
void GroupMap::SplitPtsToGroupSeg(std::deque<Line::Ptr>* lines,
                                  std::vector<GroupSegment::Ptr>* segments) {
  // 将线分割并填充到GroupSegment
  for (auto& seg : *segments) {
    // 生成LineSegments
    const auto& start_slice = seg->start_slice;
    Eigen::Vector2f start_po = start_slice.po.head<2>();
    Eigen::Vector2f start_pl = start_slice.pl.head<2>();
    const auto& end_slice = seg->end_slice;
    Eigen::Vector2f end_po = end_slice.po.head<2>();
    Eigen::Vector2f end_pl = end_slice.pl.head<2>();
    for (auto& line : *lines) {
      auto line_seg = std::make_shared<LineSegment>();
      line_seg->id = line->id;
      line_seg->type = line->type;
      line_seg->color = line->color;
      line_seg->lanepos = line->lanepos;
      line_seg->isego = line->isego;
      line_seg->mean_end_heading = line->mean_end_heading;
      line_seg->mean_end_heading_std_dev = line->mean_end_heading_std_dev;
      line_seg->mean_end_interval = line->mean_end_interval;
      while (!line->pts.empty()) {
        Eigen::Vector2f front_pt_xy = line->pts.front().pt.head<2>();
        // 第一个点已经在end_slice的右边，直接跳过这条线
        if (PointInVectorSide(end_po, end_pl, front_pt_xy) > 0) {
          break;
        }
        // 第一个点在end_slice的左边，并且在start_slice的右边，说明属于此segment，加进来
        if (PointInVectorSide(start_po, start_pl, front_pt_xy) >= 0) {
          line_seg->pts.emplace_back(line->pts.front());
        }
        // 把处理完的第一个点pop掉，防止后面重复处理
        line->pts.pop_front();
      }
      if (line_seg->pts.empty()) {
        continue;
      }

      //! TBD: 这里用首尾两点求中值，后续考虑对所有点取平均值得到中心点
      // line_seg->center =
      //     (line_seg->pts.front().pt + line_seg->pts.back().pt) * 0.5;
      int size_line_seg = line_seg->pts.size();
      Eigen::Vector3f point_tmp(0.0, 0.0, 0.0);
      for (int i = 0; i < size_line_seg; ++i) {
        point_tmp += line_seg->pts[i].pt;
      }
      line_seg->center = point_tmp / size_line_seg;
      auto dist =
          PointToVectorDist(start_slice.po, end_slice.po, line_seg->center);
      Eigen::Vector2f center_xy = line_seg->center.head<2>();
      // 在path右边，距离设为负值
      if (PointInVectorSide(start_po, end_po, center_xy) > 0) {
        dist = -1 * dist;
      }
      line_seg->dist_to_path = dist;
      seg->line_segments.emplace_back(line_seg);
    }
  }
}

// 从GroupSegment包含的所有线中，聚合出一个个小的LaneSegment
void GroupMap::GenLaneSegInGroupSeg(std::vector<GroupSegment::Ptr>* segments) {
  std::string last_seg_id = "";
  for (auto& seg : *segments) {
    // 生成LaneSegments
    const auto line_seg_num = seg->line_segments.size();
    if (line_seg_num > 1) {
      // 按与path的距离从大到小排序，得到的结果就是所有线都是从左到右排序号
      std::sort(seg->line_segments.begin(), seg->line_segments.end(),
                [](const LineSegment::Ptr& a, const LineSegment::Ptr& b) {
                  return a->dist_to_path > b->dist_to_path;
                });
      // 按边线距离生成LaneSegment
      for (size_t i = 0; i < line_seg_num - 1; ++i) {
        auto& left_line = seg->line_segments.at(i);
        auto& right_line = seg->line_segments.at(i + 1);
        auto& left_center = left_line->center;
        auto right_center = right_line->center;
        auto dist = Dist(left_center, right_center);
        if (dist < conf_.min_lane_width) {
          if (i == 0 && line_seg_num > 2) {
            std::string index =
                std::to_string(left_line->id) + "_" +
                std::to_string(seg->line_segments.at(i + 2)->id);
            if (last_seg_id.substr(0, index.length()) != index) {
              continue;
            }
            auto& right_line_2 = seg->line_segments.at(i + 2);
            auto& right_center_2 = right_line_2->center;
            auto dist2 = Dist(left_center, right_center_2);
            if (dist2 < conf_.min_lane_width || dist2 > conf_.max_lane_width) {
              continue;
            }
            right_line = right_line_2;
            right_center = right_center_2;
            i = i + 1;
          } else if (i == line_seg_num - 2 && line_seg_num > 2) {
            std::string index = std::to_string(seg->line_segments.at(i)->id);
            std::string index2 =
                index + "_" + std::to_string(seg->line_segments.at(i + 1)->id);
            auto& left_line_2 = seg->line_segments.at(i - 1);
            auto& left_center_2 = left_line_2->center;
            if ((last_seg_id.length() > index.length() &&
                 last_seg_id.substr(last_seg_id.length() - index.length(),
                                    index.length()) == index) ||
                (last_seg_id.length() > index2.length() &&
                 last_seg_id.substr(last_seg_id.length() - index2.length(),
                                    index2.length()) == index2)) {
              continue;
            }

            auto dist2 = Dist(left_center_2, right_center);
            auto dist1 = Dist(left_center_2, left_center);
            if (dist2 < conf_.min_lane_width || dist2 > conf_.max_lane_width) {
              continue;
            } else {
              auto lane_seg = std::make_shared<LaneSegment>();
              lane_seg->left_boundary = left_line_2;
              lane_seg->right_boundary = right_line;
              // 将左右边线id连起来，作为LaneSegment的str_id，
              // 这里使用边线id组成字符串id来标识lane，可以方便后续比对lane与lane是否共线
              lane_seg->str_id = std::to_string(left_line_2->id) + "_" +
                                 std::to_string(right_line->id);
              lane_seg->lanepos_id = std::to_string(left_line_2->lanepos) +
                                     "_" + std::to_string(right_line->lanepos);
              if (dist1 > conf_.min_lane_width &&
                  dist1 < conf_.max_lane_width) {
                seg->lane_segments.back() = lane_seg;
              } else {
                seg->lane_segments.emplace_back(lane_seg);
              }
              break;
            }
          } else {
            continue;
          }
        }
        // if (dist < conf_.min_lane_width || dist > conf_.max_lane_width) {
        //   continue;
        // }
        if (dist > conf_.max_lane_width) {
          continue;
        }
        auto lane_seg = std::make_shared<LaneSegment>();
        lane_seg->left_boundary = left_line;
        lane_seg->right_boundary = right_line;
        // 将左右边线id连起来，作为LaneSegment的str_id，
        // 这里使用边线id组成字符串id来标识lane，可以方便后续比对lane与lane是否共线
        lane_seg->str_id = std::to_string(left_line->id) + "_" +
                           std::to_string(right_line->id);
        lane_seg->lanepos_id = std::to_string(left_line->lanepos) + "_" +
                               std::to_string(right_line->lanepos);
        seg->lane_segments.emplace_back(lane_seg);
      }
    }

    // 将所有LaneSegment的str_id连起来，作为GroupSegment的str_id，
    // 这里GroupSegment的字符串id包含了所有内部lane
    // id信息，可以方便后续比对GroupSegment间 是否包含同样的lane
    for (const auto& lane_seg : seg->lane_segments) {
      if (!seg->str_id.empty()) {
        seg->str_id += "|";
      }
      seg->str_id += lane_seg->str_id;
    }
    last_seg_id = seg->str_id;
  }
}

// 将若干个GroupSegment聚合成Group：
// 当前采用的策略是：相邻两个GroupSegment内包含的LaneSegment一样，那就可以聚合到一起；
// 即每个Group里所有GroupSegment包含的LaneSegment数目都相同，并且LaneSegment的
// str_id（由边线id组成）也相同.
//! TBD：当前聚合方法只考虑了lane的边线id，后续可增加根据lane边线的虚实类型进行聚合
void GroupMap::UniteGroupSegmentsToGroups(
    double stamp, std::vector<GroupSegment::Ptr> group_segments,
    std::vector<Group::Ptr>* groups) {
  if (group_segments.size() > 1) {
    while (group_segments.size() > 1 &&
           group_segments[0]->str_id != group_segments[1]->str_id) {
      group_segments.erase(group_segments.begin());
    }
    if (group_segments.size() > 1) {
      for (size_t i = group_segments.size() - 1; i > 0; i--) {
        if (group_segments[i]->str_id != group_segments[i - 1]->str_id) {
          group_segments.erase(group_segments.begin() + i);
        } else {
          break;
        }
      }
    }
  }

  if (group_segments.size() > 2) {
    for (size_t i = 1; i < group_segments.size() - 1; ++i) {
      if ((group_segments[i]->str_id != group_segments[i - 1]->str_id) &&
          (group_segments[i]->str_id != group_segments[i + 1]->str_id)) {
        group_segments.erase(group_segments.begin() + i);
        i--;
      }
    }
  }

  int grp_idx = -1;
  for (const auto& seg : group_segments) {
    if (seg->str_id == "") {
      continue;
    }
    if (!groups->empty() && seg->str_id == groups->back()->seg_str_id) {
      groups->back()->group_segments.emplace_back(seg);
      continue;
    }

    grp_idx += 1;
    auto grp = std::make_shared<Group>();
    grp->group_segments.emplace_back(seg);
    grp->seg_str_id = seg->str_id;
    grp->str_id = std::string("G") + std::to_string(grp_idx) +
                  std::string("-") + grp->seg_str_id;
    grp->stamp = stamp;
    groups->emplace_back(grp);
  }
  if (grp_idx > 2) {
    for (size_t i = 1; i < groups->size() - 1; ++i) {
      if (groups->at(i - 1)->seg_str_id == groups->at(i + 1)->seg_str_id &&
          groups->at(i)->group_segments.size() < 6) {
        groups->erase(groups->begin() + i);
        i--;
      }
    }
  }
}

float GroupMap::CalculateDistPt(Lane::Ptr lane_in_next, Lane::Ptr lane_in_curr,
                                size_t sizet) {
  return pow(lane_in_next->center_line_pts[0].pt.y() -
                 lane_in_curr->center_line_pts[sizet - 1].pt.y(),
             2) +
         pow(lane_in_next->center_line_pts[0].pt.x() -
                 lane_in_curr->center_line_pts[sizet - 1].pt.x(),
             2);
}

float GroupMap::CalculatePoint2CenterLine(Lane::Ptr lane_in_next,
                                          Lane::Ptr lane_in_curr) {
  return abs(lane_in_next->center_line_pts[0].pt.y() -
             (lane_in_curr->center_line_param[0] +
              lane_in_curr->center_line_param[1] *
                  lane_in_next->center_line_pts[0].pt.x())) /
         sqrt(1 + pow(lane_in_curr->center_line_param[1], 2));
}

float GroupMap::Calculate2CenterlineAngle(Lane::Ptr lane_in_next,
                                          Lane::Ptr lane_in_curr,
                                          size_t sizet) {
  return atan((lane_in_next->center_line_pts[0].pt.y() -
               lane_in_curr->center_line_pts[sizet - 1].pt.y()) /
              (lane_in_next->center_line_pts[0].pt.x() -
               lane_in_curr->center_line_pts[sizet - 1].pt.x())) -
         atan(lane_in_curr->center_line_param[1]);
}

bool GroupMap::IsAccessLane(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next) {
  if (lane_in_curr->center_line_pts.empty() ||
      lane_in_next->center_line_pts.empty()) {
    return false;
  }
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
    Eigen::Vector3f lane_curr2next_vec =
        lane_in_curr->center_line_pts[sizet - 1].pt -
        lane_in_next->center_line_pts[0].pt;
    lane_in_next_norm = lane_in_next_norm.normalized();
    if ((lane_in_next_norm.cross(lane_curr2next_vec)).norm() > 2.0) {
      // curr点是否在next_lane的右侧
      float is_right = lane_in_next_norm.y() * lane_curr2next_vec.x() -
                       lane_in_next_norm.x() * lane_curr2next_vec.y();
      if (is_right > 0 &&
          (lane_in_next->right_boundary->type == em::LaneType_SOLID ||
           lane_in_next->right_boundary->type == em::LaneType_DOUBLE_SOLID ||
           lane_in_next->right_boundary->type ==
               em::LaneType_LEFT_SOLID_RIGHT_DASHED)) {
        HLOG_ERROR << "NOT ACCESS RIGHT!";
        return false;
      } else if (is_right < 0 &&
                 (lane_in_next->left_boundary->type == em::LaneType_SOLID ||
                  lane_in_next->left_boundary->type ==
                      em::LaneType_DOUBLE_SOLID ||
                  lane_in_next->left_boundary->type ==
                      em::LaneType_RIGHT_SOLID_LEFT_DASHED)) {
        HLOG_ERROR << "NOT ACCESS LEFT!";
        return false;
      }
    }
  } else {
    return false;
  }
  return true;
}

bool GroupMap::IsLaneConnet(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next) {
  if (!IsAccessLane(lane_in_curr, lane_in_next)) {
    return false;
  }
  size_t sizet = lane_in_curr->center_line_pts.size();
  float dis_pt = CalculateDistPt(lane_in_next, lane_in_curr, sizet);
  float angel_thresh{0.0};
  float dis_thresh{0.0};
  if (dis_pt < 5 || (lane_in_curr->lanepos_id == lane_in_next->lanepos_id &&
                     lane_in_curr->lanepos_id != "99_99")) {
    HLOG_ERROR << "lane_in_curr=" << lane_in_curr->str_id_with_group
               << "   lane_in_next" << lane_in_next->str_id_with_group
               << "  dis_pt = " << dis_pt
               << " lane_in_curr->lanepos_id =" << lane_in_curr->lanepos_id
               << "  lane_in_next->lanepos_id = " << lane_in_next->lanepos_id;
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
      angel_thresh = 10;
      dis_thresh = 1.5;
    }
    float dis = CalculatePoint2CenterLine(lane_in_next, lane_in_curr);
    float angle = Calculate2CenterlineAngle(lane_in_next, lane_in_curr, sizet);
    // HLOG_ERROR << "angle = "<<angle*180/pi_;
    HLOG_ERROR << "lane_in_curr=" << lane_in_curr->str_id_with_group
               << "   lane_in_next" << lane_in_next->str_id_with_group
               << "  dis2l = " << dis << "  dis thresh = " << dis_thresh
               << " angle = " << abs(angle) * 180 / pi_
               << "  angel_thresh = " << angel_thresh;
    if ((dis < dis_thresh && abs(angle) * 180 / pi_ < angel_thresh)) {
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

bool GroupMap::IsLaneConnet(
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
  if (dis_pt < 5 || (lane_in_curr->lanepos_id == lane_in_next->lanepos_id &&
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
      angel_thresh = 10;
      dis_thresh = 1.5;
    }
    float dis = CalculatePoint2CenterLine(lane_in_next, lane_in_curr);
    float angle = Calculate2CenterlineAngle(lane_in_next, lane_in_curr, sizet);
    // HLOG_ERROR << "angle = "<<angle*180/pi_;
    if ((dis < dis_thresh && abs(angle) * 180 / pi_ < angel_thresh)) {
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

void GroupMap::NextGroupLaneConnect(
    Group::Ptr curr_group, Group::Ptr next_group,
    const std::map<int, std::vector<int>>& curr_group_next_lane) {
  for (const auto& iter : curr_group_next_lane) {
    auto& lane_in_curr = curr_group->lanes[iter.first];
    if (iter.second.size() == 1) {
      auto& lane_in_next = next_group->lanes[iter.second[0]];
      lane_in_curr->next_lane_str_id_with_group.emplace_back(
          lane_in_next->str_id_with_group);
      lane_in_next->prev_lane_str_id_with_group.emplace_back(
          lane_in_curr->str_id_with_group);
      if (lane_in_next->center_line_pts.size() > 2 &&
          lane_in_curr->center_line_pts.size() > 4) {
        // size_t cur_num = lane_in_curr->center_line_pts.size();
        // std::vector<Eigen::Vector3f> tmp_vec;
        // tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num - 5].pt);
        // tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num - 4].pt);
        // tmp_vec.emplace_back(lane_in_next->center_line_pts[0].pt);
        // tmp_vec.emplace_back(lane_in_next->center_line_pts[1].pt);
        // std::vector<Eigen::Vector3f> center_fit;
        // CatmullRom(tmp_vec, &center_fit, 4);
        // Point pt_center;
        // pt_center.type = gm::VIRTUAL;
        // pt_center.pt = center_fit[0];
        // lane_in_curr->center_line_pts[cur_num - 3] = pt_center;
        // pt_center.pt = center_fit[1];
        // lane_in_curr->center_line_pts[cur_num - 2] = pt_center;
        // pt_center.pt = center_fit[2];
        // lane_in_curr->center_line_pts[cur_num - 1] = pt_center;
        lane_in_curr->center_line_pts.erase(
            lane_in_curr->center_line_pts.end() - 3,
            lane_in_curr->center_line_pts.end());
        lane_in_curr->center_line_pts.emplace_back(
            lane_in_next->center_line_pts.front());
      } else if (!lane_in_next->center_line_pts.empty()) {
        lane_in_curr->center_line_pts.emplace_back(
            lane_in_next->center_line_pts.front());
      }
    } else if (iter.second.size() > 1) {
      for (const auto& next_idx : iter.second) {
        auto& lane_in_next = next_group->lanes[next_idx];
        lane_in_curr->next_lane_str_id_with_group.emplace_back(
            lane_in_next->str_id_with_group);
        lane_in_next->prev_lane_str_id_with_group.emplace_back(
            lane_in_curr->str_id_with_group);
        if (lane_in_next->center_line_pts.size() > 4 &&
            lane_in_curr->center_line_pts.size() > 2) {
          // size_t cur_num = lane_in_curr->center_line_pts.size();
          // std::vector<Eigen::Vector3f> tmp_vec;
          // tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num -
          // 2].pt); tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num
          // - 1].pt);
          // tmp_vec.emplace_back(lane_in_next->center_line_pts[3].pt);
          // tmp_vec.emplace_back(lane_in_next->center_line_pts[4].pt);
          // std::vector<Eigen::Vector3f> center_fit;
          // CatmullRom(tmp_vec, &center_fit, 4);
          // Point pt_center;
          // pt_center.type = gm::VIRTUAL;
          // pt_center.pt = center_fit[0];
          // lane_in_next->center_line_pts[0] = pt_center;
          // pt_center.pt = center_fit[1];
          // lane_in_next->center_line_pts[1] = pt_center;
          // pt_center.pt = center_fit[2];
          // lane_in_next->center_line_pts[2] = pt_center;
          // lane_in_next->center_line_pts.insert(
          //     lane_in_next->center_line_pts.begin(),
          //     lane_in_curr->center_line_pts.back());
          lane_in_next->center_line_pts.erase(
              lane_in_next->center_line_pts.begin(),
              lane_in_next->center_line_pts.begin() + 2);
          lane_in_next->center_line_pts[0] =
              lane_in_curr->center_line_pts.back();
        } else if (!lane_in_curr->center_line_pts.empty()) {
          lane_in_next->center_line_pts.insert(
              lane_in_next->center_line_pts.begin(),
              lane_in_curr->center_line_pts.back());
        }
      }
    }
  }
}

float GroupMap::LaneDist(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next) {
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

void GroupMap::FindGroupNextLane(Group::Ptr curr_group, Group::Ptr next_group) {
  // 如果next_group的lanes比curr_group的lanes多或者相等的话，currgroup->lane有一个后继
  // 如果next_group的lanes比curr_group的lanes少的话，next->lane有一个前驱
  // 如果左右车道线有一根的trackid相等的话，有连接关系

  // 用hash先存下关联关系
  std::map<int, std::vector<int>>
      curr_group_next_lane;  // 当前车道groupindex,后继在下个group的index
  std::map<int, std::vector<int>> next_group_prev_lane;
  if (curr_group->lanes.size() >= next_group->lanes.size()) {
    HLOG_ERROR << "CUR >= NEXT";
    for (int i = 0; i < curr_group->lanes.size(); ++i) {
      auto& lane_in_curr = curr_group->lanes[i];
      if (lane_in_curr->next_lane_str_id_with_group.empty()) {
        int trackid_same_find = 0;
        for (int j = 0; j < next_group->lanes.size(); ++j) {
          auto& lane_in_next = next_group->lanes[j];
          if (lane_in_next->left_boundary->id ==
                  lane_in_curr->left_boundary->id ||
              lane_in_next->right_boundary->id ==
                  lane_in_curr->right_boundary->id) {
            curr_group_next_lane[i].emplace_back(j);
            next_group_prev_lane[j].emplace_back(i);
            trackid_same_find = 1;
            break;
          }
        }
        if (trackid_same_find == 0) {
          bool has_next_lane = false;
          for (int j = 0; j < next_group->lanes.size(); ++j) {
            has_next_lane =
                IsLaneConnet(curr_group, next_group, i, j,
                             &curr_group_next_lane, &next_group_prev_lane) ||
                has_next_lane;
          }
          if (!has_next_lane) {
            // 目前只考虑最左或最右线关联问题
            // 这里仅增加收缩的lane，目的是为了排除正常宽度的lane也被认为是merge的lane
            double curr_len = CalcLaneLength(lane_in_curr);
            bool shrink = IsLaneShrink(lane_in_curr);
            const float dis_thresh = 4.5;
            if (i == 0 && curr_len > kMergeLengthThreshold && shrink &&
                IsAccessLane(lane_in_curr, next_group->lanes[0]) &&
                LaneDist(lane_in_curr, next_group->lanes[0]) < dis_thresh) {
              curr_group_next_lane[i].emplace_back(0);
              next_group_prev_lane[0].emplace_back(i);
            } else if (i == curr_group->lanes.size() - 1 &&
                       curr_len > kMergeLengthThreshold && shrink &&
                       IsAccessLane(lane_in_curr, next_group->lanes.back()) &&
                       LaneDist(lane_in_curr, next_group->lanes.back()) <
                           dis_thresh) {
              curr_group_next_lane[i].emplace_back(next_group->lanes.size() -
                                                   1);

              next_group_prev_lane[static_cast<int>(next_group->lanes.size() -
                                                    1)]
                  .emplace_back(i);
            }
          }
        }
      }
    }
    for (auto& iter : curr_group_next_lane) {
      if (iter.second.size() > 1) {
        int best = iter.second[0];
        for (auto& next_lane_idx : iter.second) {
          if (best == next_lane_idx) {
            continue;
          }
          auto& lane_in_next = next_group->lanes[next_lane_idx];
          auto& lane_best = next_group->lanes[best];
          auto& lane_in_curr = curr_group->lanes[iter.first];
          if (next_group_prev_lane[next_lane_idx].size() == 1 &&
              next_group_prev_lane[best].size() > 1) {
            best = next_lane_idx;
          } else if ((next_group_prev_lane[next_lane_idx].size() > 1 &&
                      next_group_prev_lane[best].size() > 1) ||
                     (next_group_prev_lane[next_lane_idx].size() == 1 &&
                      next_group_prev_lane[best].size() == 1)) {
            size_t sizet = lane_in_curr->center_line_pts.size();
            if (!lane_in_curr->center_line_param.empty() &&
                !lane_in_next->center_line_param_front.empty() &&
                !lane_best->center_line_param_front.empty()) {
              float angle1 =
                  Calculate2CenterlineAngle(lane_in_next, lane_in_curr, sizet);
              float angle2 =
                  Calculate2CenterlineAngle(lane_best, lane_in_curr, sizet);
              float dis1 =
                  CalculatePoint2CenterLine(lane_in_next, lane_in_curr);
              float dis2 = CalculatePoint2CenterLine(lane_best, lane_in_curr);
              if (angle2 > angle1 + 1 * 180 / pi_ && dis2 > dis1 + 0.2) {
                best = next_lane_idx;
              }
            } else {
              float dis_pt = CalculateDistPt(lane_in_next, lane_in_curr, sizet);
              float dis_pt2 = CalculateDistPt(lane_best, lane_in_curr, sizet);
              if (dis_pt < dis_pt2) {
                best = next_lane_idx;
              }
            }
          }
        }
        for (auto& next_lane_idx : iter.second) {
          if (next_lane_idx == best) {
            continue;
          }
          for (auto iter_next = next_group_prev_lane[next_lane_idx].begin();
               iter_next != next_group_prev_lane[next_lane_idx].end();
               ++iter_next) {
            // 删除相关的prev
            if (*iter_next == iter.first) {
              next_group_prev_lane[next_lane_idx].erase(iter_next);
              break;
            }
          }
        }
        iter.second.clear();
        iter.second.emplace_back(best);
      }
    }
  } else {
    HLOG_ERROR << "CUR < NEXT";
    for (int i = 0; i < next_group->lanes.size(); ++i) {
      auto& lane_in_next = next_group->lanes[i];
      if (lane_in_next->prev_lane_str_id_with_group.empty()) {
        int trackid_same_find = 0;
        for (int j = 0; j < curr_group->lanes.size(); ++j) {
          auto& lane_in_curr = curr_group->lanes[j];
          if (lane_in_next->left_boundary->id ==
                  lane_in_curr->left_boundary->id ||
              lane_in_next->right_boundary->id ==
                  lane_in_curr->right_boundary->id) {
            curr_group_next_lane[j].emplace_back(i);
            next_group_prev_lane[i].emplace_back(j);
            trackid_same_find = 1;
            break;
          }
        }
        if (trackid_same_find == 0) {
          bool has_prev_lane = false;
          for (int j = 0; j < curr_group->lanes.size(); ++j) {
            has_prev_lane =
                IsLaneConnet(curr_group, next_group, j, i,
                             &curr_group_next_lane, &next_group_prev_lane) ||
                has_prev_lane;
          }
          double next_len = CalcLaneLength(lane_in_next);
          bool next_long_enough = next_len > kSplitLengthThreshold;
          if (!has_prev_lane && next_long_enough &&
              lane_in_next->center_line_pts.size() >= 2) {
            const float kSplitDistThreshold = 3.5;
            Eigen::Vector2f next_pt0(
                lane_in_next->center_line_pts.at(0).pt.x(),
                lane_in_next->center_line_pts.at(0).pt.y());
            Eigen::Vector2f next_pt1(
                lane_in_next->center_line_pts.at(1).pt.x(),
                lane_in_next->center_line_pts.at(1).pt.y());
            if (i == 0 && (!curr_group->lanes.empty()) &&
                (!curr_group->lanes.front()->center_line_pts.empty())) {
              auto& lane_in_curr = curr_group->lanes.at(0);
              Eigen::Vector2f curr_pt(
                  lane_in_curr->center_line_pts.back().pt.x(),
                  lane_in_curr->center_line_pts.back().pt.y());
              float dist = PointToVectorDist(next_pt0, next_pt1, curr_pt);
              if (dist < kSplitDistThreshold &&
                  IsAccessLane(lane_in_curr, lane_in_next)) {
                next_group_prev_lane[i].emplace_back(0);
                curr_group_next_lane[0].emplace_back(i);
              }
            } else if (i == next_group->lanes.size() - 1 &&
                       (!curr_group->lanes.empty()) &&
                       (!curr_group->lanes.back()->center_line_pts.empty())) {
              auto& lane_in_curr = curr_group->lanes.back();
              Eigen::Vector2f curr_pt(
                  lane_in_curr->center_line_pts.back().pt.x(),
                  lane_in_curr->center_line_pts.back().pt.y());
              float dist = PointToVectorDist(next_pt0, next_pt1, curr_pt);
              if (dist < kSplitDistThreshold &&
                  IsAccessLane(lane_in_curr, lane_in_next)) {
                next_group_prev_lane[i].emplace_back(curr_group->lanes.size() -
                                                     1);
                curr_group_next_lane[static_cast<int>(curr_group->lanes.size() -
                                                      1)]
                    .emplace_back(i);
              }
            }
          }
        }
      }
    }
    for (auto& iter : next_group_prev_lane) {
      if (iter.second.size() > 1) {
        int best = iter.second[0];
        for (auto& prev_lane_idx : iter.second) {
          if (best == prev_lane_idx) {
            continue;
          }
          auto& lane_in_curr = curr_group->lanes[prev_lane_idx];
          auto& lane_best = curr_group->lanes[best];
          auto& lane_in_next = next_group->lanes[iter.first];
          if (curr_group_next_lane[prev_lane_idx].size() == 1 &&
              curr_group_next_lane[best].size() > 1) {
            best = prev_lane_idx;
          } else if ((curr_group_next_lane[prev_lane_idx].size() > 1 &&
                      curr_group_next_lane[best].size() > 1) ||
                     (curr_group_next_lane[prev_lane_idx].size() == 1 &&
                      curr_group_next_lane[best].size() == 1)) {
            size_t sizet1 = lane_in_curr->center_line_pts.size();
            size_t sizet2 = lane_best->center_line_pts.size();
            if (!lane_in_curr->center_line_param.empty() &&
                !lane_best->center_line_param.empty() &&
                !lane_in_next->center_line_param_front.empty()) {
              float angle1 =
                  Calculate2CenterlineAngle(lane_in_next, lane_in_curr, sizet1);
              float angle2 =
                  Calculate2CenterlineAngle(lane_in_next, lane_best, sizet2);
              float dis1 =
                  CalculatePoint2CenterLine(lane_in_next, lane_in_curr);
              float dis2 = CalculatePoint2CenterLine(lane_in_next, lane_best);
              if (angle2 > angle1 + 1 * 180 / pi_ && dis2 > dis1 + 0.2) {
                best = prev_lane_idx;
              }
            } else {
              float dis_pt =
                  CalculateDistPt(lane_in_next, lane_in_curr, sizet1);
              float dis_pt2 = CalculateDistPt(lane_in_next, lane_best, sizet2);
              if (dis_pt < dis_pt2) {
                best = prev_lane_idx;
              }
            }
          }
        }
        for (auto& prev_lane_idx : iter.second) {
          if (prev_lane_idx == best) {
            continue;
          }
          for (auto iter_prev = curr_group_next_lane[prev_lane_idx].begin();
               iter_prev != curr_group_next_lane[prev_lane_idx].end();
               ++iter_prev) {
            if (*iter_prev == iter.first) {
              curr_group_next_lane[prev_lane_idx].erase(iter_prev);
              break;
            }
          }
        }
        iter.second.clear();
        iter.second.emplace_back(best);
      }
    }
  }
  NextGroupLaneConnect(curr_group, next_group, curr_group_next_lane);
  // for (auto& lane_in_curr : curr_group->lanes) {
  //   if (lane_in_curr->next_lane_str_id_with_group.empty()) {
  //     int trackid_same_find = 0;
  //     for (auto& lane_in_next : next_group->lanes) {
  //       if (lane_in_next->left_boundary->id ==
  //               lane_in_curr->left_boundary->id ||
  //           lane_in_next->right_boundary->id ==
  //               lane_in_curr->right_boundary->id) {
  //         BuildVirtualProSucLane(lane_in_curr, lane_in_next, 5.0);
  //         // lane_in_curr->next_lane_str_id_with_group
  //         trackid_same_find = 1;
  //         break;
  //       }
  //     }
  //     if (trackid_same_find == 0) {
  //       for (auto& lane_in_next : next_group->lanes) {
  //         size_t sizet = lane_in_curr->center_line_pts.size();
  //         float dis_pt =
  //             pow(lane_in_next->center_line_pts[0].pt.y() -
  //                     lane_in_curr->center_line_pts[sizet - 1].pt.y(),
  //                 2) +
  //             pow(lane_in_next->center_line_pts[0].pt.x() -
  //                     lane_in_curr->center_line_pts[sizet - 1].pt.x(),
  //                 2);
  //         float angel_thresh = 0.0, dis_thresh = 0.0;
  //         if (!lane_in_curr->center_line_param.empty() &&
  //             !lane_in_next->center_line_pts.empty()) {
  //           if (dis_pt > 10000) {
  //             // 差的太远不计算
  //             continue;
  //           } else if (dis_pt > 400) {
  //             angel_thresh = 2;
  //             dis_thresh = 3;
  //           } else {
  //             angel_thresh = 10;
  //             dis_thresh = 1;
  //           }
  //           float dis = abs(lane_in_next->center_line_pts[0].pt.y() -
  //                           (lane_in_curr->center_line_param[0] +
  //                             lane_in_curr->center_line_param[1] *
  //                                 lane_in_next->center_line_pts[0].pt.x())) /
  //                       sqrt(pow(lane_in_curr->center_line_param[0], 2) +
  //                             pow(lane_in_curr->center_line_param[1], 2));
  //           float angle =
  //               atan((lane_in_next->center_line_pts[0].pt.y() -
  //                     lane_in_curr->center_line_pts[sizet - 1].pt.y()) /
  //                     (lane_in_next->center_line_pts[0].pt.x() -
  //                     lane_in_curr->center_line_pts[sizet - 1].pt.x())) -
  //               atan(lane_in_curr->center_line_param[1]);
  //           // HLOG_ERROR << "angle = "<<angle*180/pi_;
  //           if ((dis < dis_thresh && abs(angle) * 180 / pi_ < angel_thresh))
  //           {
  //             //! TBD:
  //             //!
  //             这里直接把后一个lane中心线的第一个点加到前一个lane中心线的末尾，
  //             //!
  //             后续需要考虑某些异常情况，比如后一个lane中心线的第一个点在前一个lane中心线最后
  //             //!
  //             一个点的后方，这样直连就导致整个中心线往后折返了；以及还要考虑横向偏移较大时不平
  //             //! 滑的问题
  //             BuildVirtualProSucLane(lane_in_curr, lane_in_next, dis_pt);
  //             break;
  //           }
  //         }
  //         if (dis_pt < 9 ||
  //             lane_in_curr->lanepos_id == lane_in_next->lanepos_id) {
  //           BuildVirtualProSucLane(lane_in_curr, lane_in_next, dis_pt);
  //           break;
  //         }
  //       }
  //     }
  //   }
  // }
}
bool GroupMap::IsZebraIn(const Eigen::Vector2f& curr_start_pl,
                         const Eigen::Vector2f& curr_start_pr,
                         const Eigen::Vector2f& next_start_pl,
                         const Eigen::Vector2f& next_start_pr) {
  for (auto zebra : zebra_) {
    Eigen::Vector2f zebra_center(0, 0);
    for (auto pt : zebra.second->polygon.points) {
      zebra_center += pt.head<2>();
    }
    zebra_center /= static_cast<float>(zebra.second->polygon.points.size());
    if (PointInVectorSide(curr_start_pr, curr_start_pl, zebra_center) >= 0 &&
        PointInVectorSide(next_start_pr, next_start_pl, zebra_center) <= 0) {
      return true;
    }
  }
  return false;
}

void GroupMap::RelateGroups(std::vector<Group::Ptr>* groups, double stamp) {
  std::vector<Group::Ptr> group_virtual;
  int erase_grp_idx = -1;
  for (size_t grp_idx = 0; grp_idx < groups->size() - 1; ++grp_idx) {
    std::vector<Lane::Ptr> lane_virtual;
    auto& curr_group = groups->at(grp_idx);
    auto& next_group = groups->at(grp_idx + 1);
    if (curr_group->group_segments.size() < 2 ||
        next_group->group_segments.size() < 2 || curr_group->lanes.size() < 1 ||
        next_group->lanes.size() < 1) {
      continue;
    }
    auto curr_group_size = curr_group->group_segments.size();
    double group_distance =
        (curr_group->group_segments[curr_group_size - 1]->end_slice.po -
         next_group->group_segments.front()->start_slice.po)
            .norm();
    double min_dis_curr_lane_next_group = DBL_MAX;
    for (auto& lane : curr_group->lanes) {
      double diss = (lane->center_line_pts.back().pt -
                     next_group->group_segments.front()->start_slice.po)
                        .norm();
      if (diss < min_dis_curr_lane_next_group) {
        min_dis_curr_lane_next_group = diss;
      }
    }
    auto curr_grp_start_slice = curr_group->group_segments.front()->start_slice;
    auto curr_grp_end_slice = curr_group->group_segments.back()->end_slice;
    auto next_grp_start_slice = next_group->group_segments.front()->start_slice;
    Eigen::Vector2f curr_start_pl(curr_grp_start_slice.pl.x(),
                                  curr_grp_start_slice.pl.y());
    Eigen::Vector2f curr_start_pr(curr_grp_start_slice.pr.x(),
                                  curr_grp_start_slice.pr.y());
    Eigen::Vector2f curr_end_pl(curr_grp_end_slice.pl.x(),
                                curr_grp_end_slice.pl.y());
    Eigen::Vector2f curr_end_pr(curr_grp_end_slice.pr.x(),
                                curr_grp_end_slice.pr.y());
    Eigen::Vector2f next_start_pl(next_grp_start_slice.pl.x(),
                                  next_grp_start_slice.pl.y());
    Eigen::Vector2f next_start_pr(next_grp_start_slice.pr.x(),
                                  next_grp_start_slice.pr.y());
    Eigen::Vector2f curr_pos(0, 0);
    if (group_distance > 10 && min_dis_curr_lane_next_group > 10) {
      //  ||
      //  (group_distance > 5 && IsZebraIn(curr_start_pl, curr_start_pr,
      //                                  next_start_pl, next_start_pr))
      // 小路口及以上并且正处于路口
      bool veh_in_this_junction = false;
      // 判断车是否在curr和next group范围内，如果是就认为在路口内
      if (PointInVectorSide(curr_start_pr, curr_start_pl, curr_pos) >= 0 &&
          PointInVectorSide(next_start_pr, next_start_pl, curr_pos) <= 0) {
        veh_in_this_junction = true;
      }
      //! 注意： 这里判断车与curr
      //! group的距离是否小于规控要求的长度，如果小于表示前方太短了，
      //! 此时也认为是在路口内，这样就可以将next group里车道删除，正常的使用curr
      //! group向前预测
      if (PointInVectorSide(curr_end_pr, curr_end_pl, curr_pos) <= 0 &&
          PointToVectorDist(curr_end_pr, curr_end_pl, curr_pos) <
              conf_.predict_farthest_dist) {
        veh_in_this_junction = true;
      }

      auto cur_group_v =
          (curr_group->group_segments[curr_group_size - 1]->end_slice.po -
           curr_group->group_segments[curr_group_size - 2]->end_slice.po)
              .normalized();
      auto next_group_v = (next_group->group_segments[1]->end_slice.po -
                           next_group->group_segments[0]->end_slice.po)
                              .normalized();
      if (cur_group_v.dot(next_group_v) > 0.8) {
#if 0
        // 角度差别不大的情况下
        if (curr_group->lanes.size() == next_group->lanes.size()) {
          for (size_t lane_idx = 0; lane_idx < curr_group->lanes.size(); ++lane_idx) {
            if (curr_group->lanes[lane_idx]->next_lane_str_id_with_group.empty()) {
              BuildCrossingLane(&lane_virtual, curr_group->lanes[lane_idx],
                                next_group->lanes[lane_idx]);
            }
          }
        } else {
          size_t next_group_lane_min_index = 0,
                 next_group_lane_max_index = next_group->lanes.size() - 1;
          for (size_t curr_lane_idx = 0;
               curr_lane_idx < curr_group->lanes.size(); ++curr_lane_idx) {
            auto& lane_in_curr = curr_group->lanes[curr_lane_idx];
            if (!lane_in_curr->next_lane_str_id_with_group.empty()) {
              for (size_t next_lane_idx = 0;
                   next_lane_idx < next_group_lane_max_index; ++next_lane_idx) {
                if (lane_in_curr->next_lane_str_id_with_group[0] ==
                    next_group->lanes[next_lane_idx]->str_id_with_group) {
                  next_group_lane_min_index = next_lane_idx;
                  break;
                }
              }
            } else {
              if (curr_lane_idx == 0 &&
                  next_group->lanes.size() < curr_group->lanes.size()) {
                //
                BuildCrossingLane(&lane_virtual, lane_in_curr,
                                  next_group->lanes[0]);
              } else if ((curr_lane_idx == curr_group->lanes.size() - 1 &&
                          next_group->lanes.size() <
                              curr_group->lanes.size()) ||
                         next_group_lane_min_index ==
                             next_group_lane_max_index) {
                BuildCrossingLane(&lane_virtual, lane_in_curr,
                                  next_group->lanes[next_group_lane_max_index]);
              }
//              else if (curr_group->lanes.size() - 1 - curr_lane_idx ==
//                         next_group_lane_max_index -
//                             next_group_lane_min_index) {
//                BuildCrossingLane(&lane_virtual, lane_in_curr,
//                                  next_group->lanes[next_group_lane_min_index]);
//                next_group_lane_min_index += 1;
//              }
              else { // NOLINT
                if (lane_in_curr->center_line_param.size() > 0) {
                  size_t sizet = lane_in_curr->center_line_pts.size();
                  auto lane_in_next =
                      next_group->lanes[next_group_lane_min_index];
                  float angle1 =
                      atan((lane_in_next->center_line_pts[0].pt.y() -
                            lane_in_curr->center_line_pts[sizet - 1].pt.y()) /
                           (lane_in_next->center_line_pts[0].pt.x() -
                            lane_in_curr->center_line_pts[sizet - 1].pt.x())) -
                      atan(lane_in_curr->center_line_param[1]);
                  auto lane_in_next2 =
                      next_group->lanes[next_group_lane_min_index + 1];
                  float angle2 =
                      atan((lane_in_next2->center_line_pts[0].pt.y() -
                            lane_in_curr->center_line_pts[sizet - 1].pt.y()) /
                           (lane_in_next2->center_line_pts[0].pt.x() -
                            lane_in_curr->center_line_pts[sizet - 1].pt.x())) -
                      atan(lane_in_curr->center_line_param[1]);
                  if (lane_in_next->center_line_param_front.size() > 0 &&
                      lane_in_next2->center_line_param_front.size() > 0) {
                    float angle_diss1 =
                        atan(lane_in_next->center_line_param_front[1]) -
                        atan(lane_in_curr->center_line_param[1]);
                    float angle_diss2 =
                        atan(lane_in_next->center_line_param_front[1]) -
                        atan(lane_in_curr->center_line_param[1]);
//                    abs(angle_diss1) >
//                    abs(angle_diss2) + 1.0 / 180 * 3.14 &&
                    if (abs(lane_in_next->center_line_param_front[0] -
                            lane_in_curr->center_line_param[0]) >
                            abs(lane_in_next2->center_line_param_front[0] -
                                lane_in_curr->center_line_param[0])) {
                      next_group_lane_min_index += 1;
                    }
                    BuildCrossingLane(
                        &lane_virtual, lane_in_curr,
                        next_group->lanes[next_group_lane_min_index]);
                    if (next_group->lanes.size() > curr_group->lanes.size()) {
                      next_group_lane_min_index += 1;
                    }
                  } else if (abs(angle1) > abs(angle2) ||
                             lane_in_next2->lanepos_id ==
                                 lane_in_curr->lanepos_id) {
                    next_group_lane_min_index += 1;
                    BuildCrossingLane(
                        &lane_virtual, lane_in_curr,
                        next_group->lanes[next_group_lane_min_index]);
                  } else {
                    BuildCrossingLane(
                        &lane_virtual, lane_in_curr,
                        next_group->lanes[next_group_lane_min_index]);
                    if (next_group->lanes.size() > curr_group->lanes.size()) {
                      next_group_lane_min_index += 1;
                    }
                  }
                } else {
                  if (curr_group->lanes.size() > next_group->lanes.size()) {
                    size_t mod =
                        curr_lane_idx % (next_group_lane_max_index + 1);
                    size_t div =
                        curr_lane_idx / (next_group_lane_max_index + 1);
                    if (mod > (next_group_lane_max_index + 1) / 2) {
                      next_group_lane_min_index += 1;
                    }
                    next_group_lane_min_index =
                        ((mod > (next_group_lane_max_index + 1) / 2) ? 1 : 0) +
                        div;
                  } else {
                    size_t mod =
                        next_group_lane_min_index % (curr_group->lanes.size());
                    size_t div =
                        next_group_lane_min_index / (curr_group->lanes.size());
                    if (mod > (next_group_lane_max_index + 1) / 2) {
                      next_group_lane_min_index += 1;
                    }
                    next_group_lane_min_index =
                        ((mod > (next_group_lane_max_index + 1) / 2) ? 1 : 0) +
                        div;
                  }

                  BuildCrossingLane(
                      &lane_virtual, lane_in_curr,
                      next_group->lanes[next_group_lane_min_index]);
                }
              }
            }
          }
        }
#else
        auto dist_to_slice =
            PointToVectorDist(next_start_pl, next_start_pr, curr_pos);
        //! 注意：当前路口策略是：先使用预测的车道往前行驶一半路口长度，之后再使用前方车道进行关联.
        //! 为了能使用curr group的车道向前预测，这里把next
        //! group的索引标记出来，后面会将next group 里车道都删掉，这样curr
        //! group就变成最后一个group了，后续就能正常使用其向前预测了.
        if (dist_to_slice > (group_distance * 0.5) && veh_in_this_junction) {
          erase_grp_idx = grp_idx + 1;
          break;
        }

        if (veh_in_this_junction) {
          // 找到与curr_group最近的历史车辆位置
          Pose nearest;
          nearest.stamp = -1;
          float min_dist = FLT_MAX;
          for (const auto& p : path_in_curr_pose_) {
            Eigen::Vector2f pt(p.pos.x(), p.pos.y());
            float dist = PointToVectorDist(curr_end_pl, curr_end_pr, pt);
            if (dist < min_dist) {
              min_dist = dist;
              nearest = p;
              nearest.stamp = 0;
            }
          }

          // 找到与历史车辆位置最接近的curr_lane作为当前所在lane
          min_dist = FLT_MAX;
          Lane::Ptr ego_curr_lane = nullptr;
          if (nearest.stamp >= 0) {
            Eigen::Vector3f temp_pt(1, 0, 0);
            // 转到当前车体系下
            Eigen::Vector3f temp_pt_curr_veh =
                nearest.quat * temp_pt + nearest.pos;
            Eigen::Vector2f p1(nearest.pos.x(), nearest.pos.y());
            Eigen::Vector2f p0(temp_pt_curr_veh.x(), temp_pt_curr_veh.y());
            for (auto& curr_lane : curr_group->lanes) {
              Eigen::Vector2f pt(curr_lane->center_line_pts.back().pt.x(),
                                 curr_lane->center_line_pts.back().pt.y());
              float dist = PointToVectorDist(p0, p1, pt);
              if (dist < min_dist) {
                min_dist = dist;
                ego_curr_lane = curr_lane;
              }
            }
          }

          Eigen::Vector2f thresh_v(std::cos(conf_.junction_heading_diff),
                                   std::sin(conf_.junction_heading_diff));
          Eigen::Vector2f n(1, 0);  // 车前向量
          float thresh_len = std::abs(thresh_v.transpose() * n);

          Lane::Ptr best_next_lane = nullptr;
          float max_len = 0;
          for (auto& next_lane : next_group->lanes) {
            Eigen::Vector2f p0(0, 0);
            Eigen::Vector2f p1(next_lane->center_line_pts.front().pt.x(),
                               next_lane->center_line_pts.front().pt.y());
            Eigen::Vector2f v = p1 - p0;
            v.normalize();
            float len = std::abs(v.transpose() * n);
            if (len > max_len && len >= thresh_len) {
              max_len = len;
              best_next_lane = next_lane;
            }
          }

          float min_offset = FLT_MAX;
          const float kOffsetThreshold = 3.5 * 0.5;  // 半个3.5米车道宽度
          float check_offset_thresh = -10000;
          if (conf_.junction_heading_diff > 0.001) {
            check_offset_thresh =
                kOffsetThreshold / std::tan(conf_.junction_heading_diff);
          }
          // 如果此时距离足够近，并且通过角度未找到合适的next
          // lane，此时通过找最小的横向偏移来确定next lane
          if (best_next_lane == nullptr &&
              dist_to_slice <= check_offset_thresh) {
            for (auto& next_lane : next_group->lanes) {
              float offset =
                  std::abs(next_lane->center_line_pts.front().pt.y());
              if (offset < min_offset && offset <= kOffsetThreshold) {
                min_offset = offset;
                best_next_lane = next_lane;
              }
            }
          }

          if (ego_curr_lane != nullptr && best_next_lane != nullptr) {
            BuildCrossingLane(&lane_virtual, ego_curr_lane, best_next_lane);
          }
        }

#endif
      }
    } else {
      // 非路口
      FindGroupNextLane(curr_group, next_group);
    }
    if (lane_virtual.size() > 0) {
      BuildVirtualGroup(lane_virtual, &group_virtual, stamp);
    }
  }

  if (erase_grp_idx > 0) {
    groups->erase(groups->begin() + erase_grp_idx, groups->end());
    if (!groups->empty()) {
      groups->back()->is_last_after_erased = true;
    }
  }

  for (int i = groups->size() - 1; i >= 0; --i) {
    if (group_virtual.empty()) {
      break;
    }
    if (groups->at(i)->str_id + 'V' == group_virtual.back()->str_id) {
      groups->insert(groups->begin() + i + 1, group_virtual.back());
      group_virtual.pop_back();
    }
  }
}

void GroupMap::BuildCrossingLane(std::vector<Lane::Ptr>* lane_virtual,
                                 Lane::Ptr lane_in_curr,
                                 Lane::Ptr lane_in_next) {
  Lane lane_pre;
  LineSegment left_bound;
  left_bound.id = lane_in_curr->left_boundary->id;
  left_bound.lanepos = lane_in_curr->left_boundary->lanepos;
  left_bound.type = em::LaneType_DASHED;  // lane_in_curr->left_boundary->type;
  left_bound.color = em::WHITE;           // lane_in_curr->left_boundary->color;
  left_bound.isego = lane_in_curr->left_boundary->isego;
  left_bound.mean_end_heading = lane_in_curr->left_boundary->mean_end_heading;
  left_bound.mean_end_heading_std_dev =
      lane_in_curr->left_boundary->mean_end_heading_std_dev;
  left_bound.mean_end_interval = lane_in_curr->left_boundary->mean_end_interval;
  size_t index_left = lane_in_curr->left_boundary->pts.size();
  Point left_pt_pre(VIRTUAL,
                    lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                    lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                    lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
  LineSegment right_bound;
  right_bound.id = lane_in_curr->right_boundary->id;
  right_bound.lanepos = lane_in_curr->right_boundary->lanepos;
  right_bound.type =
      em::LaneType_DASHED;        // lane_in_curr->right_boundary->type;
  right_bound.color = em::WHITE;  // lane_in_curr->right_boundary->color;
  right_bound.isego = lane_in_curr->right_boundary->isego;
  right_bound.mean_end_heading = lane_in_curr->right_boundary->mean_end_heading;
  right_bound.mean_end_heading_std_dev =
      lane_in_curr->right_boundary->mean_end_heading_std_dev;
  right_bound.mean_end_interval =
      lane_in_curr->right_boundary->mean_end_interval;
  size_t index_right = lane_in_curr->right_boundary->pts.size();
  Point right_pt_pre(VIRTUAL,
                     lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
                     lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
                     lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
  std::vector<Point> ctl_pt;
  size_t index_center = lane_in_curr->center_line_pts.size();
  Point center_pt_pre(VIRTUAL,
                      lane_in_curr->center_line_pts[index_center - 1].pt.x(),
                      lane_in_curr->center_line_pts[index_center - 1].pt.y(),
                      lane_in_curr->center_line_pts[index_center - 1].pt.z());
  if (is_cross_.is_crossing_ && is_cross_.along_path_dis_.norm() > 2.0) {
    float i = 0.0;
    float len = is_cross_.along_path_dis_.norm();
    left_bound.pts.emplace_back(left_pt_pre);
    right_bound.pts.emplace_back(right_pt_pre);
    ctl_pt.emplace_back(center_pt_pre);
    while (i < len - 1.0) {
      i = i + 1.0;
      float minus_x = 1 / len * is_cross_.along_path_dis_.x();
      float minus_y = 1 / len * is_cross_.along_path_dis_.y();
      float pre_x = left_pt_pre.pt.x() - minus_x;
      float pre_y = left_pt_pre.pt.y() - minus_y;
      left_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
      left_bound.pts.emplace_back(left_pt_pre);

      pre_x = right_pt_pre.pt.x() - minus_x;
      pre_y = right_pt_pre.pt.y() - minus_y;
      right_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
      right_bound.pts.emplace_back(right_pt_pre);

      pre_x = center_pt_pre.pt.x() - minus_x;
      pre_y = center_pt_pre.pt.y() - minus_y;
      center_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
      ctl_pt.emplace_back(center_pt_pre);
    }
    i = 0.0;
    em::Point param_left =
        lane_in_next->left_boundary->pts[0].pt - left_bound.pts.back().pt;
    len = param_left.norm();
    while (i < len - 1.0) {
      left_bound.pts.emplace_back(left_pt_pre);
      i = i + 1.0;
      float pre_x = left_pt_pre.pt.x() + 1 / len * param_left.x();
      float pre_y = left_pt_pre.pt.y() + 1 / len * param_left.y();
      left_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    }
    if (left_bound.pts.empty()) {
      return;
    }
    i = 0.0;
    em::Point param_right =
        lane_in_next->right_boundary->pts[0].pt - right_bound.pts.back().pt;
    len = param_right.norm();
    while (i < len - 1.0) {
      right_bound.pts.emplace_back(right_pt_pre);
      i = i + 1.0;
      float pre_x = right_pt_pre.pt.x() + 1 / len * param_right.x();
      float pre_y = right_pt_pre.pt.y() + 1 / len * param_right.y();
      right_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    }
    if (right_bound.pts.empty()) {
      return;
    }
    i = 0.0;
    em::Point param_center =
        lane_in_next->center_line_pts[0].pt - ctl_pt.back().pt;
    len = param_center.norm();
    while (i < len - 1.0) {
      i = i + 1.0;
      ctl_pt.emplace_back(center_pt_pre);
      float pre_x = center_pt_pre.pt.x() + 1 / len * param_center.x();
      float pre_y = center_pt_pre.pt.y() + 1 / len * param_center.y();
      center_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    }
    ctl_pt.emplace_back(lane_in_next->center_line_pts.front());
    if (ctl_pt.empty()) {
      return;
    }

  } else {
    std::vector<Point> tmp;
    tmp.emplace_back(lane_in_curr->left_boundary->pts[index_left - 1]);
    tmp.emplace_back(lane_in_next->left_boundary->pts[0]);
    std::vector<double> param_left = FitLaneline(tmp);
    while (left_pt_pre.pt.x() < lane_in_next->left_boundary->pts[0].pt.x()) {
      left_bound.pts.emplace_back(left_pt_pre);
      float pre_x = left_pt_pre.pt.x() + 1.0;
      float pre_y = param_left[0] + param_left[1] * pre_x;
      left_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    }
    if (left_bound.pts.empty()) {
      return;
    }

    tmp.clear();
    tmp.emplace_back(lane_in_curr->right_boundary->pts[index_right - 1]);
    tmp.emplace_back(lane_in_next->right_boundary->pts[0]);
    param_left = FitLaneline(tmp);
    while (right_pt_pre.pt.x() < lane_in_next->right_boundary->pts[0].pt.x()) {
      right_bound.pts.emplace_back(right_pt_pre);
      float pre_x = right_pt_pre.pt.x() + 1.0;
      float pre_y = param_left[0] + param_left[1] * pre_x;
      right_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    }
    if (right_bound.pts.empty()) {
      return;
    }
    tmp.clear();
    tmp.emplace_back(lane_in_curr->center_line_pts[index_center - 1]);
    tmp.emplace_back(lane_in_next->center_line_pts[0]);
    param_left = FitLaneline(tmp);
    while (center_pt_pre.pt.x() < lane_in_next->center_line_pts[0].pt.x()) {
      ctl_pt.emplace_back(center_pt_pre);
      float pre_x = center_pt_pre.pt.x() + 1.0;
      float pre_y = param_left[0] + param_left[1] * pre_x;
      center_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
    }
    ctl_pt.emplace_back(lane_in_next->center_line_pts.front());
    if (ctl_pt.empty()) {
      return;
    }
  }

  lane_pre.str_id = lane_in_curr->str_id;
  lane_pre.lanepos_id = lane_in_curr->lanepos_id;
  size_t index = lane_in_curr->str_id_with_group.find(":");
  lane_pre.str_id_with_group =
      lane_in_curr->str_id_with_group.substr(0, index) + "V:" + lane_pre.str_id;
  lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
  lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
  lane_pre.center_line_param = lane_in_curr->center_line_param;
  lane_pre.center_line_param_front = lane_in_curr->center_line_param;
  lane_pre.center_line_pts = ctl_pt;
  lane_pre.prev_lane_str_id_with_group.emplace_back(
      lane_in_curr->str_id_with_group);
  lane_in_curr->next_lane_str_id_with_group.emplace_back(
      lane_pre.str_id_with_group);
  lane_pre.next_lane_str_id_with_group.emplace_back(
      lane_in_next->str_id_with_group);
  lane_in_next->prev_lane_str_id_with_group.emplace_back(
      lane_pre.str_id_with_group);

  if (lane_pre.left_boundary->pts.size() > 1 &&
      lane_pre.right_boundary->pts.size() > 1 &&
      lane_pre.center_line_pts.size() > 1) {
    (*lane_virtual).emplace_back(std::make_shared<Lane>(lane_pre));
  }
}

bool GroupMap::MatchLanePtAndStopLine(const em::Point& left_pt,
                                      const em::Point& right_pt,
                                      const Stpl& stop_line) {
  if (stop_line.points.size() < 2) {
    return false;
  }

  const double kLaneStopLineDistThresh = 5.;
  Eigen::Vector2f p0(stop_line.points.front().x(),
                     stop_line.points.front().y());
  Eigen::Vector2f p1(stop_line.points.back().x(), stop_line.points.back().y());
  Eigen::Vector2f left(left_pt.x(), left_pt.y());
  Eigen::Vector2f right(right_pt.x(), right_pt.y());
  bool left_in_range =
      (ProjectedInSegment(p0, p1, left) &&
       PointToVectorDist(p0, p1, left) < kLaneStopLineDistThresh);
  bool right_in_range =
      (ProjectedInSegment(p0, p1, right) &&
       PointToVectorDist(p0, p1, right) < kLaneStopLineDistThresh);
  // 左右点都在停止线范围内，认为关联
  if (left_in_range && right_in_range) {
    return true;
  }

  Eigen::Vector2f center = (left + right) * 0.5;
  bool center_in_range =
      (ProjectedInSegment(p0, p1, center) &&
       PointToVectorDist(p0, p1, center) < kLaneStopLineDistThresh);
  // 左边点在范围内且中心点也在范围内，认为关联
  if (left_in_range && center_in_range) {
    return true;
  }

  // 右边点在范围内且中心点也在范围内，认为关联
  if (right_in_range && center_in_range) {
    return true;
  }

  return false;
}

bool GroupMap::MatchLaneAndStopLine(const Lane::Ptr& lane,
                                    const Stpl::Ptr& stop_line) {
  if (lane == nullptr || stop_line == nullptr) {
    return false;
  }

  if (lane->left_boundary->pts.empty() || lane->right_boundary->pts.empty() ||
      stop_line->points.size() < 2) {
    return false;
  }

  auto start_left = lane->left_boundary->pts.front().pt;
  auto start_right = lane->right_boundary->pts.front().pt;
  auto end_left = lane->left_boundary->pts.back().pt;
  auto end_right = lane->right_boundary->pts.back().pt;

  return MatchLanePtAndStopLine(start_left, start_right, *stop_line) ||
         MatchLanePtAndStopLine(end_left, end_right, *stop_line);
}

Stpl::Ptr GroupMap::MatchedStopLine(const std::string& lane_id) {
  for (const auto& stop : stopline_) {
    if (std::find(stop.second->lane_id.begin(), stop.second->lane_id.end(),
                  lane_id) != stop.second->lane_id.end()) {
      return stop.second;
    }
  }
  return nullptr;
}
void GroupMap::FitCenterLine(Lane::Ptr lane) {
  std::vector<Vec2d> left_pts;
  std::vector<Vec2d> right_pts;
  left_pts.reserve(lane->left_boundary->pts.size());
  right_pts.reserve(lane->right_boundary->pts.size());
  for (const auto& pt : lane->left_boundary->pts) {
    left_pts.emplace_back(Vec2d(pt.pt.x(), pt.pt.y()));
  }
  for (const auto& pt : lane->right_boundary->pts) {
    right_pts.emplace_back(Vec2d(pt.pt.x(), pt.pt.y()));
  }
  std::vector<Vec2d> cent_pts;
  math::GenerateCenterPoint(left_pts, right_pts, &cent_pts);
  for (const auto& pt : cent_pts) {
    Point cpt(gm::RAW, static_cast<float>(pt.x()), static_cast<float>(pt.y()),
              0);
    lane->center_line_pts.emplace_back(cpt);
  }
  if (lane->center_line_pts.size() > 3) {
    lane->center_line_param = FitLaneline(lane->center_line_pts);
    lane->center_line_param_front = FitLanelinefront(lane->center_line_pts);
  }
}

void GroupMap::UpdateLane(Group::Ptr curr_group) {
  std::unordered_map<em::Id, em::Id> line_next;
  for (auto lane : curr_group->lanes) {
    if (lane->left_boundary->id_next != -1000) {
      line_next[lane->left_boundary->id] = lane->left_boundary->id_next;
    }
    if (lane->right_boundary->id_next != -1000) {
      line_next[lane->right_boundary->id] = lane->right_boundary->id_next;
    }
  }
  for (auto lane : curr_group->lanes) {
    if (lane->left_boundary->id_next == -1000 &&
        line_next.find(lane->left_boundary->id) != line_next.end()) {
      lane->left_boundary->id_next = line_next[lane->left_boundary->id];
    }
    if (lane->right_boundary->id_next == -1000 &&
        line_next.find(lane->right_boundary->id) != line_next.end()) {
      lane->right_boundary->id_next = line_next[lane->right_boundary->id];
    }
  }
}

std::vector<Point> GroupMap::SigmoidFunc(std::vector<Point> centerline,
                                         float sigma) {
  int size_ct = centerline.size();
  // 中心点数目少于两个点不拟合
  if (centerline.size() < 2) {
    return centerline;
  }
  float dis_x = centerline.back().pt.x() - centerline.front().pt.x();
  float dis_y = centerline.back().pt.y() - centerline.front().pt.y();

  // 距离过小不拟合
  if (dis_x < 0.1) {
    return centerline;
  }
  std::vector<Point> center;
  center.emplace_back(centerline[0]);

  for (int i = 1; i < centerline.size() - 1; ++i) {
    Point ctp(VIRTUAL, 0.0, 0.0, 0.0);
    if (center.back().pt.x() < centerline[i + 1].pt.x()) {
      ctp.pt.x() = centerline[i + 1].pt.x();
      float deta_x = centerline[i + 1].pt.x() - center[0].pt.x();
      // 1/(1+e^(-x)) sigmoid函数
      // 把x的范围圈定在[-sigma,sigma]范围内，deta_x的范围是(0,dis_x]
      // y的范围是(centerline[0].pt.y(),centerline[0].y()+dis_y)
      ctp.pt.y() = centerline[0].pt.y() +
                   dis_y / (1 + exp(-(2 * deta_x - dis_x) / dis_x * sigma));
    } else {
      float x_to_end = centerline.back().pt.x() - center.back().pt.x();
      float interval = x_to_end / static_cast<float>(centerline.size() - i - 1);
      ctp.pt.x() = center.back().pt.x() + interval;
      float deta_x = ctp.pt.x() - center[0].pt.x();
      ctp.pt.y() = centerline[0].pt.y() +
                   dis_y / (1 + exp(-(2 * deta_x - dis_x) / dis_x * sigma));
    }
    center.emplace_back(ctp);
  }
  center.emplace_back(centerline.back());
  return center;
}
std::vector<Point> GroupMap::SlidingWindow(std::vector<Point> centerline,
                                           int w) {
  int size_ct = centerline.size();
  if (size_ct < w) {
    return centerline;
  }
  std::vector<Point> center;
  Point ptc(VIRTUAL, 0.0, 0.0, 0.0);
  center.emplace_back(ptc);
  for (int i = 0; i < centerline.size(); ++i) {
    if (i < w) {
      center[0].pt += centerline[i].pt;
    } else {
      Point pt;
      pt.type = VIRTUAL;
      pt.pt = center[i - w].pt - centerline[i - w].pt + centerline[i].pt;
      center.emplace_back(pt);
      center[i - w].pt /= w;
    }
  }
  center.back().pt /= w;
  return center;
}
void GroupMap::SmoothCenterline(std::vector<Group::Ptr>* groups) {
  std::unordered_map<std::string, int>
      lane_grp_index;  // lane所在对应group的index
  for (auto& grp : *groups) {
    for (int index = 0; index < grp->lanes.size(); ++index) {
      lane_grp_index[grp->lanes[index]->str_id_with_group] = index;
    }
  }
  for (size_t i = 0; i < groups->size() - 1; ++i) {
    auto& curr_grp = groups->at(i);
    auto& next_grp = groups->at(i + 1);

    for (auto lane : curr_grp->lanes) {
      if (lane->is_smooth || lane->next_lane_str_id_with_group.empty()) {
        continue;
      }
      if (lane->next_lane_str_id_with_group.size() == 1) {
        // 从后往前滑动窗口
        //! TD:后续用距离不用点
        //! TBD:每个点计算曲率，抑制最大曲率
        int lane_index = lane_grp_index[lane->next_lane_str_id_with_group[0]];
        std::vector<Point> centerpt;
        auto& cur_centerline = lane->center_line_pts;
        int crr_size = cur_centerline.size();
        if (crr_size >= 10) {
          centerpt.insert(centerpt.begin(), cur_centerline.end() - 10,
                          cur_centerline.end());
        } else {
          centerpt.insert(centerpt.begin(), cur_centerline.begin(),
                          cur_centerline.end());
          int ctpt_size = centerpt.size();
          if (lane->prev_lane_str_id_with_group.size() == 1) {
            int pre_index =
                lane_grp_index[lane->prev_lane_str_id_with_group[0]];
            auto& prev_centerline =
                groups->at(i - 1)->lanes[pre_index]->center_line_pts;
            if (prev_centerline.size() >= 10 - ctpt_size) {
              centerpt.insert(centerpt.begin(),
                              prev_centerline.end() - (10 - ctpt_size),
                              prev_centerline.end());
            } else {
              centerpt.insert(centerpt.begin(), prev_centerline.begin(),
                              prev_centerline.end());
            }
          }
        }
        std::vector<Point> res = SigmoidFunc(centerpt, 5.0);
        int res_size = res.size();
        if (crr_size >= res_size) {
          std::copy(res.begin(), res.end(),
                    lane->center_line_pts.end() - res_size);
          if ((lane->center_line_pts.back().pt -
               next_grp->lanes[lane_index]->center_line_pts[0].pt)
                  .norm() > 0.1) {
            lane->center_line_pts.emplace_back(
                next_grp->lanes[lane_index]->center_line_pts[0]);
          }
        } else {
          std::copy(res.end() - crr_size, res.end(),
                    lane->center_line_pts.begin());
          int pre_index = lane_grp_index[lane->prev_lane_str_id_with_group[0]];
          auto& prev_centerline =
              groups->at(i - 1)->lanes[pre_index]->center_line_pts;
          std::copy(res.begin(), res.end() - crr_size,
                    prev_centerline.end() - (res_size - crr_size));
          if ((lane->center_line_pts.back().pt -
               next_grp->lanes[lane_index]->center_line_pts[0].pt)
                  .norm() > 0.1) {
            lane->center_line_pts.emplace_back(
                next_grp->lanes[lane_index]->center_line_pts[0]);
          }
          if ((prev_centerline.back().pt - lane->center_line_pts[0].pt).norm() >
              0.1) {
            prev_centerline.emplace_back(lane->center_line_pts[0]);
          }
        }
      } else {
        // 从前往后滑动
        // 一根车道最多两根后继
        std::vector<Point> next_centerpt;
        int lane_index_next_f =
            lane_grp_index[lane->next_lane_str_id_with_group[0]];
        auto& next_first_lane = next_grp->lanes[lane_index_next_f];
        auto& lane_next_f_centerline =
            next_grp->lanes[lane_index_next_f]->center_line_pts;
        int next_f_ctl_size = lane_next_f_centerline.size();
        if (next_f_ctl_size >= 10) {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_f_centerline.begin(),
                               lane_next_f_centerline.begin() + 10);
        } else {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_f_centerline.begin(),
                               lane_next_f_centerline.end());
          int ctpt_size = next_centerpt.size();

          if (next_first_lane->next_lane_str_id_with_group.size() == 1) {
            int next_next_index =
                lane_grp_index[next_first_lane->next_lane_str_id_with_group[0]];
            auto& next_next_ctline =
                groups->at(i + 2)->lanes[next_next_index]->center_line_pts;
            if (next_next_ctline.size() >= 10 - ctpt_size) {
              next_centerpt.insert(next_centerpt.end(),
                                   next_next_ctline.begin(),
                                   next_next_ctline.begin() + (10 - ctpt_size));

            } else {
              next_centerpt.insert(next_centerpt.end(),
                                   next_next_ctline.begin(),
                                   next_next_ctline.end());
            }
          }
        }
        std::vector<Point> first_ctl = SigmoidFunc(next_centerpt, 5.0);
        int first_ctl_size = first_ctl.size();
        if (next_f_ctl_size >= first_ctl_size) {
          std::copy(first_ctl.begin(), first_ctl.end(),
                    lane_next_f_centerline.begin());
          if ((lane_next_f_centerline[0].pt - lane->center_line_pts.back().pt)
                  .norm() > 0.1) {
            lane_next_f_centerline.insert(lane_next_f_centerline.begin(),
                                          lane->center_line_pts.back());
          }
        } else {
          std::copy(first_ctl.begin(), first_ctl.begin() + next_f_ctl_size,
                    lane_next_f_centerline.begin());
          int next_next_index =
              lane_grp_index[next_first_lane->next_lane_str_id_with_group[0]];
          auto& next_next_ctline =
              groups->at(i + 2)->lanes[next_next_index]->center_line_pts;
          std::copy(first_ctl.begin() + next_f_ctl_size, first_ctl.end(),
                    next_next_ctline.begin());
          if ((lane_next_f_centerline[0].pt - lane->center_line_pts.back().pt)
                  .norm() > 0.1) {
            lane_next_f_centerline.insert(lane_next_f_centerline.begin(),
                                          lane->center_line_pts.back());
          }
          if ((lane_next_f_centerline.back().pt - next_next_ctline[0].pt)
                  .norm() > 0.1) {
            next_next_ctline.insert(next_next_ctline.begin(),
                                    lane_next_f_centerline.back());
          }
        }

        next_centerpt.clear();
        int lane_index_next_s =
            lane_grp_index[lane->next_lane_str_id_with_group[1]];
        auto& next_second_lane = next_grp->lanes[lane_index_next_s];
        auto& lane_next_s_centerline =
            next_grp->lanes[lane_index_next_s]->center_line_pts;
        int next_s_ctl_size = lane_next_s_centerline.size();
        if (next_s_ctl_size >= 10) {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_s_centerline.begin(),
                               lane_next_s_centerline.begin() + 10);
        } else {
          next_centerpt.insert(next_centerpt.end(),
                               lane_next_s_centerline.begin(),
                               lane_next_s_centerline.end());
          int ctpt_size = next_centerpt.size();
          if (next_second_lane->next_lane_str_id_with_group.size() == 1) {
            int next_next_index =
                lane_grp_index[next_second_lane
                                   ->next_lane_str_id_with_group[0]];
            auto& next_next_ctline =
                groups->at(i + 2)->lanes[next_next_index]->center_line_pts;
            if (next_next_ctline.size() >= 10 - ctpt_size) {
              next_centerpt.insert(next_centerpt.end(),
                                   next_next_ctline.begin(),
                                   next_next_ctline.begin() + (10 - ctpt_size));
            } else {
              next_centerpt.insert(next_centerpt.end(),
                                   next_next_ctline.begin(),
                                   next_next_ctline.end());
            }
          }
        }
        std::vector<Point> second_ctl = SigmoidFunc(next_centerpt, 5.0);
        int second_ctl_size = second_ctl.size();
        if (next_s_ctl_size >= second_ctl_size) {
          std::copy(second_ctl.begin(), second_ctl.end(),
                    lane_next_s_centerline.begin());
          if ((lane_next_s_centerline[0].pt - lane->center_line_pts.back().pt)
                  .norm() > 0.1) {
            lane_next_s_centerline.insert(lane_next_s_centerline.begin(),
                                          lane->center_line_pts.back());
          }
        } else {
          std::copy(second_ctl.begin(), second_ctl.begin() + next_s_ctl_size,
                    lane_next_s_centerline.begin());
          int next_next_index =
              lane_grp_index[next_second_lane->next_lane_str_id_with_group[0]];
          auto& next_next_line =
              groups->at(i + 2)->lanes[next_next_index]->center_line_pts;
          std::copy(second_ctl.begin() + next_s_ctl_size, second_ctl.end(),
                    next_next_line.begin());
          if ((lane_next_s_centerline[0].pt - lane->center_line_pts.back().pt)
                  .norm() > 0.1) {
            lane_next_s_centerline.insert(lane_next_s_centerline.begin(),
                                          lane->center_line_pts.back());
          }
          if ((next_next_line[0].pt - lane_next_s_centerline.back().pt).norm() >
              0.1) {
            next_next_line.insert(next_next_line.begin(),
                                  lane_next_s_centerline.back());
          }
        }
      }
    }
  }
}
// 把Group里一个个GroupSegment中包含的小的LaneSegment，纵向上聚合成一个个大的Lane，
// 并且生成出：左右关联、前后关联、远端预测线、中心线
void GroupMap::GenLanesInGroups(std::vector<Group::Ptr>* groups, double stamp) {
  if (groups->size() < 1) {
    return;
  }
  for (auto& grp : *groups) {
    std::vector<Lane::Ptr> possible_lanes;
    possible_lanes.clear();
    // 收集可能的lane
    CollectGroupPossibleLanes(grp, &possible_lanes);
    // 过滤掉边线只有一个点的lane
    FilterGroupBadLane(possible_lanes, grp);
    // 区分出同向、反向车道
    //! 注意：当前同向、反向车道仅用于路口处排除关联到反向车道，普通路段不考虑同向、反向.
    // 同向、反向可能有多种隔离场景：单/双黄线、隔离带（路沿）、隔离栅栏等；
    // 当前仅考虑使用黄线隔离的场景，并且假设只有一根隔离（即仅考虑一根黄线）；当前不考虑双向车道.
    // 找到隔离线（当前只用黄线），判断每个车道的中心线在隔离线的左侧还是右侧，
    // 右侧为同向车道，左侧为反向车道.
    //! TBD：综合考虑黄线、路沿、栅栏等作为隔离线.
    SetGroupLaneOrient(grp);
    // 关联停止线
    MatchStopLineWithGroup(grp);
    // 关联左右
    MatchLRLane(grp);
  }

  //   |   |
  //   *   *
  //   |   |     Group
  //   *   *
  //   (中断)
  // - - - - -  SliceLine
  //   (中断)
  //   *   *
  //   |   |     Group
  //   *   *
  //   |   |
  // 对上面相邻的Group，SliceLine切分处会缺失点，导致前一个group里的线与后一个group里的线断开了，
  // 这里对于前一个group里每根线，从后一个group查找是否存在，如果存在就将后一个group那根线的front
  // 直接加到前一个group的back
  OptiPreNextLaneBoundaryPoint(groups);
  // 生成车道中心线
  GenLaneCenterLine(groups);
  // 关联前后，并且把前后lane的中心线连接上
  // 关联同一个lanepos但不同trackid的前后继
  //! TBD:后续要加上斑马线、停止线等有标识的车道线前后继关系，以及前后关系计算

  // 车道线补齐逻辑
  InferenceLaneLength(groups);
  // 设置lane属性(is_ego、is_tran)
  SetLaneStatus(groups);
  // 左车道断开或者右车道断开导致没形成道
  // if(groups.size()>2){
  //   for(size_t i = 0; i < groups.size()-2; i++){
  //   }
  // }

  // 停止线斑马线路面箭头与车道绑定
  if (groups->size() > 1) {
    RelateGroups(groups, stamp);
  }
  SmoothCenterline(groups);

#if 1
  OptiNextLane(groups);
#endif
  LaneForwardPredict(groups, stamp);
  // // 打印点
  // for (auto& grp : *groups) {
  //   for (auto& lane_grp : grp->lanes) {
  //     HLOG_ERROR << "lane_grp->str_id_with_group ="
  //                << lane_grp->str_id_with_group;
  //     for (auto& ct_p : lane_grp->center_line_pts) {
  //       HLOG_ERROR << "ct_p.pt.x() = " << ct_p.pt.x()
  //                  << "  ct_p.pt.y() = " << ct_p.pt.y();
  //     }
  //   }
  // }
}

void GroupMap::CollectGroupPossibleLanes(
    Group::Ptr grp, std::vector<Lane::Ptr>* possible_lanes) {
  for (const auto& grp_seg : grp->group_segments) {
    if (possible_lanes->empty()) {
      for (const auto& lane_seg : grp_seg->lane_segments) {
        auto lane = std::make_shared<Lane>();
        lane->str_id = lane_seg->str_id;
        lane->lanepos_id = lane_seg->lanepos_id;
        lane->str_id_with_group = grp->str_id + ":" + lane->str_id;
        lane->left_boundary = std::make_shared<LineSegment>();
        lane->left_boundary->id = lane_seg->left_boundary->id;
        lane->left_boundary->lanepos = lane_seg->left_boundary->lanepos;
        lane->left_boundary->type = lane_seg->left_boundary->type;
        lane->left_boundary->color = lane_seg->left_boundary->color;
        lane->left_boundary->isego = lane_seg->left_boundary->isego;
        lane->left_boundary->mean_end_heading =
            lane_seg->left_boundary->mean_end_heading;
        lane->left_boundary->mean_end_heading_std_dev =
            lane_seg->left_boundary->mean_end_heading_std_dev;
        lane->left_boundary->mean_end_interval =
            lane_seg->left_boundary->mean_end_interval;
        for (const auto& pt : lane_seg->left_boundary->pts) {
          lane->left_boundary->pts.emplace_back(pt);
        }
        lane->right_boundary = std::make_shared<LineSegment>();
        lane->right_boundary->id = lane_seg->right_boundary->id;
        lane->right_boundary->lanepos = lane_seg->right_boundary->lanepos;
        lane->right_boundary->type = lane_seg->right_boundary->type;
        lane->right_boundary->color = lane_seg->right_boundary->color;
        lane->right_boundary->isego = lane_seg->right_boundary->isego;
        lane->right_boundary->mean_end_heading =
            lane_seg->right_boundary->mean_end_heading;
        lane->right_boundary->mean_end_heading_std_dev =
            lane_seg->right_boundary->mean_end_heading_std_dev;
        lane->right_boundary->mean_end_interval =
            lane_seg->right_boundary->mean_end_interval;
        for (const auto& pt : lane_seg->right_boundary->pts) {
          lane->right_boundary->pts.emplace_back(pt);
        }
        possible_lanes->emplace_back(lane);
      }
      continue;
    }

    const size_t grp_lane_num = possible_lanes->size();
    if (grp_lane_num != grp_seg->lane_segments.size()) {
      HLOG_ERROR << "something is wrong, group's lane num should be equal to "
                    "group segment's lane segment num";
      return;
    }

    for (size_t i = 0; i != grp_lane_num; ++i) {
      auto& exist_lane = possible_lanes->at(i);
      const auto& lane_seg = grp_seg->lane_segments.at(i);
      for (const auto& pt : lane_seg->left_boundary->pts) {
        exist_lane->left_boundary->pts.emplace_back(pt);
      }
      for (const auto& pt : lane_seg->right_boundary->pts) {
        exist_lane->right_boundary->pts.emplace_back(pt);
      }
    }
  }
}

bool GroupMap::FilterGroupBadLane(const std::vector<Lane::Ptr>& possible_lanes,
                                  Group::Ptr grp) {
  // 过滤掉边线只有一个点的lane
  std::vector<Lane::Ptr> valid_lanes;
  for (const auto& lane : possible_lanes) {
    if (lane != nullptr && lane->left_boundary != nullptr &&
        lane->left_boundary->pts.size() > 1 &&
        lane->right_boundary != nullptr &&
        lane->right_boundary->pts.size() > 1) {
      grp->lanes.emplace_back(lane);
    }
  }
  return true;
}

bool GroupMap::MatchLRLane(Group::Ptr grp) {
  if (grp->lanes.size() > 1) {
    for (size_t i = 0; i != grp->lanes.size() - 1; ++i) {
      auto& curr = grp->lanes.at(i);
      for (size_t j = i + 1; j != grp->lanes.size(); ++j) {
        const auto& right = grp->lanes.at(j);
        curr->right_lane_str_id_with_group.emplace_back(
            right->str_id_with_group);
      }
    }
    for (size_t i = grp->lanes.size() - 1; i != 0; --i) {
      auto& curr = grp->lanes.at(i);
      for (int j = i - 1; j >= 0; --j) {
        const auto& left = grp->lanes.at(j);
        curr->left_lane_str_id_with_group.emplace_back(left->str_id_with_group);
      }
    }
  }
  return true;
}

bool GroupMap::MatchStopLineWithGroup(Group::Ptr grp) {
  // 将车道关联到停止线
  for (const auto& lane : grp->lanes) {
    for (auto& stop_line : stopline_) {
      if (MatchLaneAndStopLine(lane, stop_line.second)) {
        stop_line.second->lane_id.emplace_back(lane->str_id);
      }
    }
  }
  return true;
}

bool GroupMap::SetGroupLaneOrient(Group::Ptr grp) {
  std::vector<em::Point> separate;
  for (auto it = grp->lanes.rbegin(); it != grp->lanes.rend(); ++it) {
    LineSegment::Ptr yellow_bound = nullptr;
    if ((*it)->right_boundary->color == em::YELLOW &&
        (*it)->right_boundary->pts.size() > 1) {
      yellow_bound = (*it)->right_boundary;
    } else if ((*it)->left_boundary->color == em::YELLOW &&
               (*it)->left_boundary->pts.size() > 1) {
      yellow_bound = (*it)->left_boundary;
    }
    if (yellow_bound != nullptr) {
      auto start = yellow_bound->pts.front().pt;
      auto end = yellow_bound->pts.back().pt;
      separate.emplace_back(start);
      separate.emplace_back(end);
      break;
    }
  }
  // 没有隔离线，就默认所有车道都是同向
  if (separate.size() < 2) {
    for (auto& lane : grp->lanes) {
      lane->direction_type = em::DIRECTION_FORWARD;
    }
  } else {
    for (auto& lane : grp->lanes) {
      auto start_center = (lane->left_boundary->pts.front().pt +
                           lane->right_boundary->pts.front().pt) *
                          0.5;
      auto end_center = (lane->left_boundary->pts.back().pt +
                         lane->right_boundary->pts.back().pt) *
                        0.5;
      auto center = (start_center + end_center) * 0.5;
      Eigen::Vector2f p0(separate.front().x(), separate.front().y());
      Eigen::Vector2f p1(separate.back().x(), separate.back().y());
      Eigen::Vector2f pt(center.x(), center.y());
      if (PointInVectorSide(p0, p1, pt) > 0) {
        lane->direction_type = em::DIRECTION_FORWARD;
      } else {
        lane->direction_type = em::DIRECTION_BACKWARD;
      }
    }
  }
  return true;
}

bool GroupMap::GenLaneCenterLine(std::vector<Group::Ptr>* groups) {
  for (auto& grp : *groups) {
    for (auto& lane : grp->lanes) {
      FitCenterLine(lane);
    }
  }

  return true;
}

bool GroupMap::OptiPreNextLaneBoundaryPoint(std::vector<Group::Ptr>* groups) {
  for (size_t i = 0; i < groups->size() - 1; ++i) {
    auto& curr_grp = groups->at(i);
    auto& next_grp = groups->at(i + 1);
    std::map<em::Id, LineSegment::Ptr> curr_lines;
    std::map<em::Id, LineSegment::Ptr> next_lines;
    for (auto& lanes : curr_grp->lanes) {
      if (lanes->left_boundary != nullptr) {
        curr_lines.insert_or_assign(lanes->left_boundary->id,
                                    lanes->left_boundary);
      }
      if (lanes->right_boundary != nullptr) {
        curr_lines.insert_or_assign(lanes->right_boundary->id,
                                    lanes->right_boundary);
      }
    }
    for (auto& lanes : next_grp->lanes) {
      if (lanes->left_boundary != nullptr) {
        next_lines.insert_or_assign(lanes->left_boundary->id,
                                    lanes->left_boundary);
      }
      if (lanes->right_boundary != nullptr) {
        next_lines.insert_or_assign(lanes->right_boundary->id,
                                    lanes->right_boundary);
      }
    }
    for (auto& curr_line : curr_lines) {
      em::Id line_id = curr_line.first;
      if (curr_line.second->pts.empty()) {
        continue;
      }
      if (next_lines.find(line_id) != next_lines.end()) {
        auto& next_line = next_lines[line_id];
        if (next_line->pts.empty()) {
          continue;
        }
        curr_line.second->pts.emplace_back(next_line->pts.front());
      }
    }
  }
  return true;
}

bool GroupMap::InferenceLaneLength(std::vector<Group::Ptr>* groups) {
  // 从前往后
  if (groups->size() > 1) {
    for (size_t i = 0; i != groups->size() - 1; ++i) {
      auto& curr_group = groups->at(i);
      auto& next_group = groups->at(i + 1);
      int flag = 0;       // 前后group是否有车道根据trackid关联
      int flag_lane = 1;  // currgroup的车道是否全都有nextgroup关联
      for (auto& lane_in_curr : curr_group->lanes) {
        int next_lane_exit = 0;  // currlane是否有后继
        // HLOG_ERROR << "curr+lane = " << lane_in_curr->str_id_with_group;
        for (auto& lane_in_next : next_group->lanes) {
          if (lane_in_curr->str_id == lane_in_next->str_id ||
              lane_in_curr->left_boundary->id ==
                  lane_in_next->left_boundary->id ||
              lane_in_curr->right_boundary->id ==
                  lane_in_next->right_boundary->id) {
            // 目前这个是否有next是为了记录是否需要补道，如果有后继就不用补道
            lane_in_curr->next_lane_str_id_with_group.emplace_back(
                lane_in_next->str_id_with_group);
            // HLOG_ERROR << "the next lane is "
            //            << lane_in_next->str_id_with_group;
            lane_in_curr->left_boundary->id_next =
                lane_in_next->left_boundary->id;
            lane_in_curr->right_boundary->id_next =
                lane_in_next->right_boundary->id;
            lane_in_next->prev_lane_str_id_with_group.emplace_back(
                lane_in_curr->str_id_with_group);

            //! TBD:
            //! 这里直接把后一个lane中心线的第一个点加到前一个lane中心线的末尾，
            //! 后续需要考虑某些异常情况，比如后一个lane中心线的第一个点在前一个lane中心线最后
            //! 一个点的后方，这样直连就导致整个中心线往后折返了；以及还要考虑横向偏移较大时不平
            //! 滑的问题
            // if (!lane_in_next->center_line_pts.empty()) {
            // lane_in_curr->center_line_pts.emplace_back(
            //     lane_in_next->center_line_pts.front());
            // }
            flag = 1;
            next_lane_exit = 1;
            if (lane_in_next->center_line_param.empty()) {
              lane_in_next->center_line_param = lane_in_curr->center_line_param;
            }
            if (lane_in_next->center_line_param_front.empty()) {
              lane_in_next->center_line_param_front =
                  lane_in_next->center_line_param;
            }
            break;
          }
          if (IsLaneConnet(lane_in_curr, lane_in_next)) {
            lane_in_curr->next_lane_str_id_with_group.emplace_back(
                lane_in_next->str_id_with_group);
            lane_in_curr->left_boundary->id_next =
                lane_in_next->left_boundary->id;
            lane_in_curr->right_boundary->id_next =
                lane_in_next->right_boundary->id;

            flag = 1;
            next_lane_exit = 1;
            // if (lane_in_next->center_line_param.empty()) {
            //   lane_in_next->center_line_param =
            //   lane_in_curr->center_line_param;
            // }
            // if (lane_in_next->center_line_param_front.empty()) {
            //   lane_in_next->center_line_param_front =
            //       lane_in_next->center_line_param;
            // }
            break;
          }
        }
        if (next_lane_exit == 0) {
          flag_lane = 0;
        }
      }
      if (flag == 1) {
        // 查看trackid一致但是groupsize不一致的情况
        // 如果是线段长短不一致
        // float max_length_lane = 0.0;
        // for (auto& lane : next_group->lanes) {
        //   size_t size_lane = lane->center_line_pts.size();
        //   float len = (lane->center_line_pts[size_lane - 1].pt -
        //                lane->center_line_pts[0].pt)
        //                   .norm();
        //   if (len > max_length_lane) {
        //     max_length_lane = len;
        //   }
        // }

        // if (max_length_lane <= 10.0 && flag_lane == 0) {
        //   BuildVirtualLaneAfter(curr_group, next_group);
        //   // groups->erase(groups->begin() + i + 1);
        //   // i--;
        // }
        double group_distance =
            (curr_group->group_segments.back()->end_slice.po -
             next_group->group_segments.front()->start_slice.po)
                .norm();
        if (flag_lane == 0 && group_distance < 10.0) {
          UpdateLane(curr_group);
          BuildVirtualLaneAfter(curr_group, next_group);
        }
        for (auto& lane_in_curr : curr_group->lanes) {
          lane_in_curr->next_lane_str_id_with_group.clear();
          // lane_in_curr->left_boundary->id_next = -1000;
          // lane_in_curr->right_boundary->id_next = -1000;
        }
      }
    }
  }
  // 从后往前

  if (groups->size() > 1) {
    for (size_t i = groups->size() - 1; i > 0; --i) {
      auto& curr_group = groups->at(i - 1);
      auto& next_group = groups->at(i);
      int flag = 0;       // 前后group是否有车道根据trackid关联
      int flag_lane = 1;  // currgroup的车道是否全都有nextgroup关联
      for (auto& lane_in_next : next_group->lanes) {
        int next_lane_exit = 0;  // next_group是否有前驱
        // HLOG_ERROR << "next+lane = " << lane_in_next->str_id_with_group;
        for (auto& lane_in_curr : curr_group->lanes) {
          if (lane_in_curr->str_id == lane_in_next->str_id ||
              lane_in_curr->left_boundary->id ==
                  lane_in_next->left_boundary->id ||
              lane_in_curr->right_boundary->id ==
                  lane_in_next->right_boundary->id) {
            flag = 1;
            next_lane_exit = 1;
            // HLOG_ERROR << "the prev lane is "
            //            << lane_in_curr->str_id_with_group;
            lane_in_next->prev_lane_str_id_with_group.emplace_back(
                lane_in_curr->str_id_with_group);
            if (lane_in_curr->center_line_param.empty()) {
              lane_in_curr->center_line_param =
                  lane_in_next->center_line_param_front;
            }
            if (lane_in_curr->center_line_param_front.empty()) {
              lane_in_curr->center_line_param_front =
                  lane_in_curr->center_line_param;
            }
            break;
          }
          if (IsLaneConnet(lane_in_curr, lane_in_next)) {
            flag = 1;
            next_lane_exit = 1;
            lane_in_next->prev_lane_str_id_with_group.emplace_back(
                lane_in_curr->str_id_with_group);
            // HLOG_ERROR << "the prev lane is "
            //            << lane_in_curr->str_id_with_group;
            // if (lane_in_curr->center_line_param.empty()) {
            //   lane_in_curr->center_line_param =
            //       lane_in_next->center_line_param_front;
            // }
            // if (lane_in_curr->center_line_param_front.empty()) {
            //   lane_in_curr->center_line_param_front =
            //       lane_in_curr->center_line_param;
            // }
            break;
          }
        }
        if (next_lane_exit == 0) {
          flag_lane = 0;
        }
      }
      if (flag == 1) {
        // 查看trackid一致但是groupsize不一致的情况
        // 如果是线段长短不一致
        float max_length_lane = 0.0;
        for (auto& lane : curr_group->lanes) {
          size_t size_lane = lane->center_line_pts.size();
          float len = (lane->center_line_pts[size_lane - 1].pt -
                       lane->center_line_pts[0].pt)
                          .norm();
          if (len > max_length_lane) {
            max_length_lane = len;
          }
        }
        double group_distance =
            (curr_group->group_segments.back()->end_slice.po -
             next_group->group_segments.front()->start_slice.po)
                .norm();
        if (max_length_lane <= 10.0 && flag_lane == 0 &&
            group_distance < 10.0) {
          HLOG_ERROR << " is connect";
          BuildVirtualLaneBefore(curr_group, next_group);
          // groups->erase(groups->begin() + i - 1);
        }
        for (auto& lane : next_group->lanes) {
          lane->prev_lane_str_id_with_group.clear();
        }
      }
    }
  }

  return true;
}

bool GroupMap::SetLaneStatus(std::vector<Group::Ptr>* groups) {
  // 添加主路和是否当前朝向属性
  for (auto& group : *groups) {
    int flag = 0;
    for (auto& lane : group->lanes) {
      if (lane->left_boundary->isego == em::Ego_Road) {
        lane->is_ego = 1;
      }
      if (flag == 0) {
        if (lane->left_boundary->color == em::YELLOW) {
          flag = 1;
          lane->is_trans = 1;
        }
      } else {
        lane->is_trans = 1;
      }
    }
    if (flag == 0) {
      for (auto& lane : group->lanes) {
        if (lane->is_ego) {
          lane->is_trans = 1;
        }
      }
    }
  }
  return true;
}

bool GroupMap::OptiNextLane(std::vector<Group::Ptr>* groups) {
  //! TBD: 临时修改，解除多个后继，仅保留角度变化最小的一个后继
  HLOG_INFO << "get success of smallest angle";
  std::map<std::string, LaneWithNextLanes> lanes_has_next;
  if (groups->size() > 1) {
    for (size_t i = 0; i != groups->size() - 1; ++i) {
      auto& curr_grp = groups->at(i);
      auto& next_grp = groups->at(i + 1);
      for (auto& curr_lane : curr_grp->lanes) {
        LaneWithNextLanes lanes_next;
        lanes_next.lane = curr_lane;
        for (const auto& id : curr_lane->next_lane_str_id_with_group) {
          for (auto& next_lane : next_grp->lanes) {
            if (id == next_lane->str_id_with_group) {
              lanes_next.next_lanes.emplace_back(next_lane);
            }
          }
        }
        if (lanes_next.next_lanes.size() > 1) {
          lanes_has_next.insert_or_assign(curr_lane->str_id_with_group,
                                          lanes_next);
        }
      }
    }
  }

  for (auto it : lanes_has_next) {
    auto curr_lane = it.second.lane;
    auto curr_pts_size = curr_lane->center_line_pts.size();
    if (curr_pts_size < 2) {
      continue;
    }
    Eigen::Vector2f c0(curr_lane->center_line_pts.at(curr_pts_size - 2).pt.x(),
                       curr_lane->center_line_pts.at(curr_pts_size - 2).pt.y());
    Eigen::Vector2f c1(curr_lane->center_line_pts.at(curr_pts_size - 1).pt.x(),
                       curr_lane->center_line_pts.at(curr_pts_size - 1).pt.y());
    Eigen::Vector2f c = c1 - c0;
    c.normalize();
    Lane::Ptr best_next = nullptr;
    float max_len = 0;
    for (auto next_lane : it.second.next_lanes) {
      auto next_pts_size = next_lane->center_line_pts.size();
      if (next_pts_size < 3) {
        continue;
      }
      Eigen::Vector2f n0(next_lane->center_line_pts.at(0).pt.x(),
                         next_lane->center_line_pts.at(0).pt.y());
      Eigen::Vector2f n1(next_lane->center_line_pts.at(1).pt.x(),
                         next_lane->center_line_pts.at(1).pt.y());
      Eigen::Vector2f n = n1 - n0;
      if (n.norm() < 0.01) {
        n0 = n1;
        n1 << next_lane->center_line_pts.at(2).pt.x(),
            next_lane->center_line_pts.at(2).pt.y();
        n = n1 - n0;
      }
      n.normalize();
      float len = std::abs(n.transpose() * c);
      if (len > max_len) {
        max_len = len;
        best_next = next_lane;
      }
    }
    if (best_next != nullptr) {
      curr_lane->next_lane_str_id_with_group.clear();
      curr_lane->next_lane_str_id_with_group.emplace_back(
          best_next->str_id_with_group);
      best_next->prev_lane_str_id_with_group.clear();
      best_next->prev_lane_str_id_with_group.emplace_back(
          curr_lane->str_id_with_group);
    }
  }

  return true;
}

bool GroupMap::LaneForwardPredict(std::vector<Group::Ptr>* groups,
                                  const double& stamp) {
  // 对远处车道线进行预测，仅对无后继的lane尝试预测
  HLOG_INFO << "predict success lane";
  if (conf_.predict_farthest_dist > conf_.robust_percep_dist) {
    Group::Ptr last_grp = nullptr;
    // 找到最后一个非空的group，只预测最后一个group里的lane
    for (auto it = groups->rbegin(); it != groups->rend(); ++it) {
      if ((*it) == nullptr || (*it)->lanes.empty()) {
        continue;
      }
      last_grp = *it;
      break;
    }

    if (last_grp != nullptr) {
      bool check_back = true;
      if (last_grp->is_last_after_erased) {
        check_back = false;
      }
      std::vector<Lane::Ptr> lanes_wo_next;  // 末端lane，即无后继的lane
      Eigen::Vector2f curr_end_pl(
          last_grp->group_segments.back()->end_slice.pl.x(),
          last_grp->group_segments.back()->end_slice.pl.y());
      Eigen::Vector2f curr_end_pr(
          last_grp->group_segments.back()->end_slice.pr.x(),
          last_grp->group_segments.back()->end_slice.pr.y());
      Eigen::Vector2f curr_pos(0.0, 0.0);
      bool veh_in_this_junction = false;
      // HLOG_INFO << "calculate veh_in_this_junction";
      // HLOG_INFO << "curr_end_pl = " << curr_end_pl.x() << "  "
      //           << curr_end_pl.y();
      // HLOG_INFO << "curr_end_pr = " << curr_end_pr.x() << "  "
      //           << curr_end_pr.y();
      if ((PointInVectorSide(curr_end_pr, curr_end_pl, curr_pos) >= 0 &&
           PointToVectorDist(curr_end_pr, curr_end_pl, curr_pos) < 40) ||
          PointToVectorDist(curr_end_pr, curr_end_pl, curr_pos) < 20 ||
          (!check_back)) {
        veh_in_this_junction = true;
      }
      if (veh_in_this_junction) {
        ForwardCrossVirtual(last_grp, groups, stamp);
      } else {
        for (const auto& lane : last_grp->lanes) {
          if (lane == nullptr) {
            HLOG_ERROR << "found nullptr lane";
            continue;
          }
          if (lane->next_lane_str_id_with_group.empty()) {
            lanes_wo_next.emplace_back(lane);
          }
        }
        std::vector<LineSegment::Ptr> lines_need_pred;
        std::vector<Lane::Ptr> center_line_need_pred;
        for (auto& lane : lanes_wo_next) {
          int lane_center_need_pre = 0;
          if (lane->left_boundary != nullptr &&
              LaneLineNeedToPredict(*lane->left_boundary, check_back)) {
            lines_need_pred.emplace_back(lane->left_boundary);
            lane_center_need_pre++;
          }
          if (lane->right_boundary != nullptr &&
              LaneLineNeedToPredict(*lane->right_boundary, check_back)) {
            lines_need_pred.emplace_back(lane->right_boundary);
            lane_center_need_pre++;
          }
          if (lane_center_need_pre == 2) {
            center_line_need_pred.emplace_back(lane);
          }
        }
        // 使用平均heading，这样可以使得预测的线都是平行的，不交叉
        // 由于部分弯道heading偏差较大，导致整体平均heading发生偏差，
        // 现增加pred_end_heading字段用于预测，使用k-means算法或者PCL欧式聚类对heading进行聚类
        if (!lines_need_pred.empty()) {
          // k-means聚类
          // int k = 3;
          // HeadingCluster(&lines_need_pred, &k);

          // PCL欧式聚类 阈值为10度
          const double heading_threshold = 10.0 / 180. * M_PI;
          HeadingCluster(&lines_need_pred, heading_threshold);

          for (auto& line : lines_need_pred) {
            PredictLaneLine(line.get());
          }
          for (auto& lane : center_line_need_pred) {
            auto avg_heading = (lane->left_boundary->pred_end_heading +
                                lane->right_boundary->pred_end_heading) /
                               2;
            PredictLaneLine(avg_heading, &(lane->center_line_pts),
                            lane->left_boundary->mean_end_interval);
          }
        }
      }
    }
  }

  return true;
}

void GroupMap::ForwardCrossVirtual(Group::Ptr curr_group,
                                   std::vector<Group::Ptr>* groups,
                                   double stamp) {
  Group grp;
  grp.stamp = stamp;
  grp.str_id = curr_group->str_id + "P";

  double heading = 0;
  int lines = 0;
  for (auto lane_in_curr : curr_group->lanes) {
    if (lane_in_curr->left_boundary != nullptr) {
      heading += lane_in_curr->left_boundary->mean_end_heading;
      lines++;
    }
    if (lane_in_curr->right_boundary != nullptr) {
      heading += lane_in_curr->right_boundary->mean_end_heading;
      lines++;
    }
  }
  heading /= static_cast<double>(lines);
  for (auto lane_in_curr : curr_group->lanes) {
    if (lane_in_curr->left_boundary->pts.empty() ||
        lane_in_curr->right_boundary->pts.empty() ||
        lane_in_curr->center_line_pts.empty()) {
      continue;
    }
    Lane lane_pre;
    LineSegment left_bound;
    left_bound.id = lane_in_curr->left_boundary->id;
    left_bound.lanepos = lane_in_curr->left_boundary->lanepos;
    left_bound.type =
        em::LaneType_DASHED;       // lane_in_curr->left_boundary->type;
    left_bound.color = em::WHITE;  // lane_in_curr->left_boundary->color;
    left_bound.isego = lane_in_curr->left_boundary->isego;
    left_bound.mean_end_heading = lane_in_curr->left_boundary->mean_end_heading;
    left_bound.mean_end_heading_std_dev =
        lane_in_curr->left_boundary->mean_end_heading_std_dev;
    left_bound.mean_end_interval =
        lane_in_curr->left_boundary->mean_end_interval;
    size_t index_left = lane_in_curr->left_boundary->pts.size();
    Point left_pt_pre(VIRTUAL,
                      lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                      lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                      lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
    left_bound.pts.emplace_back(left_pt_pre);
    float dist_to_veh = left_pt_pre.pt.norm();
    auto pred_length = conf_.predict_farthest_dist - dist_to_veh;
    auto mean_interval = left_bound.mean_end_interval;
    auto pred_counts = static_cast<int>(pred_length / mean_interval);
    Eigen::Vector2f n(std::cos(heading), std::sin(heading));
    for (int i = 0; i < pred_counts; ++i) {
      Eigen::Vector2f pt = left_pt_pre.pt.head<2>() + (i + 1) * n;
      Point pred_pt;
      pred_pt.pt << pt.x(), pt.y(), 0;
      pred_pt.type = gm::VIRTUAL;
      left_bound.pts.emplace_back(pred_pt);
    }
    LineSegment right_bound;
    right_bound.id = lane_in_curr->right_boundary->id;
    right_bound.lanepos = lane_in_curr->right_boundary->lanepos;
    right_bound.type =
        em::LaneType_DASHED;        // lane_in_curr->right_boundary->type;
    right_bound.color = em::WHITE;  // lane_in_curr->right_boundary->color;
    right_bound.isego = lane_in_curr->right_boundary->isego;
    right_bound.mean_end_heading =
        lane_in_curr->right_boundary->mean_end_heading;
    right_bound.mean_end_heading_std_dev =
        lane_in_curr->right_boundary->mean_end_heading_std_dev;
    right_bound.mean_end_interval =
        lane_in_curr->right_boundary->mean_end_interval;
    size_t index_right = lane_in_curr->right_boundary->pts.size();
    Point right_pt_pre(
        VIRTUAL, lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
        lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
        lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
    right_bound.pts.emplace_back(right_pt_pre);
    dist_to_veh = right_pt_pre.pt.norm();
    pred_length = conf_.predict_farthest_dist - dist_to_veh;
    mean_interval = right_bound.mean_end_interval;
    pred_counts = static_cast<int>(pred_length / mean_interval);
    for (int i = 0; i < pred_counts; ++i) {
      Eigen::Vector2f pt = right_pt_pre.pt.head<2>() + (i + 1) * n;
      Point pred_pt;
      pred_pt.pt << pt.x(), pt.y(), 0;
      pred_pt.type = gm::VIRTUAL;
      right_bound.pts.emplace_back(pred_pt);
    }
    std::vector<Point> ctl_pt;
    size_t index_center = lane_in_curr->center_line_pts.size();
    Point center_pt_pre(VIRTUAL,
                        lane_in_curr->center_line_pts[index_center - 1].pt.x(),
                        lane_in_curr->center_line_pts[index_center - 1].pt.y(),
                        lane_in_curr->center_line_pts[index_center - 1].pt.z());
    ctl_pt.emplace_back(center_pt_pre);
    dist_to_veh = center_pt_pre.pt.norm();
    pred_length = conf_.predict_farthest_dist - dist_to_veh;
    mean_interval = left_bound.mean_end_interval;
    pred_counts = static_cast<int>(pred_length / mean_interval);
    for (int i = 0; i < pred_counts; ++i) {
      Eigen::Vector2f pt = center_pt_pre.pt.head<2>() + (i + 1) * n;
      Point pred_pt;
      pred_pt.pt << pt.x(), pt.y(), 0;
      pred_pt.type = gm::VIRTUAL;
      ctl_pt.emplace_back(pred_pt);
    }
    lane_pre.str_id = lane_in_curr->str_id;
    lane_pre.lanepos_id = lane_in_curr->lanepos_id;
    size_t index = lane_in_curr->str_id_with_group.find(":");
    lane_pre.str_id_with_group =
        lane_in_curr->str_id_with_group.substr(0, index) +
        "P:" + lane_pre.str_id;
    lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
    lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
    lane_pre.center_line_param = lane_in_curr->center_line_param;
    lane_pre.center_line_param_front = lane_in_curr->center_line_param;
    lane_pre.center_line_pts = ctl_pt;
    lane_pre.prev_lane_str_id_with_group.emplace_back(
        lane_in_curr->str_id_with_group);
    lane_in_curr->next_lane_str_id_with_group.emplace_back(
        lane_pre.str_id_with_group);
    if (lane_pre.left_boundary->pts.size() > 1 &&
        lane_pre.right_boundary->pts.size() > 1 &&
        lane_pre.center_line_pts.size() > 1) {
      grp.lanes.emplace_back(std::make_shared<Lane>(lane_pre));
    }
  }
  (*groups).emplace_back(std::make_shared<Group>(grp));
}

void GroupMap::SetAllOverlaps(std::map<em::Id, Zebra::Ptr>* zebra,
                              std::map<em::Id, Arrow::Ptr>* arrow,
                              std::map<em::Id, Stpl::Ptr>* stopline,
                              std::map<em::Id, Overlaps::Ptr>* overlap,
                              Lane::Ptr lane) {
  // for (auto& zeb : *zebra) {
  //   auto pts = zeb.second->polygon.points;
  //   if ((pts[0] - pts[1]).norm() < (pts[1] - pts[2]).norm()) {
  //   }
  // }

  // for (auto& arw : *arrow) {
  //   auto center =
  //       (arw.second->polygon.points[0] + arw.second->polygon.points[1] +
  //        arw.second->polygon.points[2] + arw.second->polygon.points[3]) /
  //       4;
  //   for(size_t i = 0; i < lane->center_line_pts.size() - 1; ++i){
  //     if(center.x() >= lane->center_line_pts[i].pt.x() && center.x() <=
  //     lane->center_line_pts[i].pt.x()){
  //       if()
  //       break;
  //     }
  //   }
  // }
}

void GroupMap::BuildVirtualGroup(std::vector<Lane::Ptr> lane_virtual,
                                 std::vector<Group::Ptr>* group_virtual,
                                 double stamp) {
  Group grp;
  grp.stamp = stamp;
  for (const auto& lane : lane_virtual) {
    grp.lanes.emplace_back(lane);
  }
  grp.str_id = lane_virtual[0]->str_id_with_group.substr(
      0, lane_virtual[0]->str_id_with_group.find(":"));
  (*group_virtual).emplace_back(std::make_shared<Group>(grp));
}

void GroupMap::CatmullRom(const std::vector<Eigen::Vector3f>& pts,
                          std::vector<Eigen::Vector3f>* fit_points, int num) {
  auto func = [](double p0, double p1, double p2, double p3, double t) {
    double s = 0.5;
    double a = -s * p0 + (2 - s) * p1 + (s - 2) * p2 + s * p3;
    double b = 2 * s * p0 + (s - 3) * p1 + (3 - 2 * s) * p2 - s * p3;
    double c = -s * p0 + s * p2;
    double d = p1;
    double t2 = t * t;
    double t3 = t2 * t;
    return (a * t3 + b * t2 + c * t + d);
  };
  for (size_t i = 0; i < pts.size() - 3; ++i) {
    double t = 0;
    while (t < 1) {
      double px =
          func(pts[i].x(), pts[i + 1].x(), pts[i + 2].x(), pts[i + 3].x(), t);
      double py =
          func(pts[i].y(), pts[i + 1].y(), pts[i + 2].y(), pts[i + 3].y(), t);
      Eigen::Vector3f point = {px, py, 0.0};
      fit_points->emplace_back(point);
      t += 1.0 / num;
    }
  }
}

void GroupMap::HeadingCluster(std::vector<LineSegment::Ptr>* lines_need_pred,
                              int* num) {
  if (num == nullptr) {
    HLOG_ERROR << "k is not specified for k-means cluster";
    return;
  }
  std::vector<double> heading_data;
  for (const auto& line : *lines_need_pred) {
    heading_data.emplace_back(line->mean_end_heading);
  }

  if (*num > heading_data.size()) {
    *num = heading_data.size();
  }

  std::vector<double> centroids;
  double min_val = *std::min_element(heading_data.begin(), heading_data.end());
  double max_val = *std::max_element(heading_data.begin(), heading_data.end());
  double step = (max_val - min_val) / (*num - 1);
  for (int i = 0; i < *num; ++i) {
    centroids.push_back(min_val + i * step);
  }

  // 迭代更新聚类中心点
  std::vector<int> cluster_count(*num, 0);
  std::vector<double> cluster_sum(*num, 0.0);
  std::vector<int> assignments(heading_data.size(), -1);
  bool converged = false;
  while (!converged) {
    converged = true;
    for (int i = 0; i < heading_data.size(); ++i) {
      double min_dist = std::numeric_limits<double>::max();
      int closest_centroid = -1;
      for (int j = 0; j < *num; ++j) {
        double dist = std::abs(heading_data[i] - centroids[j]);
        if (dist < min_dist) {
          min_dist = dist;
          closest_centroid = j;
        }
      }
      if (assignments[i] != closest_centroid) {
        assignments[i] = closest_centroid;
        converged = false;
      }
      cluster_count[closest_centroid]++;
      cluster_sum[closest_centroid] += heading_data[i];
    }

    // 更新聚类中心点
    for (int i = 0; i < *num; ++i) {
      if (cluster_count[i] > 0) {
        centroids[i] = cluster_sum[i] / cluster_count[i];
      }
    }
  }

  // 填充pred_end_heading字段用于预测
  int i = 0;
  for (auto& line : *lines_need_pred) {
    line->pred_end_heading = centroids[assignments[i]];
    ++i;
  }
}

void GroupMap::HeadingCluster(std::vector<LineSegment::Ptr>* lines_need_pred,
                              double threshold) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr heading_data(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& line : *lines_need_pred) {
    pcl::PointXYZ point;
    point.x =
        static_cast<float>(line->mean_end_heading);  // 将一维heading添加到x轴上
    point.y = 0.0;
    point.z = 0.0;
    heading_data->emplace_back(point);
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr heading_data_tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  heading_data_tree->setInputCloud(heading_data);

  // 对heading进行欧式聚类
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> heading_cluster;
  heading_cluster.setClusterTolerance(threshold);
  heading_cluster.setMinClusterSize(1);
  heading_cluster.setMaxClusterSize(heading_data->size());
  heading_cluster.setSearchMethod(heading_data_tree);
  heading_cluster.setInputCloud(heading_data);

  std::vector<pcl::PointIndices> heading_cluster_indices;
  heading_cluster.extract(heading_cluster_indices);

  std::vector<double> mean_heading;
  std::vector<int> assignments(heading_data->size(), -1);

  // std::unordered_map<double, int> heading_data_map;
  // int heading_index = 0;
  // for (const auto& line : *lines_need_pred) {
  //   heading_data_map[line->mean_end_heading] = heading_index;
  //   heading_index = heading_index + 1;
  // }

  for (int i = 0; i < heading_cluster_indices.size(); ++i) {
    double heading_sum = 0.;
    int heading_data_size = 0;
    for (const auto& idx : heading_cluster_indices[i].indices) {
      auto heading_data_element = (*heading_data)[idx].x;
      heading_sum = heading_sum + heading_data_element;
      // auto it = heading_data_map.find(heading_data_element);
      assignments[idx] = i;
      heading_data_size = heading_data_size + 1;
    }
    if (heading_data_size != 0) {
      mean_heading.emplace_back(heading_sum / heading_data_size);
    }
  }

  // 填充pred_end_heading字段用于预测
  int i = 0;
  for (auto& line : *lines_need_pred) {
    line->pred_end_heading = mean_heading[assignments[i]];
    ++i;
  }
}

void GroupMap::BuildVirtualProSucLane(Lane::Ptr lane_in_curr,
                                      Lane::Ptr lane_in_next, float dis_pt) {
  // ! TBD: 要写成是否路口判断条件，如是否有斑马线等
  if (dis_pt < 100.0) {
    lane_in_curr->next_lane_str_id_with_group.emplace_back(
        lane_in_next->str_id_with_group);
    lane_in_next->prev_lane_str_id_with_group.emplace_back(
        lane_in_curr->str_id_with_group);
    if (lane_in_next->center_line_pts.size() > 2 &&
        lane_in_curr->center_line_pts.size() > 4) {
      size_t cur_num = lane_in_curr->center_line_pts.size();
      std::vector<Eigen::Vector3f> tmp_vec;
      tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num - 5].pt);
      tmp_vec.emplace_back(lane_in_curr->center_line_pts[cur_num - 4].pt);
      tmp_vec.emplace_back(lane_in_next->center_line_pts[0].pt);
      tmp_vec.emplace_back(lane_in_next->center_line_pts[1].pt);
      std::vector<Eigen::Vector3f> center_fit;
      CatmullRom(tmp_vec, &center_fit, 4);
      Point pt_center;
      pt_center.type = gm::VIRTUAL;
      pt_center.pt = center_fit[0];
      lane_in_curr->center_line_pts[cur_num - 3] = pt_center;
      pt_center.pt = center_fit[1];
      lane_in_curr->center_line_pts[cur_num - 2] = pt_center;
      pt_center.pt = center_fit[2];
      lane_in_curr->center_line_pts[cur_num - 1] = pt_center;
      lane_in_curr->center_line_pts.emplace_back(
          lane_in_next->center_line_pts.front());
    } else if (!lane_in_next->center_line_pts.empty()) {
      lane_in_curr->center_line_pts.emplace_back(
          lane_in_next->center_line_pts.front());
    }
  }
  // else {
  //   if (lane_in_curr->center_line_param.empty()) {
  //     std::vector<Point> tmp;
  //     size_t center_size = lane_in_curr->center_line_pts.size();
  //     tmp.emplace_back(lane_in_curr->center_line_pts[center_size - 1]);
  //     tmp.emplace_back(lane_in_next->center_line_pts[0]);
  //     lane_in_curr->center_line_param = FitLaneline(tmp);
  //   }
  //   size_t index_left = lane_in_curr->left_boundary->pts.size();
  //   Point left_pt_pre(VIRTUAL,
  //                     lane_in_curr->left_boundary->pts[index_left -
  //                     1].pt.x(), lane_in_curr->left_boundary->pts[index_left
  //                     - 1].pt.y(),
  //                     lane_in_curr->left_boundary->pts[index_left -
  //                     1].pt.z());
  //   double b_left = left_pt_pre.pt.y() -
  //                   lane_in_curr->center_line_param[1] * left_pt_pre.pt.x();
  //   while (left_pt_pre.pt.x() <
  //          lane_in_next->left_boundary->pts[0].pt.x() - 1.0) {
  //     lane_in_curr->left_boundary->pts.emplace_back(left_pt_pre);
  //     float pre_x = left_pt_pre.pt.x() + 1.0;
  //     float pre_y = b_left + lane_in_curr->center_line_param[1] * pre_x;
  //     left_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  //   }
  //   size_t index_right = lane_in_curr->right_boundary->pts.size();
  //   Point right_pt_pre(
  //       VIRTUAL, lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
  //       lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
  //       lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
  //   double b_right = right_pt_pre.pt.y() -
  //                    lane_in_curr->center_line_param[1] *
  //                    right_pt_pre.pt.x();
  //   while (right_pt_pre.pt.x() <
  //          lane_in_next->right_boundary->pts[0].pt.x() - 1.0) {
  //     lane_in_curr->right_boundary->pts.emplace_back(right_pt_pre);
  //     float pre_x = right_pt_pre.pt.x() + 1.0;
  //     float pre_y = b_right + lane_in_curr->center_line_param[1] * pre_x;
  //     right_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  //   }
  //   size_t index_center = lane_in_curr->center_line_pts.size();
  //   Point center_pt_pre(VIRTUAL,
  //                       lane_in_curr->center_line_pts[index_center -
  //                       1].pt.x(), lane_in_curr->center_line_pts[index_center
  //                       - 1].pt.y(),
  //                       lane_in_curr->center_line_pts[index_center -
  //                       1].pt.z());
  //   while (center_pt_pre.pt.x() <
  //          lane_in_next->center_line_pts[0].pt.x() - 1.0) {
  //     lane_in_curr->center_line_pts.emplace_back(center_pt_pre);
  //     float pre_x = center_pt_pre.pt.x() + 1.0;
  //     float pre_y = lane_in_curr->center_line_param[0] +
  //                   lane_in_curr->center_line_param[1] * pre_x;
  //     center_pt_pre = Point(VIRTUAL, pre_x, pre_y, static_cast<float>(0.0));
  //   }
  //   lane_in_curr->center_line_pts.emplace_back(
  //       lane_in_next->center_line_pts.front());
  // }
}
std::vector<double> GroupMap::FitLaneline(
    const std::vector<Point>& centerline) {
  int size_c = centerline.size();

  double k = 0.0, kk = 0.0;
  double y1 = centerline[size_c - 1].pt.y(), x1 = centerline[size_c - 1].pt.x();
  double y2 = centerline[size_c - 2].pt.y(), x2 = centerline[size_c - 2].pt.x();
  int idx = size_c - 2;
  while (sqrt(pow(y1 - y2, 2) + pow(x1 - x2, 2)) < 4 && idx > 0) {
    idx--;
    y2 = centerline[idx].pt.y();
    x2 = centerline[idx].pt.x();
  }
  if (idx < 0) {
    return {};
  } else {
    kk = (y2 - y1) / (x2 - x1);
    k = y1 - kk * x1;
  }
  return {k, kk};
}

std::vector<double> GroupMap::FitLanelinefront(
    const std::vector<Point>& centerline) {
  int size_c = centerline.size();

  double k = 0.0, kk = 0.0;
  double y1 = centerline[0].pt.y(), x1 = centerline[0].pt.x();
  double y2 = centerline[1].pt.y(), x2 = centerline[1].pt.x();
  int idx = 1;
  while (sqrt(pow(y1 - y2, 2) + pow(x1 - x2, 2)) < 4 && idx < size_c) {
    y2 = centerline[idx].pt.y();
    x2 = centerline[idx].pt.x();
    idx++;
  }
  if (idx >= size_c) {
    return {};
  } else {
    kk = (y2 - y1) / (x2 - x1);
    k = y1 - kk * x1;
  }
  return {k, kk};
}
void GroupMap::BuildConnectLane(Lane::Ptr lane_in_curr, Group::Ptr next_group,
                                Lane::Ptr lane_in_next_next) {
  if (lane_in_curr->next_lane_str_id_with_group.empty()) {
    Lane lane_pre;
    LineSegment left_bound;
    left_bound.id = lane_in_curr->left_boundary->id;
    left_bound.lanepos = lane_in_curr->left_boundary->lanepos;
    left_bound.type = lane_in_curr->left_boundary->type;
    left_bound.color = lane_in_curr->left_boundary->color;
    left_bound.mean_end_heading = lane_in_curr->left_boundary->mean_end_heading;
    left_bound.mean_end_heading_std_dev =
        lane_in_curr->left_boundary->mean_end_heading_std_dev;
    left_bound.mean_end_interval =
        lane_in_curr->left_boundary->mean_end_interval;
    size_t index_left = lane_in_curr->left_boundary->pts.size();
    Point left_pt_pre(PREDICTED,
                      lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                      lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                      lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
    Eigen::Vector3f left_vec_norm;
    left_vec_norm = (lane_in_next_next->left_boundary->pts[0].pt -
                     lane_in_curr->left_boundary->pts[index_left - 1].pt)
                        .normalized();
    while (left_pt_pre.pt.x() <
           lane_in_next_next->left_boundary->pts[0].pt.x() - 1.0) {
      left_bound.pts.emplace_back(left_pt_pre);
      float pre_x = left_pt_pre.pt.x() + left_vec_norm.x();
      float pre_y = left_pt_pre.pt.y() + left_vec_norm.y();
      left_pt_pre = Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
    }
    if (left_bound.pts.empty()) {
      return;
    }
    LineSegment right_bound;
    right_bound.id = lane_in_curr->right_boundary->id;
    right_bound.lanepos = lane_in_curr->right_boundary->lanepos;
    right_bound.type = lane_in_curr->right_boundary->type;
    right_bound.color = lane_in_curr->right_boundary->color;
    right_bound.mean_end_heading =
        lane_in_curr->right_boundary->mean_end_heading;
    right_bound.mean_end_heading_std_dev =
        lane_in_curr->right_boundary->mean_end_heading_std_dev;
    right_bound.mean_end_interval =
        lane_in_curr->right_boundary->mean_end_interval;
    size_t index_right = lane_in_curr->right_boundary->pts.size();
    Point right_pt_pre(
        PREDICTED, lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
        lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
        lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
    Eigen::Vector3f right_vec_norm;
    right_vec_norm = (lane_in_next_next->right_boundary->pts[0].pt -
                      lane_in_curr->right_boundary->pts[index_right - 1].pt)
                         .normalized();
    while (right_pt_pre.pt.x() <
           lane_in_next_next->right_boundary->pts[0].pt.x() - 1.0) {
      right_bound.pts.emplace_back(right_pt_pre);
      float pre_x = right_pt_pre.pt.x() + right_vec_norm.x();
      float pre_y = right_pt_pre.pt.y() + right_vec_norm.y();
      right_pt_pre = Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
    }
    if (right_bound.pts.empty()) {
      return;
    }
    std::vector<Point> ctl_pt;
    size_t index_center = lane_in_curr->center_line_pts.size();
    Point center_pt_pre(PREDICTED,
                        lane_in_curr->center_line_pts[index_center - 1].pt.x(),
                        lane_in_curr->center_line_pts[index_center - 1].pt.y(),
                        lane_in_curr->center_line_pts[index_center - 1].pt.z());
    Eigen::Vector3f cent_vec_norm;
    cent_vec_norm = (lane_in_next_next->center_line_pts[0].pt -
                     lane_in_curr->center_line_pts[index_center - 1].pt)
                        .normalized();
    while (center_pt_pre.pt.x() <
           lane_in_next_next->center_line_pts[0].pt.x() - 1.0) {
      ctl_pt.emplace_back(center_pt_pre);
      float pre_x = center_pt_pre.pt.x() + cent_vec_norm.x();
      float pre_y = center_pt_pre.pt.y() + cent_vec_norm.y();
      center_pt_pre = Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
    }
    if (ctl_pt.empty()) {
      return;
    }
    lane_pre.str_id = lane_in_curr->str_id;
    lane_pre.lanepos_id = lane_in_curr->lanepos_id;
    lane_pre.str_id_with_group = next_group->str_id + ":" + lane_pre.str_id;
    lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
    lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
    lane_pre.center_line_param = lane_in_curr->center_line_param;
    lane_pre.center_line_param_front = lane_in_curr->center_line_param;
    lane_pre.center_line_pts = ctl_pt;
    if (lane_pre.center_line_pts.size() > 1) {
      next_group->lanes.emplace_back(std::make_shared<Lane>(lane_pre));
    }
  }
}

void GroupMap::FillLineSegment(LineSegment::Ptr line, LineSegment* line_set) {
  for (auto& line_pt : line->pts) {
    line_set->pts.emplace_back(line_pt);
  }
}

void GroupMap::BuildVirtualLaneAfter(Group::Ptr curr_group,
                                     Group::Ptr next_group) {
  for (auto& lane_in_curr : curr_group->lanes) {
    if (lane_in_curr->next_lane_str_id_with_group.empty()) {
      // && !lane_in_curr->center_line_param.empty()
      Lane lane_pre;
      LineSegment left_bound;
      LineSegment right_bound;
      left_bound.id = lane_in_curr->left_boundary->id;
      left_bound.lanepos = lane_in_curr->left_boundary->lanepos;
      left_bound.type = lane_in_curr->left_boundary->type;
      left_bound.color = lane_in_curr->left_boundary->color;
      left_bound.mean_end_heading =
          lane_in_curr->left_boundary->mean_end_heading;
      left_bound.mean_end_heading_std_dev =
          lane_in_curr->left_boundary->mean_end_heading_std_dev;
      left_bound.mean_end_interval =
          lane_in_curr->left_boundary->mean_end_interval;
      right_bound.id = lane_in_curr->right_boundary->id;
      right_bound.lanepos = lane_in_curr->right_boundary->lanepos;
      right_bound.type = lane_in_curr->right_boundary->type;
      right_bound.color = lane_in_curr->right_boundary->color;
      right_bound.mean_end_heading =
          lane_in_curr->right_boundary->mean_end_heading;
      right_bound.mean_end_heading_std_dev =
          lane_in_curr->right_boundary->mean_end_heading_std_dev;
      right_bound.mean_end_interval =
          lane_in_curr->right_boundary->mean_end_interval;
      // HLOG_ERROR << "lane_in_curr->str_id_with_group = "
      //            << lane_in_curr->str_id_with_group
      //            << "  lane_in_curr->left_boundary->id = "
      //            << lane_in_curr->left_boundary->id
      //            << "  lane_in_curr->left_boundary->id_next = "
      //            << lane_in_curr->left_boundary->id_next
      //            << "  lane_in_curr->right_boundary->id = "
      //            << lane_in_curr->right_boundary->id
      //            << "  lane_in_curr->right_boundary->id_next = "
      //            << lane_in_curr->right_boundary->id_next;
      // 判断现有的line是否已经存在
      int left_bound_exist = 0, right_bound_exist = 0;
      for (auto& lane_in_next : next_group->lanes) {
        if (lane_in_next->left_boundary->id ==
            lane_in_curr->left_boundary->id_next) {
          // left_bound = *(lane_in_next->left_boundary);
          // FillLineSegment(lane_in_next->left_boundary, &left_bound);
          for (auto& line_pt : lane_in_next->left_boundary->pts) {
            Point pt_pre(PREDICTED, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = 1;
        } else if (lane_in_next->right_boundary->id ==
                   lane_in_curr->left_boundary->id_next) {
          // left_bound = *(lane_in_next->right_boundary);
          // FillLineSegment(lane_in_next->right_boundary, &left_bound);
          for (auto& line_pt : lane_in_next->right_boundary->pts) {
            Point pt_pre(PREDICTED, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = 1;
        } else if (lane_in_next->left_boundary->id ==
                   lane_in_curr->right_boundary->id_next) {
          // right_bound = *(lane_in_next->left_boundary);
          // FillLineSegment(lane_in_next->left_boundary, &right_bound);
          for (auto& line_pt : lane_in_next->left_boundary->pts) {
            Point pt_pre(PREDICTED, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = 1;
        } else if (lane_in_next->right_boundary->id ==
                   lane_in_curr->right_boundary->id_next) {
          // right_bound = *(lane_in_next->right_boundary);
          // FillLineSegment(lane_in_next->right_boundary, &right_bound);
          for (auto& line_pt : lane_in_next->right_boundary->pts) {
            Point pt_pre(PREDICTED, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = 1;
        }
      }

      if (left_bound_exist == 1 && right_bound_exist == 0) {
        size_t index_right = lane_in_curr->right_boundary->pts.size();
        Point right_pt_pre(
            PREDICTED,
            lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
            lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
            lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
        right_bound.pts.emplace_back(right_pt_pre);
        if ((right_bound.pts[0].pt - left_bound.pts[0].pt).norm() > 4.0) {
          right_pt_pre.pt.x() = right_pt_pre.pt.x() + left_bound.pts[0].pt.x() -
                                lane_in_curr->left_boundary->pts.back().pt.x();
          right_pt_pre.pt.y() = right_pt_pre.pt.y() + left_bound.pts[0].pt.y() -
                                lane_in_curr->left_boundary->pts.back().pt.y();
          right_bound.pts.emplace_back(right_pt_pre);
        }
        size_t left_index = 1;
        size_t left_bound_size = left_bound.pts.size();
        float dx = left_bound.pts[left_index].pt.x() -
                   left_bound.pts[left_index - 1].pt.x();
        float dy = left_bound.pts[left_index].pt.y() -
                   left_bound.pts[left_index - 1].pt.y();
        while (right_pt_pre.pt.x() <
                   next_group->group_segments.back()->end_slice.po.x() &&
               left_index < left_bound_size &&
               (dx > 0 && dx * dx + dy * dy > 0.4 || dx * dx + dy * dy < 0.4)) {
          dx = left_bound.pts[left_index].pt.x() -
               left_bound.pts[left_index - 1].pt.x();
          dy = left_bound.pts[left_index].pt.y() -
               left_bound.pts[left_index - 1].pt.y();
          float pre_x = right_pt_pre.pt.x() + dx;
          float pre_y = right_pt_pre.pt.y() + dy;
          right_pt_pre =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          right_bound.pts.emplace_back(right_pt_pre);
          left_index++;
        }
        if (right_bound.pts.empty()) {
          return;
        }
      } else if (left_bound_exist == 0 && right_bound_exist == 1) {
        size_t index_left = lane_in_curr->left_boundary->pts.size();
        Point left_pt_pre(
            PREDICTED, lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
            lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
            lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
        size_t right_index = 1;
        size_t right_bound_size = right_bound.pts.size();
        left_bound.pts.emplace_back(left_pt_pre);
        if ((right_bound.pts[0].pt - left_bound.pts[0].pt).norm() > 4.0) {
          left_pt_pre.pt.x() = left_pt_pre.pt.x() + right_bound.pts[0].pt.x() -
                               lane_in_curr->right_boundary->pts.back().pt.x();
          left_pt_pre.pt.y() = left_pt_pre.pt.y() + right_bound.pts[0].pt.y() -
                               lane_in_curr->right_boundary->pts.back().pt.y();
          left_bound.pts.emplace_back(left_pt_pre);
        }
        float dx = right_bound.pts[right_index].pt.x() -
                   right_bound.pts[right_index - 1].pt.x();
        float dy = right_bound.pts[right_index].pt.y() -
                   right_bound.pts[right_index - 1].pt.y();
        // HLOG_ERROR << "dx = " << dx << " dy = " << dy;
        while (left_pt_pre.pt.x() <
                   next_group->group_segments.back()->end_slice.po.x() &&
               right_index < right_bound_size &&
               (dx > 0 && dx * dx + dy * dy > 0.4 || dx * dx + dy * dy < 0.4)) {
          dx = right_bound.pts[right_index].pt.x() -
               right_bound.pts[right_index - 1].pt.x();
          dy = right_bound.pts[right_index].pt.y() -
               right_bound.pts[right_index - 1].pt.y();
          // HLOG_ERROR << "dx = " << dx << " dy = " << dy;
          float pre_x = left_pt_pre.pt.x() + dx;
          float pre_y = left_pt_pre.pt.y() + dy;
          left_pt_pre = Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          left_bound.pts.emplace_back(left_pt_pre);
          right_index++;
        }
        if (left_bound.pts.empty()) {
          return;
        }
      } else if (left_bound_exist == 0 && right_bound_exist == 0) {
        if (lane_in_curr->center_line_param.empty()) {
          return;
        }
        size_t index_left = lane_in_curr->left_boundary->pts.size();
        Point left_pt_pre(
            PREDICTED, lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
            lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
            lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
        double b_left = left_pt_pre.pt.y() -
                        lane_in_curr->center_line_param[1] * left_pt_pre.pt.x();
        left_bound.pts.emplace_back(left_pt_pre);
        while (left_pt_pre.pt.x() <
               next_group->group_segments.back()->end_slice.po.x()) {
          float pre_x = left_pt_pre.pt.x() + 1.0;
          float pre_y = b_left + lane_in_curr->center_line_param[1] * pre_x;
          left_pt_pre = Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          left_bound.pts.emplace_back(left_pt_pre);
        }
        if (left_bound.pts.empty()) {
          return;
        }
        size_t index_right = lane_in_curr->right_boundary->pts.size();
        Point right_pt_pre(
            PREDICTED,
            lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
            lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
            lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
        double b_right =
            right_pt_pre.pt.y() -
            lane_in_curr->center_line_param[1] * right_pt_pre.pt.x();
        right_bound.pts.emplace_back(right_pt_pre);
        while (right_pt_pre.pt.x() <
               next_group->group_segments.back()->end_slice.po.x()) {
          float pre_x = right_pt_pre.pt.x() + 1.0;
          float pre_y = b_right + lane_in_curr->center_line_param[1] * pre_x;
          right_pt_pre =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          right_bound.pts.emplace_back(right_pt_pre);
        }
        if (right_bound.pts.empty()) {
          return;
        }
      }

      // std::vector<Point> ctl_pt;
      // size_t index_center = lane_in_curr->center_line_pts.size();
      // Point center_pt_pre(
      //     PREDICTED, lane_in_curr->center_line_pts[index_center - 1].pt.x(),
      //     lane_in_curr->center_line_pts[index_center - 1].pt.y(),
      //     lane_in_curr->center_line_pts[index_center - 1].pt.z());
      // while (center_pt_pre.pt.x() <
      //        next_group->group_segments.back()->end_slice.po.x() - 2.0) {
      //   ctl_pt.emplace_back(center_pt_pre);
      //   float pre_x = center_pt_pre.pt.x() + 1.0;
      //   float pre_y = lane_in_curr->center_line_param[0] +
      //                 lane_in_curr->center_line_param[1] * pre_x;
      //   center_pt_pre = Point(PREDICTED, pre_x, pre_y, float(0.0));
      // }
      // if (ctl_pt.empty()) {
      //   return;
      // }

      lane_pre.str_id = lane_in_curr->str_id;
      lane_pre.lanepos_id = lane_in_curr->lanepos_id;
      lane_pre.str_id_with_group = next_group->str_id + ":" + lane_pre.str_id;
      lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
      lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
      Lane::Ptr lane_ptr = std::make_shared<Lane>(lane_pre);
      FitCenterLine(lane_ptr);
      // HLOG_ERROR << "lane_pre.center_line_pts.size() = "
      //            << lane_pre.center_line_pts.size() << "   "
      //            << lane_ptr->center_line_pts.size()
      //            << "  lane_pre.left_boundary = "
      //            << lane_pre.left_boundary->pts.size()
      //            << "  lane_pre.right_boundary = "
      //            << lane_pre.right_boundary->pts.size();

      if (lane_ptr->center_line_pts.empty()) {
        return;
      }
      // lane_pre.center_line_param = lane_in_curr->center_line_param;
      // lane_pre.center_line_param_front = lane_in_curr->center_line_param;
      // lane_pre.center_line_pts = ctl_pt;
      // lane_pre.prev_lane_str_id_with_group.emplace_back(
      //     lane_in_curr->str_id_with_group);
      // lane_in_curr->next_lane_str_id_with_group.emplace_back(
      //     lane_pre.str_id_with_group);

      for (const auto& lane_right :
           lane_in_curr->right_lane_str_id_with_group) {
        size_t index = lane_right.find(":");
        std::string right_str =
            next_group->str_id +
            lane_right.substr(index, lane_right.size() - index);
        lane_pre.right_lane_str_id_with_group.emplace_back(right_str);
      }
      for (const auto& lane_left : lane_in_curr->left_lane_str_id_with_group) {
        size_t index = lane_left.find(":");
        std::string left_str =
            next_group->str_id +
            lane_left.substr(index, lane_left.size() - index);
        lane_pre.left_lane_str_id_with_group.emplace_back(left_str);
      }

      // next_group->lanes.emplace_back(std::make_shared<Lane>(lane_pre));
      next_group->lanes.emplace_back(lane_ptr);
    } else {
      lane_in_curr->next_lane_str_id_with_group.clear();
    }
  }
  // for (auto& lane_in_curr : curr_group->lanes) {
  //   if (lane_in_curr->next_lane_str_id_with_group.empty()) {
  //     if (!lane_in_curr->center_line_param.empty()) {
  //       size_t index_left = lane_in_curr->left_boundary->pts.size();
  //       Point left_pt_pre(
  //           VIRTUAL, lane_in_curr->left_boundary->pts[index_left -
  //           1].pt.x(), lane_in_curr->left_boundary->pts[index_left -
  //           1].pt.y(), lane_in_curr->left_boundary->pts[index_left -
  //           1].pt.z());
  //       double b_left = left_pt_pre.pt.y() -
  //                       lane_in_curr->center_line_param[1] *
  //                       left_pt_pre.pt.x();
  //       while (left_pt_pre.pt.x() <
  //              next_group->group_segments.back()->end_slice.po.x() - 2.0) {
  //         lane_in_curr->left_boundary->pts.emplace_back(left_pt_pre);
  //         float pre_x = left_pt_pre.pt.x() + 1.0;
  //         float pre_y = b_left + lane_in_curr->center_line_param[1] *
  //         pre_x; left_pt_pre = Point(VIRTUAL, pre_x, pre_y,
  //         static_cast<float>(0.0));
  //       }
  //       size_t index_right = lane_in_curr->right_boundary->pts.size();
  //       Point right_pt_pre(
  //           VIRTUAL, lane_in_curr->right_boundary->pts[index_right -
  //           1].pt.x(), lane_in_curr->right_boundary->pts[index_right -
  //           1].pt.y(), lane_in_curr->right_boundary->pts[index_right -
  //           1].pt.z());
  //       double b_right =
  //           right_pt_pre.pt.y() -
  //           lane_in_curr->center_line_param[1] * right_pt_pre.pt.x();
  //       while (right_pt_pre.pt.x() <
  //              next_group->group_segments.back()->end_slice.po.x() - 2.0) {
  //         lane_in_curr->right_boundary->pts.emplace_back(right_pt_pre);
  //         float pre_x = right_pt_pre.pt.x() + 1.0;
  //         float pre_y = b_right + lane_in_curr->center_line_param[1] *
  //         pre_x; right_pt_pre = Point(VIRTUAL, pre_x, pre_y,
  //         static_cast<float>(0.0));
  //       }
  //       size_t index_center = lane_in_curr->center_line_pts.size();
  //       Point center_pt_pre(
  //           VIRTUAL, lane_in_curr->center_line_pts[index_center -
  //           1].pt.x(), lane_in_curr->center_line_pts[index_center -
  //           1].pt.y(), lane_in_curr->center_line_pts[index_center -
  //           1].pt.z());
  //       while (center_pt_pre.pt.x() <
  //              next_group->group_segments.back()->end_slice.po.x() - 2.0) {
  //         lane_in_curr->center_line_pts.emplace_back(center_pt_pre);
  //         float pre_x = center_pt_pre.pt.x() + 1.0;
  //         float pre_y = lane_in_curr->center_line_param[0] +
  //                       lane_in_curr->center_line_param[1] * pre_x;
  //         center_pt_pre = Point(VIRTUAL, pre_x, pre_y,
  //         static_cast<float>(0.0));
  //       }
  //     }
  //   } else {
  //     for (auto& lane_in_next : next_group->lanes) {
  //       if (lane_in_curr->next_lane_str_id_with_group[0] ==
  //           lane_in_next->str_id_with_group) {
  //         lane_in_curr->left_boundary->pts.insert(
  //             lane_in_curr->left_boundary->pts.end(),
  //             lane_in_next->left_boundary->pts.begin(),
  //             lane_in_next->left_boundary->pts.end());
  //         lane_in_curr->right_boundary->pts.insert(
  //             lane_in_curr->right_boundary->pts.end(),
  //             lane_in_next->right_boundary->pts.begin(),
  //             lane_in_next->right_boundary->pts.end());
  //         lane_in_curr->center_line_pts.insert(
  //             lane_in_curr->center_line_pts.end(),
  //             lane_in_next->center_line_pts.begin(),
  //             lane_in_next->center_line_pts.end());
  //         lane_in_curr->center_line_param =
  //             FitLaneline(lane_in_curr->center_line_pts);
  //         break;
  //       }
  //     }
  //     lane_in_curr->next_lane_str_id_with_group.clear();
  //   }
  // }
}

void GroupMap::BuildVirtualLaneBefore(Group::Ptr curr_group,
                                      Group::Ptr next_group) {
  for (auto& lane_in_next : next_group->lanes) {
    if (lane_in_next->prev_lane_str_id_with_group.empty()) {
      //  && !lane_in_next->center_line_param_front.empty()
      Lane lane_pre;
      LineSegment left_bound;
      LineSegment right_bound;
      left_bound.id = lane_in_next->left_boundary->id;
      left_bound.lanepos = lane_in_next->left_boundary->lanepos;
      left_bound.type = lane_in_next->left_boundary->type;
      left_bound.color = lane_in_next->left_boundary->color;
      left_bound.mean_end_heading =
          lane_in_next->left_boundary->mean_end_heading;
      left_bound.mean_end_heading_std_dev =
          lane_in_next->left_boundary->mean_end_heading_std_dev;
      left_bound.mean_end_interval =
          lane_in_next->left_boundary->mean_end_interval;
      right_bound.id = lane_in_next->right_boundary->id;
      right_bound.lanepos = lane_in_next->right_boundary->lanepos;
      right_bound.type = lane_in_next->right_boundary->type;
      right_bound.color = lane_in_next->right_boundary->color;
      right_bound.mean_end_heading =
          lane_in_next->right_boundary->mean_end_heading;
      right_bound.mean_end_heading_std_dev =
          lane_in_next->right_boundary->mean_end_heading_std_dev;
      right_bound.mean_end_interval =
          lane_in_next->right_boundary->mean_end_interval;
      // 判断左右line是否已经存在
      int left_bound_exist = 0, right_bound_exist = 0;
      for (auto& lane_in_curr : curr_group->lanes) {
        if (lane_in_curr->left_boundary->id ==
            lane_in_next->left_boundary->id) {
          // left_bound = *(lane_in_curr->left_boundary);
          for (auto& line_pt : lane_in_curr->left_boundary->pts) {
            Point pt_pre(PREDICTED, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = 1;
        } else if (lane_in_curr->left_boundary->id ==
                   lane_in_next->right_boundary->id) {
          // right_bound = *(lane_in_curr->left_boundary);
          for (auto& line_pt : lane_in_curr->left_boundary->pts) {
            Point pt_pre(PREDICTED, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = 1;
        } else if (lane_in_curr->right_boundary->id ==
                   lane_in_next->left_boundary->id) {
          // left_bound = *(lane_in_curr->right_boundary);
          for (auto& line_pt : lane_in_curr->right_boundary->pts) {
            Point pt_pre(PREDICTED, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            left_bound.pts.emplace_back(pt_pre);
          }
          left_bound_exist = 1;
        } else if (lane_in_curr->right_boundary->id ==
                   lane_in_next->right_boundary->id) {
          // right_bound = *(lane_in_curr->right_boundary);
          for (auto& line_pt : lane_in_curr->right_boundary->pts) {
            Point pt_pre(PREDICTED, line_pt.pt.x(), line_pt.pt.y(),
                         line_pt.pt.z());
            right_bound.pts.emplace_back(pt_pre);
          }
          right_bound_exist = 1;
        }
      }
      if (left_bound_exist == 1 && right_bound_exist == 0) {
        Point right_pt_pre(PREDICTED,
                           lane_in_next->right_boundary->pts[0].pt.x(),
                           lane_in_next->right_boundary->pts[0].pt.y(),
                           lane_in_next->right_boundary->pts[0].pt.z());
        size_t left_index = left_bound.pts.size() - 1;
        right_bound.pts.insert(right_bound.pts.begin(), right_pt_pre);
        float dx = left_bound.pts[left_index].pt.x() -
                   left_bound.pts[left_index - 1].pt.x();
        float dy = left_bound.pts[left_index].pt.y() -
                   left_bound.pts[left_index - 1].pt.y();
        while (right_pt_pre.pt.x() >
                   curr_group->group_segments[0]->end_slice.po.x() &&
               left_index > 0 &&
               (dx > 0 && dx * dx + dy * dy > 0.4 || dx * dx + dy * dy < 0.4)) {
          dx = left_bound.pts[left_index].pt.x() -
               left_bound.pts[left_index - 1].pt.x();
          dy = left_bound.pts[left_index].pt.y() -
               left_bound.pts[left_index - 1].pt.y();
          float pre_x = right_pt_pre.pt.x() - dx;
          float pre_y = right_pt_pre.pt.y() - dy;
          right_pt_pre =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          right_bound.pts.insert(right_bound.pts.begin(), right_pt_pre);
          left_index--;
        }
        if (right_bound.pts.empty()) {
          return;
        }
      } else if (left_bound_exist == 0 && right_bound_exist == 1) {
        Point left_pt_pre(PREDICTED, lane_in_next->left_boundary->pts[0].pt.x(),
                          lane_in_next->left_boundary->pts[0].pt.y(),
                          lane_in_next->left_boundary->pts[0].pt.z());
        size_t right_index = right_bound.pts.size() - 1;
        left_bound.pts.insert(left_bound.pts.begin(), left_pt_pre);
        float dx = right_bound.pts[right_index].pt.x() -
                   right_bound.pts[right_index - 1].pt.x();
        float dy = right_bound.pts[right_index].pt.y() -
                   right_bound.pts[right_index - 1].pt.y();
        while (left_pt_pre.pt.x() >
                   curr_group->group_segments[0]->end_slice.po.x() &&
               right_index > 0 &&
               (dx > 0 && dx * dx + dy * dy > 0.4 || dx * dx + dy * dy < 0.4)) {
          dx = right_bound.pts[right_index].pt.x() -
               right_bound.pts[right_index - 1].pt.x();
          dy = right_bound.pts[right_index].pt.y() -
               right_bound.pts[right_index - 1].pt.y();
          float pre_x = left_pt_pre.pt.x() - dx;
          float pre_y = left_pt_pre.pt.y() - dy;
          left_pt_pre = Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
          left_bound.pts.insert(left_bound.pts.begin(), left_pt_pre);
          right_index--;
        }
        if (left_bound.pts.empty()) {
          return;
        }
      } else if (left_bound_exist == 0 && right_bound_exist == 0) {
        if (lane_in_next->center_line_param_front.empty()) {
          return;
        }
        Point left_pt_pre(PREDICTED, lane_in_next->left_boundary->pts[0].pt.x(),
                          lane_in_next->left_boundary->pts[0].pt.y(),
                          lane_in_next->left_boundary->pts[0].pt.z());
        double b_left =
            left_pt_pre.pt.y() -
            lane_in_next->center_line_param_front[1] * left_pt_pre.pt.x();
        while (left_pt_pre.pt.x() >
               curr_group->group_segments[0]->end_slice.po.x()) {
          left_bound.pts.insert(left_bound.pts.begin(), left_pt_pre);
          float pre_x = left_pt_pre.pt.x() - 1.0;
          float pre_y =
              b_left + lane_in_next->center_line_param_front[1] * pre_x;
          left_pt_pre = Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
        }
        if (left_bound.pts.size() < 2) {
          return;
        }
        Point right_pt_pre(PREDICTED,
                           lane_in_next->right_boundary->pts[0].pt.x(),
                           lane_in_next->right_boundary->pts[0].pt.y(),
                           lane_in_next->right_boundary->pts[0].pt.z());
        double b_right =
            right_pt_pre.pt.y() -
            lane_in_next->center_line_param_front[1] * right_pt_pre.pt.x();
        while (right_pt_pre.pt.x() >
               curr_group->group_segments[0]->end_slice.po.x()) {
          right_bound.pts.insert(right_bound.pts.begin(), right_pt_pre);
          float pre_x = right_pt_pre.pt.x() - 1.0;
          float pre_y =
              b_right + lane_in_next->center_line_param_front[1] * pre_x;
          right_pt_pre =
              Point(PREDICTED, pre_x, pre_y, static_cast<float>(0.0));
        }
        if (right_bound.pts.size() < 2) {
          return;
        }
      }

      // std::vector<Point> ctl_pt;
      // size_t index_center = lane_in_next->center_line_pts.size();
      // Point center_pt_pre(PREDICTED, lane_in_next->center_line_pts[0].pt.x(),
      //                     lane_in_next->center_line_pts[0].pt.y(),
      //                     lane_in_next->center_line_pts[0].pt.z());
      // while (center_pt_pre.pt.x() >
      //        curr_group->group_segments[0]->end_slice.po.x() + 2.0) {
      //   ctl_pt.insert(ctl_pt.begin(), center_pt_pre);
      //   float pre_x = center_pt_pre.pt.x() - 1.0;
      //   float pre_y = lane_in_next->center_line_param_front[0] +
      //                 lane_in_next->center_line_param_front[1] * pre_x;
      //   center_pt_pre = Point(PREDICTED, pre_x, pre_y, float(0.0));
      // }
      // if (ctl_pt.empty()) {
      //   return;
      // }
      lane_pre.str_id = lane_in_next->str_id;
      lane_pre.lanepos_id = lane_in_next->lanepos_id;
      lane_pre.str_id_with_group = curr_group->str_id + ":" + lane_pre.str_id;
      lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
      lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
      // lane_pre.center_line_param = lane_in_next->center_line_param_front;
      // lane_pre.center_line_param_front =
      // lane_in_next->center_line_param_front; lane_pre.center_line_pts =
      // ctl_pt;
      Lane::Ptr lane_ptr = std::make_shared<Lane>(lane_pre);
      FitCenterLine(lane_ptr);
      HLOG_ERROR << "lane_pre.center_line_pts.size() = "
                 << lane_pre.center_line_pts.size() << "   "
                 << lane_ptr->center_line_pts.size()
                 << "  lane_pre.left_boundary = "
                 << lane_pre.left_boundary->pts.size()
                 << "  lane_pre.right_boundary = "
                 << lane_pre.right_boundary->pts.size();

      if (lane_ptr->center_line_pts.empty()) {
        return;
      }
      // lane_pre.next_lane_str_id_with_group.emplace_back(
      //     lane_in_next->str_id_with_group);
      // lane_in_next->prev_lane_str_id_with_group.emplace_back(
      //     lane_pre.str_id_with_group);
      for (const auto& lane_right :
           lane_in_next->right_lane_str_id_with_group) {
        size_t index = lane_right.find(":");
        std::string right_str =
            curr_group->str_id +
            lane_right.substr(index, lane_right.size() - index);
        lane_pre.right_lane_str_id_with_group.emplace_back(right_str);
      }
      for (const auto& lane_left : lane_in_next->left_lane_str_id_with_group) {
        size_t index = lane_left.find(":");
        std::string left_str =
            curr_group->str_id +
            lane_left.substr(index, lane_left.size() - index);
        lane_pre.left_lane_str_id_with_group.emplace_back(left_str);
      }
      curr_group->lanes.emplace_back(lane_ptr);

    } else {
      lane_in_next->prev_lane_str_id_with_group.clear();
    }
  }
  // for (auto& lane_in_next : next_group->lanes) {
  //   if (lane_in_next->prev_lane_str_id_with_group.empty()) {
  //     if (!lane_in_next->center_line_param_front.empty()) {
  //       Point left_pt_pre(VIRTUAL,
  //       lane_in_next->left_boundary->pts[0].pt.x(),
  //                         lane_in_next->left_boundary->pts[0].pt.y(),
  //                         lane_in_next->left_boundary->pts[0].pt.z());
  //       double b_left =
  //           left_pt_pre.pt.y() -
  //           lane_in_next->center_line_param_front[1] * left_pt_pre.pt.x();
  //       while (left_pt_pre.pt.x() >
  //              curr_group->group_segments[0]->start_slice.po.x() + 2.0) {
  //         lane_in_next->left_boundary->pts.insert(
  //             lane_in_next->left_boundary->pts.begin(), left_pt_pre);
  //         float pre_x = left_pt_pre.pt.x() - 1.0;
  //         float pre_y =
  //             b_left + lane_in_next->center_line_param_front[1] * pre_x;
  //         left_pt_pre = Point(VIRTUAL, pre_x, pre_y,
  //         static_cast<float>(0.0));
  //       }

  //       Point right_pt_pre(PointType::VIRTUAL,
  //                          lane_in_next->right_boundary->pts[0].pt.x(),
  //                          lane_in_next->right_boundary->pts[0].pt.y(),
  //                          lane_in_next->right_boundary->pts[0].pt.z());
  //       double b_right =
  //           right_pt_pre.pt.y() -
  //           lane_in_next->center_line_param_front[1] * right_pt_pre.pt.x();
  //       while (right_pt_pre.pt.x() >
  //              curr_group->group_segments[0]->start_slice.po.x() + 2.0) {
  //         lane_in_next->right_boundary->pts.insert(
  //             lane_in_next->right_boundary->pts.begin(), right_pt_pre);
  //         float pre_x = right_pt_pre.pt.x() - 1.0;
  //         float pre_y =
  //             b_right + lane_in_next->center_line_param_front[1] * pre_x;
  //         right_pt_pre = Point(VIRTUAL, pre_x, pre_y,
  //         static_cast<float>(0.0));
  //       }
  //       size_t index_center = lane_in_next->center_line_pts.size();
  //       Point center_pt_pre(VIRTUAL,
  //       lane_in_next->center_line_pts[0].pt.x(),
  //                           lane_in_next->center_line_pts[0].pt.y(),
  //                           lane_in_next->center_line_pts[0].pt.z());
  //       while (center_pt_pre.pt.x() >
  //              curr_group->group_segments[0]->start_slice.po.x() + 2.0) {
  //         lane_in_next->center_line_pts.insert(
  //             lane_in_next->center_line_pts.begin(), center_pt_pre);
  //         float pre_x = center_pt_pre.pt.x() - 1.0;
  //         float pre_y = lane_in_next->center_line_param_front[0] +
  //                       lane_in_next->center_line_param_front[1] * pre_x;
  //         center_pt_pre = Point(VIRTUAL, pre_x, pre_y,
  //         static_cast<float>(0.0));
  //       }
  //     }
  //   } else {
  //     for (auto& lane_in_curr : curr_group->lanes) {
  //       if (lane_in_next->prev_lane_str_id_with_group[0] ==
  //           lane_in_curr->str_id_with_group) {
  //         lane_in_next->left_boundary->pts.insert(
  //             lane_in_next->left_boundary->pts.begin(),
  //             lane_in_curr->left_boundary->pts.begin(),
  //             lane_in_curr->left_boundary->pts.end());
  //         lane_in_next->right_boundary->pts.insert(
  //             lane_in_next->right_boundary->pts.begin(),
  //             lane_in_curr->right_boundary->pts.begin(),
  //             lane_in_curr->right_boundary->pts.end());
  //         lane_in_next->center_line_pts.insert(
  //             lane_in_next->center_line_pts.begin(),
  //             lane_in_curr->center_line_pts.begin(),
  //             lane_in_curr->center_line_pts.end());
  //         lane_in_next->center_line_param_front =
  //             FitLanelinefront(lane_in_next->center_line_pts);
  //         break;
  //       }
  //     }
  //     lane_in_next->prev_lane_str_id_with_group.clear();
  //   }
  // }
}

void GroupMap::BuildVirtualMergeLane(Group::Ptr curr_group,
                                     Group::Ptr next_group, size_t curr_lane,
                                     size_t curr_lane_next) {
  size_t next_lane = -1;
  for (size_t i = 0; i < next_group->lanes.size(); ++i) {
    if (next_group->lanes[i]->str_id_with_group ==
        curr_group->lanes[curr_lane_next]->next_lane_str_id_with_group[0]) {
      next_lane = i;
      break;
    }
  }
  if (next_lane == -1 ||
      next_group->lanes[next_lane]->center_line_param_front.empty() ||
      next_group->lanes[next_lane]->center_line_pts.size() < 11) {
    return;
  }
  size_t grp_seg_size = curr_group->group_segments.size();
  float y1 = -1.0, y2 = -1.0;
  for (const auto& lane_iter :
       curr_group->group_segments[grp_seg_size - 1]->lane_segments) {
    if (lane_iter->str_id == curr_group->lanes[curr_lane]->str_id) {
      y1 = Dist(lane_iter->left_boundary->center,
                lane_iter->right_boundary->center);
    }
  }
  for (const auto& lane_iter :
       curr_group->group_segments[grp_seg_size - 3]->lane_segments) {
    if (lane_iter->str_id == curr_group->lanes[curr_lane]->str_id) {
      y2 = Dist(lane_iter->left_boundary->center,
                lane_iter->right_boundary->center);
    }
  }
  if (y2 < y1 + 0.2) {
    return;
  }
  Lane lane_pre;
  LineSegment left_bound;
  auto& lane_in_curr = curr_group->lanes[curr_lane];
  auto& lane_in_next = next_group->lanes[next_lane];
  left_bound.id = lane_in_curr->left_boundary->id;
  left_bound.lanepos = lane_in_curr->left_boundary->lanepos;
  left_bound.type = lane_in_curr->left_boundary->type;
  left_bound.color = lane_in_curr->left_boundary->color;
  left_bound.isego = lane_in_curr->left_boundary->isego;
  left_bound.mean_end_heading = lane_in_curr->left_boundary->mean_end_heading;
  left_bound.mean_end_heading_std_dev =
      lane_in_curr->left_boundary->mean_end_heading_std_dev;
  left_bound.mean_end_interval = lane_in_curr->left_boundary->mean_end_interval;
  size_t index_left = lane_in_curr->left_boundary->pts.size();

  Point left_pt_pre(PREDICTED,
                    lane_in_curr->left_boundary->pts[index_left - 1].pt.x(),
                    lane_in_curr->left_boundary->pts[index_left - 1].pt.y(),
                    lane_in_curr->left_boundary->pts[index_left - 1].pt.z());
  // float b_left = left_pt_pre.pt.y() -
  //                lane_in_curr->center_line_param[1] * left_pt_pre.pt.x();

  // float b_next_left = lane_in_next->left_boundary->pts[0].pt.y() -
  //                     lane_in_next->center_line_param_front[1] *
  //                         lane_in_next->left_boundary->pts[0].pt.x();

  // float st_sl_x = left_pt_pre.pt.x();
  // float distance = st_sl_x * lane_in_curr->center_line_param[1] + b_left -
  //                  st_sl_x * lane_in_next->center_line_param_front[1] -
  //                  b_next_left;
  // int flag = 1;
  // while (abs(distance) > 0.2 && flag > 0) {
  //   left_bound.pts.emplace_back(left_pt_pre);
  //   st_sl_x += 1.0;
  //   float pre_y = b_left + lane_in_curr->center_line_param[1] * st_sl_x;
  //   left_pt_pre = Point(PREDICTED, st_sl_x, pre_y, float(0.0));
  //   float distance2 = pre_y -
  //                     st_sl_x * lane_in_next->center_line_param_front[1] -
  //                     b_next_left;
  //   flag = (abs(distance2) < abs(distance)) ? 1 : 0;
  //   distance = distance2;
  // }
  auto dis_left = lane_in_next->left_boundary->pts[9].pt - left_pt_pre.pt;
  for (int i = 0; i < 10; ++i) {
    left_pt_pre =
        Point(PREDICTED, left_pt_pre.pt.x() + dis_left.x() * i,
              left_pt_pre.pt.y() + dis_left.y() * i, static_cast<float>(0.0));
    left_bound.pts.emplace_back(left_pt_pre);
  }

  if (left_bound.pts.empty()) {
    return;
  }
  LineSegment right_bound;
  right_bound.id = lane_in_curr->right_boundary->id;
  right_bound.lanepos = lane_in_curr->right_boundary->lanepos;
  right_bound.type = lane_in_curr->right_boundary->type;
  right_bound.color = lane_in_curr->right_boundary->color;
  right_bound.isego = lane_in_curr->right_boundary->isego;
  right_bound.mean_end_heading = lane_in_curr->right_boundary->mean_end_heading;
  right_bound.mean_end_heading_std_dev =
      lane_in_curr->right_boundary->mean_end_heading_std_dev;
  right_bound.mean_end_interval =
      lane_in_curr->right_boundary->mean_end_interval;
  size_t index_right = lane_in_curr->right_boundary->pts.size();
  Point right_pt_pre(PREDICTED,
                     lane_in_curr->right_boundary->pts[index_right - 1].pt.x(),
                     lane_in_curr->right_boundary->pts[index_right - 1].pt.y(),
                     lane_in_curr->right_boundary->pts[index_right - 1].pt.z());
  // float b_right = right_pt_pre.pt.y() -
  //                 lane_in_curr->center_line_param[1] * right_pt_pre.pt.x();
  float b_next_right = lane_in_next->right_boundary->pts[0].pt.y() -
                       lane_in_next->center_line_param_front[1] *
                           lane_in_next->right_boundary->pts[0].pt.x();
  // st_sl_x = right_pt_pre.pt.x();
  // distance = st_sl_x * lane_in_curr->center_line_param[1] + b_right -
  //            st_sl_x * lane_in_next->center_line_param_front[1] -
  //            b_next_right;
  // flag = 1;
  // while (abs(distance) > 0.2 && flag > 0) {
  //   right_bound.pts.emplace_back(right_pt_pre);
  //   st_sl_x += 1.0;
  //   float pre_y = b_right + lane_in_curr->center_line_param[1] * st_sl_x;
  //   right_pt_pre = Point(PREDICTED, st_sl_x, pre_y, float(0.0));
  //   float distance2 = pre_y -
  //                     st_sl_x * lane_in_next->center_line_param_front[1] -
  //                     b_next_right;
  //   flag = (abs(distance2) < abs(distance)) ? 1 : 0;
  //   distance = distance2;
  // }
  Eigen::Vector3f poly_right(lane_in_next->left_boundary->pts[9].pt.x(),
                             lane_in_next->left_boundary->pts[9].pt.x() *
                                     lane_in_next->center_line_param_front[1] +
                                 b_next_right,
                             0.0);
  auto dis_right = poly_right - right_pt_pre.pt;
  for (int i = 0; i < 10; ++i) {
    right_pt_pre =
        Point(PointType::PREDICTED, right_pt_pre.pt.x() + dis_right.x() * i,
              right_pt_pre.pt.y() + dis_right.y() * i, static_cast<float>(0.0));
    right_bound.pts.emplace_back(right_pt_pre);
  }

  if (right_bound.pts.empty()) {
    return;
  }

  std::vector<Point> ctl_pt;
  size_t index_center = lane_in_curr->center_line_pts.size();
  Point center_pt_pre(PointType::PREDICTED,
                      lane_in_curr->center_line_pts[index_center - 1].pt.x(),
                      lane_in_curr->center_line_pts[index_center - 1].pt.y(),
                      lane_in_curr->center_line_pts[index_center - 1].pt.z());
  Eigen::Vector3f poly_center(lane_in_next->left_boundary->pts[9].pt.x(),
                              lane_in_next->left_boundary->pts[9].pt.x() *
                                      lane_in_next->center_line_param_front[1] +
                                  lane_in_next->center_line_param_front[0],
                              0.0);
  auto dis_center = poly_center - center_pt_pre.pt;
  for (int i = 0; i < 10; ++i) {
    center_pt_pre = Point(
        PointType::PREDICTED, center_pt_pre.pt.x() + dis_center.x() * i,
        center_pt_pre.pt.y() + dis_center.y() * i, static_cast<float>(0.0));
    ctl_pt.emplace_back(center_pt_pre);
  }
  // st_sl_x = center_pt_pre.pt.x();
  // distance = lane_in_curr->center_line_param[1] * st_sl_x +
  //            lane_in_curr->center_line_param[0] -
  //            lane_in_next->center_line_param_front[1] * st_sl_x -
  //            lane_in_next->center_line_param_front[0];
  // flag = 1;
  // while (abs(distance) > 0.2 && flag > 0) {
  //   ctl_pt.emplace_back(center_pt_pre);
  //   st_sl_x += 1.0;
  //   float pre_y = lane_in_curr->center_line_param[1] * st_sl_x +
  //                 lane_in_curr->center_line_param[0];
  //   center_pt_pre = Point(PREDICTED, st_sl_x, pre_y, float(0.0));
  //   float distance2 = pre_y -
  //                     st_sl_x * lane_in_next->center_line_param_front[1] -
  //                     lane_in_next->center_line_param_front[0];
  //   flag = (abs(distance2) < abs(distance)) ? 1 : 0;
  //   distance = distance2;
  // }

  // HLOG_ERROR <<"distance = "<<distance;
  if (ctl_pt.empty()) {
    return;
  }
  lane_pre.str_id = lane_in_curr->str_id;
  lane_pre.lanepos_id = lane_in_curr->lanepos_id;
  lane_pre.str_id_with_group = next_group->str_id + ":" + lane_pre.str_id;
  lane_pre.left_boundary = std::make_shared<LineSegment>(left_bound);
  lane_pre.right_boundary = std::make_shared<LineSegment>(right_bound);
  lane_pre.center_line_pts = ctl_pt;

  lane_pre.prev_lane_str_id_with_group.emplace_back(
      lane_in_curr->str_id_with_group);
  lane_in_curr->next_lane_str_id_with_group.emplace_back(
      lane_pre.str_id_with_group);
  lane_pre.next_lane_str_id_with_group.emplace_back(
      lane_in_next->str_id_with_group);
  if (lane_pre.left_boundary->pts.size() > 2 &&
      lane_pre.right_boundary->pts.size() > 2) {
    next_group->lanes.emplace_back(std::make_shared<Lane>(lane_pre));
  }
}
// 判断车道线是否需要预测，当前预测的条件是：
// 1.线的末端点必须在车前方；
// 2.线的末端点距离自车距离在可信赖的感知范围之外，即可信赖的感知范围的线是不会预测的；
// 3.线末端平均heading足够小，即heading与自车行驶方向差异较大的线也不会预测；
// 4.线末端heading的标准差足够小（太大说明heading变换较大，可能是曲线）；
// 5.线末端平均点间距足够大；
//! TBD：当前仅对比较平直的线进行直线预测，后续考虑对弯道进行预测
bool GroupMap::LaneLineNeedToPredict(const LineSegment& line, bool check_back) {
  Eigen::Vector2f back_pt = line.pts.back().pt.head<2>();
  auto mean_heading = line.mean_end_heading;
  auto mean_heading_std = line.mean_end_heading_std_dev;
  auto mean_interval = line.mean_end_interval;
  float dist_to_veh = back_pt.norm();
  bool valid_back = true;
  if (check_back && back_pt.x() < -40) {
    valid_back = false;
  }
  if (valid_back &&
      // dist_to_veh > conf_.robust_percep_dist &&
      dist_to_veh < conf_.predict_farthest_dist &&
      mean_heading < conf_.max_heading_rad &&
      // mean_heading_std < conf_.max_heading_std_dev &&
      mean_interval > conf_.min_predict_interval) {
    return true;
  }
  return false;
}

// 对line从最后一个点开始，沿着heading方向直线预测
//! TBD：当前直接按直线预测，后续考虑对弯道继续非直线预测，比如按曲率
void GroupMap::PredictLaneLine(LineSegment* line) {
  if (line == nullptr || line->pts.empty()) {
    return;
  }

  Eigen::Vector2f back_pt = line->pts.back().pt.head<2>();
  auto mean_interval = line->mean_end_interval;
  float dist_to_veh = back_pt.norm();
  auto pred_length = conf_.predict_farthest_dist - dist_to_veh;
  auto pred_counts = static_cast<int>(pred_length / mean_interval);
  Eigen::Vector2f n(std::cos(line->pred_end_heading),
                    std::sin(line->pred_end_heading));
  for (int i = 0; i < pred_counts; ++i) {
    Eigen::Vector2f pt = back_pt + (i + 1) * n;
    Point pred_pt;
    pred_pt.pt << pt.x(), pt.y(), 0;
    pred_pt.type = gm::PREDICTED;
    line->pts.emplace_back(pred_pt);
  }
}

void GroupMap::PredictLaneLine(double heading, std::vector<Point>* line,
                               double mean_end_interval) {
  if (line == nullptr || line->empty()) {
    return;
  }

  Eigen::Vector2f back_pt = line->back().pt.head<2>();
  auto mean_interval = mean_end_interval;
  float dist_to_veh = back_pt.norm();
  auto pred_length = conf_.predict_farthest_dist - dist_to_veh;
  auto pred_counts = static_cast<int>(pred_length / mean_interval);
  Eigen::Vector2f n(std::cos(heading), std::sin(heading));
  for (int i = 0; i < pred_counts; ++i) {
    Eigen::Vector2f pt = back_pt + (i + 1) * n;
    Point pred_pt;
    pred_pt.pt << pt.x(), pt.y(), 0;
    pred_pt.type = gm::PREDICTED;
    line->emplace_back(pred_pt);
  }
}

// 判断车道是否收缩，即宽度越来越小
bool GroupMap::IsLaneShrink(Lane::Ptr lane) {
  // 为空或点太少，默认非收缩
  if (lane == nullptr || lane->left_boundary->pts.size() < 3 ||
      lane->right_boundary->pts.size() < 3) {
    return false;
  }
  const auto& left = lane->left_boundary->pts;
  const auto& right = lane->right_boundary->pts;
  const auto& left_back_pt = left.back().pt;
  const auto& left_mid_pt = left.at(left.size() / 2).pt;

  const auto& right_back_v_pt0 = right.at(right.size() - 2).pt;
  const auto& right_back_v_pt1 = right.at(right.size() - 1).pt;

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
  const float kShrinkDiffThreshold = 0.5;
  if (left_diff_dist > 0 && std::abs(left_diff_dist) > kShrinkDiffThreshold) {
    return true;
  }

  return false;
}

double GroupMap::CalcLaneLength(Lane::Ptr lane) {
  if (lane == nullptr || lane->center_line_pts.size() < 2) {
    return 0;
  }
  double len = 0;
  for (size_t i = 0; i < lane->center_line_pts.size() - 1; ++i) {
    Eigen::Vector3f p0(lane->center_line_pts.at(i).pt.x(),
                       lane->center_line_pts.at(i).pt.y(), 0);
    Eigen::Vector3f p1(lane->center_line_pts.at(i + 1).pt.x(),
                       lane->center_line_pts.at(i + 1).pt.y(), 0);
    len += Dist(p0, p1);
  }
  return len;
}

std::shared_ptr<hozon::mp::mf::em::ElementMapOut> GroupMap::ConvertToElementMap(
    const std::vector<Group::Ptr>& groups, const KinePose::Ptr& curr_pose,
    const em::ElementMap::Ptr& ele) {
  for (const auto& grp : groups) {
    if (grp == nullptr) {
      HLOG_ERROR << "found nullptr group";
      return nullptr;
    }
  }
  auto ele_map = std::make_shared<hozon::mp::mf::em::ElementMapOut>();
  ele_map->map_info.stamp = groups.front()->stamp;
  int lane_id = 0;
  std::map<std::string, int> lane_id_hash;
  for (const auto& grp : groups) {
    for (const auto& lane : grp->lanes) {
      lane_id_hash.insert_or_assign(lane->str_id_with_group, lane_id++);
    }
  }

  std::vector<std::vector<int32_t>> lane_ids_in_group;

  std::map<std::string, int> node_id_hash;
  int boundary_id = 0, node_id = 0, center_id = 0;
  for (const auto& grp : groups) {
    lane_ids_in_group.emplace_back(std::vector<int32_t>());
    for (const auto& lane : grp->lanes) {
      em::Lane ele_lane;
      ele_lane.left_track_id = lane->left_boundary->id;
      ele_lane.right_track_id = lane->right_boundary->id;
      // id
      ele_lane.id = lane_id_hash[lane->str_id_with_group];
      lane_ids_in_group.back().emplace_back(ele_lane.id);
      // central_curve
      em::CenterLineDetailed centerline;
      centerline.id = center_id++;
      double length = 0;
      em::Point prev_pt;
      for (size_t i = 0; i != lane->center_line_pts.size(); ++i) {
        const auto& pt = lane->center_line_pts[i];
        if (i > 0) {
          length += Dist(pt.pt, prev_pt);
        }
        prev_pt = pt.pt;
        em::PointDetailed center_pt;
        center_pt.point = pt.pt;
        if (pt.type == gm::RAW) {
          center_pt.pointtype = em::RAW;
        } else if (pt.type == gm::INTERPOLATED) {
          center_pt.pointtype = em::INTERPOLATED;
        } else {
          center_pt.pointtype = em::PREDICTED;
        }
        centerline.points.emplace_back(center_pt);
        // HLOG_ERROR<<"center_pt.pointtype = "<<center_pt.pointtype;
      }
      ele_lane.center_line_id = centerline.id;
      ele_map->center_lines[centerline.id] =
          std::make_shared<em::CenterLineDetailed>(centerline);

      // leftboundary
      em::BoundaryDetailed left_bound;
      {
        left_bound.id = boundary_id++;
        for (size_t i = 0; i < lane->left_boundary->pts.size(); ++i) {
          const auto& pt = lane->left_boundary->pts[i];
          em::PointDetailed left_pt;
          left_pt.point = pt.pt;
          if (pt.type == gm::RAW) {
            left_pt.pointtype = em::RAW;
          } else if (pt.type == gm::INTERPOLATED) {
            left_pt.pointtype = em::INTERPOLATED;
          } else {
            left_pt.pointtype = em::PREDICTED;
          }
          left_bound.nodes.emplace_back(left_pt);
        }
        left_bound.color = lane->left_boundary->color;
        left_bound.is_ego = lane->left_boundary->isego;
        left_bound.linetype = lane->left_boundary->type;
        left_bound.lanepos = lane->left_boundary->lanepos;
        ele_lane.left_boundary_ids.emplace_back(left_bound.id);
        ele_map->boundaries[left_bound.id] =
            std::make_shared<em::BoundaryDetailed>(left_bound);
      }
      // rightboundary
      em::BoundaryDetailed right_bound;
      {
        right_bound.id = boundary_id++;
        for (size_t i = 0; i < lane->right_boundary->pts.size(); ++i) {
          const auto& pt = lane->right_boundary->pts[i];
          em::PointDetailed left_pt;
          left_pt.point = pt.pt;
          if (pt.type == gm::RAW) {
            left_pt.pointtype = em::RAW;
          } else if (pt.type == gm::INTERPOLATED) {
            left_pt.pointtype = em::INTERPOLATED;
          } else {
            left_pt.pointtype = em::PREDICTED;
          }
          right_bound.nodes.emplace_back(left_pt);
        }
        right_bound.color = lane->right_boundary->color;
        right_bound.is_ego = lane->right_boundary->isego;
        right_bound.linetype = lane->right_boundary->type;
        right_bound.lanepos = lane->right_boundary->lanepos;
        ele_lane.right_boundary_ids.emplace_back(right_bound.id);
        ele_map->boundaries[right_bound.id] =
            std::make_shared<em::BoundaryDetailed>(right_bound);
      }
      ele_lane.length = static_cast<float>(length);
      for (size_t i = 0; i < lane->next_lane_str_id_with_group.size(); ++i) {
        ele_lane.next_lane_ids.emplace_back(
            lane_id_hash[lane->next_lane_str_id_with_group[i]]);
      }
      for (size_t i = 0; i < lane->prev_lane_str_id_with_group.size(); ++i) {
        ele_lane.prev_lane_ids.emplace_back(
            lane_id_hash[lane->prev_lane_str_id_with_group[i]]);
      }
      for (size_t i = 0; i < lane->left_lane_str_id_with_group.size(); ++i) {
        ele_lane.left_forward_lane_ids.emplace_back(
            lane_id_hash[lane->left_lane_str_id_with_group[i]]);
      }
      for (size_t i = 0; i < lane->right_lane_str_id_with_group.size(); ++i) {
        ele_lane.right_forward_lane_ids.emplace_back(
            lane_id_hash[lane->right_lane_str_id_with_group[i]]);
      }
      ele_map->lanes[ele_lane.id] = std::make_shared<em::Lane>(ele_lane);
    }
  }
  if (lane_ids_in_group.size() != groups.size()) {
    HLOG_ERROR << "lane_ids_in_group size not matched with groups, "
               << lane_ids_in_group.size() << ", " << groups.size()
               << ", not add roads";
    return ele_map;
  }

  int grp_idx = -1;
  int grp_size = lane_ids_in_group.size();
  for (const auto& grp : lane_ids_in_group) {
    grp_idx += 1;
    if (grp.empty()) {
      continue;
    }
    em::Road ele_road;
    ele_road.id = grp_idx;
    for (const auto& lane_ids : grp) {
      ele_road.lane_ids.emplace_back(lane_ids);
    }
    if (grp_idx > 1) {
      ele_road.prev_road_ids.emplace_back(grp_idx - 1);
    }
    if (grp_idx < grp_size) {
      ele_road.next_road_ids.emplace_back(grp_idx + 1);
    }
    ele_map->roads[ele_road.id] = std::make_shared<em::Road>(ele_road);
  }

  // stop lines arrows crosswalk
  ele_map->stop_lines = ele->stop_lines;
  ele_map->arrows = ele->arrows;
  ele_map->cross_walks = ele->cross_walks;
  return ele_map;
}

// 把Group里元素转换成map proto，并且坐标变换到local系
std::shared_ptr<hozon::hdmap::Map> GroupMap::ConvertToProtoMap(
    const std::vector<Group::Ptr>& groups, const KinePose::Ptr& curr_pose,
    const em::ElementMap::Ptr& ele_map, HistoryId* history_id) {
  for (const auto& grp : groups) {
    if (grp == nullptr) {
      HLOG_ERROR << "found nullptr group";
      return nullptr;
    }
  }

  auto map = std::make_shared<hozon::hdmap::Map>();
  map->Clear();
  // 填充header
  map->mutable_header()->mutable_header()->set_data_stamp(
      groups.front()->stamp);

  int lane_id = history_id->lane_id;  // 0;
  std::map<std::string, int> lane_id_hash;
  for (const auto& grp : groups) {
    for (const auto& lane : grp->lanes) {
      lane_id++;
      lane_id = lane_id % history_id->cicle;
      lane_id_hash.insert_or_assign(lane->str_id_with_group, lane_id);
    }
  }
  history_id->lane_id = lane_id;
  std::vector<std::vector<std::string>> lane_ids_in_group;

  for (const auto& grp : groups) {
    lane_ids_in_group.emplace_back(std::vector<std::string>());
    for (const auto& lane : grp->lanes) {
      auto* proto_lane = map->add_lane();
      // id
      auto proto_lane_id =
          std::to_string(lane_id_hash[lane->str_id_with_group]);
      proto_lane->mutable_id()->set_id(proto_lane_id);
      lane_ids_in_group.back().emplace_back(proto_lane_id);

      // central_curve
      auto* central_seg = proto_lane->mutable_central_curve()->add_segment();
      double length = 0;
      em::Point prev_pt;
      for (size_t i = 0; i != lane->center_line_pts.size(); ++i) {
        const auto& pt = lane->center_line_pts[i].pt;
        if (i > 0) {
          length += Dist(pt, prev_pt);
        }
        prev_pt = pt;
        em::Point pt_local = curr_pose->TransformPoint(pt);
        auto* proto_pt = central_seg->mutable_line_segment()->add_point();
        proto_pt->set_x(pt_local.x());
        proto_pt->set_y(pt_local.y());
        proto_pt->set_z(pt_local.z());
      }
      central_seg->set_length(length);
      //! TBD: heading
      // central_seg->set_heading();

      // left_boundary
      bool is_virtual = false;
      {
        auto* left_boundary = proto_lane->mutable_left_boundary();
        auto* left_seg = left_boundary->mutable_curve()->add_segment();
        length = 0;
        prev_pt.setZero();
        for (size_t i = 0; i != lane->left_boundary->pts.size(); ++i) {
          const auto& pt = lane->left_boundary->pts[i].pt;
          if (i > 0) {
            length += Dist(pt, prev_pt);
          }
          if (i == 1 && lane->left_boundary->pts[i].type == VIRTUAL) {
            is_virtual = true;
          }
          prev_pt = pt;
          em::Point pt_local = curr_pose->TransformPoint(pt);
          auto* proto_pt = left_seg->mutable_line_segment()->add_point();
          proto_pt->set_x(pt_local.x());
          proto_pt->set_y(pt_local.y());
          proto_pt->set_z(pt_local.z());
        }
        left_seg->set_length(length);
        //! TBD: heading
        // left_seg->set_heading();
        left_boundary->set_virtual_(is_virtual);
        auto* boundary_type = left_boundary->add_boundary_type();
        //! TBD: 这里s设为整个线的长度行不行?
        boundary_type->set_s(0.0);
        auto type = ProtoBoundType::UNKNOWN;
        std::set<em::LineType> dotted = {
            em::LaneType_DASHED,
            em::LaneType_SHORT_DASHED,
            em::LaneType_DOUBLE_DASHED,
            em::LaneType_FISHBONE_DASHED,
        };
        std::set<em::LineType> solid = {
            em::LaneType_SOLID,
            em::LaneType_FISHBONE_SOLID,
        };
        if (lane->left_boundary->lanepos > 0) {
          dotted.insert(em::LaneType_RIGHT_SOLID_LEFT_DASHED);
          solid.insert(em::LaneType_LEFT_SOLID_RIGHT_DASHED);
        } else {
          dotted.insert(em::LaneType_LEFT_SOLID_RIGHT_DASHED);
          solid.insert(em::LaneType_RIGHT_SOLID_LEFT_DASHED);
        }
        if (dotted.find(lane->left_boundary->type) != dotted.end() &&
            lane->left_boundary->color == em::YELLOW) {
          type = ProtoBoundType::DOTTED_YELLOW;
        } else if (dotted.find(lane->left_boundary->type) != dotted.end() &&
                   lane->left_boundary->color == em::WHITE) {
          type = ProtoBoundType::DOTTED_WHITE;
        } else if (solid.find(lane->left_boundary->type) != solid.end() &&
                   lane->left_boundary->color == em::YELLOW) {
          type = ProtoBoundType::SOLID_YELLOW;
        } else if (solid.find(lane->left_boundary->type) != solid.end() &&
                   lane->left_boundary->color == em::WHITE) {
          type = ProtoBoundType::SOLID_WHITE;
        } else if (lane->left_boundary->type == em::LaneType_DOUBLE_SOLID &&
                   lane->left_boundary->color == em::YELLOW) {
          type = ProtoBoundType::DOUBLE_YELLOW;
        }
        boundary_type->add_types(type);
      }
      // right_boundary
      is_virtual = false;
      {
        auto* right_boundary = proto_lane->mutable_right_boundary();
        auto* right_seg = right_boundary->mutable_curve()->add_segment();
        length = 0;
        prev_pt.setZero();
        for (size_t i = 0; i != lane->right_boundary->pts.size(); ++i) {
          const auto& pt = lane->right_boundary->pts[i].pt;
          if (i > 0) {
            length += Dist(pt, prev_pt);
          }
          if (i == 1 && lane->right_boundary->pts[i].type == VIRTUAL) {
            is_virtual = true;
          }
          prev_pt = pt;
          em::Point pt_local = curr_pose->TransformPoint(pt);
          auto* proto_pt = right_seg->mutable_line_segment()->add_point();
          proto_pt->set_x(pt_local.x());
          proto_pt->set_y(pt_local.y());
          proto_pt->set_z(pt_local.z());
        }
        right_seg->set_length(length);
        //! TBD: heading
        // right_seg->set_heading();
        right_boundary->set_virtual_(is_virtual);
        auto* boundary_type = right_boundary->add_boundary_type();
        //! TBD: 这里s设为整个线的长度行不行?
        boundary_type->set_s(0.0);
        auto type = ProtoBoundType::UNKNOWN;
        std::set<em::LineType> dotted = {
            em::LaneType_DASHED,
            em::LaneType_SHORT_DASHED,
            em::LaneType_DOUBLE_DASHED,
            em::LaneType_FISHBONE_DASHED,
        };
        std::set<em::LineType> solid = {
            em::LaneType_SOLID,
            em::LaneType_FISHBONE_SOLID,
        };
        if (lane->right_boundary->lanepos > 0) {
          dotted.insert(em::LaneType_RIGHT_SOLID_LEFT_DASHED);
          solid.insert(em::LaneType_LEFT_SOLID_RIGHT_DASHED);
        } else {
          dotted.insert(em::LaneType_LEFT_SOLID_RIGHT_DASHED);
          solid.insert(em::LaneType_RIGHT_SOLID_LEFT_DASHED);
        }
        if (dotted.find(lane->right_boundary->type) != dotted.end() &&
            lane->right_boundary->color == em::YELLOW) {
          type = ProtoBoundType::DOTTED_YELLOW;
        } else if (dotted.find(lane->right_boundary->type) != dotted.end() &&
                   lane->right_boundary->color == em::WHITE) {
          type = ProtoBoundType::DOTTED_WHITE;
        } else if (solid.find(lane->right_boundary->type) != solid.end() &&
                   lane->right_boundary->color == em::YELLOW) {
          type = ProtoBoundType::SOLID_YELLOW;
        } else if (solid.find(lane->right_boundary->type) != solid.end() &&
                   lane->right_boundary->color == em::WHITE) {
          type = ProtoBoundType::SOLID_WHITE;
        } else if (lane->right_boundary->type == em::LaneType_DOUBLE_SOLID &&
                   lane->right_boundary->color == em::YELLOW) {
          type = ProtoBoundType::DOUBLE_YELLOW;
        }
        boundary_type->add_types(type);
      }

      // length
      //! TBD: 设为中心线长度行不行？
      proto_lane->set_length(central_seg->length());
      //! TBD: speed_limit
      proto_lane->set_speed_limit(conf_.lane_speed_limit_kmph / 3.6);

      // left_neighbor_forward_lane_id
      for (const auto& left_id : lane->left_lane_str_id_with_group) {
        int id = -1;
        if (lane_id_hash.find(left_id) != lane_id_hash.end()) {
          id = lane_id_hash[left_id];
          proto_lane->add_left_neighbor_forward_lane_id()->set_id(
              std::to_string(id));
        } else {
          HLOG_ERROR << "cannot find " << left_id << " in lane_id_hash";
        }
      }

      // right_neighbor_forward_lane_id
      for (const auto& right_id : lane->right_lane_str_id_with_group) {
        int id = -1;
        if (lane_id_hash.find(right_id) != lane_id_hash.end()) {
          id = lane_id_hash[right_id];
          proto_lane->add_right_neighbor_forward_lane_id()->set_id(
              std::to_string(id));
        } else {
          HLOG_ERROR << "cannot find " << right_id << " in lane_id_hash";
        }
      }

      // successor_id
      for (const auto& next_id : lane->next_lane_str_id_with_group) {
        int id = -1;
        if (lane_id_hash.find(next_id) != lane_id_hash.end()) {
          id = lane_id_hash[next_id];
          proto_lane->add_successor_id()->set_id(std::to_string(id));
        } else {
          HLOG_ERROR << "cannot find " << next_id << " in lane_id_hash";
        }
      }

      // predecessor_id
      for (const auto& prev_id : lane->prev_lane_str_id_with_group) {
        int id = -1;
        if (lane_id_hash.find(prev_id) != lane_id_hash.end()) {
          id = lane_id_hash[prev_id];
          proto_lane->add_predecessor_id()->set_id(std::to_string(id));
        } else {
          HLOG_ERROR << "cannot find " << prev_id << " in lane_id_hash";
        }
      }

      //! TBD: LaneType都设为城区行不行?
      proto_lane->set_type(hozon::hdmap::Lane_LaneType_CITY_DRIVING);

      //! TBD: LaneTurn暂时都设为NOTURN行不行?
      proto_lane->set_turn(hozon::hdmap::Lane_LaneTurn_NO_TURN);

      //! TBD: MapLaneType暂时都设为normal行不行?
      proto_lane->mutable_map_lane_type()->set_normal(true);
    }
  }

  if (lane_ids_in_group.size() != groups.size()) {
    HLOG_ERROR << "lane_ids_in_group size not matched with groups, "
               << lane_ids_in_group.size() << ", " << groups.size()
               << ", not add roads";
    return map;
  }

  //! TBD: 当前认为始终一条Road，每个group对应一个RoadSection
  //! TBD: 暂时未加RoadBoundary
  auto* proto_road = map->add_road();
  proto_road->mutable_id()->set_id("0");
  int grp_idx = history_id->road_id;  // -1;
  for (const auto& grp : lane_ids_in_group) {
    grp_idx += 1;
    grp_idx = grp_idx % history_id->cicle;
    if (grp.empty()) {
      continue;
    }
    auto* proto_road_section = proto_road->add_section();
    proto_road_section->set_max_max_speed(conf_.road_max_max_speed_kmph / 3.6);
    proto_road_section->set_min_max_speed(conf_.road_min_max_speed_kmph / 3.6);
    proto_road_section->mutable_id()->set_id(std::to_string(grp_idx));
    for (const auto& id : grp) {
      auto* proto_lane_id = proto_road_section->add_lane_id();
      proto_lane_id->set_id(id);
    }
  }
  history_id->road_id = grp_idx;
  // stop lines
  // for (const auto& stopline_it : ele_map->stop_lines) {
  //   auto* stop_line = map->add_stop_line();
  //   stop_line->set_id(std::to_string(stopline_it.first));
  //   Eigen::Vector3f left_point =
  //       curr_pose->TransformPoint(stopline_it.second->points[0]);
  //   auto* point_left = stop_line->mutable_shape()->add_point();
  //   point_left->set_x(static_cast<double>(left_point.x()));
  //   point_left->set_y(static_cast<double>(left_point.y()));
  //   point_left->set_z(static_cast<double>(left_point.z()));
  //   Eigen::Vector3f right_point =
  //       curr_pose->TransformPoint(stopline_it.second->points[1]);

  //   auto* point_right = stop_line->mutable_shape()->add_point();
  //   point_right->set_x(static_cast<double>(right_point.x()));
  //   point_right->set_y(static_cast<double>(right_point.y()));
  //   point_right->set_z(static_cast<double>(right_point.z()));
  // }
  for (const auto& stopline_it : ele_map->stop_lines) {
    auto* signal = map->add_signal();
    signal->mutable_id()->set_id(std::to_string(stopline_it.first));
    Eigen::Vector3f left_point =
        curr_pose->TransformPoint(stopline_it.second->points[0]);
    auto* point_left = signal->add_stop_line()
                           ->add_segment()
                           ->mutable_line_segment()
                           ->add_point();
    point_left->set_x(static_cast<double>(left_point.x()));
    point_left->set_y(static_cast<double>(left_point.y()));
    point_left->set_z(static_cast<double>(left_point.z()));
    Eigen::Vector3f right_point =
        curr_pose->TransformPoint(stopline_it.second->points[1]);
    auto* point_right = signal->add_stop_line()
                            ->add_segment()
                            ->mutable_line_segment()
                            ->add_point();
    point_right->set_x(static_cast<double>(right_point.x()));
    point_right->set_y(static_cast<double>(right_point.y()));
    point_right->set_z(static_cast<double>(right_point.z()));
  }

  // crosswalk
  for (const auto& crosswalk_it : ele_map->cross_walks) {
    if (crosswalk_it.second->polygon.points.size() != 4) {
      continue;
    }
    auto* cross_walk = map->add_crosswalk();
    cross_walk->mutable_id()->set_id(std::to_string(crosswalk_it.first));
    for (const auto& point_it : crosswalk_it.second->polygon.points) {
      Eigen::Vector3f pt = curr_pose->TransformPoint(point_it);
      auto* cross_walk_pt = cross_walk->mutable_polygon()->add_point();
      cross_walk_pt->set_x(static_cast<double>(pt.x()));
      cross_walk_pt->set_y(static_cast<double>(pt.y()));
      cross_walk_pt->set_z(static_cast<double>(pt.z()));
    }
  }

  // arrows
  for (const auto& arrow_it : ele_map->arrows) {
    if (arrow_it.second->polygon.points.size() != 4) {
      continue;
    }
    auto* arrow = map->add_arraw();
    arrow->set_id(std::to_string(arrow_it.first));
    double center_x = 0.0, center_y = 0.0;
    for (const auto& point_it : arrow_it.second->polygon.points) {
      Eigen::Vector3f pt = curr_pose->TransformPoint(point_it);
      auto* arrow_pt = arrow->mutable_shape()->add_point();
      arrow_pt->set_x(static_cast<double>(pt.x()));
      arrow_pt->set_y(static_cast<double>(pt.y()));
      arrow_pt->set_z(static_cast<double>(pt.z()));
      center_x += pt.x();
      center_y += pt.y();
    }
    arrow->mutable_center_point()->set_x(center_x / 4);
    arrow->mutable_center_point()->set_y(center_y / 4);
    Eigen::Quaternionf quat_arrow_in_veh(
        Eigen::AngleAxisf(arrow_it.second->heading, Eigen::Vector3f::UnitZ()));
    Eigen::Quaternionf quat_arrow_in_local_enu =
        curr_pose->quat * quat_arrow_in_veh;
    Eigen::Vector3f euler_arrow =
        quat_arrow_in_local_enu.toRotationMatrix().eulerAngles(2, 0, 1);
    arrow->set_heading(static_cast<double>(euler_arrow[0]));
    arrow->set_type(FillArrowType(arrow_it.second->type));
  }
  return map;
}

hozon::hdmap::ArrowData_Type GroupMap::FillArrowType(em::ArrowType arrowtype) {
  switch (arrowtype) {
    case em::ArrowType::UNKNOWN_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_UNKNOWN_TURN;
    case em::ArrowType::STRAIGHT_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT;
    case em::ArrowType::RIGHT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_TURN;
    case em::ArrowType::LEFT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_TURN;
    case em::ArrowType::U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_U_TURN;
    case em::ArrowType::STRAIGHT_LEFT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_LEFT_TURN;
    case em::ArrowType::STRAIGHT_RIGHT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_RIGHT_TURN;
    case em::ArrowType::STRAIGHT_U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_U_TURN;
    case em::ArrowType::LEFT_U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_U_TURN;
    case em::ArrowType::LEFT_RIGHT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_RIGHT_TURN;
    case em::ArrowType::LEFT_FRONT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_LEFT_FRONT_TURN;
    case em::ArrowType::RIGHT_FRONT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_FRONT_TURN;
    case em::ArrowType::STRAIGHT_LEFT_RIGHT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::
          ArrowData_Type_STRAIGHT_LEFT_RIGHT_TURN;
    case em::ArrowType::STRAIGHT_LEFT_U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_STRAIGHT_LEFT_U_TURN;
    case em::ArrowType::RIGHT_U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_RIGHT_U_TURN;
    case em::ArrowType::FORBID_LEFT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_LEFT_TURN;
    case em::ArrowType::FORBID_RIGHT_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_RIGHT_TURN;
    case em::ArrowType::FORBID_U_TURN_ARROW:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_FORBID_U_TURN;
    default:
      return hozon::hdmap::ArrowData_Type::ArrowData_Type_FRONT_NEAR_CROSSWALK;
  }
  return hozon::hdmap::ArrowData_Type::ArrowData_Type_UNKNOWN_TURN;
}

// 获取内部Group
void GroupMap::GetGroups(std::vector<Group::Ptr>* groups) {
  if (groups == nullptr) {
    return;
  }
  groups->reserve(groups->size());
  for (const auto& grp : groups_) {
    groups->emplace_back(grp);
  }
}

// 导出为map proto
std::shared_ptr<hozon::hdmap::Map> GroupMap::Export(
    const em::ElementMap::Ptr& ele_map, HistoryId* history_id) {
  if (curr_pose_ == nullptr || groups_.empty()) {
    return nullptr;
  }
  auto map = ConvertToProtoMap(groups_, curr_pose_, ele_map, history_id);
  return map;
}

std::shared_ptr<hozon::mp::mf::em::ElementMapOut> GroupMap::AddElementMap(
    const em::ElementMap::Ptr& ele_map) {
  if (curr_pose_ == nullptr || groups_.empty()) {
    return nullptr;
  }
  auto ele_m = ConvertToElementMap(groups_, curr_pose_, ele_map);
  return ele_m;
}

void GroupMap::UpdatePathInCurrPose(const std::vector<KinePose::Ptr>& path,
                                    const KinePose& curr_pose) {
  path_in_curr_pose_.clear();
  for (const auto& p : path) {
    if (p == nullptr) {
      HLOG_ERROR << "found nullptr pose in path";
      continue;
    }
    Eigen::Isometry3f T_local_v1;
    T_local_v1.setIdentity();
    T_local_v1.rotate(p->quat);
    T_local_v1.pretranslate(p->pos);

    Eigen::Isometry3f T_local_v2;
    T_local_v2.setIdentity();
    T_local_v2.rotate(curr_pose.quat);
    T_local_v2.pretranslate(curr_pose.pos);

    Eigen::Isometry3f T_v2_v1;
    T_v2_v1.setIdentity();
    T_v2_v1 = T_local_v2.inverse() * T_local_v1;
    if (T_v2_v1.translation().x() < 0) {
      Pose pose;
      pose.stamp = p->stamp;
      pose.pos << T_v2_v1.translation().x(), T_v2_v1.translation().y(),
          T_v2_v1.translation().z();
      pose.quat = T_v2_v1.rotation();
      path_in_curr_pose_.emplace_back(pose);
    }
  }
  // 自车原点也加进来
  Pose pose;
  pose.stamp = curr_pose.stamp;
  pose.pos.setZero();
  pose.quat.setIdentity();
  path_in_curr_pose_.emplace_back(pose);
}

}  // namespace gm
}  // namespace mf
}  // namespace mp
}  // namespace hozon
