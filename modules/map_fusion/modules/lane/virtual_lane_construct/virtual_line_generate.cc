/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： virtual_line_generate.cc
 *   author     ： zhangzhike
 *   date       ： 2024.08
 ******************************************************************************/

#include "modules/map_fusion/modules/lane/virtual_lane_construct/virtual_line_generate.h"

#include <algorithm>
#include <cfloat>

#include "Eigen/src/Core/Matrix.h"
#include "base/element_base.h"
#include "base/group.h"
#include "base/processor.h"
#include "base/utils/log.h"

namespace hozon {
namespace mp {
namespace mf {
bool VirtualLineGen::Init(const LaneFusionProcessOption& conf) {
  conf_ = conf;
  HLOG_INFO << "VirtualLineGen construct init";
  return true;
}

LineSegment::Ptr VirtualLineGen::RoadEdgecvtLine(
    const RoadEdge::Ptr& road_edge) {
  if (road_edge == nullptr || road_edge->points.empty()) {
    return nullptr;
  }
  LineSegment line_road;
  for (const auto& pt : road_edge->points) {
    Point pt_tmp;
    pt_tmp.pt = pt;
    pt_tmp.type = VIRTUAL;
    line_road.pts.emplace_back(pt_tmp);
    line_road.id = 5000;
  }
  line_road.dist_to_path = road_edge->dist_to_path;
  LineSegment::Ptr line_virtual_ptr = std::make_shared<LineSegment>(line_road);
  return line_virtual_ptr;
}

bool VirtualLineGen::ConstructVirtualLine(std::vector<Group::Ptr>* groups) {
  if (groups->empty()) {
    HLOG_WARN << "NO GROUPS INPUT!";
    return true;
  }
  // ExtendLineToAlignSlice(groups);
  groups_lane_name_.resize(groups->size());
  for (int i = 0; i < groups->size(); ++i) {
    for (auto lane : groups->at(i)->lanes) {
      groups_lane_name_[i].insert(lane->str_id);
    }
  }

  for (auto& grp : *groups) {
    if (grp->group_state == Group::NORMAL && grp->road_edges.size() >= 2) {
      if (grp->road_edges[0]->points.empty() ||
          grp->road_edges[1]->points.empty()) {
        continue;
      }
      GroupVirtualLine grp_lines;
      grp_lines.lines.emplace_back(RoadEdgecvtLine(grp->road_edges[0]));
      // HLOG_ERROR << "RoadEdgecvtLine(grp->road_edges[0])->DIS = "
      //            << RoadEdgecvtLine(grp->road_edges[0])->dist_to_path;
      // if (grp->road_edges[0]->road_edge_type == RoadEdgeType::OCC) {
      //   grp_lines.lines_type.emplace_back(1);
      // } else if (grp->road_edges[0]->road_edge_type == RoadEdgeType::MODEL) {
      //   grp_lines.lines_type.emplace_back(2);
      // } else {
      //   grp_lines.lines_type.emplace_back(1);
      // }
      grp_lines.lines_type.emplace_back(2);
      for (const auto& line : grp->line_segments) {
        grp_lines.lines.emplace_back(line);
        grp_lines.lines_type.emplace_back(1);
        // HLOG_ERROR << "line->DIS = " << line->dist_to_path;
      }
      grp_lines.lines.emplace_back(RoadEdgecvtLine(grp->road_edges[1]));
      // HLOG_ERROR << "RoadEdgecvtLine(grp->road_edges[1])->DIS = "
      //            << RoadEdgecvtLine(grp->road_edges[1])->dist_to_path;
      // if (grp->road_edges[1]->road_edge_type == RoadEdgeType::OCC) {
      //   grp_lines.lines_type.emplace_back(1);
      // } else if (grp->road_edges[1]->road_edge_type == RoadEdgeType::MODEL) {
      //   grp_lines.lines_type.emplace_back(2);
      // } else {
      //   grp_lines.lines_type.emplace_back(1);
      // }
      grp_lines.lines_type.emplace_back(2);
      groups_lines_[grp->str_id] = grp_lines;
    }
  }

  for (int i = 0; i < groups->size(); ++i) {
    auto curr_grp = groups->at(i);
    Group::Ptr prev_grp = nullptr;
    Group::Ptr next_grp = nullptr;
    GroupVirtualLine curr_grp_lines = groups_lines_[curr_grp->str_id];
    GroupVirtualLine prev_grp_lines, next_grp_lines;
    Eigen::Vector3f start_po = curr_grp->start_slice.po;
    Eigen::Vector3f start_pr = curr_grp->start_slice.pr;
    Eigen::Vector3f end_po = curr_grp->end_slice.po;
    Eigen::Vector3f end_pr = curr_grp->end_slice.pr;
    if (i > 0) {
      if ((PointToVectorDist(start_po, start_pr,
                             groups->at(i - 1)->end_slice.po) < 1.0 ||
           PointToVectorDist(groups->at(i - 1)->end_slice.po,
                             groups->at(i - 1)->end_slice.pr,
                             curr_grp->start_slice.po) < 1.0) &&
          groups->at(i - 1)->group_state == Group::NORMAL) {
        prev_grp = groups->at(i - 1);
        prev_grp_lines = groups_lines_[prev_grp->str_id];
      }
    }
    if (i < groups->size() - 1) {
      if ((PointToVectorDist(end_po, end_pr,
                             groups->at(i + 1)->start_slice.po) < 1.0 ||
           PointToVectorDist(groups->at(i + 1)->start_slice.po,
                             groups->at(i + 1)->start_slice.pr,
                             curr_grp->end_slice.po) < 1.0) &&
          groups->at(i + 1)->group_state == Group::NORMAL) {
        next_grp = groups->at(i + 1);
        next_grp_lines = groups_lines_[next_grp->str_id];
      }
    }
    if (curr_grp->road_edges.size() < 2) {
      // HLOG_INFO << "curr_grp->road_edges.size() = "
      //            << curr_grp->road_edges.size();
      continue;
    }
    if (curr_grp->road_edges[0]->points.empty() ||
        curr_grp->road_edges[1]->points.empty()) {
      // HLOG_INFO << "  curr_grp->road_edges[0]->points.size() = "
      //            << curr_grp->road_edges[0]->points.size()
      //            << "  curr_grp->road_edges[1]->points.size() = "
      //            << curr_grp->road_edges[1]->points.size();
      continue;
    }
    // 道路宽度
    // float road_width =
    //     GetTwoBoundayDis(curr_grp->road_edges[0], curr_grp->road_edges[1]);
    float road_width =
        GetTwoBoundayDis(curr_grp_lines.lines[0], curr_grp_lines.lines.back());
    if (road_width < conf_.min_lane_width) {
      continue;
    }

    // . 计算车道宽度
    float lane_width = GetCurrGroupLaneWidth(curr_grp);
    if (lane_width < 0) {
      float prev_grp_lane_width =
          (prev_grp != nullptr) ? GetCurrGroupLaneWidth(prev_grp) : -1.0f;
      if (prev_grp_lane_width < 0) {
        float next_grp_lane_width =
            (next_grp != nullptr) ? GetCurrGroupLaneWidth(next_grp) : -1.0f;
        if (next_grp_lane_width < 0) {
          lane_width = averge_exit_lane_width_;
        } else {
          lane_width = next_grp_lane_width;
        }
      } else {
        lane_width = prev_grp_lane_width;
      }
    }

    //! TBD:车道宽度用前后当前group的lane减去最小和最大值，再求平均

    HLOG_INFO << "currgroup id is " << curr_grp->str_id
              << "   lane_width = " << lane_width;

    // 计算点在slice上的的投影
    std::vector<float> prev_grp_lines2start_slice;
    std::vector<float> next_grp_lines2end_slice;

    if (prev_grp != nullptr) {
      prev_grp_lines2start_slice = StartSlicePrevLineIntersecPointProjec(
          prev_grp, curr_grp);  // prev_grp->lines与curr_grp->start_slice的投影
      std::sort(prev_grp_lines2start_slice.begin(),
                prev_grp_lines2start_slice.end());  // left to right
    }
    HLOG_INFO << "prev_grp_lines2start_slice.size() = "
              << prev_grp_lines2start_slice.size();
    for (auto& start_line_dis : prev_grp_lines2start_slice) {
      HLOG_INFO << "start_line_dis = " << start_line_dis;
    }

    // if (next_grp != nullptr) {
    //   next_grp_lines2end_slice =
    //       EndSliceNextLineIntersecPointProjec(curr_grp, next_grp);
    //   std::sort(next_grp_lines2end_slice.begin(),
    //             next_grp_lines2end_slice.end());
    // }
    // HLOG_ERROR << "next_grp_lines2end_slice.size() = "
    //            << next_grp_lines2end_slice.size();
    // for (auto& end_line_dis : next_grp_lines2end_slice) {
    //   HLOG_ERROR << " end_line_dis = " << end_line_dis;
    // }
    std::vector<float> start_between_lr;  // 相邻两根线的夹着的交点们
    std::vector<LineSegment::Ptr> line_virtual_vec;  // 构建虚拟车道线
    for (int line_right = 1; line_right < curr_grp_lines.lines.size();
         ++line_right) {
      auto left_line = curr_grp_lines.lines[line_right - 1];
      auto right_line = curr_grp_lines.lines[line_right];
      if (left_line->dist_to_path < right_line->dist_to_path) {
        continue;
      }
      int left_line_type = curr_grp_lines.lines_type[line_right - 1];
      int right_line_type = curr_grp_lines.lines_type[line_right];
      // 构成道的两边线不进行线排查
      std::string id =
          std::to_string(left_line->id) + "_" + std::to_string(right_line->id);
      if (groups_lane_name_[i].find(id) != groups_lane_name_[i].end()) {
        continue;
      }
      std::vector<float> line_dis =
          GetTwoBoundayFrontBackDis(left_line, right_line);
      // HLOG_INFO << "curr group id is " << curr_grp->str_id << "   left_line
      // is "
      //           << left_line->id << "  right_line is " << right_line->id
      //           << "  line_dis is " << line_dis[0] << " " << line_dis[1];
      start_between_lr.clear();
      NeighborLineStSlP(start_po, start_pr, left_line->pts[0].pt,
                        right_line->pts[0].pt, prev_grp_lines2start_slice,
                        &start_between_lr);
      if (abs(line_dis[0] - line_dis[1]) < conf_.max_lane_width) {
        if (line_dis[0] > conf_.min_lane_width &&
            line_dis[1] > conf_.min_lane_width) {
          if (start_between_lr.size() < 2) {
            SameLineNumVirtualBuild(curr_grp, &line_virtual_vec, line_dis,
                                    left_line, right_line, lane_width,
                                    left_line_type, right_line_type);
          } else {
            SlicePointVirtualLine(curr_grp, &line_virtual_vec, start_between_lr,
                                  line_dis, left_line, right_line, lane_width,
                                  left_line_type, right_line_type);
          }
        }
      }
    }
    // 同一个group一起添加virtual_line_segment
    for (const auto& line_vir : line_virtual_vec) {
      curr_grp->line_segments.emplace_back(line_vir);
    }
    std::sort(curr_grp->line_segments.begin(), curr_grp->line_segments.end(),
              [](const LineSegment::Ptr& a, const LineSegment::Ptr& b) {
                return a->dist_to_path > b->dist_to_path;
              });  // 重新排序
    // for (auto& line : curr_grp->line_segments) {
    //   HLOG_INFO << "line->id = " << line->id
    //             << " line->dist_to_path =  " << line->dist_to_path;
    // }
    GenGroupAllLanes(curr_grp);  // 重新生成车道

#if 0
      // 1. 0 line_segment (2 road_edge)
      // 2. 1 line_segment (>=1 road_edge)
      // 3. >=2 line_segment (>=0 road_edge)
      if (curr_grp->line_segments.empty()) {
        int assume_nums = static_cast<int>(std::max(
            static_cast<float>(std::floor(road_width / lane_width +
            0.3)), 1.0F));
        std::vector<LineSegment::Ptr> line_virtual_vec;  // 虚拟车道线
        std::vector<float> road_edge_dis = GetTwoBoundayFrontBackDis(
            curr_grp->road_edges[0], curr_grp->road_edges[1]);
        std::vector<float> start_between_lr;
        std::vector<float> end_between_lr;
        NeighborLineStSlP(start_po, start_pr, end_po, end_pr,
                          curr_grp->road_edges[0]->points[0],
                          curr_grp->road_edges[1]->points[0],
                          curr_grp->road_edges[0]->points.back(),
                          curr_grp->road_edges[1]->points.back(),
                          prev_grp_lines2start_slice,
                          next_grp_lines2end_slice, &start_between_lr,
                          &end_between_lr);
        if (abs(road_edge_dis[0] - road_edge_dis[1]) < conf_.min_lane_width) {
          if (road_edge_dis[0] > conf_.min_lane_width &&
              road_edge_dis[1] > conf_.min_lane_width) {
            if (start_between_lr.empty() && end_between_lr.empty()) {
              SameLineNumVirtualBuild(curr_grp, &line_virtual_vec,
              road_edge_dis,
                                      curr_grp->road_edges[0],
                                      curr_grp->road_edges[1], lane_width);
            } else {
              SlicePointVirtualLine(curr_grp, &line_virtual_vec,
              start_between_lr,
                                    end_between_lr, road_edge_dis,
                                    curr_grp->road_edges[0],
                                    curr_grp->road_edges[1], lane_width);
            }
          }
        }
        for (const auto& line_vir : line_virtual_vec) {
          curr_grp->line_segments.emplace_back(line_vir);
        }
        // 添加相应的虚拟车道线
      } else if (curr_grp->line_segments.size() == 1) {
        std::vector<LineSegment::Ptr> line_virtual_vec;  // 虚拟车道线

        // 1. road_edge_left between line dis
        std::vector<float> road_edge_left_between_line_dis =
            GetRoadedgeLineFrontEndDis(curr_grp->road_edges[0],
                                       curr_grp->line_segments[0]);
        HLOG_ERROR << "road_edge_left_between_line_dis[0] = "
                   << road_edge_left_between_line_dis[0]
                   << " road_edge_left_between_line_dis[1] = "
                   << road_edge_left_between_line_dis[1];
        std::vector<float> start_between_lr;
        std::vector<float> end_between_lr;
        NeighborLineStSlP(start_po, start_pr, end_po, end_pr,
                          curr_grp->road_edges[0]->points[0],
                          curr_grp->line_segments[0]->pts[0].pt,
                          curr_grp->road_edges[0]->points.back(),
                          curr_grp->line_segments[0]->pts.back().pt,
                          prev_grp_lines2start_slice,
                          next_grp_lines2end_slice, &start_between_lr,
                          &end_between_lr);
        // if (back_distance - front_distance)>lane_width
        // find turn point
        if (abs(road_edge_left_between_line_dis[0] -
                road_edge_left_between_line_dis[1]) < conf_.min_lane_width) {
          if (road_edge_left_between_line_dis[0] > conf_.min_lane_width &&
              road_edge_left_between_line_dis[1] > conf_.min_lane_width) {
            if (start_between_lr.empty() && end_between_lr.empty()) {
              SameLineNumVirtualBuild(curr_grp, &line_virtual_vec,
                                      road_edge_left_between_line_dis,
                                      curr_grp->road_edges[0],
                                      curr_grp->line_segments[0],
                                      lane_width);
            } else {
              SlicePointVirtualLine(
                  curr_grp, &line_virtual_vec, start_between_lr,
                  end_between_lr, road_edge_left_between_line_dis,
                  curr_grp->road_edges[0], curr_grp->line_segments[0],
                  lane_width, 1);
            }
          }
        } else if (road_edge_left_between_line_dis[0] >
                   road_edge_left_between_line_dis[1]) {
        } else {
        }

        // 2. line between road_edge_right dis
        std::vector<float> line_between_road_edge_right =
            GetRoadedgeLineFrontEndDis(curr_grp->road_edges[1],
                                       curr_grp->line_segments[0]);
        HLOG_ERROR << "line_between_road_edge_right[0] = "
                   << line_between_road_edge_right[0]
                   << " line_between_road_edge_right[1] = "
                   << line_between_road_edge_right[1];
        start_between_lr.clear();
        end_between_lr.clear();
        NeighborLineStSlP(start_po, start_pr, end_po, end_pr,
                          curr_grp->line_segments[0]->pts[0].pt,
                          curr_grp->road_edges[1]->points[0],
                          curr_grp->line_segments[0]->pts.back().pt,
                          curr_grp->road_edges[1]->points.back(),
                          prev_grp_lines2start_slice,
                          next_grp_lines2end_slice, &start_between_lr,
                          &end_between_lr);
        if (abs(line_between_road_edge_right[0] -
                line_between_road_edge_right[1]) < conf_.min_lane_width) {
          if (line_between_road_edge_right[0] > conf_.min_lane_width &&
              line_between_road_edge_right[1] > conf_.min_lane_width) {
            if (start_between_lr.empty() && end_between_lr.empty()) {
              SameLineNumVirtualBuild(curr_grp, &line_virtual_vec,
                                      line_between_road_edge_right,
                                      curr_grp->road_edges[1],
                                      curr_grp->line_segments[0],
                                      lane_width);
            } else {
              SlicePointVirtualLine(
                  curr_grp, &line_virtual_vec, start_between_lr,
                  end_between_lr, road_edge_left_between_line_dis,
                  curr_grp->road_edges[1], curr_grp->line_segments[0],
                  lane_width, 0);
            }
          }
        }

        for (const auto& line_vir : line_virtual_vec) {
          curr_grp->line_segments.emplace_back(line_vir);
        }
      } else {
        std::vector<LineSegment::Ptr> line_virtual_vec;  // 虚拟车道线
        // 1. road_edge_left between left_line dis
        std::vector<float> road_edge_left_between_left_line_dis =
            GetRoadedgeLineFrontEndDis(curr_grp->road_edges[0],
                                       curr_grp->line_segments[0]);
        HLOG_ERROR << "road_edge_left_between_left_line_dis[0] = "
                   << road_edge_left_between_left_line_dis[0]
                   << " road_edge_left_between_left_line_dis[1] = "
                   << road_edge_left_between_left_line_dis[1];
        std::vector<float> start_between_lr;
        std::vector<float> end_between_lr;
        NeighborLineStSlP(start_po, start_pr, end_po, end_pr,
                          curr_grp->road_edges[0]->points[0],
                          curr_grp->line_segments[0]->pts[0].pt,
                          curr_grp->road_edges[0]->points.back(),
                          curr_grp->line_segments[0]->pts.back().pt,
                          prev_grp_lines2start_slice,
                          next_grp_lines2end_slice, &start_between_lr,
                          &end_between_lr);
        if (abs(road_edge_left_between_left_line_dis[0] -
                road_edge_left_between_left_line_dis[1]) <
                conf_.min_lane_width) {
          if (road_edge_left_between_left_line_dis[0] > conf_.min_lane_width
          &&
              road_edge_left_between_left_line_dis[1] > conf_.min_lane_width) {
            if (start_between_lr.empty() && end_between_lr.empty()) {
              SameLineNumVirtualBuild(curr_grp, &line_virtual_vec,
                                      road_edge_left_between_left_line_dis,
                                      curr_grp->road_edges[0],
                                      curr_grp->line_segments[0],
                                      lane_width);
            } else {
              SlicePointVirtualLine(
                  curr_grp, &line_virtual_vec, start_between_lr,
                  end_between_lr, road_edge_left_between_left_line_dis,
                  curr_grp->road_edges[0], curr_grp->line_segments[0],
                  lane_width, 1);
            }
          }
        }
        //
        // 2. lines dis
        for (int line_seg_size = 0;
             line_seg_size < curr_grp->line_segments.size() - 1;
             ++line_seg_size) {
          auto left_line = curr_grp->line_segments[line_seg_size];
          auto right_line = curr_grp->line_segments[line_seg_size + 1];
          // 构成道的两边线不进行不线排查
          std::string id = std::to_string(left_line->id) + "_" +
                           std::to_string(right_line->id);
          if (groups_lane_name_[i].find(id) != groups_lane_name_[i].end()) {
            continue;
          }
          std::vector<float> line_dis =
              GetTwoBoundayFrontBackDis(left_line, right_line);
          HLOG_ERROR << "curr group id is " << curr_grp->str_id
                     << "   left_line is " << left_line->id << "  right_line is"
                     << right_line->id << "  line_dis is " << line_dis[0] <<
                     " " << line_dis[1];
          start_between_lr.clear();
          end_between_lr.clear();
          NeighborLineStSlP(start_po, start_pr, end_po, end_pr,
                            left_line->pts[0].pt, right_line->pts[0].pt,
                            left_line->pts.back().pt,
                            right_line->pts.back().pt,
                            prev_grp_lines2start_slice,
                            next_grp_lines2end_slice, &start_between_lr,
                            &end_between_lr);
          if (abs(line_dis[0] - line_dis[1]) < conf_.min_lane_width) {
            if (line_dis[0] > conf_.min_lane_width &&
                line_dis[1] > conf_.min_lane_width) {
              if (start_between_lr.empty() && end_between_lr.empty()) {
                SameLineNumVirtualBuild(curr_grp, &line_virtual_vec,
                line_dis,
                                        left_line, right_line, lane_width);
              } else {
                SlicePointVirtualLine(curr_grp, &line_virtual_vec,
                                      start_between_lr, end_between_lr,
                                      line_dis, left_line, right_line,
                                      lane_width);
              }
            }
          }
        }

        // 3. right_line between road_edge_right dis
        std::vector<float> road_edge_right_between_right_line_dis =
            GetRoadedgeLineFrontEndDis(curr_grp->road_edges.back(),
                                       curr_grp->line_segments.back());
        HLOG_ERROR << "road_edge_right_between_right_line_dis[0] = "
                   << road_edge_right_between_right_line_dis[0]
                   << " road_edge_right_between_right_line_dis[1] = "
                   << road_edge_right_between_right_line_dis[1];
        start_between_lr.clear();
        end_between_lr.clear();
        NeighborLineStSlP(start_po, start_pr, end_po, end_pr,
                          curr_grp->line_segments.back()->pts[0].pt,
                          curr_grp->road_edges[1]->points[0],
                          curr_grp->line_segments.back()->pts.back().pt,
                          curr_grp->road_edges[1]->points.back(),
                          prev_grp_lines2start_slice,
                          next_grp_lines2end_slice, &start_between_lr,
                          &end_between_lr);
        if (abs(road_edge_right_between_right_line_dis[0] -
                road_edge_right_between_right_line_dis[1]) <
            conf_.min_lane_width) {
          if (road_edge_right_between_right_line_dis[0] >
          conf_.min_lane_width &&
              road_edge_right_between_right_line_dis[1] >
              conf_.min_lane_width) {
            if (start_between_lr.empty() && end_between_lr.empty()) {
              SameLineNumVirtualBuild(curr_grp, &line_virtual_vec,
                                      road_edge_right_between_right_line_dis,
                                      curr_grp->road_edges.back(),
                                      curr_grp->line_segments.back(),
                                      lane_width);
            } else {
              SlicePointVirtualLine(
                  curr_grp, &line_virtual_vec, start_between_lr,
                  end_between_lr, road_edge_right_between_right_line_dis,
                  curr_grp->road_edges[1], curr_grp->line_segments[0],
                  lane_width, 0);
            }
          }
        }
        // 同一个group一起添加virtual_line_segment
        for (const auto& line_vir : line_virtual_vec) {
          curr_grp->line_segments.emplace_back(line_vir);
        }
      }
#endif
  }

  return true;
}

float VirtualLineGen::Pt2BaselineDis(const RoadEdge::Ptr& base_line,
                                     const Eigen::Vector3f& vec_popr,
                                     const Eigen::Vector3f& po, float length) {
  float last_dis = 0.0;
  Eigen::Vector3f pt1 = vec_popr * length + po;
  auto it = std::min_element(
      base_line->points.begin(), base_line->points.end(),
      [&pt1](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
        return (pt1 - a).norm() < (pt1 - b).norm();
      });
  if (base_line->points.size() == 1) {
    last_dis = ((*it) - pt1).norm();
  } else if (it == std::prev(base_line->points.end())) {
    last_dis = perpendicular_distance((*std::prev(it)), (*it), pt1);
  } else {
    last_dis = perpendicular_distance((*std::next(it)), (*it), pt1);
  }
  return last_dis;
}
float VirtualLineGen::Pt2BaselineDis(const LineSegment::Ptr& base_line,
                                     const Eigen::Vector3f& vec_popr,
                                     const Eigen::Vector3f& po, float length) {
  float last_dis = 0.0;
  Eigen::Vector3f pt1 = vec_popr * length + po;
  auto it = std::min_element(base_line->pts.begin(), base_line->pts.end(),
                             [&pt1](const Point& a, const Point& b) {
                               return (pt1 - a.pt).norm() < (pt1 - b.pt).norm();
                             });
  if (base_line->pts.size() == 1) {
    last_dis = ((*it).pt - pt1).norm();
  } else if (it == std::prev(base_line->pts.end())) {
    last_dis = perpendicular_distance((*std::prev(it)).pt, (*it).pt, pt1);
  } else {
    last_dis = perpendicular_distance((*std::next(it)).pt, (*it).pt, pt1);
  }
  return last_dis;
}
void VirtualLineGen::NeighborLineStSlP(
    const Eigen::Vector3f& start_po, const Eigen::Vector3f& start_pr,
    const Eigen::Vector3f& left_line_front_pt,
    const Eigen::Vector3f& right_line_front_pt,
    const std::vector<float>& start_grp_lines2slice,
    std::vector<float>* start_between_lr) {
  start_between_lr->clear();
  float start_left = -FLT_MAX;
  if (PointToVectorDist(start_po, start_pr, left_line_front_pt) < 5) {
    start_left = ProjectionToVector(start_po, start_pr, left_line_front_pt);
    HLOG_ERROR << "start_left = " << start_left;
  }
  float start_right = FLT_MAX;
  if (PointToVectorDist(start_po, start_pr, right_line_front_pt) < 5) {
    start_right = ProjectionToVector(start_po, start_pr, right_line_front_pt);
    HLOG_ERROR << "start_right = " << start_right;
  }
  std::copy_if(start_grp_lines2slice.begin(), start_grp_lines2slice.end(),
               std::back_inserter(*start_between_lr),
               [start_left, start_right](float x) {
                 if (start_left > -FLT_MAX && start_right < FLT_MAX) {
                   return x >= (start_left - 0.1) && x <= (start_right + 0.1);
                 } else if (start_left > -FLT_MAX) {
                   return x >= (start_left - 0.1);
                 }
                 return x <= (start_right + 0.1);
               });
  if (start_left > -FLT_MAX && !(*start_between_lr).empty() &&
      (start_between_lr->front() - start_left) > conf_.min_lane_width) {
    start_between_lr->insert(start_between_lr->begin(), start_left);
  }
  if (start_right < FLT_MAX && !(*start_between_lr).empty() &&
      (start_right - (*start_between_lr).back()) > conf_.min_lane_width) {
    start_between_lr->emplace_back(start_right);
  }
}
void VirtualLineGen::NeighborLineStSlP(
    const Eigen::Vector3f& start_po, const Eigen::Vector3f& start_pr,
    const Eigen::Vector3f& end_po, const Eigen::Vector3f& end_pr,
    const Eigen::Vector3f& left_line_front_pt,
    const Eigen::Vector3f& right_line_front_pt,
    const Eigen::Vector3f& left_line_back_pt,
    const Eigen::Vector3f& right_line_back_pt,
    const std::vector<float>& start_grp_lines2slice,
    const std::vector<float>& end_grp_lines2slice,
    std::vector<float>* start_between_lr, std::vector<float>* end_between_lr) {
  start_between_lr->clear();
  end_between_lr->clear();
  float start_left = -FLT_MAX;
  if (PointToVectorDist(start_po, start_pr, left_line_front_pt) < 5) {
    start_left = ProjectionToVector(start_po, start_pr, left_line_front_pt);
  }
  float start_right = FLT_MAX;
  if (PointToVectorDist(start_po, start_pr, right_line_front_pt) < 5) {
    start_right = ProjectionToVector(start_po, start_pr, right_line_front_pt);
  }
  std::copy_if(start_grp_lines2slice.begin(), start_grp_lines2slice.end(),
               std::back_inserter(*start_between_lr),
               [start_left, start_right](float x) {
                 return x > start_left && x < start_right;
               });
  if (start_left > -FLT_MAX && !start_grp_lines2slice.empty() &&
      (start_grp_lines2slice[0] - start_left) > conf_.min_lane_width) {
    start_between_lr->insert(start_between_lr->begin(), start_left);
  }
  if (start_right < FLT_MAX && !start_grp_lines2slice.empty() &&
      (start_right - start_grp_lines2slice.back()) > conf_.min_lane_width) {
    start_between_lr->emplace_back(start_right);
  }

  float end_left = -FLT_MAX;
  if (PointToVectorDist(end_po, end_pr, left_line_back_pt) < 5) {
    end_left = ProjectionToVector(end_po, end_pr, left_line_back_pt);
  }
  float end_right = FLT_MAX;
  if (PointToVectorDist(end_po, end_pr, right_line_back_pt) < 5) {
    end_right = ProjectionToVector(end_po, end_pr, right_line_back_pt);
  }
  std::copy_if(
      end_grp_lines2slice.begin(), end_grp_lines2slice.end(),
      std::back_inserter(*end_between_lr),
      [end_left, end_right](float x) { return x > end_left && x < end_right; });

  if (end_left > -FLT_MAX && !end_grp_lines2slice.empty() &&
      (end_grp_lines2slice[0] - end_left) > conf_.min_lane_width) {
    end_between_lr->insert(end_between_lr->begin(), end_left);
  }
  if (end_right < FLT_MAX && !end_grp_lines2slice.empty() &&
      (end_right - end_grp_lines2slice.back()) > conf_.min_lane_width) {
    end_between_lr->emplace_back(end_right);
  }
}
void VirtualLineGen::SlicePointVirtualLine(
    const Group::Ptr& curr_group,
    std::vector<LineSegment::Ptr>* line_virtual_vec,
    const std::vector<float>& start_between_lr,
    const std::vector<float>& end_between_lr,
    const std::vector<float>& line_dis, const RoadEdge::Ptr& road,
    const LineSegment::Ptr& line, float lane_width, int is_line_left) {
  if (line_dis.size() < 2 || road == nullptr || line == nullptr ||
      (start_between_lr.empty() && end_between_lr.empty()) ||
      road->points.empty() || line->pts.size() < 2) {
    return;
  }
  HLOG_ERROR << "start_between_lr.size() = " << start_between_lr.size();
  for (auto& start_lr : start_between_lr) {
    HLOG_ERROR << "start_lr = " << start_lr;
  }
  HLOG_ERROR << "end_between_lr.size() = " << end_between_lr.size();
  for (auto& end_lr : end_between_lr) {
    HLOG_ERROR << "end_lr = " << end_lr;
  }
  auto start_po = curr_group->start_slice.po;
  auto start_pr = curr_group->start_slice.pr;
  auto end_po = curr_group->end_slice.po;
  auto end_pr = curr_group->end_slice.pr;
  Eigen::Vector3f vec_popr = (start_pr - start_po).normalized();
  Eigen::Vector3f vec_popr_end = (end_pr - end_po).normalized();

  std::vector<float> startslicept_bsl_dis;
  std::vector<Eigen::Vector3f> startslicept;
  float last_dis = 100;
  for (int i = 1; i < start_between_lr.size(); ++i) {
    float p1 = start_between_lr[i - 1];
    float p2 = start_between_lr[i];
    if (last_dis > 99) {
      last_dis = Pt2BaselineDis(line, vec_popr, start_po, p1);
      startslicept_bsl_dis.emplace_back(last_dis);
      startslicept.emplace_back(start_po + vec_popr * p1);
    }
    float p2_dis = Pt2BaselineDis(line, vec_popr, start_po, p2);
    float line_width = abs(last_dis - p2_dis);
    if (line_width > conf_.max_lane_width) {
      int assume_nums = static_cast<int>(std::max(
          static_cast<float>(std::floor(line_width / lane_width + 0.3)), 1.0F));
      float l_width = (p2_dis - last_dis) / static_cast<float>(assume_nums);
      for (int assume_num = 1; assume_num < assume_nums; assume_num++) {
        startslicept_bsl_dis.emplace_back(
            last_dis + l_width * static_cast<float>(assume_num));
        startslicept.emplace_back(
            start_po +
            vec_popr * (p1 + (p2 - p1) / static_cast<float>(assume_nums) *
                                 static_cast<float>(assume_num)));
      }
    }
    startslicept_bsl_dis.emplace_back(p2_dis);
    startslicept.emplace_back(start_po + vec_popr * p2);
    last_dis = p2_dis;
  }
  if (start_between_lr.size() == 1) {
    last_dis = Pt2BaselineDis(line, vec_popr, start_po, start_between_lr[0]);
    startslicept_bsl_dis.emplace_back(last_dis);
    startslicept.emplace_back(start_po + vec_popr * start_between_lr[0]);
  }

  HLOG_ERROR << "startslicept_bsl_dis.size() = " << startslicept_bsl_dis.size();
  for (auto& dis : startslicept_bsl_dis) {
    HLOG_ERROR << "startslicept_bsl_dis  dis = " << dis;
  }
  std::vector<float> endslicept_bsl_dis;
  std::vector<Eigen::Vector3f> endslicept;
  last_dis = 100;
  for (int i = 1; i < end_between_lr.size(); ++i) {
    float p1 = end_between_lr[i - 1];
    float p2 = end_between_lr[i];
    if (last_dis > 99) {
      last_dis = Pt2BaselineDis(line, vec_popr_end, end_po, p1);
      endslicept_bsl_dis.emplace_back(last_dis);
      endslicept.emplace_back(end_po + vec_popr_end * p1);
    }

    float p2_dis = Pt2BaselineDis(line, vec_popr_end, end_po, p2);
    float line_width = abs(last_dis - p2_dis);
    if (line_width > conf_.max_lane_width) {
      int assume_nums = static_cast<int>(std::max(
          static_cast<float>(std::floor(line_width / lane_width + 0.3)), 1.0F));
      float l_width = (p2_dis - last_dis) / static_cast<float>(assume_nums);
      for (int assume_num = 1; assume_num < assume_nums; assume_num++) {
        endslicept_bsl_dis.emplace_back(
            last_dis + l_width * static_cast<float>(assume_num));
        endslicept.emplace_back(
            end_po +
            vec_popr_end * (p1 + (p2 - p1) / static_cast<float>(assume_nums) *
                                     static_cast<float>(assume_num)));
      }
    }
    endslicept_bsl_dis.emplace_back(p2_dis);
    endslicept.emplace_back(end_po + vec_popr_end * p2);
    last_dis = p2_dis;
  }
  if (end_between_lr.size() == 1) {
    last_dis = Pt2BaselineDis(line, vec_popr_end, end_po, end_between_lr[0]);
    endslicept_bsl_dis.emplace_back(last_dis);
    endslicept.emplace_back(end_po + vec_popr_end * end_between_lr[0]);
  }

  HLOG_ERROR << "endslicept_bsl_dis.size() = " << endslicept_bsl_dis.size();
  for (auto& dis : endslicept_bsl_dis) {
    HLOG_ERROR << "endslicept_bsl_dis  dis = " << dis;
  }

  if (is_line_left == 0) {
    float left_start_last_dis = 0.0;
    float left_end_last_dis = 0.0;
    int left_start_last_index = 0;
    int left_end_last_index = 0;
    while (left_start_last_index < startslicept_bsl_dis.size() &&
           left_end_last_index < endslicept_bsl_dis.size()) {
      while (left_start_last_index < startslicept_bsl_dis.size() &&
             startslicept_bsl_dis[left_start_last_index] - left_start_last_dis <
                 conf_.min_lane_width) {
        left_start_last_index++;
      }
      while (left_end_last_index < endslicept_bsl_dis.size() &&
             endslicept_bsl_dis[left_end_last_index] - left_end_last_dis <
                 conf_.min_lane_width) {
        left_end_last_index++;
      }
      if (left_start_last_index < startslicept_bsl_dis.size() &&
          left_end_last_index < endslicept_bsl_dis.size()) {
        float start_lane_width =
            startslicept_bsl_dis[left_start_last_index] - left_start_last_dis;
        float end_lane_width =
            endslicept_bsl_dis[left_end_last_index] - left_end_last_dis;
        HLOG_ERROR << "start_lane_width = " << start_lane_width
                   << "  end_lane_width = " << end_lane_width;
        if (abs(start_lane_width - end_lane_width) > conf_.min_lane_width) {
          LineSegment line_virtual;
          Point tmp;
          tmp.type = VIRTUAL;
          tmp.pt = endslicept[left_end_last_index];
          TranslateLineBack(&line_virtual, tmp, line, curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);

          LineSegment line_virtual2;
          tmp.type = VIRTUAL;
          tmp.pt = startslicept[left_start_last_index];
          TranslateLine(&line_virtual2, tmp, line, curr_group);
          LineSegment::Ptr line_virtual_ptr2 =
              std::make_shared<LineSegment>(line_virtual2);
          (*line_virtual_vec).emplace_back(line_virtual_ptr2);
          left_start_last_dis =
              std::max(startslicept_bsl_dis[left_start_last_index],
                       endslicept_bsl_dis[left_end_last_index]);
          left_end_last_dis = left_start_last_dis;
        } else {
          LineSegment line_virtual;
          Point start_pt, end_pt;
          start_pt.type = VIRTUAL;
          start_pt.pt = startslicept[left_start_last_index];
          end_pt.type = VIRTUAL;
          end_pt.pt = endslicept[left_end_last_index];

          TranslateAndRotateLine(&line_virtual, start_pt, end_pt, line,
                                 curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);
          left_start_last_dis = startslicept_bsl_dis[left_start_last_index];
          left_end_last_dis = endslicept_bsl_dis[left_end_last_index];
        }
      }
    }
    while (left_start_last_index < startslicept_bsl_dis.size()) {
      while (left_start_last_index < startslicept_bsl_dis.size() &&
             startslicept_bsl_dis[left_start_last_index] - left_start_last_dis <
                 conf_.min_lane_width) {
        left_start_last_index++;
      }
      if (left_start_last_index < startslicept_bsl_dis.size()) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = startslicept[left_start_last_index];
        TranslateLine(&line_virtual, tmp, line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      left_start_last_dis = startslicept_bsl_dis[left_start_last_index];
    }
    while (left_end_last_index < endslicept_bsl_dis.size()) {
      while (left_end_last_index < endslicept_bsl_dis.size() &&
             endslicept_bsl_dis[left_end_last_index] - left_end_last_dis <
                 conf_.min_lane_width) {
        left_end_last_index++;
      }
      if (left_end_last_index < endslicept_bsl_dis.size()) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = endslicept[left_end_last_index];
        TranslateLineBack(&line_virtual, tmp, line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      left_end_last_dis = endslicept_bsl_dis[left_end_last_index];
    }
  } else {
    float right_start_last_dis = 0.0;
    float right_end_last_dis = 0.0;
    int right_start_last_index =
        static_cast<int>(startslicept_bsl_dis.size()) - 1;
    int right_end_last_index = static_cast<int>(endslicept_bsl_dis.size()) - 1;
    while (right_start_last_index >= 0 && right_end_last_index >= 0) {
      while (right_start_last_index >= 0 &&
             startslicept_bsl_dis[right_start_last_index] -
                     right_start_last_dis <
                 conf_.min_lane_width) {
        right_start_last_index--;
      }
      while (right_end_last_index >= 0 &&
             endslicept_bsl_dis[right_end_last_index] - right_end_last_dis <
                 conf_.min_lane_width) {
        right_end_last_index--;
      }
      if (right_start_last_index >= 0 && right_end_last_index >= 0) {
        float start_lane_width =
            startslicept_bsl_dis[right_start_last_index] - right_start_last_dis;
        float end_lane_width =
            endslicept_bsl_dis[right_end_last_index] - right_end_last_dis;
        if (abs(start_lane_width - end_lane_width) > conf_.min_lane_width) {
          LineSegment line_virtual;
          Point tmp;
          tmp.type = VIRTUAL;
          tmp.pt = endslicept[right_end_last_index];
          TranslateLineBack(&line_virtual, tmp, line, curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);

          LineSegment line_virtual2;
          tmp.type = VIRTUAL;
          tmp.pt = startslicept[right_start_last_index];
          TranslateLine(&line_virtual2, tmp, line, curr_group);
          LineSegment::Ptr line_virtual_ptr2 =
              std::make_shared<LineSegment>(line_virtual2);
          (*line_virtual_vec).emplace_back(line_virtual_ptr2);
          right_start_last_dis =
              std::max(startslicept_bsl_dis[right_start_last_index],
                       endslicept_bsl_dis[right_end_last_index]);
          right_end_last_dis = right_start_last_dis;
        } else {
          LineSegment line_virtual;
          Point start_pt, end_pt;
          start_pt.type = VIRTUAL;
          start_pt.pt = startslicept[right_start_last_index];
          end_pt.type = VIRTUAL;
          end_pt.pt = endslicept[right_end_last_index];

          TranslateAndRotateLine(&line_virtual, start_pt, end_pt, line,
                                 curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);
          right_start_last_dis = startslicept_bsl_dis[right_start_last_index];
          right_end_last_dis = endslicept_bsl_dis[right_end_last_index];
        }
      }
    }
    while (right_start_last_index >= 0) {
      while (right_start_last_index >= 0 &&
             startslicept_bsl_dis[right_start_last_index] -
                     right_start_last_dis <
                 conf_.min_lane_width) {
        right_start_last_index--;
      }
      if (right_start_last_index >= 0) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = startslicept[right_start_last_index];
        TranslateLine(&line_virtual, tmp, line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      right_start_last_dis = startslicept_bsl_dis[right_start_last_index];
    }
    while (right_end_last_index >= 0) {
      while (right_end_last_index >= 0 &&
             endslicept_bsl_dis[right_end_last_index] - right_end_last_dis <
                 conf_.min_lane_width) {
        right_end_last_index--;
      }
      if (right_end_last_index >= 0) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = endslicept[right_end_last_index];
        TranslateLineBack(&line_virtual, tmp, line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      right_end_last_dis = endslicept_bsl_dis[right_end_last_index];
    }
  }
}
void VirtualLineGen::SlicePointVirtualLine(
    const Group::Ptr& curr_group,
    std::vector<LineSegment::Ptr>* line_virtual_vec,
    const std::vector<float>& start_between_lr,
    const std::vector<float>& line_dis, const LineSegment::Ptr& left_line,
    const LineSegment::Ptr& right_line, float lane_width, int left_type,
    int right_type) {
  if (line_dis.size() < 2 || left_line == nullptr || right_line == nullptr ||
      start_between_lr.empty() || left_line->pts.empty() ||
      right_line->pts.empty()) {
    return;
  }
  int base_line_flag = 0;
  auto base_line = left_line;
  if (left_type == right_type) {
    if (left_line->pts.size() < 2 ||
        (right_line->pts.size() > 1 &&
         Dist(right_line->pts[0].pt, right_line->pts.back().pt) >
             Dist(left_line->pts[0].pt, left_line->pts.back().pt))) {
      base_line = right_line;
      base_line_flag = 1;
    }
  } else if (right_type == 1) {
    base_line = right_line;
    base_line_flag = 1;
  }
  // HLOG_ERROR << "start_between_lr.size() = " << start_between_lr.size();
  // for (auto& start_lr : start_between_lr) {
  //   HLOG_ERROR << "start_lr = " << start_lr;
  // }
  auto start_po = curr_group->start_slice.po;
  auto start_pr = curr_group->start_slice.pr;

  Eigen::Vector3f vec_popr = (start_pr - start_po).normalized();

  float start_left = -FLT_MAX;
  if (PointToVectorDist(start_po, start_pr, left_line->pts.front().pt) < 5) {
    start_left =
        ProjectionToVector(start_po, start_pr, left_line->pts.front().pt);
    start_left = Pt2BaselineDis(base_line, vec_popr, start_po, start_left);
    // HLOG_ERROR << "start_left = " << start_left;
  }
  float start_right = FLT_MAX;
  if (PointToVectorDist(start_po, start_pr, right_line->pts.front().pt) < 5) {
    start_right =
        ProjectionToVector(start_po, start_pr, right_line->pts.front().pt);
    start_right = Pt2BaselineDis(base_line, vec_popr, start_po, start_right);
    // HLOG_ERROR << "start_right = " << start_right;
  }
  std::vector<float> startslicept_bsl_dis;
  std::vector<Eigen::Vector3f> startslicept;
  HLOG_ERROR << "lane_width = " << lane_width;
  float last_dis = 100;
  for (int i = 1; i < start_between_lr.size(); ++i) {
    float p1 = start_between_lr[i - 1];
    float p2 = start_between_lr[i];
    if (last_dis > 99) {
      last_dis = Pt2BaselineDis(base_line, vec_popr, start_po, p1);
      startslicept_bsl_dis.emplace_back(last_dis);
      startslicept.emplace_back(start_po + vec_popr * p1);
    }
    float p2_dis = Pt2BaselineDis(base_line, vec_popr, start_po, p2);
    float line_width = abs(last_dis - p2_dis);
    // HLOG_ERROR << "last_dis = " << last_dis << " p2_dis = " << p2_dis
    //            << " line_width = " << line_width;
    if (line_width > lane_width) {
      int assume_nums = static_cast<int>(std::max(
          static_cast<float>(std::floor(line_width / lane_width + 0.3)), 1.0F));
      HLOG_ERROR << "assume_nums = " << assume_nums;
      float l_width = (p2_dis - last_dis) / static_cast<float>(assume_nums);
      for (int assume_num = 1; assume_num < assume_nums; assume_num++) {
        startslicept_bsl_dis.emplace_back(
            last_dis + l_width * static_cast<float>(assume_num));
        startslicept.emplace_back(
            start_po +
            vec_popr * (p1 + (p2 - p1) / static_cast<float>(assume_nums) *
                                 static_cast<float>(assume_num)));
      }
      // HLOG_ERROR << "line_width - static_cast<float>(assume_nums - 1) * "
      //               "abs(l_width) = "
      //            << line_width -
      //                   static_cast<float>(assume_nums - 1) * abs(l_width);

      float lane_w = std::min(lane_width, averge_exit_lane_width_);
      if (line_width - static_cast<float>(assume_nums - 1) * abs(l_width) >
              conf_.max_lane_width ||
          ((left_type == 2 || right_type == 2) &&
           line_width - static_cast<float>(assume_nums - 1) * abs(l_width) >
               lane_w)) {
        if (l_width < 0.0) {
          lane_w = -lane_w;
        }
        startslicept_bsl_dis.emplace_back(
            last_dis + l_width * static_cast<float>(assume_nums - 1) + lane_w);
        startslicept.emplace_back(
            start_po + vec_popr * (p1 +
                                   (p2 - p1) / static_cast<float>(assume_nums) *
                                       static_cast<float>(assume_nums - 1) +
                                   (p2 - p1) / (p2_dis - last_dis) * lane_w));
      }
    }
    startslicept_bsl_dis.emplace_back(p2_dis);
    startslicept.emplace_back(start_po + vec_popr * p2);
    last_dis = p2_dis;
  }
  if (start_between_lr.size() == 1) {
    last_dis =
        Pt2BaselineDis(base_line, vec_popr, start_po, start_between_lr[0]);
    startslicept_bsl_dis.emplace_back(last_dis);
    startslicept.emplace_back(start_po + vec_popr * start_between_lr[0]);
  }
  // HLOG_ERROR << "startslicept_bsl_dis.size() = " <<
  // startslicept_bsl_dis.size(); for (auto& dis : startslicept_bsl_dis) {
  //   HLOG_ERROR << "startslicept_bsl_dis  dis = " << dis;
  // }
  if (base_line_flag == 0) {
    float left_start_last_dis = 0.0;
    int left_start_last_index = 0;
    if (left_type == 2) {
      if (startslicept_bsl_dis.front() - left_start_last_dis <
          conf_.min_lane_width) {
        // HLOG_ERROR << "startslicept_bsl_dis.front() "
        //            << startslicept_bsl_dis.front();
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = startslicept.front();
        double dist_to_path_line =
            base_line->dist_to_path - startslicept_bsl_dis.front();
        TranslateLine(&line_virtual, tmp, base_line, curr_group);
        line_virtual.dist_to_path = dist_to_path_line;
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
        left_start_last_dis = startslicept_bsl_dis.front();
      }
    }
    while (left_start_last_index < startslicept_bsl_dis.size()) {
      while (left_start_last_index < startslicept_bsl_dis.size() &&
             startslicept_bsl_dis[left_start_last_index] - left_start_last_dis <
                 conf_.min_lane_width) {
        left_start_last_index++;
      }
      if (left_start_last_index < startslicept_bsl_dis.size()) {
        if (start_right < FLT_MAX &&
            (start_right - startslicept_bsl_dis[left_start_last_index]) <
                conf_.min_lane_width) {
          break;
        }
        // HLOG_ERROR << "startslicept_bsl_dis[left_start_last_index] = "
        //            << startslicept_bsl_dis[left_start_last_index];
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = startslicept[left_start_last_index];
        double dist_to_path_line = base_line->dist_to_path -
                                   startslicept_bsl_dis[left_start_last_index];
        TranslateLine(&line_virtual, tmp, base_line, curr_group);
        line_virtual.dist_to_path = dist_to_path_line;
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      left_start_last_dis = startslicept_bsl_dis[left_start_last_index];
    }
    if (right_type == 2) {
      // if (PointToVectorDist(start_po, start_pr, right_line->pts[0].pt) < 5
      // ||
      //     abs(startslicept_bsl_dis.back() - start_right) > 0.5) {
      // HLOG_ERROR << "startslicept_bsl_dis.back() "
      //            << startslicept_bsl_dis.back();
      LineSegment line_virtual;
      Point tmp;
      tmp.type = VIRTUAL;
      tmp.pt = startslicept.back();
      double dist_to_path_line =
          base_line->dist_to_path - startslicept_bsl_dis.back();
      TranslateLine(&line_virtual, tmp, base_line, curr_group);
      line_virtual.dist_to_path = dist_to_path_line;
      LineSegment::Ptr line_virtual_ptr =
          std::make_shared<LineSegment>(line_virtual);
      (*line_virtual_vec).emplace_back(line_virtual_ptr);
      // }
    }
  } else {
    float right_start_last_dis = 0.0;
    int right_start_last_index =
        static_cast<int>(startslicept_bsl_dis.size()) - 1;
    if (right_type == 2) {
      if (startslicept_bsl_dis.back() - right_start_last_dis <
          conf_.min_lane_width) {
        // HLOG_ERROR << "startslicept_bsl_dis.front() = "
        //            << startslicept_bsl_dis.back();

        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = startslicept.back();
        double dist_to_path_line =
            base_line->dist_to_path + startslicept_bsl_dis.back();
        TranslateLine(&line_virtual, tmp, base_line, curr_group);
        line_virtual.dist_to_path = dist_to_path_line;
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
        right_start_last_dis = startslicept_bsl_dis.back();
      }
    }
    while (right_start_last_index >= 0) {
      while (right_start_last_index >= 0 &&
             startslicept_bsl_dis[right_start_last_index] -
                     right_start_last_dis <
                 conf_.min_lane_width) {
        right_start_last_index--;
      }
      if (right_start_last_index >= 0) {
        if (start_left > -FLT_MAX &&
            start_left - startslicept_bsl_dis[right_start_last_index] <
                conf_.min_lane_width) {
          break;
        }
        // HLOG_ERROR << "startslicept_bsl_dis[right_start_last_index] = "
        //            << startslicept_bsl_dis[right_start_last_index];
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = startslicept[right_start_last_index];
        double dist_to_path_line = base_line->dist_to_path +
                                   startslicept_bsl_dis[right_start_last_index];
        TranslateLine(&line_virtual, tmp, base_line, curr_group);
        line_virtual.dist_to_path = dist_to_path_line;
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      right_start_last_dis = startslicept_bsl_dis[right_start_last_index];
    }
    if (left_type == 2) {
      // if (PointToVectorDist(start_po, start_pr, left_line->pts[0].pt) < 5 ||
      //     abs(startslicept_bsl_dis.front() - start_left) > 0.5) {
      // HLOG_ERROR << "startslicept_bsl_dis.front() = "
      //            << startslicept_bsl_dis.front();

      LineSegment line_virtual;
      Point tmp;
      tmp.type = VIRTUAL;
      tmp.pt = startslicept.front();
      double dist_to_path_line =
          base_line->dist_to_path + startslicept_bsl_dis.front();
      TranslateLine(&line_virtual, tmp, base_line, curr_group);
      line_virtual.dist_to_path = dist_to_path_line;
      LineSegment::Ptr line_virtual_ptr =
          std::make_shared<LineSegment>(line_virtual);
      (*line_virtual_vec).emplace_back(line_virtual_ptr);
      // }
    }
  }
}

void VirtualLineGen::SlicePointVirtualLine(
    const Group::Ptr& curr_group,
    std::vector<LineSegment::Ptr>* line_virtual_vec,
    const std::vector<float>& start_between_lr,
    const std::vector<float>& end_between_lr,
    const std::vector<float>& line_dis, const LineSegment::Ptr& left_line,
    const LineSegment::Ptr& right_line, float lane_width) {
  if (line_dis.size() < 2 || left_line == nullptr || right_line == nullptr ||
      (start_between_lr.empty() && end_between_lr.empty()) ||
      left_line->pts.empty() || right_line->pts.empty()) {
    return;
  }
  int base_line_flag = 0;
  auto base_line = left_line;
  if (left_line->pts.size() < 2 ||
      (right_line->pts.size() > 1 &&
       Dist(right_line->pts[0].pt, right_line->pts.back().pt) >
           Dist(left_line->pts[0].pt, left_line->pts.back().pt))) {
    base_line = right_line;
    base_line_flag = 1;
  }
  HLOG_ERROR << "start_between_lr.size() = " << start_between_lr.size();
  for (auto& start_lr : start_between_lr) {
    HLOG_ERROR << "start_lr = " << start_lr;
  }
  HLOG_ERROR << "end_between_lr.size() = " << end_between_lr.size();
  for (auto& end_lr : end_between_lr) {
    HLOG_ERROR << "end_lr = " << end_lr;
  }
  auto start_po = curr_group->start_slice.po;
  auto start_pr = curr_group->start_slice.pr;
  auto end_po = curr_group->end_slice.po;
  auto end_pr = curr_group->end_slice.pr;
  Eigen::Vector3f vec_popr = (start_pr - start_po).normalized();
  Eigen::Vector3f vec_popr_end = (end_pr - end_po).normalized();
  std::vector<float> startslicept_bsl_dis;
  std::vector<Eigen::Vector3f> startslicept;
  float last_dis = 100;
  for (int i = 1; i < start_between_lr.size(); ++i) {
    float p1 = start_between_lr[i - 1];
    float p2 = start_between_lr[i];
    if (last_dis > 99) {
      last_dis = Pt2BaselineDis(base_line, vec_popr, start_po, p1);
      startslicept_bsl_dis.emplace_back(last_dis);
      startslicept.emplace_back(start_po + vec_popr * p1);
    }
    float p2_dis = Pt2BaselineDis(base_line, vec_popr, start_po, p2);
    float line_width = abs(last_dis - p2_dis);
    if (line_width > conf_.max_lane_width) {
      int assume_nums = static_cast<int>(std::max(
          static_cast<float>(std::floor(line_width / lane_width + 0.3)), 1.0F));
      float l_width = (p2_dis - last_dis) / static_cast<float>(assume_nums);
      for (int assume_num = 1; assume_num < assume_nums; assume_num++) {
        startslicept_bsl_dis.emplace_back(
            last_dis + l_width * static_cast<float>(assume_num));
        startslicept.emplace_back(
            start_po +
            vec_popr * (p1 + (p2 - p1) / static_cast<float>(assume_nums) *
                                 static_cast<float>(assume_num)));
      }
    }
    startslicept_bsl_dis.emplace_back(p2_dis);
    startslicept.emplace_back(start_po + vec_popr * p2);
    last_dis = p2_dis;
  }
  if (start_between_lr.size() == 1) {
    last_dis =
        Pt2BaselineDis(base_line, vec_popr, start_po, start_between_lr[0]);
    startslicept_bsl_dis.emplace_back(last_dis);
    startslicept.emplace_back(start_po + vec_popr * start_between_lr[0]);
  }
  HLOG_ERROR << "startslicept_bsl_dis.size() = " << startslicept_bsl_dis.size();
  for (auto& dis : startslicept_bsl_dis) {
    HLOG_ERROR << "startslicept_bsl_dis  dis = " << dis;
  }
  std::vector<float> endslicept_bsl_dis;
  std::vector<Eigen::Vector3f> endslicept;
  last_dis = 100;

  for (int i = 1; i < end_between_lr.size(); ++i) {
    float p1 = end_between_lr[i - 1];
    float p2 = end_between_lr[i];
    if (last_dis > 99) {
      last_dis = Pt2BaselineDis(base_line, vec_popr_end, end_po, p1);
      endslicept_bsl_dis.emplace_back(last_dis);
      endslicept.emplace_back(end_po + vec_popr_end * p1);
    }
    float p2_dis = Pt2BaselineDis(base_line, vec_popr_end, end_po, p2);
    float line_width = abs(last_dis - p2_dis);
    if (line_width > conf_.max_lane_width) {
      int assume_nums = static_cast<int>(std::max(
          static_cast<float>(std::floor(line_width / lane_width + 0.3)), 1.0F));
      float l_width = (p2_dis - last_dis) / static_cast<float>(assume_nums);
      for (int assume_num = 1; assume_num < assume_nums; assume_num++) {
        endslicept_bsl_dis.emplace_back(
            last_dis + l_width * static_cast<float>(assume_num));
        endslicept.emplace_back(
            end_po +
            vec_popr_end * (p1 + (p2 - p1) / static_cast<float>(assume_nums) *
                                     static_cast<float>(assume_num)));
      }
    }
    endslicept_bsl_dis.emplace_back(p2_dis);
    endslicept.emplace_back(end_po + vec_popr_end * p2);
    last_dis = p2_dis;
  }
  if (end_between_lr.size() == 1) {
    last_dis =
        Pt2BaselineDis(base_line, vec_popr_end, end_po, end_between_lr[0]);
    endslicept_bsl_dis.emplace_back(last_dis);
    endslicept.emplace_back(end_po + vec_popr_end * end_between_lr[0]);
  }
  HLOG_ERROR << "endslicept_bsl_dis.size() = " << endslicept_bsl_dis.size();
  for (auto& dis : endslicept_bsl_dis) {
    HLOG_ERROR << "endslicept_bsl_dis  dis = " << dis;
  }
  if (base_line_flag == 0) {
    float left_start_last_dis = 0.0;
    float left_end_last_dis = 0.0;
    int left_start_last_index = 0;
    int left_end_last_index = 0;
    while (left_start_last_index < startslicept_bsl_dis.size() &&
           left_end_last_index < endslicept_bsl_dis.size()) {
      while (left_start_last_index < startslicept_bsl_dis.size() &&
             startslicept_bsl_dis[left_start_last_index] - left_start_last_dis <
                 conf_.min_lane_width) {
        left_start_last_index++;
      }
      while (left_end_last_index < endslicept_bsl_dis.size() &&
             endslicept_bsl_dis[left_end_last_index] - left_end_last_dis <
                 conf_.min_lane_width) {
        left_end_last_index++;
      }
      if (left_start_last_index < startslicept_bsl_dis.size() &&
          left_end_last_index < endslicept_bsl_dis.size()) {
        float start_lane_width =
            startslicept_bsl_dis[left_start_last_index] - left_start_last_dis;
        float end_lane_width =
            endslicept_bsl_dis[left_end_last_index] - left_end_last_dis;
        if (abs(start_lane_width - end_lane_width) > conf_.min_lane_width) {
          LineSegment line_virtual;
          Point tmp;
          tmp.type = VIRTUAL;
          tmp.pt = endslicept[left_end_last_index];
          TranslateLineBack(&line_virtual, tmp, base_line, curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);

          LineSegment line_virtual2;
          tmp.type = VIRTUAL;
          tmp.pt = startslicept[left_start_last_index];
          TranslateLine(&line_virtual2, tmp, base_line, curr_group);
          LineSegment::Ptr line_virtual_ptr2 =
              std::make_shared<LineSegment>(line_virtual2);
          (*line_virtual_vec).emplace_back(line_virtual_ptr2);
          left_start_last_dis =
              std::max(startslicept_bsl_dis[left_start_last_index],
                       endslicept_bsl_dis[left_end_last_index]);
          left_end_last_dis = left_start_last_dis;
        } else {
          LineSegment line_virtual;
          Point start_pt, end_pt;
          start_pt.type = VIRTUAL;
          start_pt.pt = startslicept[left_start_last_index];
          end_pt.type = VIRTUAL;
          end_pt.pt = endslicept[left_end_last_index];

          TranslateAndRotateLine(&line_virtual, start_pt, end_pt, base_line,
                                 curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);
          left_start_last_dis = startslicept_bsl_dis[left_start_last_index];
          left_end_last_dis = endslicept_bsl_dis[left_end_last_index];
        }
      }
    }
    while (left_start_last_index < startslicept_bsl_dis.size()) {
      while (left_start_last_index < startslicept_bsl_dis.size() &&
             startslicept_bsl_dis[left_start_last_index] - left_start_last_dis <
                 conf_.min_lane_width) {
        left_start_last_index++;
      }
      if (left_start_last_index < startslicept_bsl_dis.size()) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = startslicept[left_start_last_index];
        TranslateLine(&line_virtual, tmp, base_line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      left_start_last_dis = startslicept_bsl_dis[left_start_last_index];
    }
    while (left_end_last_index < endslicept_bsl_dis.size()) {
      while (left_end_last_index < endslicept_bsl_dis.size() &&
             endslicept_bsl_dis[left_end_last_index] - left_end_last_dis <
                 conf_.min_lane_width) {
        left_end_last_index++;
      }
      if (left_end_last_index < endslicept_bsl_dis.size()) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = endslicept[left_end_last_index];
        TranslateLineBack(&line_virtual, tmp, base_line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      left_end_last_dis = endslicept_bsl_dis[left_end_last_index];
    }
  } else {
    float right_start_last_dis = 0.0;
    float right_end_last_dis = 0.0;
    int right_start_last_index =
        static_cast<int>(startslicept_bsl_dis.size()) - 1;
    int right_end_last_index = static_cast<int>(endslicept_bsl_dis.size()) - 1;
    while (right_start_last_index >= 0 && right_end_last_index >= 0) {
      while (right_start_last_index >= 0 &&
             startslicept_bsl_dis[right_start_last_index] -
                     right_start_last_dis <
                 conf_.min_lane_width) {
        right_start_last_index--;
      }
      while (right_end_last_index >= 0 &&
             endslicept_bsl_dis[right_end_last_index] - right_end_last_dis <
                 conf_.min_lane_width) {
        right_end_last_index--;
      }
      if (right_start_last_index >= 0 && right_end_last_index >= 0) {
        float start_lane_width =
            startslicept_bsl_dis[right_start_last_index] - right_start_last_dis;
        float end_lane_width =
            endslicept_bsl_dis[right_end_last_index] - right_end_last_dis;
        if (abs(start_lane_width - end_lane_width) > conf_.min_lane_width) {
          LineSegment line_virtual;
          Point tmp;
          tmp.type = VIRTUAL;
          tmp.pt = endslicept[right_end_last_index];
          TranslateLineBack(&line_virtual, tmp, base_line, curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);

          LineSegment line_virtual2;
          tmp.type = VIRTUAL;
          tmp.pt = startslicept[right_start_last_index];
          TranslateLine(&line_virtual2, tmp, base_line, curr_group);
          LineSegment::Ptr line_virtual_ptr2 =
              std::make_shared<LineSegment>(line_virtual2);
          (*line_virtual_vec).emplace_back(line_virtual_ptr2);
          right_start_last_dis =
              std::max(startslicept_bsl_dis[right_start_last_index],
                       endslicept_bsl_dis[right_end_last_index]);
          right_end_last_dis = right_start_last_dis;
        } else {
          LineSegment line_virtual;
          Point start_pt, end_pt;
          start_pt.type = VIRTUAL;
          start_pt.pt = startslicept[right_start_last_index];
          end_pt.type = VIRTUAL;
          end_pt.pt = endslicept[right_end_last_index];

          TranslateAndRotateLine(&line_virtual, start_pt, end_pt, base_line,
                                 curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);
          right_start_last_dis = startslicept_bsl_dis[right_start_last_index];
          right_end_last_dis = endslicept_bsl_dis[right_end_last_index];
        }
      }
    }
    while (right_start_last_index >= 0) {
      while (right_start_last_index >= 0 &&
             startslicept_bsl_dis[right_start_last_index] -
                     right_start_last_dis <
                 conf_.min_lane_width) {
        right_start_last_index--;
      }
      if (right_start_last_index >= 0) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = startslicept[right_start_last_index];
        TranslateLine(&line_virtual, tmp, base_line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      right_start_last_dis = startslicept_bsl_dis[right_start_last_index];
    }
    while (right_end_last_index >= 0) {
      while (right_end_last_index >= 0 &&
             endslicept_bsl_dis[right_end_last_index] - right_end_last_dis <
                 conf_.min_lane_width) {
        right_end_last_index--;
      }
      if (right_end_last_index >= 0) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = endslicept[right_end_last_index];
        TranslateLineBack(&line_virtual, tmp, base_line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      right_end_last_dis = endslicept_bsl_dis[right_end_last_index];
    }
  }
}
void VirtualLineGen::SlicePointVirtualLine(
    const Group::Ptr& curr_group,
    std::vector<LineSegment::Ptr>* line_virtual_vec,
    const std::vector<float>& start_between_lr,
    const std::vector<float>& end_between_lr,
    const std::vector<float>& line_dis, const RoadEdge::Ptr& road_left,
    const RoadEdge::Ptr& road_right, float lane_width) {
  if (line_dis.size() < 2 || road_left == nullptr || road_right == nullptr ||
      (start_between_lr.empty() && end_between_lr.empty()) ||
      road_left->points.empty() || road_right->points.empty()) {
    return;
  }
  int base_line_flag = 0;  // 0 represent left 1 right
  auto base_line = road_left;
  if (road_left->points.size() < 2 ||
      (road_right->points.size() > 1 &&
       Dist(road_right->points[0], road_right->points.back()) >
           Dist(road_left->points[0], road_left->points.back()))) {
    base_line = road_right;
    base_line_flag = 1;
  }
  auto start_po = curr_group->start_slice.po;
  auto start_pr = curr_group->start_slice.pr;
  auto end_po = curr_group->end_slice.po;
  auto end_pr = curr_group->end_slice.pr;
  Eigen::Vector3f vec_popr = (start_pr - start_po).normalized();
  Eigen::Vector3f vec_popr_end = (end_pr - end_po).normalized();

  std::vector<float> startslicept_bsl_dis;
  std::vector<Eigen::Vector3f> startslicept;
  float last_dis = 100;
  HLOG_ERROR << "start_between_lr.size() = " << start_between_lr.size();
  for (auto& start_lr : start_between_lr) {
    HLOG_ERROR << "start_lr = " << start_lr;
  }
  HLOG_ERROR << "end_between_lr.size() = " << end_between_lr.size();
  for (auto& end_lr : end_between_lr) {
    HLOG_ERROR << "end_lr = " << end_lr;
  }
  for (int i = 1; i < start_between_lr.size(); ++i) {
    float p1 = start_between_lr[i - 1];
    float p2 = start_between_lr[i];
    if (last_dis > 99) {
      last_dis = Pt2BaselineDis(base_line, vec_popr, start_po, p1);
      startslicept_bsl_dis.emplace_back(last_dis);
      startslicept.emplace_back(start_po + vec_popr * p1);
    }
    float p2_dis = Pt2BaselineDis(base_line, vec_popr, start_po, p2);
    float line_width = abs(last_dis - p2_dis);
    if (line_width > conf_.max_lane_width) {
      int assume_nums = static_cast<int>(std::max(
          static_cast<float>(std::floor(line_width / lane_width + 0.3)), 1.0F));
      float l_width = (p2_dis - last_dis) / static_cast<float>(assume_nums);
      for (int assume_num = 1; assume_num < assume_nums; assume_num++) {
        startslicept_bsl_dis.emplace_back(
            last_dis + l_width * static_cast<float>(assume_num));
        startslicept.emplace_back(
            start_po +
            vec_popr * (p1 + (p2 - p1) / static_cast<float>(assume_nums) *
                                 static_cast<float>(assume_num)));
      }
    }
    startslicept_bsl_dis.emplace_back(p2_dis);
    startslicept.emplace_back(start_po + vec_popr * p2);
    last_dis = p2_dis;
  }
  if (start_between_lr.size() == 1) {
    last_dis =
        Pt2BaselineDis(base_line, vec_popr, start_po, start_between_lr[0]);
    startslicept_bsl_dis.emplace_back(last_dis);
    startslicept.emplace_back(start_po + vec_popr * start_between_lr[0]);
  }
  HLOG_ERROR << "startslicept_bsl_dis.size() = " << startslicept_bsl_dis.size();
  for (auto& dis : startslicept_bsl_dis) {
    HLOG_ERROR << "startslicept_bsl_dis  dis = " << dis;
  }
  std::vector<float> endslicept_bsl_dis;
  std::vector<Eigen::Vector3f> endslicept;
  last_dis = 100;
  for (int i = 1; i < end_between_lr.size(); ++i) {
    float p1 = end_between_lr[i - 1];
    float p2 = end_between_lr[i];
    if (last_dis > 99) {
      last_dis = Pt2BaselineDis(base_line, vec_popr_end, end_po, p1);
      endslicept_bsl_dis.emplace_back(last_dis);
      endslicept.emplace_back(end_po + vec_popr_end * p1);
    }
    float p2_dis = Pt2BaselineDis(base_line, vec_popr_end, end_po, p2);
    float line_width = abs(last_dis - p2_dis);
    if (line_width > conf_.max_lane_width) {
      int assume_nums = static_cast<int>(std::max(
          static_cast<float>(std::floor(line_width / lane_width + 0.3)), 1.0F));
      float l_width = (p2_dis - last_dis) / static_cast<float>(assume_nums);
      for (int assume_num = 1; assume_num < assume_nums; assume_num++) {
        endslicept_bsl_dis.emplace_back(
            last_dis + l_width * static_cast<float>(assume_num));
        endslicept.emplace_back(
            end_po +
            vec_popr_end * (p1 + (p2 - p1) / static_cast<float>(assume_nums) *
                                     static_cast<float>(assume_num)));
      }
    }
    endslicept_bsl_dis.emplace_back(p2_dis);
    endslicept.emplace_back(end_po + vec_popr_end * p2);
    last_dis = p2_dis;
  }
  if (end_between_lr.size() == 1) {
    last_dis =
        Pt2BaselineDis(base_line, vec_popr_end, end_po, end_between_lr[0]);
    endslicept_bsl_dis.emplace_back(last_dis);
    endslicept.emplace_back(end_po + vec_popr_end * end_between_lr[0]);
  }
  HLOG_ERROR << "endslicept_bsl_dis.size() = " << endslicept_bsl_dis.size();
  for (auto& dis : endslicept_bsl_dis) {
    HLOG_ERROR << "endslicept_bsl_dis  dis = " << dis;
  }
  if (base_line_flag == 0) {
    float left_start_last_dis = 0.0;
    float left_end_last_dis = 0.0;
    int left_start_last_index = 0;
    int left_end_last_index = 0;
    while (left_start_last_index < startslicept_bsl_dis.size() &&
           left_end_last_index < endslicept_bsl_dis.size()) {
      while (left_start_last_index < startslicept_bsl_dis.size() &&
             startslicept_bsl_dis[left_start_last_index] - left_start_last_dis <
                 conf_.min_lane_width) {
        left_start_last_index++;
      }
      while (left_end_last_index < endslicept_bsl_dis.size() &&
             endslicept_bsl_dis[left_end_last_index] - left_end_last_dis <
                 conf_.min_lane_width) {
        left_end_last_index++;
      }
      if (left_start_last_index < startslicept_bsl_dis.size() &&
          left_end_last_index < endslicept_bsl_dis.size()) {
        float start_lane_width =
            startslicept_bsl_dis[left_start_last_index] - left_start_last_dis;
        float end_lane_width =
            endslicept_bsl_dis[left_end_last_index] - left_end_last_dis;
        if (abs(start_lane_width - end_lane_width) > conf_.min_lane_width) {
          LineSegment line_virtual;
          Point tmp;
          tmp.type = VIRTUAL;
          tmp.pt = endslicept[left_end_last_index];
          TranslateRoadBack(&line_virtual, tmp, base_line, curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);

          LineSegment line_virtual2;
          tmp.type = VIRTUAL;
          tmp.pt = startslicept[left_start_last_index];
          TranslateRoad(&line_virtual2, tmp, base_line, curr_group);
          LineSegment::Ptr line_virtual_ptr2 =
              std::make_shared<LineSegment>(line_virtual2);
          (*line_virtual_vec).emplace_back(line_virtual_ptr2);
          left_start_last_dis =
              std::max(startslicept_bsl_dis[left_start_last_index],
                       endslicept_bsl_dis[left_end_last_index]);
          left_end_last_dis = left_start_last_dis;
        } else {
          LineSegment line_virtual;
          Point start_pt, end_pt;
          start_pt.type = VIRTUAL;
          start_pt.pt = startslicept[left_start_last_index];
          end_pt.type = VIRTUAL;
          end_pt.pt = endslicept[left_end_last_index];

          TranslateAndRotateRoad(&line_virtual, start_pt, end_pt, base_line,
                                 curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);
          left_start_last_dis = startslicept_bsl_dis[left_start_last_index];
          left_end_last_dis = endslicept_bsl_dis[left_end_last_index];
        }
      }
    }
    while (left_start_last_index < startslicept_bsl_dis.size()) {
      while (left_start_last_index < startslicept_bsl_dis.size() &&
             startslicept_bsl_dis[left_start_last_index] - left_start_last_dis <
                 conf_.min_lane_width) {
        left_start_last_index++;
      }
      if (left_start_last_index < startslicept_bsl_dis.size()) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = startslicept[left_start_last_index];
        TranslateRoad(&line_virtual, tmp, base_line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      left_start_last_dis = startslicept_bsl_dis[left_start_last_index];
    }
    while (left_end_last_index < endslicept_bsl_dis.size()) {
      while (left_end_last_index < endslicept_bsl_dis.size() &&
             endslicept_bsl_dis[left_end_last_index] - left_end_last_dis <
                 conf_.min_lane_width) {
        left_end_last_index++;
      }
      if (left_end_last_index < endslicept_bsl_dis.size()) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = endslicept[left_end_last_index];
        TranslateRoadBack(&line_virtual, tmp, base_line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      left_end_last_dis = endslicept_bsl_dis[left_end_last_index];
    }
  } else {
    float right_start_last_dis = 0.0;
    float right_end_last_dis = 0.0;
    int right_start_last_index =
        static_cast<int>(startslicept_bsl_dis.size()) - 1;
    int right_end_last_index = static_cast<int>(endslicept_bsl_dis.size()) - 1;
    while (right_start_last_index >= 0 && right_end_last_index >= 0) {
      while (right_start_last_index >= 0 &&
             startslicept_bsl_dis[right_start_last_index] -
                     right_start_last_dis <
                 conf_.min_lane_width) {
        right_start_last_index--;
      }
      while (right_end_last_index >= 0 &&
             endslicept_bsl_dis[right_end_last_index] - right_end_last_dis <
                 conf_.min_lane_width) {
        right_end_last_index--;
      }
      if (right_start_last_index >= 0 && right_end_last_index >= 0) {
        float start_lane_width =
            startslicept_bsl_dis[right_start_last_index] - right_start_last_dis;
        float end_lane_width =
            endslicept_bsl_dis[right_end_last_index] - right_end_last_dis;
        if (abs(start_lane_width - end_lane_width) > conf_.min_lane_width) {
          LineSegment line_virtual;
          Point tmp;
          tmp.type = VIRTUAL;
          tmp.pt = endslicept[right_end_last_index];
          TranslateRoadBack(&line_virtual, tmp, base_line, curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);

          LineSegment line_virtual2;
          tmp.type = VIRTUAL;
          tmp.pt = startslicept[right_start_last_index];
          TranslateRoad(&line_virtual2, tmp, base_line, curr_group);
          LineSegment::Ptr line_virtual_ptr2 =
              std::make_shared<LineSegment>(line_virtual2);
          (*line_virtual_vec).emplace_back(line_virtual_ptr2);
          right_start_last_dis =
              std::max(startslicept_bsl_dis[right_start_last_index],
                       endslicept_bsl_dis[right_end_last_index]);
          right_end_last_dis = right_start_last_dis;
        } else {
          LineSegment line_virtual;
          Point start_pt, end_pt;
          start_pt.type = VIRTUAL;
          start_pt.pt = startslicept[right_start_last_index];
          end_pt.type = VIRTUAL;
          end_pt.pt = endslicept[right_end_last_index];

          TranslateAndRotateRoad(&line_virtual, start_pt, end_pt, base_line,
                                 curr_group);
          LineSegment::Ptr line_virtual_ptr =
              std::make_shared<LineSegment>(line_virtual);
          (*line_virtual_vec).emplace_back(line_virtual_ptr);
          right_start_last_dis = startslicept_bsl_dis[right_start_last_index];
          right_end_last_dis = endslicept_bsl_dis[right_end_last_index];
        }
      }
    }
    while (right_start_last_index >= 0) {
      while (right_start_last_index >= 0 &&
             startslicept_bsl_dis[right_start_last_index] -
                     right_start_last_dis <
                 conf_.min_lane_width) {
        right_start_last_index--;
      }
      if (right_start_last_index >= 0) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = startslicept[right_start_last_index];
        TranslateRoad(&line_virtual, tmp, base_line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      right_start_last_dis = startslicept_bsl_dis[right_start_last_index];
    }
    while (right_end_last_index >= 0) {
      while (right_end_last_index >= 0 &&
             endslicept_bsl_dis[right_end_last_index] - right_end_last_dis <
                 conf_.min_lane_width) {
        right_end_last_index--;
      }
      if (right_end_last_index >= 0) {
        LineSegment line_virtual;
        Point tmp;
        tmp.type = VIRTUAL;
        tmp.pt = endslicept[right_end_last_index];
        TranslateRoadBack(&line_virtual, tmp, base_line, curr_group);
        LineSegment::Ptr line_virtual_ptr =
            std::make_shared<LineSegment>(line_virtual);
        (*line_virtual_vec).emplace_back(line_virtual_ptr);
      }
      right_end_last_dis = endslicept_bsl_dis[right_end_last_index];
    }
  }
}

Eigen::Matrix2f rotationMatrix(const Eigen::Vector2f& a,
                               const Eigen::Vector2f& b) {
  float cos_theta = a.dot(b) / (a.norm() * b.norm());
  float angle = std::acos(cos_theta);  // 计算角度

  // 计算旋转矩阵
  Eigen::Matrix2f R;
  R << std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle);

  return R;
}

void VirtualLineGen::TranslateAndRotateRoad(LineSegment* line_virtual,
                                            Point start_pt, Point end_pt,
                                            const RoadEdge::Ptr& road_edge,
                                            const Group::Ptr& curr_group) {
  if (road_edge->points.size() < 2) {
    return;
  }
  Eigen::Vector3f end_po = curr_group->end_slice.po;
  Eigen::Vector3f end_pr = curr_group->end_slice.pr;

  float scale = Dist(end_pt.pt, start_pt.pt) /
                Dist(road_edge->points.front(), road_edge->points.back());
  Eigen::Vector2f start_pt_end_nm =
      (end_pt.pt - start_pt.pt).normalized().head<2>();
  Eigen::Vector2f road_edge_vec_nm =
      (road_edge->points.back() - road_edge->points.front())
          .normalized()
          .head<2>();
  Eigen::Matrix2f R = rotationMatrix(start_pt_end_nm, road_edge_vec_nm);
  (*line_virtual).pts.emplace_back(start_pt);
  for (int pt_index = 0; pt_index < road_edge->points.size() - 1; pt_index++) {
    Eigen::Vector2f p1 = road_edge->points[pt_index].head<2>();
    Eigen::Vector2f p2 = road_edge->points[pt_index + 1].head<2>();
    Eigen::Vector2f deta = (p2 - p1) * scale;
    start_pt.pt.x() = start_pt.pt.x() + R(0, 0) * deta.x() + R(0, 1) * deta.y();
    start_pt.pt.y() = start_pt.pt.y() + R(1, 0) * deta.x() + R(1, 1) * deta.y();
    // 补的点不能超过end_slice
    if (PointInVectorSide(end_po, end_pr, start_pt.pt) < 0.0) {
      break;
    }
    (*line_virtual).pts.emplace_back(start_pt);
  }
  // HLOG_ERROR << "line_virtual.pts.size() = " << line_virtual.pts.size();
  (*line_virtual).type = LaneType_LANE_VIRTUAL_MARKING;
  (*line_virtual).color = WHITE;
  (*line_virtual).lanepos = LanePositionType_OTHER;
  (*line_virtual).id = (virtual_line_id_++ % 3000);
}

void VirtualLineGen::TranslateAndRotateLine(LineSegment* line_virtual,
                                            Point start_pt, Point end_pt,
                                            const LineSegment::Ptr& line,
                                            const Group::Ptr& curr_group) {
  if (line->pts.size() < 2) {
    return;
  }
  Eigen::Vector3f end_po = curr_group->end_slice.po;
  Eigen::Vector3f end_pr = curr_group->end_slice.pr;

  float scale = Dist(end_pt.pt, start_pt.pt) /
                Dist(line->pts.front().pt, line->pts.back().pt);
  Eigen::Vector2f start_pt_end_nm =
      (end_pt.pt - start_pt.pt).normalized().head<2>();
  Eigen::Vector2f road_edge_vec_nm =
      (line->pts.back().pt - line->pts.front().pt).normalized().head<2>();
  Eigen::Matrix2f R = rotationMatrix(start_pt_end_nm, road_edge_vec_nm);
  (*line_virtual).pts.emplace_back(start_pt);
  for (int pt_index = 0; pt_index < line->pts.size() - 1; pt_index++) {
    Eigen::Vector2f p1 = line->pts[pt_index].pt.head<2>();
    Eigen::Vector2f p2 = line->pts[pt_index + 1].pt.head<2>();
    Eigen::Vector2f deta = (p2 - p1) * scale;
    start_pt.pt.x() = start_pt.pt.x() + R(0, 0) * deta.x() + R(0, 1) * deta.y();
    start_pt.pt.y() = start_pt.pt.y() + R(1, 0) * deta.x() + R(1, 1) * deta.y();
    // 补的点不能超过end_slice
    if (PointInVectorSide(end_po, end_pr, start_pt.pt) < 0.0) {
      break;
    }
    (*line_virtual).pts.emplace_back(start_pt);
  }
  // HLOG_ERROR << "line_virtual.pts.size() = " << line_virtual.pts.size();
  (*line_virtual).type = LaneType_LANE_VIRTUAL_MARKING;
  (*line_virtual).color = WHITE;
  (*line_virtual).lanepos = LanePositionType_OTHER;
  (*line_virtual).id = (virtual_line_id_++ % 3000);
}

void VirtualLineGen::TranslateLine(LineSegment* line_virtual, Point tmp,
                                   const LineSegment::Ptr& left_line,
                                   const Group::Ptr& curr_group) {
  Eigen::Vector3f end_po = curr_group->end_slice.po;
  Eigen::Vector3f end_pr = curr_group->end_slice.pr;
  Eigen::Vector3f start_po = curr_group->start_slice.po;
  Eigen::Vector3f start_pr = curr_group->start_slice.pr;
  (*line_virtual).pts.emplace_back(tmp);
  for (int pt_index = 0; pt_index < left_line->pts.size() - 1; pt_index++) {
    const auto& p1 = left_line->pts[pt_index];
    const auto& p2 = left_line->pts[pt_index + 1];
    tmp.pt = tmp.pt + p2.pt - p1.pt;
    // 补的点不能超过end_slice
    if (PointInVectorSide(end_po, end_pr, tmp.pt) < 0.0) {
      break;
    }
    (*line_virtual).pts.emplace_back(tmp);
  }
  while (!(*line_virtual).pts.empty()) {
    auto pt = (*line_virtual).pts.front();
    if (PointInVectorSide(start_po, start_pr, pt.pt) > 0.0) {
      (*line_virtual).pts.erase((*line_virtual).pts.begin());
    } else {
      break;
    }
  }
  // HLOG_ERROR << "line_virtual.pts.size() = " << line_virtual.pts.size();
  (*line_virtual).type = LaneType_DASHED;
  (*line_virtual).color = WHITE;
  (*line_virtual).lanepos = LanePositionType_OTHER;
  (*line_virtual).id = (virtual_line_id_++ % 3000);
}
void VirtualLineGen::TranslateRoad(LineSegment* line_virtual, Point tmp,
                                   const RoadEdge::Ptr& road_edge,
                                   const Group::Ptr& curr_group) {
  Eigen::Vector3f end_po = curr_group->end_slice.po;
  Eigen::Vector3f end_pr = curr_group->end_slice.pr;
  (*line_virtual).pts.emplace_back(tmp);
  for (int pt_index = 0; pt_index < road_edge->points.size() - 1; pt_index++) {
    const auto& p1 = road_edge->points[pt_index];
    const auto& p2 = road_edge->points[pt_index + 1];
    tmp.pt = tmp.pt + p2 - p1;
    // 补的点不能超过end_slice
    if (PointInVectorSide(end_po, end_pr, tmp.pt) < 0.0) {
      break;
    }
    (*line_virtual).pts.emplace_back(tmp);
  }
  // HLOG_ERROR << "line_virtual.pts.size() = " << line_virtual.pts.size();
  (*line_virtual).type = LaneType_LANE_VIRTUAL_MARKING;
  (*line_virtual).color = WHITE;
  (*line_virtual).lanepos = LanePositionType_OTHER;
  (*line_virtual).id = (virtual_line_id_++ % 3000);
}
void VirtualLineGen::TranslateLineBack(LineSegment* line_virtual, Point tmp,
                                       const LineSegment::Ptr& left_line,
                                       const Group::Ptr& curr_group) {
  // Eigen::Vector3f end_po = curr_group->end_slice.po;
  // Eigen::Vector3f end_pr = curr_group->end_slice.pr;
  Eigen::Vector3f start_po = curr_group->start_slice.po;
  Eigen::Vector3f start_pr = curr_group->start_slice.pr;
  (*line_virtual).pts.emplace_back(tmp);
  for (int pt_index = left_line->pts.size() - 2; pt_index >= 0; pt_index--) {
    const auto& p1 = left_line->pts[pt_index];
    const auto& p2 = left_line->pts[pt_index + 1];
    tmp.pt = tmp.pt - (p2.pt - p1.pt);
    // 补的点不能超过start_slice
    if (PointInVectorSide(start_po, start_pr, tmp.pt) > 0.0) {
      break;
    }
    (*line_virtual).pts.insert((*line_virtual).pts.begin(), tmp);
  }
  // HLOG_ERROR << "line_virtual.pts.size() = " << line_virtual.pts.size();
  (*line_virtual).type = LaneType_LANE_VIRTUAL_MARKING;
  (*line_virtual).color = WHITE;
  (*line_virtual).lanepos = LanePositionType_OTHER;
  (*line_virtual).id = (virtual_line_id_++ % 3000);
}
void VirtualLineGen::TranslateRoadBack(LineSegment* line_virtual, Point tmp,
                                       const RoadEdge::Ptr& road_edge,
                                       const Group::Ptr& curr_group) {
  Eigen::Vector3f start_po = curr_group->start_slice.po;
  Eigen::Vector3f start_pr = curr_group->start_slice.pr;
  (*line_virtual).pts.emplace_back(tmp);
  for (int pt_index = road_edge->points.size() - 2; pt_index >= 0; pt_index--) {
    const auto& p1 = road_edge->points[pt_index];
    const auto& p2 = road_edge->points[pt_index + 1];
    tmp.pt = tmp.pt - (p2 - p1);
    // 补的点不能超过end_slice
    if (PointInVectorSide(start_po, start_pr, tmp.pt) > 0.0) {
      break;
    }
    (*line_virtual).pts.insert((*line_virtual).pts.begin(), tmp);
  }
  // HLOG_ERROR << "line_virtual.pts.size() = " << line_virtual.pts.size();
  (*line_virtual).type = LaneType_LANE_VIRTUAL_MARKING;
  (*line_virtual).color = WHITE;
  (*line_virtual).lanepos = LanePositionType_OTHER;
  (*line_virtual).id = (virtual_line_id_++ % 3000);
}
std::vector<float> VirtualLineGen::StartSlicePrevLineIntersecPointProjec(
    const Group::Ptr& prev_group, const Group::Ptr& curr_group) {
  std::vector<float> intersec_pt;
  if (prev_group == nullptr) {
    return intersec_pt;
  }
  const auto& start_po = curr_group->start_slice.po;
  const auto& start_pr = curr_group->start_slice.pr;
  for (const auto& line : prev_group->line_segments) {
    if (line->pts.empty()) {
      continue;
    }
    HLOG_ERROR << "prev_group->line_segments.id is " << line->id;
    const auto& last_pt = line->pts.back().pt;
    intersec_pt.emplace_back(ProjectionToVector(start_po, start_pr, last_pt));
  }
  return intersec_pt;
}

std::vector<float> VirtualLineGen::EndSliceNextLineIntersecPointProjec(
    const Group::Ptr& curr_group, const Group::Ptr& next_group) {
  std::vector<float> intersec_pt;
  if (next_group == nullptr) {
    return intersec_pt;
  }
  const auto& start_po = curr_group->end_slice.po;
  const auto& start_pr = curr_group->end_slice.pr;
  for (const auto& line : next_group->line_segments) {
    if (line->pts.empty()) {
      continue;
    }
    const auto& front_pt = line->pts.front().pt;
    intersec_pt.emplace_back(ProjectionToVector(start_po, start_pr, front_pt));
  }
  return intersec_pt;
}

void VirtualLineGen::SameLineNumVirtualBuild(
    const Group::Ptr& curr_group,
    std::vector<LineSegment::Ptr>* line_virtual_vec,
    const std::vector<float>& line_dis, const LineSegment::Ptr& left_line,
    const LineSegment::Ptr& right_line, float lane_width) {
  if (line_dis.size() < 2 || left_line == nullptr || right_line == nullptr) {
    return;
  }
  float line_width = (line_dis[0] + line_dis[1]) / 2.0;
  int assume_nums = static_cast<int>(std::max(
      static_cast<float>(std::floor(line_width / lane_width + 0.3)), 1.0F));
  HLOG_ERROR << "assume_nums IS " << assume_nums;
  if (assume_nums > 0) {
    for (int num = 1; num <= assume_nums - 1; ++num) {
      LineSegment line_virtual;
      Point tmp;
      tmp.type = VIRTUAL;
      auto vec = right_line->pts[0].pt - left_line->pts[0].pt;
      HLOG_ERROR << "VEC " << vec.x() << "  " << vec.y();
      tmp.pt = vec / assume_nums * num + left_line->pts[0].pt;
      // HLOG_ERROR << "tmp.pt = " << tmp.pt.x() << "  " <<
      // tmp.pt.y();
      TranslateLine(&line_virtual, tmp, left_line, curr_group);
      LineSegment::Ptr line_virtual_ptr =
          std::make_shared<LineSegment>(line_virtual);
      (*line_virtual_vec).emplace_back(line_virtual_ptr);
    }
  }
}

void VirtualLineGen::SameLineNumVirtualBuild(
    const Group::Ptr& curr_group,
    std::vector<LineSegment::Ptr>* line_virtual_vec,
    const std::vector<float>& line_dis, const LineSegment::Ptr& left_line,
    const LineSegment::Ptr& right_line, float lane_width, int left_type,
    int right_type) {
  if (line_dis.size() < 2 || left_line == nullptr || right_line == nullptr) {
    return;
  }
  auto base_line = left_line;
  auto other_line = right_line;
  double dis_to_path_norm = right_line->dist_to_path - left_line->dist_to_path;
  if (left_line->pts.size() >= 2 && right_line->pts.size() >= 2) {
    if (left_type == right_type) {
      float left_line_length =
          Dist(left_line->pts.back().pt, right_line->pts.front().pt);
      float right_line_length =
          Dist(right_line->pts.back().pt, right_line->pts.front().pt);
      // base线的选取需要优化 根据曲率等
      if (left_line_length < right_line_length) {
        base_line = right_line;
        other_line = left_line;
        dis_to_path_norm = left_line->dist_to_path - right_line->dist_to_path;
      }
    } else if (right_type == 1) {
      base_line = right_line;
      other_line = left_line;
      dis_to_path_norm = left_line->dist_to_path - right_line->dist_to_path;
    }
  } else if (left_line->pts.size() < 2) {
    base_line = right_line;
    other_line = left_line;
    dis_to_path_norm = left_line->dist_to_path - right_line->dist_to_path;
  }

  float line_width = (line_dis[0] + line_dis[1]) / 2.0;
  int assume_nums = static_cast<int>(std::max(
      static_cast<float>(std::floor(line_width / lane_width + 0.3)), 1.0F));
  // HLOG_ERROR << "assume_nums IS " << assume_nums;
  if (assume_nums > 0) {
    int n_index = 1;
    while (n_index < base_line->pts.size() &&
           (base_line->pts[n_index].pt - base_line->pts[0].pt).norm() < 0.5) {
      n_index++;
    }
    if (n_index >= base_line->pts.size()) {
      return;
    }
    Eigen::Vector3f norm_vec =
        (base_line->pts[n_index].pt - base_line->pts[0].pt).normalized();
    Eigen::Vector3f vec(norm_vec.y(), -norm_vec.x(), 0.0f);
    if (PointInVectorSide(base_line->pts[0].pt, base_line->pts[n_index].pt,
                          other_line->pts[0].pt) < 0.0) {
      vec.x() = -vec.x();
      vec.y() = -vec.y();
    }
    if (left_type == 1 && right_type == 1) {
      lane_width = line_width / static_cast<float>(assume_nums);
    }
    for (int num = 1; num <= assume_nums - 1; ++num) {
      LineSegment line_virtual;
      Point tmp;
      tmp.type = VIRTUAL;
      tmp.pt =
          vec * lane_width * static_cast<float>(num) + base_line->pts[0].pt;
      double dist_to_path_line = dis_to_path_norm /
                                     static_cast<double>(line_width) *
                                     static_cast<double>(num) * lane_width +
                                 base_line->dist_to_path;
      // HLOG_ERROR << "tmp.pt = " << tmp.pt.x() << "  " << tmp.pt.y();
      TranslateLine(&line_virtual, tmp, base_line, curr_group);
      line_virtual.dist_to_path = dist_to_path_line;
      LineSegment::Ptr line_virtual_ptr =
          std::make_shared<LineSegment>(line_virtual);
      (*line_virtual_vec).emplace_back(line_virtual_ptr);
    }
    // HLOG_ERROR
    //     << "line_width = " << line_width
    //     << " static_cast<float>(assume_nums - 1) * lane_width = "
    //     << static_cast<float>(assume_nums - 1) * lane_width
    //     << "  line_width - static_cast<float>(assume_nums - 1) * lane_width
    //     ="
    //     << line_width - static_cast<float>(assume_nums - 1) * lane_width;
    if (line_width - static_cast<float>(assume_nums - 1) * lane_width >
            conf_.max_lane_width - 0.5 ||
        (line_width - static_cast<float>(assume_nums - 1) * lane_width >
             lane_width &&
         (left_type == 2 || right_type == 2))) {
      LineSegment line_virtual;
      Point tmp;
      tmp.type = VIRTUAL;

      tmp.pt = vec * lane_width * static_cast<float>(assume_nums) +
               base_line->pts[0].pt;
      double dist_to_path_line =
          dis_to_path_norm / static_cast<double>(line_width) *
              static_cast<double>(assume_nums) * lane_width +
          base_line->dist_to_path;
      // HLOG_ERROR << "tmp.pt = " << tmp.pt.x() << "  " << tmp.pt.y();
      TranslateLine(&line_virtual, tmp, base_line, curr_group);
      line_virtual.dist_to_path = dist_to_path_line;
      LineSegment::Ptr line_virtual_ptr =
          std::make_shared<LineSegment>(line_virtual);
      (*line_virtual_vec).emplace_back(line_virtual_ptr);
    }
  }
}
void VirtualLineGen::SameLineNumVirtualBuild(
    const Group::Ptr& curr_group,
    std::vector<LineSegment::Ptr>* line_virtual_vec,
    const std::vector<float>& line_dis, const RoadEdge::Ptr& road_edge,
    const LineSegment::Ptr& line, float lane_width) {
  if (line_dis.size() < 2 || line == nullptr || road_edge == nullptr) {
    return;
  }
  auto end_slice = curr_group->end_slice;
  auto start_slice = curr_group->start_slice;

  float line_width = (line_dis[0] + line_dis[1]) / 2.0;
  int assume_nums = static_cast<int>(std::max(
      static_cast<float>(std::floor(line_width / lane_width + 0.3)), 1.0F));
  HLOG_ERROR << "assume_nums IS " << assume_nums;
  if (assume_nums > 0) {
    if (static_cast<float>(assume_nums) > line_width / lane_width) {
      lane_width = line_width / static_cast<float>(assume_nums);
    }
    for (int num = 1; num <= assume_nums; ++num) {
      LineSegment line_virtual;
      Point tmp;
      tmp.type = VIRTUAL;

      // change
      int n_index = 1;
      while (n_index < line->pts.size() &&
             (line->pts[n_index].pt - line->pts[0].pt).norm() < 0.5) {
        n_index++;
      }
      if (n_index >= line->pts.size()) {
        return;
      }

      Eigen::Vector3f norm_vec =
          (line->pts[n_index].pt - line->pts[0].pt).normalized();

      Eigen::Vector3f vec(norm_vec.y(), -norm_vec.x(), 0.0f);
      if (PointInVectorSide(line->pts[0].pt, line->pts[n_index].pt,
                            road_edge->points[0]) < 0.0) {
        vec.x() = -vec.x();
        vec.y() = -vec.y();
      }
      // HLOG_ERROR << "vec = " << vec.x() << "  " << vec.y()
      //            << "  line_width = " << line_width
      //            << "  assume_nums = " << assume_nums;
      tmp.pt = vec * lane_width * static_cast<float>(num) + line->pts[0].pt;

      // auto vec = road_edge->points[0] - line->pts[0].pt;
      // tmp.pt = vec / assume_nums * num + line->pts[0].pt;

      TranslateLine(&line_virtual, tmp, line, curr_group);
      // line_virtual.pts.emplace_back(tmp);
      // for (int pt_index = 0; pt_index < line->pts.size() - 1; pt_index++) {
      //   auto p1 = line->pts[pt_index];
      //   auto p2 = line->pts[pt_index + 1];
      //   tmp.pt = tmp.pt + p2.pt - p1.pt;
      //   // 补的点不能超过end_slice
      //   if (PointInVectorSide(end_slice.po, end_slice.pr, tmp.pt) < 0.0) {
      //     break;
      //   }
      //   line_virtual.pts.emplace_back(tmp);
      // }
      // // 补的点不能超过start_slice
      // for (int pt_index = 0; pt_index < line_virtual.pts.size(); pt_index++)
      // {
      //   auto pt = line_virtual.pts[pt_index];
      //   if (PointInVectorSide(start_slice.po, start_slice.pr, pt.pt) > 0.0) {
      //     line_virtual.pts.erase(line_virtual.pts.begin() + pt_index);
      //     pt_index--;
      //   } else {
      //     break;
      //   }
      // }
      // HLOG_ERROR << "line_virtual.pts.size() = " << line_virtual.pts.size();
      // line_virtual.type = LaneType_LANE_VIRTUAL_MARKING;
      // line_virtual.color = WHITE;
      // line_virtual.lanepos = LanePositionType_OTHER;
      // line_virtual.id = (virtual_line_id_++ % 3000);

      LineSegment::Ptr line_virtual_ptr =
          std::make_shared<LineSegment>(line_virtual);
      (*line_virtual_vec).emplace_back(line_virtual_ptr);
    }
  }
}

void VirtualLineGen::SameLineNumVirtualBuild(
    const Group::Ptr& curr_group,
    std::vector<LineSegment::Ptr>* line_virtual_vec,
    const std::vector<float>& line_dis, const RoadEdge::Ptr& road_left,
    const RoadEdge::Ptr& road_right, float lane_width) {
  if (line_dis.size() < 2 || road_left == nullptr || road_right == nullptr) {
    return;
  }
  auto end_slice = curr_group->end_slice;
  Eigen::Vector2f end_po = end_slice.po.head<2>();
  Eigen::Vector2f end_pr = end_slice.pr.head<2>();
  float line_width = (line_dis[0] + line_dis[1]) / 2.0;
  int assume_nums = static_cast<int>(std::max(
      static_cast<float>(std::floor(line_width / lane_width + 0.3)), 1.0F));
  HLOG_ERROR << "assume_nums IS " << assume_nums;
  if (assume_nums > 0) {
    auto base_line = road_left;
    Eigen::Vector3f vec = road_right->points[0] - road_left->points[0];
    if (road_left->points.size() < 2 ||
        (road_right->points.size() > 1 &&
         Dist(road_right->points[0], road_right->points.back()) >
             Dist(road_left->points[0], road_left->points.back()))) {
      base_line = road_right;
      vec = road_left->points[0] - road_right->points[0];
    }
    for (int num = 1; num <= assume_nums; ++num) {
      LineSegment line_virtual;
      Point tmp;
      tmp.type = VIRTUAL;

      HLOG_ERROR << "VEC " << vec.x() << "  " << vec.y();
      tmp.pt = vec / assume_nums * num + base_line->points[0];

      TranslateRoad(&line_virtual, tmp, base_line, curr_group);

      LineSegment::Ptr line_virtual_ptr =
          std::make_shared<LineSegment>(line_virtual);
      (*line_virtual_vec).emplace_back(line_virtual_ptr);
    }
  }
}

std::vector<float> VirtualLineGen::GetTwoBoundayFrontBackDis(
    const LineSegment::Ptr& left_boundary,
    const LineSegment::Ptr& right_boundary) {
  float lane_front_dis = 0.0;
  float lane_end_dis = 0.0;
  LineSegment::Ptr query_line = nullptr;
  LineSegment::Ptr value_line = nullptr;
  if (left_boundary->pts.front().pt.x() < right_boundary->pts.front().pt.x()) {
    query_line = right_boundary;
    value_line = left_boundary;
  } else {
    query_line = left_boundary;
    value_line = right_boundary;
  }

  const auto& query_point = query_line->pts.front().pt;
  auto it =
      std::min_element(value_line->pts.begin(), value_line->pts.end(),
                       [&point = query_point](const Point& a, const Point& b) {
                         return (point - a.pt).norm() < (point - b.pt).norm();
                       });

  if (value_line->pts.size() == 1) {
    lane_front_dis = ((*it).pt - query_point).norm();
  } else if (it == std::prev(value_line->pts.end())) {
    lane_front_dis =
        perpendicular_distance((*std::prev(it)).pt, (*it).pt, query_point);
  } else {
    lane_front_dis =
        perpendicular_distance((*std::next(it)).pt, (*it).pt, query_point);
  }

  const auto& query_end_point = query_line->pts.back().pt;
  auto it_end = std::min_element(
      value_line->pts.begin(), value_line->pts.end(),
      [&point = query_end_point](const Point& a, const Point& b) {
        return (point - a.pt).norm() < (point - b.pt).norm();
      });

  if (value_line->pts.size() == 1) {
    lane_end_dis = ((*it_end).pt - query_end_point).norm();
  } else if (it_end == std::prev(value_line->pts.end())) {
    lane_end_dis = perpendicular_distance((*std::prev(it_end)).pt, (*it_end).pt,
                                          query_end_point);
  } else {
    lane_end_dis = perpendicular_distance((*std::next(it_end)).pt, (*it_end).pt,
                                          query_end_point);
  }

  return {lane_front_dis, lane_end_dis};
}
std::vector<float> VirtualLineGen::GetTwoBoundayFrontBackDis(
    const RoadEdge::Ptr& left_road_edge, const RoadEdge::Ptr& right_road_edge) {
  float lane_front_dis = 0.0;
  float lane_end_dis = 0.0;
  RoadEdge::Ptr query_line = nullptr;
  RoadEdge::Ptr value_line = nullptr;
  if (left_road_edge->points.front().x() <
      right_road_edge->points.front().x()) {
    query_line = right_road_edge;
    value_line = left_road_edge;
  } else {
    query_line = left_road_edge;
    value_line = right_road_edge;
  }

  const auto& query_point = query_line->points.front();
  auto it =
      std::min_element(value_line->points.begin(), value_line->points.end(),
                       [&point = query_point](const Eigen::Vector3f& a,
                                              const Eigen::Vector3f& b) {
                         return (point - a).norm() < (point - b).norm();
                       });

  if (value_line->points.size() == 1) {
    lane_front_dis = ((*it) - query_point).norm();
  } else if (it == std::prev(value_line->points.end())) {
    lane_front_dis =
        perpendicular_distance((*std::prev(it)), (*it), query_point);
  } else {
    lane_front_dis =
        perpendicular_distance((*std::next(it)), (*it), query_point);
  }

  const auto& query_end_point = query_line->points.back();
  auto it_end =
      std::min_element(value_line->points.begin(), value_line->points.end(),
                       [&point = query_end_point](const Eigen::Vector3f& a,
                                                  const Eigen::Vector3f& b) {
                         return (point - a).norm() < (point - b).norm();
                       });

  if (value_line->points.size() == 1) {
    lane_end_dis = ((*it_end) - query_end_point).norm();
  } else if (it_end == std::prev(value_line->points.end())) {
    lane_end_dis = perpendicular_distance((*std::prev(it_end)), (*it_end),
                                          query_end_point);
  } else {
    lane_end_dis = perpendicular_distance((*std::next(it_end)), (*it_end),
                                          query_end_point);
  }

  return {lane_front_dis, lane_end_dis};
}
float VirtualLineGen::GetCurrGroupLaneWidth(const Group::Ptr& curr_grp) {
  if (curr_grp->line_segments.size() > 1) {
    float curr_group_lane_width = GetEntranceLaneWidth(curr_grp->line_segments);
    if (curr_group_lane_width > 0) {
      return curr_group_lane_width;
    }
  }
  return -1.0;  // 负值表示无效
}
std::vector<float> VirtualLineGen::GetRoadedgeLineFrontEndDis(
    const RoadEdge::Ptr& road_edge, const LineSegment::Ptr& boundary) {
  float front_dis = 0.0;
  float end_dis = 0.0;
  const auto& query_point = road_edge->points.front();
  auto it =
      std::min_element(boundary->pts.begin(), boundary->pts.end(),
                       [&point = query_point](const Point& a, const Point& b) {
                         return (point - a.pt).norm() < (point - b.pt).norm();
                       });

  if (boundary->pts.size() == 1) {
    front_dis = ((*it).pt - query_point).norm();
  } else if (it == std::prev(boundary->pts.end())) {
    front_dis =
        perpendicular_distance((*std::prev(it)).pt, (*it).pt, query_point);
  } else {
    front_dis =
        perpendicular_distance((*std::next(it)).pt, (*it).pt, query_point);
  }

  const auto& query_end_point = road_edge->points.back();
  auto it_end = std::min_element(
      boundary->pts.begin(), boundary->pts.end(),
      [&point = query_end_point](const Point& a, const Point& b) {
        return (point - a.pt).norm() < (point - b.pt).norm();
      });

  if (boundary->pts.size() == 1) {
    end_dis = ((*it_end).pt - query_end_point).norm();
  } else if (it_end == std::prev(boundary->pts.end())) {
    end_dis = perpendicular_distance((*std::prev(it_end)).pt, (*it_end).pt,
                                     query_end_point);
  } else {
    end_dis = perpendicular_distance((*std::next(it_end)).pt, (*it_end).pt,
                                     query_end_point);
  }
  return {front_dis, end_dis};
}

float VirtualLineGen::GetTwoBoundayDis(const LineSegment::Ptr& left_boundary,
                                       const LineSegment::Ptr& right_boundary) {
  float lane_space = 3.5;
  LineSegment::Ptr query_line = nullptr;
  LineSegment::Ptr value_line = nullptr;
  if (left_boundary->pts.front().pt.x() < right_boundary->pts.front().pt.x()) {
    query_line = right_boundary;
    value_line = left_boundary;
  } else {
    query_line = left_boundary;
    value_line = right_boundary;
  }

  const auto& query_point = query_line->pts.front().pt;
  auto it =
      std::min_element(value_line->pts.begin(), value_line->pts.end(),
                       [&point = query_point](const Point& a, const Point& b) {
                         return (point - a.pt).norm() < (point - b.pt).norm();
                       });

  if (value_line->pts.size() == 1) {
    lane_space = ((*it).pt - query_point).norm();
  } else if (it == std::prev(value_line->pts.end())) {
    lane_space =
        perpendicular_distance((*std::prev(it)).pt, (*it).pt, query_point);
  } else {
    lane_space =
        perpendicular_distance((*std::next(it)).pt, (*it).pt, query_point);
  }
  return lane_space;
}

float VirtualLineGen::GetTwoBoundayDis(const RoadEdge::Ptr& left_boundary,
                                       const RoadEdge::Ptr& right_boundary) {
  float lane_space = 3.5;
  RoadEdge::Ptr query_line = nullptr;
  RoadEdge::Ptr value_line = nullptr;
  if (left_boundary->points.front().x() < right_boundary->points.front().x()) {
    query_line = right_boundary;
    value_line = left_boundary;
  } else {
    query_line = left_boundary;
    value_line = right_boundary;
  }

  const auto& query_point = query_line->points.front();
  auto it =
      std::min_element(value_line->points.begin(), value_line->points.end(),
                       [&point = query_point](const Eigen::Vector3f& a,
                                              const Eigen::Vector3f& b) {
                         return (point - a).norm() < (point - b).norm();
                       });

  if (value_line->points.size() == 1) {
    lane_space = ((*it) - query_point).norm();
  } else if (it == std::prev(value_line->points.end())) {
    lane_space = perpendicular_distance((*std::prev(it)), (*it), query_point);
  } else {
    lane_space = perpendicular_distance((*std::next(it)), (*it), query_point);
  }
  return lane_space;
}

float VirtualLineGen::perpendicular_distance(const Eigen::Vector3f& A,
                                             const Eigen::Vector3f& B,
                                             const Eigen::Vector3f& P) {
  // 向量 AB
  Eigen::Vector3f AB = B - A;
  // 向量 AP
  Eigen::Vector3f AP = P - A;
  // 叉乘 AB x AP
  Eigen::Vector3f crossProduct = AB.cross(AP);
  // AB 的模长
  float lengthAB = AB.norm();
  // 垂线距离
  float distance = crossProduct.norm() / (lengthAB + 0.001);

  return distance;
}

float VirtualLineGen::GetEntranceLaneWidth(
    const std::vector<LineSegment::Ptr>& bev_lanelines) {
  float averge_entrance_lane_width = averge_exit_lane_width_;
  if (bev_lanelines.size() < 2) {
    return -1.0;  // 负值表示无效
  }

  float front_lane_width_totally = 0.0;
  int front_lane_num_calu = 0;
  for (int i = 0; i < static_cast<int>(bev_lanelines.size()) - 1; ++i) {
    if (bev_lanelines.at(i)->pts.empty() ||
        bev_lanelines.at(i + 1)->pts.empty()) {
      continue;
    }
    float lane_space =
        GetTwoBoundayDis(bev_lanelines.at(i), bev_lanelines.at(i + 1));

    if (lane_space < conf_.max_lane_width &&
        lane_space > conf_.min_lane_width) {
      front_lane_width_totally += lane_space;
      ++front_lane_num_calu;
    }
  }

  if (front_lane_num_calu != 0) {
    averge_entrance_lane_width =
        front_lane_width_totally / static_cast<float>(front_lane_num_calu);
  }
  if (averge_entrance_lane_width < conf_.max_lane_width &&
      averge_entrance_lane_width > conf_.min_lane_width) {
    return averge_entrance_lane_width;
  }

  return -1.0;  // 负值表示无效
}

void VirtualLineGen::ExtendLineToAlignSlice(std::vector<Group::Ptr>* groups) {
  for (auto& grp : *groups) {
    auto start_slice = grp->start_slice;
    auto end_slice = grp->end_slice;
    Eigen::Vector2f start_po = start_slice.po.head<2>();
    Eigen::Vector2f start_pr = start_slice.pr.head<2>();
    Eigen::Vector2f end_po = end_slice.po.head<2>();
    Eigen::Vector2f end_pr = end_slice.pr.head<2>();
    for (auto& lane : grp->lanes) {
      if (!lane->left_boundary->pts.empty()) {
        auto left_front = lane->left_boundary->pts[0].pt;

        if (PointInVectorSide(start_po, start_pr,
                              Eigen::Vector2f(left_front.head<2>())) < 0.0) {
          int index = 1;
          while (index < lane->left_boundary->pts.size() &&
                 Dist(lane->left_boundary->pts[index].pt, left_front) < 2.0) {
            index++;
          }
          if (index >= lane->left_boundary->pts.size()) {
            continue;
          }
          auto left_second = lane->left_boundary->pts[index].pt;
          auto unit = (left_second - left_front).normalized();
          while (PointToVectorDist(start_slice.po, start_slice.pr,
                                   lane->left_boundary->pts[0].pt) > 1 &&
                 PointInVectorSide(
                     start_po, start_pr,
                     Eigen::Vector2f(
                         lane->left_boundary->pts[0].pt.head<2>())) < 0.0) {
            auto start_point = lane->left_boundary->pts[0].pt;
            Point t(VIRTUAL, start_point.x() - unit.x(),
                    start_point.y() - unit.y(), static_cast<float>(0.0));
            lane->left_boundary->pts.insert(lane->left_boundary->pts.begin(),
                                            t);
          }
        }
        auto left_last = lane->left_boundary->pts.back().pt;
        if (PointInVectorSide(end_po, end_pr,
                              Eigen::Vector2f(left_last.head<2>())) > 0.0) {
          int index = lane->left_boundary->pts.size() - 2;
          while (index >= 0 &&
                 Dist(lane->left_boundary->pts[index].pt, left_last) < 2.0) {
            index--;
          }
          if (index < 0) {
            continue;
          }
          auto left_second_last = lane->left_boundary->pts[index].pt;
          auto unit = (left_second_last - left_last).normalized();
          while (PointToVectorDist(end_slice.po, end_slice.pr,
                                   lane->left_boundary->pts.back().pt) > 1 &&
                 PointInVectorSide(
                     end_po, end_pr,
                     Eigen::Vector2f(
                         lane->left_boundary->pts.back().pt.head<2>())) > 0.0) {
            auto end_point = lane->left_boundary->pts.back().pt;
            Point t(VIRTUAL, end_point.x() - unit.x(), end_point.y() - unit.y(),
                    static_cast<float>(0.0));
            lane->left_boundary->pts.emplace_back(t);
          }
        }
      }

      if (!lane->right_boundary->pts.empty()) {
        auto left_front = lane->right_boundary->pts[0].pt;
        if (PointInVectorSide(start_po, start_pr,
                              Eigen::Vector2f(left_front.head<2>())) < 0.0) {
          int index = 1;
          while (index < lane->right_boundary->pts.size() &&
                 Dist(lane->right_boundary->pts[index].pt, left_front) < 2.0) {
            index++;
          }
          if (index >= lane->right_boundary->pts.size()) {
            continue;
          }
          auto left_second = lane->right_boundary->pts[index].pt;
          auto unit = (left_second - left_front).normalized();
          while (PointToVectorDist(start_slice.po, start_slice.pr,
                                   lane->right_boundary->pts[0].pt) > 1 &&
                 PointInVectorSide(
                     start_po, start_pr,
                     Eigen::Vector2f(
                         lane->right_boundary->pts[0].pt.head<2>())) < 0.0) {
            auto start_point = lane->right_boundary->pts[0].pt;
            Point t(VIRTUAL, start_point.x() - unit.x(),
                    start_point.y() - unit.y(), static_cast<float>(0.0));
            lane->right_boundary->pts.insert(lane->right_boundary->pts.begin(),
                                             t);
          }
        }
        auto left_last = lane->right_boundary->pts.back().pt;
        if (PointInVectorSide(end_po, end_pr,
                              Eigen::Vector2f(left_last.head<2>())) > 0.0) {
          int index = lane->right_boundary->pts.size() - 2;
          while (index >= 0 &&
                 Dist(lane->right_boundary->pts[index].pt, left_last) < 2.0) {
            index--;
          }
          if (index < 0) {
            continue;
          }
          auto left_second_last = lane->right_boundary->pts[index].pt;
          auto unit = (left_second_last - left_last).normalized();
          while (PointToVectorDist(end_slice.po, end_slice.pr,
                                   lane->right_boundary->pts.back().pt) > 1 &&
                 PointInVectorSide(
                     end_po, end_pr,
                     Eigen::Vector2f(
                         lane->right_boundary->pts.back().pt.head<2>())) >
                     0.0) {
            auto end_point = lane->right_boundary->pts.back().pt;
            Point t(VIRTUAL, end_point.x() - unit.x(), end_point.y() - unit.y(),
                    static_cast<float>(0.0));
            lane->right_boundary->pts.emplace_back(t);
          }
        }
      }
    }
  }
}
float VirtualLineGen::DistPointNew(const Point& ref_point,
                                   const LineSegment& lineSegment) {
  if (std::isnan(ref_point.pt.x()) || std::isnan(ref_point.pt.y())) {
    return 0.0;
  }

  // 找到最近的点
  auto it = std::min_element(lineSegment.pts.begin(), lineSegment.pts.end(),
                             [&ref_point](const Point& a, const Point& b) {
                               return (ref_point.pt - a.pt).norm() <
                                      (ref_point.pt - b.pt).norm();
                             });
  float lane_front_dis = 0.;
  if (lineSegment.pts.size() == 1) {
    lane_front_dis = ((*it).pt - ref_point.pt).norm();
  } else if (it == std::prev(lineSegment.pts.end())) {
    lane_front_dis =
        perpendicular_distance((*std::prev(it)).pt, (*it).pt, ref_point.pt);
  } else {
    lane_front_dis =
        perpendicular_distance((*std::next(it)).pt, (*it).pt, ref_point.pt);
  }
  return lane_front_dis;
}
void VirtualLineGen::ComputeCenterPoints(Lane::Ptr lane) {
  auto main_ptr = lane->left_boundary;
  auto less_ptr = lane->right_boundary;
  if (lane->left_boundary->pts.size() <= lane->right_boundary->pts.size()) {
    main_ptr = lane->right_boundary;
    less_ptr = lane->left_boundary;
  }
  const auto& main_pts = main_ptr->pts;
  const auto& less_pts = less_ptr->pts;
  size_t less_pts_start_idx = 0;
  for (size_t i = 0; i < main_pts.size(); i++) {
    const auto& main_pt = main_pts[i];

    size_t nearest_idx = 0;
    float nearest_dis = FLT_MAX;
    for (size_t j = less_pts_start_idx; j < less_pts.size(); j++) {
      float dis = (main_pt.pt - less_pts[j].pt).norm();
      if (dis < nearest_dis) {
        nearest_dis = dis;
        nearest_idx = j;
      }
    }
    less_pts_start_idx = nearest_idx;
    const auto& less_pt = less_pts[nearest_idx];
    auto center_pt = (main_pt.pt + less_pt.pt) / 2;
    Point ct_pt(RAW, static_cast<float>(center_pt.x()),
                static_cast<float>(center_pt.y()),
                static_cast<float>(center_pt.z()));
    lane->center_line_pts.emplace_back(ct_pt);
  }
  if (lane->center_line_pts.size() > 3) {
    lane->center_line_param = math::FitLaneline(lane->center_line_pts);
    lane->center_line_param_front =
        math::FitLanelinefront(lane->center_line_pts);
  }
}

void VirtualLineGen::GenGroupAllLanes(const Group::Ptr& grp) {
  // 生成Lane
  grp->lanes.clear();
  const auto line_seg_num = grp->line_segments.size();
  if (line_seg_num > 1) {
    // 按边线距离生成Lane
    for (size_t i = 0; i < line_seg_num - 1; ++i) {
      auto& left_line = grp->line_segments.at(i);
      auto& right_line = grp->line_segments.at(i + 1);
      int l_size = static_cast<int>(left_line->pts.size());
      if (l_size < 2 || right_line->pts.size() < 2) {
        continue;
      }
      auto& left_distp = left_line->pts[static_cast<int>(l_size * 0.5)];
      auto right_center = right_line->center;
      //! 商汤切分点情况下，由于center点是用首尾两点求中值，如果在弯道，该点会在车道外，导致计算距离错误
      auto dist = DistPointNew(left_distp, *right_line);
      HLOG_INFO << "dist: " << dist;
      if (dist < conf_.min_lane_width) {
        continue;
      }

      if (dist > conf_.max_lane_width) {
        continue;
      }
      auto lane = std::make_shared<Lane>();
      lane->left_boundary = left_line;
      lane->right_boundary = right_line;
      lane->str_id = std::to_string(lane->left_boundary->id) + "_" +
                     std::to_string(lane->right_boundary->id);
      lane->lanepos_id = std::to_string(lane->left_boundary->lanepos) + "_" +
                         std::to_string(lane->right_boundary->lanepos);
      lane->str_id_with_group = grp->str_id + ":" + lane->str_id;
      ComputeCenterPoints(lane);
      grp->lanes.emplace_back(lane);
    }
  }
}
void VirtualLineGen::Clear() {
  groups_lane_name_.clear();
  groups_lines_.clear();
}
}  // namespace mf
}  // namespace mp
}  // namespace hozon
