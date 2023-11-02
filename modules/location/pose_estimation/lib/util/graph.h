/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： graph.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#include <string>

#include "modules/location/pose_estimation/lib/perception/perception_lane_line.h"

namespace hozon {
namespace mp {
namespace loc {

// undirected graph, represent the connection relationship between lane
// lines, providing DFS function
class Graph {
 public:
  Graph(const std::unordered_map<std::string, hozon::mp::loc::BoundaryLine>
            &boundary_lines,
        const std::list<std::list<std::shared_ptr<PerceptionLaneLine>>>
            &percep_lanes,
        const SE3 &T_V_W)
      : boundary_lines_(boundary_lines),
        percep_lanes_(percep_lanes),
        T_V_W_(T_V_W) {}

  void AddEdge(std::string v, std::string w) {
    adj_[v].push_back(w);
    adj_[w].push_back(v);
    visited_[v] = false;
    visited_[w] = false;
  }

  void AddNode(std::string v) {
    if (adj_.find(v) == adj_.end()) {
      adj_[v] = std::list<std::string>();
    }
    visited_[v] = false;
  }

  // note that bfs and dfs here are mutual exclusion.
  void BFS(std::string pivot, std::vector<std::string> *ordered_cluster) {
    if (!ordered_cluster) {
      return;
    }
    if (visited_[pivot]) {
      return;
    }
    int loop_cnt = 0;
    double most_left = std::numeric_limits<double>::max();
    double most_right = std::numeric_limits<double>::min();
    double cur_left, cur_right, best_left, best_right;
    bool has_branch = false;
    std::queue<std::string> que;
    MostXValue(pivot, &most_left, &most_right);
    que.push(pivot);
    visited_[pivot] = true;
    while (!que.empty()) {
      std::string temp_idx = que.front();
      que.pop();
      (*ordered_cluster).emplace_back(temp_idx);
      hozon::mp::loc::LaneLinePerceptionPtr best_percep_lane;
      if (!FindBestPercepLane(temp_idx, &best_percep_lane)) {
        continue;
      }
      double min_dist = std::numeric_limits<double>::max(), m_p_dist;
      std::string best_successor = "";
      bool find_successor = false;
      for (auto iter = adj_[temp_idx].begin(); iter != adj_[temp_idx].end();
           ++iter) {
        loop_cnt++;
        if (visited_[*iter]) {
          continue;
        }
        if (DistPercepMap(*iter, best_percep_lane, &m_p_dist)) {
          // if has big intersection with all serached mLans along X axis
          MostXValue(*iter, &cur_left, &cur_right);
          if (BigIOU(cur_left, cur_right, most_left, most_right)) {
            continue;
          }

          if (min_dist > m_p_dist) {
            min_dist = m_p_dist;
            best_successor = *iter;
            find_successor = true;
            best_left = cur_left;
            best_right = cur_right;
          }
        }
      }
      if (find_successor) {
        que.push(best_successor);
        visited_[best_successor] = true;
        most_left = std::min(most_left, best_left);
        most_right = std::max(most_right, best_right);
      }
      // prevent dead-loop
      if (loop_cnt > 10000) {
        HLOG_ERROR << "Bfs loop exceed maximum";
        break;
      }
    }
  }

  void GetNodes(std::vector<std::string> *nodes) {
    if (nodes == nullptr) {
      return;
    }
    (*nodes).clear();
    for (auto iter = visited_.begin(); iter != visited_.end(); ++iter) {
      (*nodes).push_back(iter->first);
    }
  }

 private:
  std::map<std::string, bool> visited_;
  std::map<std::string, std::list<std::string>> adj_;
  std::unordered_map<std::string, hozon::mp::loc::BoundaryLine> boundary_lines_;
  const std::list<std::list<std::shared_ptr<PerceptionLaneLine>>>
      &percep_lanes_;
  const SE3 &T_V_W_;

  bool FindBestPercepLane(std::string m_idx,
                          hozon::mp::loc::LaneLinePerceptionPtr *best_lane) {
    double best_dist = std::numeric_limits<double>::max();
    for (auto &lane_list : percep_lanes_) {
      for (auto &p_lane : lane_list) {
        double mean_dist, sum_dist = 0;
        int cnt = 0;
        for (auto &cpt : boundary_lines_[m_idx].control_point) {
          auto p_v = T_V_W_ * cpt.point;
          if (p_lane->IsIn(p_v.x())) {
            sum_dist += fabs(p_lane->Y(p_v.x()) - p_v.y());
            cnt += 1;
          }
        }
        if (cnt >= 0) {
          mean_dist = sum_dist / cnt;
          if (best_dist > mean_dist) {
            best_dist = mean_dist;
            *best_lane = p_lane;
          }
        }
      }
    }
    if (best_dist < 2) {
      return true;
    }
    return false;
  }

  bool DistPercepMap(const std::string &m_idx,
                     const hozon::mp::loc::LaneLinePerceptionPtr &p_lane,
                     double *m_p_dist) {
    double sum_dist = 0;
    int cnt = 0;
    for (auto &cpt : boundary_lines_[m_idx].control_point) {
      auto p_v = T_V_W_ * cpt.point;
      if (p_lane->IsIn(p_v.x())) {
        sum_dist += fabs(p_lane->Y(p_v.x()) - p_v.y());
        cnt += 1;
      }
    }
    if (cnt > 0) {
      (*m_p_dist) = sum_dist / cnt;
      return true;
    }
    return false;
  }

  // get most left & right value on X axis
  void MostXValue(const std::string &mLanId, double *most_left,
                  double *most_right) {
    if (most_left == nullptr || most_right == nullptr) {
      return;
    }
    auto compX = [&](ControlPoint p1, ControlPoint p2) {
      return (T_V_W_ * p1.point).x() < (T_V_W_ * p2.point).x();
    };

    auto left_elem =
        std::min_element(boundary_lines_[mLanId].control_point.begin(),
                         boundary_lines_[mLanId].control_point.end(), compX);
    auto right_elem =
        std::max_element(boundary_lines_[mLanId].control_point.begin(),
                         boundary_lines_[mLanId].control_point.end(), compX);

    *most_left = (T_V_W_ * left_elem->point).x();
    *most_right = (T_V_W_ * right_elem->point).x();
  }

  // if intersect along X axis
  bool BigIOU(const double &left_1, const double &right_1, const double &left_2,
              const double &right_2) {
    double seg1_len = right_1 - left_1;
    double seg2_len = right_2 - left_2;
    double union_len = std::max(right_1, right_2) - std::min(left_1, left_2);
    // valid check
    if (seg1_len <= 0 || seg2_len < 0) {
      return false;
    }
    // cal IOU
    if ((seg1_len + seg2_len) - union_len > 1) {
      return true;
    }
    return false;
  }
};

class GraphPure {
 public:
  GraphPure() {}

  void AddEdge(int v, int w) {
    adj_[v].push_back(w);
    adj_[w].push_back(v);
    visited_[v] = false;
    visited_[w] = false;
  }

  void AddNode(int v) { visited_[v] = false; }

  void BFS(int pivot, std::vector<int> *ordered_cluster) {
    if (visited_[pivot]) {
      return;
    }
    int loop_cnt = 0;
    std::queue<int> que;
    que.push(pivot);
    visited_[pivot] = true;
    while (!que.empty()) {
      int temp_idx = que.front();
      que.pop();
      (*ordered_cluster).emplace_back(temp_idx);
      for (auto iter = adj_[temp_idx].begin(); iter != adj_[temp_idx].end();
           iter++) {
        loop_cnt++;
        if (visited_[*iter]) {
          continue;
        }
        que.push(*iter);
        visited_[*iter] = true;
      }
      // prevent dead-loop
      if (loop_cnt > 10000) {
        HLOG_ERROR << "BfsPure loop exceed maximum";
        break;
      }
    }
  }

  void GetNodes(std::vector<int> *nodes) {
    if (nodes == nullptr) {
      return;
    }
    (*nodes).clear();
    for (auto iter = visited_.begin(); iter != visited_.end(); ++iter) {
      (*nodes).push_back(iter->first);
    }
  }

 private:
  std::map<int, bool> visited_;
  std::map<int, std::list<int>> adj_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
