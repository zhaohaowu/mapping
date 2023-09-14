/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_prediction.cc
 *   author     ： zhangshuo
 *   date       ： 2023.09
 ******************************************************************************/

#include "map_fusion/map_prediction/map_prediction.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <numeric>

#include "modules/util/include/util/geo.h"
// #include "util/log.h"

#include "util/rviz_agent/rviz_agent.h"
#include "util/temp_log.h"

namespace hozon {
namespace mp {
namespace mf {

int MapPrediction::Init(const std::string& conf) {
  running_.store(true);
  proc_th_ = std::make_shared<std::thread>(&MapPrediction::Prov, this);
  return 0;
}

void MapPrediction::Term() {
  running_.store(false);

  if (proc_th_ && proc_th_->joinable()) {
    proc_th_->join();
  }
}

void MapPrediction::OnInsNodeInfo(
    const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  local_enu_center_ << msg->pos_gcj02().x(), msg->pos_gcj02().y(),
      msg->pos_gcj02().z();
}

void MapPrediction::OnHqMap(
    const std::shared_ptr<const hozon::hdmap::Map>& hqMap) {
  std::lock_guard<std::mutex> lock(mtx_);
  hqMapRoadEdge.clear();
  if (hqMap->road().empty()) {
    return;
  }
  // 存储所有的HQ道路边界线
  for (const auto& hq_road : hqMap->road()) {
    for (const auto& hq_road_section : hq_road.section()) {
      for (const auto& hq_road_section_line :
           hq_road_section.boundary().outer_polygon().edge()) {
        std::vector<Eigen::Vector3d> edgePoint;
        for (const auto& edge : hq_road_section_line.curve().segment()) {
          for (const auto& point : edge.line_segment().point()) {
            Eigen::Vector3d point_(point.x(), point.y(), point.z());
            Eigen::Vector3d point_enu =
                util::Geo::Gcj02ToEnu(point_, local_enu_center_);
            edgePoint.emplace_back(point_enu);
          }
        }
        hqMapRoadEdge.emplace_back(std::make_pair(id, edgePoint));
        id -= 1;
      }
    }
  }
}

void MapPrediction::OnLocalMap(const std::shared_ptr<LocalMap>& msg) {
  // 接收消息
  std::lock_guard<std::mutex> lock(mtx_);
  localMapLaneLines.clear();
  if (msg->lanes().empty()) {
    return;
  }

  for (const auto& line : msg->lanes()) {
    std::vector<Eigen::Vector3d> linePoints;
    for (const auto& point : line.points()) {
      Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
      linePoints.emplace_back(point_enu);
    }
    localMapLaneLines.emplace_back(std::make_pair(line.track_id(), linePoints));
  }

  if (flag) {
    int ret = Init("O");
    flag = false;
  }
}

// void MapPrediction::OnLocalMap(
//     const std::shared_ptr<const hozon::hdmap::Map>& localMap) {
//   std::lock_guard<std::mutex> lock(mtx_);
//   localMapLaneLines.clear();
//   if (localMap->lane().empty()) {
//     return;
//   }
//   // 存储所有的车道线并赋予ID
//   for (const auto& local_lane : localMap->lane()) {
//     // 存储车道左边线
//     std::vector<Eigen::Vector3d> leftLine;
//     for (const auto& left_line :
//     local_lane.left_boundary().curve().segment()) {
//       for (const auto& point : left_line.line_segment().point()) {
//         Eigen::Vector3d point_(point.x(), point.y(), point.z());
//         leftLine.emplace_back(point_);
//       }
//     }
//     localMapLaneLines.emplace_back(std::make_pair(id, leftLine));
//     id -= 1;

//     // 存储车道右边线
//     std::vector<Eigen::Vector3d> rightLine;
//     for (const auto& right_line :
//          local_lane.right_boundary().curve().segment()) {
//       for (const auto& point : right_line.line_segment().point()) {
//         Eigen::Vector3d point_(point.x(), point.y(), point.z());
//         Eigen::Vector3d point_enu =
//             util::Geo::Gcj02ToEnu(point_, local_enu_center_);
//         rightLine.emplace_back(point_enu);
//       }
//     }
//     localMapLaneLines.emplace_back(std::make_pair(id, rightLine));
//     id -= 1;
//   }
// }

void MapPrediction::PredictLeftRightLaneline(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        hqMapRoadEdge,
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        localMapLaneLines) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (hqMapRoadEdge.empty() || localMapLaneLines.empty()) {
    return;
  }
  // 求解每根HQ道路边界线到每根localMap车道线的距离来判断关联对和缺失的车道数量
  std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>> disEdgeToLine;
  ComputeDisLineToEdge(hqMapRoadEdge, localMapLaneLines, disEdgeToLine);

  // 根据距离来判断缺失的车道线数量
  int numMissLines = 0;
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>> addLaneLines;

  for (const auto& it : disEdgeToLine) {
    numMissLines = uint32_t(it.second / 3);
    if (numMissLines == 0) {
      continue;
    }
    std::vector<std::vector<Eigen::Vector3d>> predictLine(numMissLines);
    // 查找每个车道边线所对应的道路边沿线,然后开始填充点
    for (const auto& line : localMapLaneLines) {
      for (const auto& edge : hqMapRoadEdge) {
        if (line.first == it.first.second && edge.first == it.first.first) {
          // 开始增补车缺失的车道线或者车道边沿线
          AddLeftRightLine(it, line, edge, &predictLine);
          break;
        }
      }
    }

    for (const auto& it : predictLine) {
      addLaneLines.emplace_back(std::make_pair(id, it));
      id -= 1;  // 赋予ID
    }
  }

  for (const auto& line : addLaneLines) {
    localMapLaneLines.emplace_back(std::make_pair(line.first, line.second));
  }

  // 使用RVIZ对其进行可视化操作

  // 拟合车道中心线(当前拟合车道中心线的方法有点问题)
  // std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
  //     allCenterLaneLines;
  // FitLaneCenterline(completeLaneLines, allCenterLaneLines);
}

void MapPrediction::ComputeDisLineToEdge(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        hqMapRoadEdge,
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        localMapLaneLines,
    std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>>&
        disLineToEdge) {
  // 用于计算每个HQ道路边界与localMap车道线的平均距离，保存距离最小的车道线关联对，并保存距离
  if (hqMapRoadEdge.empty() && localMapLaneLines.empty()) {
    return;
  }
  // 计算每个HQ道路边界与localMap车道线的距离，并求得最小距离
  for (const auto& edge : hqMapRoadEdge) {
    double min_distance = std::numeric_limits<double>::max();
    uint32_t line_id;
    for (const auto& laneline : localMapLaneLines) {
      std::vector<double> minDistance;  // 存储最小距离
      for (const auto& point_edge : edge.second) {
        for (int i = 1; i < laneline.second.size(); ++i) {
          Eigen::Vector3d A = laneline.second[i - 1];
          Eigen::Vector3d B = laneline.second[i];
          Eigen::Vector3d P = point_edge;
          // 计算A点和B点的方向向量
          Eigen::Vector3d AB = B - A;
          Eigen::Vector3d AP = P - A;
          double ABLengthSquared = AB.squaredNorm();
          double t = AB.dot(AP) / ABLengthSquared;
          if (t < 0 || t > 1) {
            continue;
          }
          t = std::max(0.0, std::min(t, 1.0));
          Eigen::Vector3d C = A + t * AB;    // 点到线段的最近点
          double distance = (P - C).norm();  // 点到线段的距离
          minDistance.emplace_back(distance);
          break;
        }
      }
      // 当前HQ车道边界线到localMap车道线的平均距离
      if (minDistance.empty()) {
        continue;
      }
      double sum = std::accumulate(minDistance.begin(), minDistance.end(), 0.0);
      double mean_distance = sum / minDistance.size();
      if (mean_distance < min_distance) {
        min_distance = mean_distance;
        line_id = laneline.first;
      }
    }
    // 存储其HQ车道边界线的关联对及其距离
    std::pair<uint32_t, uint32_t> id_pair = std::make_pair(edge.first, line_id);
    disLineToEdge.emplace_back(std::make_pair(id_pair, min_distance));
  }
}

void MapPrediction::AddLeftRightLine(
    const std::pair<std::pair<uint32_t, uint32_t>, double>& LineToEdge,
    const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& line,
    const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& edge,
    std::vector<std::vector<Eigen::Vector3d>>* predictLine) {
  // 传过来的是单侧的信息
  uint32_t num_miss_line = uint32_t(LineToEdge.second / 3);

  if (num_miss_line != 0) {
    for (const auto& line_point : line.second) {
      for (int i = 1; i < edge.second.size(); ++i) {
        Eigen::Vector3d A = edge.second[i - 1];
        Eigen::Vector3d B = edge.second[i];
        Eigen::Vector3d P = line_point;
        // 计算A点和B点的方向向量
        Eigen::Vector3d AB = B - A;
        Eigen::Vector3d AP = P - A;
        // 计算AB模长
        double ABLengthSquared = AB.squaredNorm();
        double t = AB.dot(AP) / ABLengthSquared;

        if (t < 0 || t > 1) {
          continue;
        }

        t = std::max(0.0, std::min(t, 1.0));
        Eigen::Vector3d C = A + t * AB;   // 点到线段的最近点
        predictLine->at(0).push_back(C);  // 先存储边界节点
        for (int j = 1; j < num_miss_line; ++j) {
          predictLine->at(j).push_back((line_point + C) / (j + 1));
        }
        break;
      }
    }
  }
}

void MapPrediction::PredictAheadLaneLine(
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        localMapLaneLines,
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        hqMapRoadEdge,
    const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& location) {
  // 用于预测前方车道线150米
  if (localMapLaneLines.empty() || hqMapRoadEdge.empty()) {
    return;
  }
  Eigen::Vector3d pos_gcj(location->pos_gcj02().x(), location->pos_gcj02().y(),
                          location->pos_gcj02().z());
  Eigen::Quaterniond orient(
      location->quaternion().w(), location->quaternion().x(),
      location->quaternion().y(), location->quaternion().z());
  Eigen::Vector3d pos_enu = util::Geo::Gcj02ToEnu(pos_gcj, local_enu_center_);
  Eigen::Matrix3d orient_ = orient.toRotationMatrix();

  // 这里只取横向距离的最大值
  std::vector<double> endX;
  for (const auto& line : localMapLaneLines) {
    if (line.second.empty()) {
      continue;
    }
    Eigen::Vector3d point = orient_.inverse() * (line.second.back() - pos_enu);
    endX.emplace_back(point.x());
  }
  std::sort(endX.begin(), endX.end());
  double max_x = endX[endX.size() - 1];

  // 根据max_x开始对HQ进行裁剪(150米)
  // 确定每根线末端点的最小的x值，根据最小值进行裁剪(150米)
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>> allEdges;
  for (const auto& edge : hqMapRoadEdge) {
    std::vector<Eigen::Vector3d> edgePoints;
    for (const auto& point_enu : edge.second) {
      // enu系转到vehicle系下
      Eigen::Vector3d point_vehicle = orient_.inverse() * (point_enu - pos_enu);
      if (point_vehicle.x() >= max_x && (point_vehicle.x() - max_x) <= 200.0) {
        // 重新将坐标转到enu坐标系下
        Eigen::Vector3d point_enu_ = orient_ * point_vehicle + pos_enu;
        edgePoints.emplace_back(point_enu_);
      }
    }
    if (!edgePoints.empty()) {
      allEdges.emplace_back(std::make_pair(edge.first, edgePoints));
    }
  }

  // 接下来通过地图中的道路边界来拟合预测的车道线信息
  std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>> boundaryPairs;
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
      predictLaneLines;
  if (allEdges.empty()) {
    return;
  }

  // 确定对应的道路边界线关联对
  DetermineEdgeAssPair(
      allEdges, boundaryPairs);  // 这里得到的boundaryPairs中的关联对可能有重复的
  // 根据关联对的信息开始拟合需要预测的车道线信息
  FitAheadLaneLine(boundaryPairs, allEdges, predictLaneLines);
}

void MapPrediction::DetermineEdgeAssPair(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        allEdges,
    std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>>&
        boundaryPairs) {
  // 求取车道之间的关联对
  for (const auto& edge : allEdges) {
    double min_distance = std::numeric_limits<double>::max();
    double ass_id;
    for (int i = 0; i < allEdges.size(); ++i) {
      if (allEdges[i].first == edge.first) {
        continue;
      }
      // 遍历edge中的所有点,并存储每个点到对应点的平均距离
      std::vector<double> distance;
      if (edge.second.size() == 1 && allEdges[i].second.size() == 1 &&
          uint32_t((edge.second.back() - allEdges[i].second.back()).norm() /
                   3.5) == 4) {
        double distance =
            (edge.second.back() - allEdges[i].second.back()).norm();
        std::pair<uint32_t, uint32_t> id_pair =
            std::make_pair(edge.first, allEdges[i].first);
        boundaryPairs.emplace_back(std::make_pair(id_pair, distance));
      }
      for (const auto& point : edge.second) {
        for (int j = 1; j < allEdges[i].second.size(); ++j) {
          Eigen::Vector3d A = allEdges[i].second[j - 1];
          Eigen::Vector3d B = allEdges[i].second[j];
          Eigen::Vector3d P = point;

          // 计算A点和B点的方向向量
          Eigen::Vector3d AB = B - A;
          Eigen::Vector3d AP = P - A;
          // 计算AB模长
          double ABLengthSquared = AB.squaredNorm();
          double t = AB.dot(AP) / ABLengthSquared;

          if (t < 0 || t > 1) {
            continue;
          }

          t = std::max(0.0, std::min(t, 1.0));
          Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点
          double dis = (point - C).norm();
          distance.emplace_back(dis);
          break;
        }
      }
      if (distance.empty()) {
        continue;
      }
      double sum = std::accumulate(distance.begin(), distance.end(), 0.0);
      double mean_distance = sum / distance.size();
      if (mean_distance < min_distance && sum != 0) {
        min_distance = mean_distance;
        ass_id = allEdges[i].first;
      }
    }
    if (min_distance != std::numeric_limits<double>::max()) {
      std::pair<uint32_t, uint32_t> id_pair =
          std::make_pair(edge.first, ass_id);
      boundaryPairs.emplace_back(std::make_pair(id_pair, min_distance));
    }
  }
}

void MapPrediction::FitAheadLaneLine(
    const std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>>&
        boundaryPairs,
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        allEdges,
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        predictLaneLines) {
  struct MyHash {
    std::size_t operator()(
        const std::pair<std::pair<uint32_t, uint32_t>, double>& p) const {
      using std::hash;
      using std::size_t;
      using std::string;

      // 计算哈希值
      return ((hash<uint32_t>()(p.first.first) ^
               (hash<uint32_t>()(p.first.second) << 1)) >>
              1);
    }
  };
  // 对boundaryPairs进行过滤，剔除相等的pair
  std::unordered_set<std::pair<std::pair<uint32_t, uint32_t>, double>, MyHash>
      uniquePairs;
  std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>> filteredPairs;
  for (const auto& pair : boundaryPairs) {
    if (uniquePairs.find(pair) == uniquePairs.end()) {
      uniquePairs.insert(pair);
      filteredPairs.push_back(pair);
    }
  }
  // 开始对预测的车道线
  for (const auto& pair : boundaryPairs) {
    for (const auto& edge_1 : allEdges) {
      for (const auto& edge_2 : allEdges) {
        if (edge_1.first == pair.first.first &&
            edge_2.first == pair.first.second) {
          uint32_t num_lines = uint32_t(pair.second / 3.5);
          std::vector<std::vector<Eigen::Vector3d>> predictLine(num_lines + 1);
          if (edge_1.second.size() == 1 && edge_2.second.size() == 1) {
            double k = (edge_1.second.back().y() - edge_2.second.back().y()) /
                       (edge_1.second.back().x() - edge_2.second.back().x());
            double b = edge_2.second.back().y() - k * edge_2.second.back().x();
            for (double j = 0.0; j < num_lines; ++j) {
              double ratio = j / num_lines;
              double x = ratio * (edge_2.second.back().x() -
                                  edge_1.second.back().x()) +
                         edge_1.second.back().x();
              double y = k * x + b;
              Eigen::Vector3d new_point(x, y, 0);
              predictLine[j].push_back(new_point);
            }
          }
          for (const auto& point : edge_1.second) {
            for (int i = 1; i < edge_2.second.size(); ++i) {
              Eigen::Vector3d A = edge_2.second[i - 1];
              Eigen::Vector3d B = edge_2.second[i];
              Eigen::Vector3d P = point;

              // 计算A点和B点的方向向量
              Eigen::Vector3d AB = B - A;
              Eigen::Vector3d AP = P - A;
              // 计算AB模长
              double ABLengthSquared = AB.squaredNorm();
              double t = AB.dot(AP) / ABLengthSquared;

              if (t < 0 || t > 1) {
                continue;
              }
              t = std::max(0.0, std::min(t, 1.0));
              Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点

              // predictLine[0].push_back(point);
              // predictLine[num_lines].push_back(C);

              double k = (point.y() - C.y()) / (point.x() - C.x());
              double b = C.y() - k * C.x();
              // predictLine[0].push_back(point);
              // predictLine[num_lines].push_back(C);
              for (double j = 0.0; j < num_lines + 1; ++j) {
                double ratio = j / num_lines;
                double x = ratio * (C.x() - point.x()) + point.x();
                double y = k * x + b;
                Eigen::Vector3d new_point(x, y, 0);
                predictLine[j].push_back(new_point);
              }

              break;
            }
          }
          if (predictLine.empty()) {
            continue;
          }
          for (const auto& it : predictLine) {
            predictLaneLines.emplace_back(std::make_pair(id, it));
            id -= 1;
          }
        }
      }
    }
  }
}

void MapPrediction::FitLaneCenterline(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        completeLaneLines,
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        allCenterLaneLines) {
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>> LaneLines =
      completeLaneLines;
  // 使用lambda表达式和std::sort函数对数组进行排序
  std::sort(LaneLines.begin(), LaneLines.end(),
            [](const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& line1,
               const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& line2) {
              return (line1.second.front().x() < line2.second.front().x());
            });
  // 排序之后开始对两两车道线求车道中心线
  for (int i = 1; i < LaneLines.size(); ++i) {
    std::vector<Eigen::Vector3d> centerlinePoints;
    for (const auto& it : LaneLines[i - 1].second) {
      for (int j = 1; j < LaneLines[i].second.size(); ++j) {
        Eigen::Vector3d A = LaneLines[i].second[j - 1];
        Eigen::Vector3d B = LaneLines[i].second[j];
        Eigen::Vector3d P = it;

        // 计算A点和B点的方向向量
        Eigen::Vector3d AB = B - A;
        Eigen::Vector3d AP = P - A;
        // 计算AB模长
        double ABLengthSquared = AB.squaredNorm();
        double t = AB.dot(AP) / ABLengthSquared;

        if (t < 0 || t > 1) {
          continue;
        }

        t = std::max(0.0, std::min(t, 1.0));
        Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点
        centerlinePoints.emplace_back((it + C) /
                                      2);  // 存入最近投影点的之间的投影点
        break;
      }
    }
    allCenterLaneLines.emplace_back(std::make_pair(id, centerlinePoints));
    id -= 1;
  }
}

void MapPrediction::Prov() {
  while (running_.load()) {
    // std::this_thread::sleep_for(100ms);
    // 这里要注意对laneline和map的时间戳是否一致的，这里要着重注意；
    // AnalyzeLaneline(laneline, map, location);
    PredictLeftRightLaneline(hqMapRoadEdge, localMapLaneLines);

    // 预测前方的车道线(大概150)
    PredictAheadLaneLine(localMapLaneLines, hqMapRoadEdge, location);
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
