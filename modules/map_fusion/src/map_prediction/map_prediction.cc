/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_prediction.cc
 *   author     ： zhangshuo
 *   date       ： 2023.09
 ******************************************************************************/

#include "map_fusion/map_prediction/map_prediction.h"
#include <gflags/gflags.h>
#include <pthread.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "boost/thread/exceptions.hpp"
#include "common/time/clock.h"
#include "common/time/time.h"
#include "common/utm_projection/coordinate_convertor.h"
#include "map/hdmap/hdmap.h"
#include "map/hdmap/hdmap_common.h"
#include "map_fusion/map_fusion.h"
#include "modules/util/include/util/geo.h"

// #include "util/log.h"

#include "proto/map/map.pb.h"
#include "proto/map/map_lane.pb.h"
#include "proto/map/map_road.pb.h"
#include "util/rviz_agent/rviz_agent.h"
#include "util/temp_log.h"

DEFINE_bool(pred_run, true, "pred thread run");
DEFINE_uint32(pred_thread_interval, 100, "pred thread interval ms");

namespace hozon {
namespace mp {
namespace mf {

MapPrediction::~MapPrediction() {
  is_pred_proc_stop_ = true;
  pred_proc_.get();
}

int MapPrediction::Init() {
  topo_map_ = std::make_shared<hozon::hdmap::Map>();
  hq_map_server_ = std::make_shared<hozon::hdmap::HDMap>();
  local_msg_ = std::make_shared<hozon::hdmap::Map>();
  location_ = std::make_shared<hozon::localization::HafNodeInfo>();

  is_pred_proc_stop_ = !FLAGS_pred_run;
  pred_proc_ = std::async(&MapPrediction::Prediction, this);
  return 0;
}

void MapPrediction::OnInsNodeInfo(
    const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
  std::lock_guard<std::mutex> lock(mtx_);
  if (local_enu_center_flag_) {
    local_enu_center_ << msg->pos_gcj02().x(), msg->pos_gcj02().y(),
        msg->pos_gcj02().z();
    local_enu_center_flag_ = false;
  }
  location_ = msg;
}

void MapPrediction::OnHqMap(const std::shared_ptr<hozon::hdmap::Map>& hqmap) {
  // std::lock_guard<std::mutex> lock(mtx_);
  hqmap_road_edge.clear();
  if (!hqmap) {
    return;
  }
  int ret = hq_map_server_->LoadMapFromProto(*hqmap);

  if (ret != 0 || hq_map_server_->Empty()) {
    return;
  }

  // PredictAheadLaneLine(localMapLaneLines, hqMapRoadEdge, location);

  // 这里拿到了所有车道的id，可以通过lane id找到其所在的所有road id

  // 假设现在拿到的road只是左边第二个车道的左边界
  /*
  通过车道数量在vehicle道路上进行补充，得到道路的两个边界
  */
  // std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>> left_edge;
  // std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>> right_edge;
  // for (const auto& road : hqMap->road()) {
  //   for (const auto& section : road.section()) {
  //     //
  //     这里开始拿到了一条road(section.boundary().outer_polygon().edge().end())
  //     int lane_num;
  //     std::vector<Eigen::Vector3d> left;
  //     std::vector<Eigen::Vector3d> right;
  //     for (const auto& point : section.boundary()
  //                                  .outer_polygon()
  //                                  .edge()
  //                                  .end()
  //                                  ->curve()
  //                                  .segment()
  //                                  .end()
  //                                  ->line_segment()
  //                                  .point()) {
  //       // 开始遍历所有的点
  //       // 将点转到vehcle坐标系下
  //       Eigen::Vector3d point_left(point.x(), point.y() + 3.5, 0);
  //       Eigen::Vector3d point_right(point.x(), point.y() - 3.5 * (lane_num -
  //       1), 0); left.emplace_back(point_left);
  //       right.emplace_back(point_right);
  //     }
  //     left_edge.emplace_back(std::make_pair(id, left));
  //     right_edge.emplace_back(std::make_pair(id, right));
  //   }
  // }
}

void MapPrediction::OnTopoMap(const std::shared_ptr<hozon::hdmap::Map>& msg) {
  // std::lock_guard<std::mutex> lock(mtx_);
  if (!msg) {
    HLOG_ERROR << "nullptr topo map";
    return;
  }

  localmap_lanelines.clear();
  if (msg->lane().empty()) {
    HLOG_ERROR << "no OnTopoMap info!";
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    local_msg_ = msg;
  }
}

void MapPrediction::PredictLeftRightLaneline(
    const std::shared_ptr<hozon::hdmap::Map>& topo_map_,
    const std::set<std::string>& curr_road_id_,
    const std::vector<hozon::hdmap::RoadInfoConstPtr>& roads) {
  if (!topo_map_ || curr_road_id_.empty() || roads.empty()) {
    return;
  }

  for (const auto& road : roads) {
    for (const auto& it : curr_road_id_) {
      if (road->id().id() == it) {
        for (const auto& road_section : road->sections()) {
          // 存储road_section中的lane_id
          std::vector<std::string> section_id;
          for (const auto& it : road_section.lane_id()) {
            section_id.emplace_back(it.id());
          }
          // 找取road_section中的两个道路边界
          std::vector<std::vector<Eigen::Vector3d>> boundary1;
          std::vector<std::vector<Eigen::Vector3d>> boundary2;
          for (const auto& edge :
               road_section.boundary().outer_polygon().edge()) {
            if (edge.type() == 2) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary1.emplace_back(edge_point);
              }
            }

            if (edge.type() == 3) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary2.emplace_back(edge_point);
              }
            }
          }
          std::vector<Eigen::Vector3d> edge1_;
          for (const auto& bound : boundary1) {
            for (const auto& point : bound) {
              edge1_.emplace_back(point);
            }
          }

          std::vector<Eigen::Vector3d> edge2_;
          for (const auto& bound : boundary2) {
            for (const auto& point : bound) {
              edge2_.emplace_back(point);
            }
          }
          // 以上存储了当前section中的两个道路边界，使用新的策略对时间进行优化
          // 接下来计算每个车道的左边线与boundary1/boundary2道路边界的匹配情况
          std::pair<std::string, std::vector<Eigen::Vector3d>> curr_left_line;
          std::pair<std::string, std::vector<Eigen::Vector3d>> curr_right_line;

          for (const auto& lane : local_msg_->lane()) {
            for (const auto& id : section_id) {
              if (id == lane.id().id() &&
                  lane.left_neighbor_forward_lane_id().empty()) {
                // 记录左边线
                std::vector<Eigen::Vector3d> left_line;
                for (const auto& seg : lane.left_boundary().curve().segment()) {
                  for (const auto& point : seg.line_segment().point()) {
                    // 此时的point是在enu系下
                    Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
                    left_line.emplace_back(point_enu);
                  }
                }
                if (!left_line.empty()) {
                  curr_left_line.first = lane.id().id();
                  curr_left_line.second = left_line;
                }

                // 如果左边线是空，存储右边线
                if (left_line.empty()) {
                  // 记录右边线
                  std::vector<Eigen::Vector3d> right_line;
                  for (const auto& seg :
                       lane.right_boundary().curve().segment()) {
                    for (const auto& point : seg.line_segment().point()) {
                      // 此时的point是在enu系下
                      Eigen::Vector3d point_enu(point.x(), point.y(),
                                                point.z());
                      right_line.emplace_back(point_enu);
                    }
                  }
                  if (!right_line.empty()) {
                    curr_left_line.first = lane.id().id();
                    curr_left_line.second = right_line;
                  }
                }
              }

              if (id == lane.id().id() &&
                  lane.right_neighbor_forward_lane_id().empty()) {
                // 记录右边线
                std::vector<Eigen::Vector3d> right_line;
                for (const auto& seg :
                     lane.right_boundary().curve().segment()) {
                  for (const auto& point : seg.line_segment().point()) {
                    // 此时的point是在enu系下
                    Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
                    right_line.emplace_back(point_enu);
                  }
                }
                if (!right_line.empty()) {
                  curr_right_line.first = lane.id().id();
                  curr_right_line.second = right_line;
                }

                if (right_line.empty()) {
                  // 存储左边线
                  std::vector<Eigen::Vector3d> left_line;
                  for (const auto& seg :
                       lane.left_boundary().curve().segment()) {
                    for (const auto& point : seg.line_segment().point()) {
                      // 此时的point是在enu系下
                      Eigen::Vector3d point_enu(point.x(), point.y(),
                                                point.z());
                      left_line.emplace_back(point_enu);
                    }
                  }
                  if (!left_line.empty()) {
                    curr_right_line.first = lane.id().id();
                    curr_right_line.second = left_line;
                  }
                }
              }
            }
          }
          double left_dist = 0.;
          ComputeDistLineToEdge(curr_left_line, edge1_,
                                &left_dist);  // 边-->线
          AddLeftOrRightLine(edge1_, curr_left_line, left_dist, 0);

          double right_dist = 0.;
          ComputeDistLineToEdge(curr_right_line, edge2_, &right_dist);
          AddLeftOrRightLine(edge2_, curr_right_line, right_dist, 1);
        }
      }
    }
  }
}

void MapPrediction::ComputeDistLineToEdge(
    const std::pair<std::string, std::vector<Eigen::Vector3d>>& curr_line,
    const std::vector<Eigen::Vector3d>& edge, double* dist) {
  // 分别计算每条车道边线到edge的距离
  if (curr_line.second.empty() || edge.empty()) {
    return;
  }

  double min_distance = 0;  // 存储最小距离
  uint32_t count = 0;
  for (const auto& point : curr_line.second) {
    bool flag = false;
    for (size_t i = 1; i < edge.size(); i++) {
      Eigen::Vector3d A = edge[i - 1];
      Eigen::Vector3d B = edge[i];
      Eigen::Vector3d P = point;
      // 计算A点和B点的方向向量
      Eigen::Vector3d AB = B - A;
      Eigen::Vector3d AP = P - A;
      double ABLengthSquared = AB.squaredNorm();
      if (ABLengthSquared == 0) {
        continue;
      }
      double t = AB.dot(AP) / ABLengthSquared;
      if (t < 0 || t > 1) {
        continue;
      }
      t = std::max(0.0, std::min(t, 1.0));
      Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点
      // min_distance = (P - C).norm();   // 点到线段的距离
      min_distance += (P - C).norm();
      count += 1;
      flag = true;
      break;
    }
    // if (flag) {
    //   break;
    // }
  }
  double mean_distance = min_distance / count;
  *dist = mean_distance;
}

void MapPrediction::AddLeftOrRightLine(
    const std::vector<Eigen::Vector3d>& edge,
    const std::pair<std::string, std::vector<Eigen::Vector3d>>& curr_line,
    const double& dist, const uint32_t& record) {
  // 开始拟合车道线并赋予拓扑关系
  if (edge.empty() || curr_line.second.empty()) {
    return;
  }
  uint32_t num_miss_line = uint32_t(dist / 3.5);  // 判断缺失车道数量
  if (num_miss_line == 0) {
    return;
  }

  std::vector<std::vector<Eigen::Vector3d>> predict_line(num_miss_line);

  // 拟合中间点
  for (const auto& line_point : curr_line.second) {
    for (size_t i = 1; i < edge.size(); ++i) {
      Eigen::Vector3d A = edge[i - 1];
      Eigen::Vector3d B = edge[i];
      Eigen::Vector3d P = line_point;
      // 计算A点和B点的方向向量
      Eigen::Vector3d AB = B - A;
      Eigen::Vector3d AP = P - A;
      // 计算AB模长
      double ABLengthSquared = AB.squaredNorm();
      if (ABLengthSquared == 0) {
        continue;
      }
      double t = AB.dot(AP) / ABLengthSquared;
      if (t < 0 || t > 1) {
        continue;
      }

      t = std::max(0.0, std::min(t, 1.0));
      Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点

      double k = (P.y() - C.y()) / (P.x() - C.x());
      double b = C.y() - k * C.x();
      for (uint32_t j = 1; j < num_miss_line + 1; j++) {
        double ratio = j * 1.0 / num_miss_line;
        double x = ratio * (C.x() - P.x()) + P.x();
        double y = k * x + b;
        Eigen::Vector3d new_point(x, y, 0);
        predict_line[j - 1].push_back(new_point);
      }
      break;
    }
  }

  if (predict_line.empty()) {
    return;
  }
  // 对预测的左右缺失车道线进行可视化
  viz_map_.VizAddSideLaneLine(predict_line);

  // 将预测的车道线添加到local_msg中，并赋予拓扑关系
  // AddSideTopological(predict_line, record, curr_line.first);
}

void MapPrediction::AddSideTopological(
    const std::vector<std::vector<Eigen::Vector3d>>& predict_line,
    const uint32_t& record, const std::string& curr_id) {
  // 将新的车道线添加到local_msg中，并赋予拓扑关系
  if (predict_line.empty()) {
    return;
  }
  // 添加左
  if (record == 0) {
    // 预测缺失的左边线
    std::string id_ = curr_id;
    for (const auto& line : predict_line) {
      // 现将当前lane的左边线补充完整
      if (id_ == curr_id) {
        uint32_t count_curr = 0;
        for (auto& lane : local_msg_->lane()) {
          if (lane.id().id() == id_ && !id_.empty()) {
            for (const auto& point : line) {
              auto pt = local_msg_->mutable_lane(count_curr)
                            ->mutable_left_boundary()
                            ->mutable_curve()
                            ->add_segment()
                            ->mutable_line_segment()
                            ->add_point();
              pt->set_x(point.x());
              pt->set_y(point.y());
              pt->set_z(point.z());
            }
          }
          count_curr += 1;
        }

        std::string id_left_id;
        for (const auto& topo_lane_ : topo_map_->lane()) {
          if (topo_lane_.id().id() == id_ &&
              !topo_lane_.left_neighbor_forward_lane_id().empty()) {
            id_left_id = topo_lane_.left_neighbor_forward_lane_id(0).id();
            break;
          }
        }
        id_ = id_left_id;
        continue;
      }
      // 定义一个新的车道
      hozon::hdmap::Lane new_lane;
      // 从topo_map_中找取对应id的左邻和后继id
      std::string id_left_id;
      std::string id_next_id;

      // 这里要注意id_left_id/id_next_id是否为空
      for (const auto& topo_lane_ : topo_map_->lane()) {
        if (topo_lane_.id().id() == id_ &&
            !topo_lane_.left_neighbor_forward_lane_id().empty()) {
          id_left_id = topo_lane_.left_neighbor_forward_lane_id(0).id();
          // break;
        }
        if (topo_lane_.id().id() == id_ && !topo_lane_.successor_id().empty()) {
          id_next_id = topo_lane_.successor_id(0).id();
        }
      }
      // new_lane赋予新id
      if (!id_left_id.empty()) {
        new_lane.mutable_id()->set_id(id_left_id);
      } else {
        new_lane.mutable_id()->set_id(std::to_string(id));
        id -= 1;
      }
      // new_lane赋予新几何
      for (const auto& point : line) {
        auto pt = new_lane.mutable_left_boundary()
                      ->mutable_curve()
                      ->add_segment()
                      ->mutable_line_segment()
                      ->add_point();
        pt->set_x(point.x());
        pt->set_y(point.y());
        pt->set_z(point.z());
      }
      // 确定当前lane后继lane的左邻lane
      std::string id_next_left_id;
      for (const auto& lane : topo_map_->lane()) {
        if (!id_next_id.empty() && lane.id().id() == id_next_id &&
            !lane.left_neighbor_forward_lane_id().empty()) {
          id_next_left_id = lane.left_neighbor_forward_lane_id(0).id();
          break;
        }
      }
      // new_lane的后继指向id_next_left_id
      if (!id_next_left_id.empty()) {
        new_lane.add_successor_id()->set_id(id_next_left_id);
      }

      // new_lane加入到local_msg中
      uint32_t count1 = 0;
      for (auto& lane : local_msg_->lane()) {
        if (lane.id().id() == id_ && !id_.empty()) {
          new_lane.mutable_right_boundary()->CopyFrom(lane.left_boundary());
          new_lane.add_right_neighbor_forward_lane_id()->set_id(lane.id().id());
          local_msg_->add_lane()->CopyFrom(new_lane);
          local_msg_->mutable_lane(count1)
              ->add_left_neighbor_forward_lane_id()
              ->set_id(new_lane.id().id());
          break;
        }
        count1 += 1;
      }
      // 完善前后继关系
      uint32_t count2 = 0;
      for (auto& lane : local_msg_->lane()) {
        if (lane.id().id() == id_next_left_id && !id_next_left_id.empty()) {
          local_msg_->mutable_lane(count2)->add_predecessor_id()->set_id(
              new_lane.id().id());
          break;
        }
        count2 += 1;
      }

      id_ = id_left_id;
    }
  }

  // 添加右
  if (record == 1) {
    // 预测缺失的右边线
    std::string id_ = curr_id;
    for (const auto& line : predict_line) {
      // 现将当前lane的右边线补充完整
      if (id_ == curr_id) {
        uint32_t count_curr = 0;
        for (auto& lane : local_msg_->lane()) {
          if (lane.id().id() == id_ && !id_.empty()) {
            for (const auto& point : line) {
              auto pt = local_msg_->mutable_lane(count_curr)
                            ->mutable_right_boundary()
                            ->mutable_curve()
                            ->add_segment()
                            ->mutable_line_segment()
                            ->add_point();
              pt->set_x(point.x());
              pt->set_y(point.y());
              pt->set_z(point.z());
            }
          }
          count_curr += 1;
        }

        std::string id_right_id;
        for (const auto& topo_lane_ : topo_map_->lane()) {
          if (topo_lane_.id().id() == id_ &&
              !topo_lane_.right_neighbor_forward_lane_id().empty()) {
            id_right_id = topo_lane_.right_neighbor_forward_lane_id(0).id();
            break;
          }
        }
        id_ = id_right_id;
        continue;
      }
      // 定义一个新的车道
      hozon::hdmap::Lane new_lane;
      // 从topo_map_中找取对应id的左邻和后继id
      std::string id_right_id;
      std::string id_next_id;

      // 这里要注意id_left_id/id_next_id是否为空
      for (const auto& topo_lane_ : topo_map_->lane()) {
        if (topo_lane_.id().id() == id_ &&
            !topo_lane_.left_neighbor_forward_lane_id().empty()) {
          id_right_id = topo_lane_.left_neighbor_forward_lane_id(0).id();
          // break;
        }
        if (topo_lane_.id().id() == id_ && !topo_lane_.successor_id().empty()) {
          id_next_id = topo_lane_.successor_id(0).id();
        }
      }
      // new_lane赋予新id
      if (!id_right_id.empty()) {
        new_lane.mutable_id()->set_id(id_right_id);
      } else {
        new_lane.mutable_id()->set_id(std::to_string(id));
        id -= 1;
      }
      // new_lane赋予新几何
      for (const auto& point : line) {
        auto pt = new_lane.mutable_right_boundary()
                      ->mutable_curve()
                      ->add_segment()
                      ->mutable_line_segment()
                      ->add_point();
        pt->set_x(point.x());
        pt->set_y(point.y());
        pt->set_z(point.z());
      }
      // 确定当前lane后继lane的右邻lane
      std::string id_next_right_id;
      for (const auto& lane : topo_map_->lane()) {
        if (!id_next_id.empty() && lane.id().id() == id_next_id &&
            !lane.right_neighbor_forward_lane_id().empty()) {
          id_next_right_id = lane.right_neighbor_forward_lane_id(0).id();
          break;
        }
      }
      // new_lane的后继指向id_next_left_id
      if (!id_next_right_id.empty()) {
        new_lane.add_successor_id()->set_id(id_next_right_id);
      }

      // new_lane加入到local_msg中
      uint32_t count1 = 0;
      for (auto& lane : local_msg_->lane()) {
        if (lane.id().id() == id_ && !id_.empty()) {
          new_lane.mutable_left_boundary()->CopyFrom(lane.right_boundary());
          new_lane.add_left_neighbor_forward_lane_id()->set_id(lane.id().id());
          local_msg_->add_lane()->CopyFrom(new_lane);
          local_msg_->mutable_lane(count1)
              ->add_right_neighbor_forward_lane_id()
              ->set_id(new_lane.id().id());
          break;
        }
        count1 += 1;
      }
      // 完善前后继关系
      uint32_t count2 = 0;
      for (auto& lane : local_msg_->lane()) {
        if (lane.id().id() == id_next_right_id && !id_next_right_id.empty()) {
          local_msg_->mutable_lane(count2)->add_predecessor_id()->set_id(
              new_lane.id().id());
          break;
        }
        count2 += 1;
      }

      id_ = id_right_id;
    }
  }
  // 拓扑关系和几何添加完毕
}

void MapPrediction::PredictAheadLaneLine(
    const std::shared_ptr<hozon::hdmap::Map>& topo_map_,
    const std::set<std::string>& add_road_id_,
    const std::vector<hozon::hdmap::RoadInfoConstPtr>& roads) {
  // 用于预测前方车道线
  if (!topo_map_ || add_road_id_.empty() || roads.empty()) {
    return;
  }
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
      predict_lanelines;
  for (const auto& road : roads) {
    for (const auto& id : add_road_id_) {
      if (road->id().id().c_str() == id) {
        // 通过两侧的道路边界拟合车道线
        for (const auto& road_section : road->sections()) {
          /*
          会用到road_section中的id,用于补拓扑关系
          */
          // std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
          // edge_; int edge_id = 0;
          std::vector<std::vector<Eigen::Vector3d>> boundary1;
          std::vector<std::vector<Eigen::Vector3d>> boundary2;
          for (const auto& edge :
               road_section.boundary().outer_polygon().edge()) {
            if (edge.type() == 2) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary1.emplace_back(edge_point);
                // edge_.emplace_back(std::make_pair(edge_id, edge_point));
                // edge_id += 1;
              }
            }

            if (edge.type() == 3) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary2.emplace_back(edge_point);
                // edge_.emplace_back(std::make_pair(edge_id, edge_point));
                // edge_id += 1;
              }
            }
            // std::vector<Eigen::Vector3d> edge_point;
            // for (const auto& seg : edge.curve().segment()) {
            //   for (const auto& point : seg.line_segment().point()) {
            //     Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
            //     int zone = 51;
            //     double x = point_utm.x();
            //     double y = point_utm.y();
            //     hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
            //     Eigen::Vector3d point_gcj(y, x, 0);
            //     Eigen::Vector3d point_enu =
            //         util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
            //     edge_point.emplace_back(point_enu);
            //   }
            // }
            // if (!edge_point.empty()) {
            //   edge_.emplace_back(std::make_pair(edge_id, edge_point));
            //   edge_id += 1;
            // }
          }
          // std::vector<std::vector<Eigen::Vector3d>> boundary1;
          // std::vector<std::vector<Eigen::Vector3d>> boundary2;
          // separateBoundaries(edge_, &boundary1, &boundary2);
          // 求boundary1和boundary2的最小投影点
          uint32_t lane_num = road_section.lane_id_size();
          FitAheadLaneLine(boundary1, boundary2, predict_lanelines, lane_num);
          // 可视化
          viz_map_.VizAddAheadLaneLine(predict_lanelines);
          // 对预测的车道继承HQ的ID并添加拓扑关系
          std::vector<std::string> section_lane_id;
          for (const auto& it : road_section.lane_id()) {
            section_lane_id.emplace_back(it.id().c_str());
          }
          AheadTopological(predict_lanelines, topo_map_, section_lane_id);
        }
      }
    }
  }
  viz_map_.VizLaneID(local_msg_, local_enu_center_);
  // viz_map_.VizAddAheadLaneLine(predict_lanelines);
}

void MapPrediction::CompleteLaneline(
    const std::vector<std::string>& end_lane_id_,
    const std::set<std::string>& end_road_id_,
    const std::vector<hozon::hdmap::RoadInfoConstPtr>& roads) {
  if (end_lane_id_.empty() || end_road_id_.empty() || roads.empty()) {
    return;
  }
  // 对原车道信息进行补全
  for (const auto& road : roads) {
    for (const auto& it : end_road_id_) {
      if (road->id().id() == it) {
        // 对当前road的两个边界拟合车道线
        for (const auto& road_section : road->sections()) {
          // 存储road_section中的lane_id
          std::vector<std::string> sec_id;
          for (const auto& it : road_section.lane_id()) {
            sec_id.emplace_back(it.id());
          }
          // 是否要先判断当前road_section中所包含的lane_id是否与end_road_id_有交集,
          // 有交集就保存
          std::vector<std::string> com_id;
          bool flag = false;
          for (const auto& sec_lane_id : sec_id) {
            if (std::find(end_lane_id_.begin(), end_lane_id_.end(),
                          sec_lane_id) != end_lane_id_.end()) {
              com_id.emplace_back(sec_lane_id);
              flag = true;
            }
          }
          if (!flag) {
            continue;
          }
          std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
              complete_lines;
          // 找当前road_section所在的两个road
          // std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
          // edge_; int edge_id = 0;
          std::vector<std::vector<Eigen::Vector3d>> boundary1;
          std::vector<std::vector<Eigen::Vector3d>> boundary2;
          for (const auto& edge :
               road_section.boundary().outer_polygon().edge()) {
            if (edge.type() == 2) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary1.emplace_back(edge_point);
                // edge_.emplace_back(std::make_pair(edge_id, edge_point));
                // edge_id += 1;
              }
            }

            if (edge.type() == 3) {
              std::vector<Eigen::Vector3d> edge_point;
              for (const auto& seg : edge.curve().segment()) {
                for (const auto& point : seg.line_segment().point()) {
                  Eigen::Vector3d point_utm(point.x(), point.y(), point.z());
                  int zone = 51;
                  double x = point_utm.x();
                  double y = point_utm.y();
                  hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
                  Eigen::Vector3d point_gcj(y, x, 0);
                  Eigen::Vector3d point_enu =
                      util::Geo::Gcj02ToEnu(point_gcj, local_enu_center_);
                  edge_point.emplace_back(point_enu);
                }
              }
              if (!edge_point.empty()) {
                boundary2.emplace_back(edge_point);
                // edge_.emplace_back(std::make_pair(edge_id, edge_point));
                // edge_id += 1;
              }
            }
          }
          // std::vector<std::vector<Eigen::Vector3d>> boundary1;
          // std::vector<std::vector<Eigen::Vector3d>> boundary2;
          // separateBoundaries(edge_, &boundary1, &boundary2);
          // 求boundary1和boundary2的最小投影点
          uint32_t lane_num = road_section.lane_id_size();
          FitAheadLaneLine(boundary1, boundary2, complete_lines, lane_num);
          // viz_map_.VizAddAheadLaneLine(complete_lines);
          // 这里是否要加入一个判断
          // 通过拟合出的车道线对当前lane进行延伸
          ExpansionLaneLine(complete_lines, com_id, sec_id);
        }
      }
    }
  }
}

void MapPrediction::ExpansionLaneLine(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        complete_lines,
    const std::vector<std::string>& com_id,
    const std::vector<std::string>& sec_id) {
  // 开始对当前的车道进行延伸
  if (complete_lines.empty() || com_id.empty() || sec_id.empty()) {
    return;
  }
  // 将填充的车道线保存并可视化出来
  std::vector<std::vector<Eigen::Vector3d>> compan_lines;
  uint32_t lane_count = 0;  // 车道计数
  for (auto& lane : local_msg_->lane()) {
    for (const auto& end_id : com_id) {
      if (end_id == lane.id().id()) {
        // 求该道路的两个边界到拟合的车道线的最近距离

        // 优化一下
        int seq = 0;  // section_id从右-->左
        for (const auto& it : sec_id) {
          if (it == lane.id().id()) {
            break;
          }
          seq += 1;
        }
        // complete_lines从左-->右
        // 左
        int left_line_id = complete_lines.size() - 2 - seq;
        // 右
        int right_line_id = complete_lines.size() - 1 - seq;

        // 左边界
        // 临时加了一个判断条件
        if (lane.left_boundary().curve().segment().empty()) {
          continue;
        }
        double seg_size = lane.left_boundary().curve().segment().size();
        double point_size = lane.left_boundary()
                                .curve()
                                .segment()[seg_size - 1]
                                .line_segment()
                                .point()
                                .size();
        // 临时加了一个判断条件
        if (seg_size == 0 || point_size == 0) {
          continue;
        }
        // 左边界最后一个点
        Eigen::Vector3d left_end_point(lane.left_boundary()
                                           .curve()
                                           .segment()[seg_size - 1]
                                           .line_segment()
                                           .point()[point_size - 1]
                                           .x(),
                                       lane.left_boundary()
                                           .curve()
                                           .segment()[seg_size - 1]
                                           .line_segment()
                                           .point()[point_size - 1]
                                           .y(),
                                       lane.left_boundary()
                                           .curve()
                                           .segment()[seg_size - 1]
                                           .line_segment()
                                           .point()[point_size - 1]
                                           .z());
        // 左边界最后一个点到新拟合车道线的投影点的距离
        double min_distance_left = std::numeric_limits<double>::max();
        uint32_t flag_count_left;

        uint32_t count_left;  // 记录从第几个点开始扩充
        for (size_t i = 1; i < complete_lines[left_line_id].second.size();
             ++i) {
          Eigen::Vector3d A = complete_lines[left_line_id].second[i - 1];
          Eigen::Vector3d B = complete_lines[left_line_id].second[i];
          Eigen::Vector3d P = left_end_point;

          // 计算A点和B点的方向向量
          Eigen::Vector3d AB = B - A;
          Eigen::Vector3d AP = P - A;
          // 计算AB模长
          double ABLengthSquared = AB.squaredNorm();
          if (ABLengthSquared == 0) {
            continue;
          }
          double t = AB.dot(AP) / ABLengthSquared;
          count_left = i;
          if (t < 0 || t > 1) {
            continue;
          }

          t = std::max(0.0, std::min(t, 1.0));
          Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点
          double dis = (P - C).norm();
          if (dis < min_distance_left && dis < 3.75) {
            min_distance_left = dis;
            flag_count_left = count_left;
          }
          break;
        }
        // 判断是否满足距离要求
        if (min_distance_left != std::numeric_limits<double>::max()) {
          // 满足要求，对该车道的边界点进行延伸
          std::vector<Eigen::Vector3d> com_line;
          for (size_t i = flag_count_left;
               i < complete_lines[left_line_id].second.size(); i++) {
            com_line.emplace_back(complete_lines[left_line_id].second[i]);
            // 对lane的左边界的点记性扩充
            auto seg_num = local_msg_->lane(lane_count)
                               .left_boundary()
                               .curve()
                               .segment_size();
            // 临时加入判断条件
            if (seg_num == 0) {
              continue;
            }
            auto pt = local_msg_->mutable_lane(lane_count)
                          ->mutable_left_boundary()
                          ->mutable_curve()
                          ->mutable_segment(seg_num - 1)
                          ->mutable_line_segment()
                          ->add_point();
            pt->set_x(complete_lines[left_line_id].second[i].x());
            pt->set_y(complete_lines[left_line_id].second[i].y());
            pt->set_z(complete_lines[left_line_id].second[i].z());
          }
          compan_lines.emplace_back(com_line);
        }

        // 右边界
        // 临时加了一个判断条件
        if (lane.right_neighbor_forward_lane_id().empty()) {
          if (lane.right_boundary().curve().segment().empty()) {
            continue;
          }
          double seg_size_ = lane.right_boundary().curve().segment().size();
          double point_size_ = lane.right_boundary()
                                   .curve()
                                   .segment()[seg_size_ - 1]
                                   .line_segment()
                                   .point()
                                   .size();
          // 临时加了一个判断条件
          if (seg_size_ == 0 || point_size_ == 0) {
            continue;
          }
          // 右边界最后一个点
          Eigen::Vector3d right_end_point(lane.right_boundary()
                                              .curve()
                                              .segment()[seg_size_ - 1]
                                              .line_segment()
                                              .point()[point_size_ - 1]
                                              .x(),
                                          lane.right_boundary()
                                              .curve()
                                              .segment()[seg_size_ - 1]
                                              .line_segment()
                                              .point()[point_size_ - 1]
                                              .y(),
                                          lane.right_boundary()
                                              .curve()
                                              .segment()[seg_size_ - 1]
                                              .line_segment()
                                              .point()[point_size_ - 1]
                                              .z());
          // 左边界最后一个点到新拟合车道线的投影点的距离
          double min_distance_right = std::numeric_limits<double>::max();
          uint32_t flag_count_right;

          uint32_t count_right = 1;  // 记录从第几个点开始扩充
          for (size_t i = 1; i < complete_lines[right_line_id].second.size();
               ++i) {
            Eigen::Vector3d A = complete_lines[right_line_id].second[i - 1];
            Eigen::Vector3d B = complete_lines[right_line_id].second[i];
            Eigen::Vector3d P = right_end_point;

            // 计算A点和B点的方向向量
            Eigen::Vector3d AB = B - A;
            Eigen::Vector3d AP = P - A;
            // 计算AB模长
            double ABLengthSquared = AB.squaredNorm();
            if (ABLengthSquared == 0) {
              continue;
            }
            double t = AB.dot(AP) / ABLengthSquared;
            count_right = i;
            if (t < 0 || t > 1) {
              continue;
            }

            t = std::max(0.0, std::min(t, 1.0));
            Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点
            double dis = (P - C).norm();
            if (dis < min_distance_left && dis < 3.75) {
              min_distance_right = dis;
              flag_count_right = count_right;
            }
            break;
          }
          // 判断是否满足距离要求
          if (min_distance_right != std::numeric_limits<double>::max()) {
            // 满足要求，对该车道的边界点进行延伸
            std::vector<Eigen::Vector3d> com_line;
            auto seg_num = local_msg_->lane(lane_count)
                               .right_boundary()
                               .curve()
                               .segment_size();
            for (size_t i = flag_count_right;
                 i < complete_lines[right_line_id].second.size(); i++) {
              com_line.emplace_back(complete_lines[right_line_id].second[i]);
              // 对lane的左边界的点记性扩充
              // 临时加入判断条件
              if (seg_num == 0) {
                continue;
              }
              auto pt = local_msg_->mutable_lane(lane_count)
                            ->mutable_right_boundary()
                            ->mutable_curve()
                            ->mutable_segment(seg_num - 1)
                            ->mutable_line_segment()
                            ->add_point();
              pt->set_x(complete_lines[right_line_id].second[i].x());
              pt->set_y(complete_lines[right_line_id].second[i].y());
              pt->set_z(complete_lines[right_line_id].second[i].z());
            }
            compan_lines.emplace_back(com_line);
          }
        }
        // 延伸结束
      }
    }
    lane_count += 1;
  }
  // 可视化延伸的车道线
  viz_map_.VizCompanLane(compan_lines);
}

void MapPrediction::separateBoundaries(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>& edge_,
    std::vector<std::vector<Eigen::Vector3d>>* boundary1,
    std::vector<std::vector<Eigen::Vector3d>>* boundary2) {
  if (!boundary1 || !boundary2) {
    return;
  }

  std::vector<Eigen::Vector3d> startSegment1 = edge_.front().second;
  findConnectedSegments(startSegment1, edge_, boundary1);

  std::vector<Eigen::Vector3d> startSegment2;
  for (const auto& segment : edge_) {
    bool isSegmentInBoundary1 = false;
    for (const auto& resultSegment : *boundary1) {
      if (resultSegment == segment.second) {
        isSegmentInBoundary1 = true;
        break;
      }
    }
    if (!isSegmentInBoundary1) {
      startSegment2 = segment.second;
    }
  }
  findConnectedSegments(startSegment2, edge_, boundary2);
}

void MapPrediction::findConnectedSegments(
    const std::vector<Eigen::Vector3d>& startSegment1,
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>& edge_,
    std::vector<std::vector<Eigen::Vector3d>>* boundary1) {
  if (!boundary1) {
    return;
  }

  boundary1->push_back(startSegment1);

  for (const auto& segment : edge_) {
    bool isSegmentInResult = false;
    for (const auto& resultSegment : *boundary1) {
      if (resultSegment == segment.second) {
        isSegmentInResult = true;
        break;
      }
    }

    if (!isSegmentInResult &&
        (segment.second.front() == boundary1->back().back() ||
         segment.second.back() == boundary1->back().front())) {
      findConnectedSegments(segment.second, edge_, boundary1);
      break;
    }
  }
}

void MapPrediction::DetermineEdgeAssPair(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        all_edges,
    std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>>&
        boundary_pairs) {
  // 求取车道之间的关联对
  for (const auto& edge : all_edges) {
    double min_distance = std::numeric_limits<double>::max();
    double ass_id;
    for (size_t i = 0; i < all_edges.size(); ++i) {
      if (all_edges[i].first == edge.first) {
        continue;
      }
      // 遍历edge中的所有点,并存储每个点到对应点的平均距离
      std::vector<double> distance;
      if (edge.second.size() == 1 && all_edges[i].second.size() == 1 &&
          uint32_t((edge.second.back() - all_edges[i].second.back()).norm() /
                   3.5) == 3) {
        double distance =
            (edge.second.back() - all_edges[i].second.back()).norm();
        std::pair<uint32_t, uint32_t> id_pair =
            std::make_pair(edge.first, all_edges[i].first);
        boundary_pairs.emplace_back(std::make_pair(id_pair, distance));
      }
      for (const auto& point : edge.second) {
        for (size_t j = 1; j < all_edges[i].second.size(); ++j) {
          Eigen::Vector3d A = all_edges[i].second[j - 1];
          Eigen::Vector3d B = all_edges[i].second[j];
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
        ass_id = all_edges[i].first;
      }
    }
    if (min_distance != std::numeric_limits<double>::max()) {
      std::pair<uint32_t, uint32_t> id_pair =
          std::make_pair(edge.first, ass_id);
      boundary_pairs.emplace_back(std::make_pair(id_pair, min_distance));
    }
  }
}

void MapPrediction::FitAheadLaneLine(
    const std::vector<std::vector<Eigen::Vector3d>>& boundary1,
    const std::vector<std::vector<Eigen::Vector3d>>& boundary2,
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        predict_lanelines,
    const uint32_t& lane_num) {
  // 跟据boundary1和boundary2拟合车道线
  if (boundary1.empty() || boundary2.empty()) {
    return;
  }
  std::vector<Eigen::Vector3d> edge1_;
  std::vector<Eigen::Vector3d> edge2_;
  for (const auto& bound : boundary1) {
    for (const auto& point : bound) {
      edge1_.push_back(point);
    }
  }
  // 去除重复元素
  edge1_.erase(std::unique(edge1_.begin(), edge1_.end()), edge1_.end());
  for (const auto& bound : boundary2) {
    for (const auto& point : bound) {
      edge2_.push_back(point);
    }
  }
  // 去除重复元素
  edge2_.erase(std::unique(edge2_.begin(), edge2_.end()), edge2_.end());

  std::vector<std::vector<Eigen::Vector3d>> predict_line(lane_num + 1);
  // 将起点加入进去
  // if (edge1.begin()->x() == edge2_.front().x()) {
  //   // k的值是无穷
  //   for (double j = 0.0; j < num_lines + 1; ++j) {
  //     double ratio = j / num_lines;
  //     double x = ratio * (edge2_.front().x() - edge1.begin()->x()) +
  //                edge1.begin()->x();
  //     double y = edge1.begin()->y() +
  //                (edge2_.front().y() - edge1.begin()->y()) * ratio;
  //     Eigen::Vector3d new_point(x, y, 0);
  //     predict_line[j].push_back(new_point);
  //   }
  // }
  Eigen::Vector3d edge1_begin = edge1_.front();
  Eigen::Vector3d edge2_begin = edge2_.front();
  double k =
      (edge1_begin.y() - edge2_begin.y()) / (edge1_begin.x() - edge2_begin.x());
  double b = edge2_begin.y() - k * edge2_begin.x();
  for (uint32_t j = 0; j < lane_num + 1; ++j) {
    double ratio = j * 1.0 / lane_num;
    double x = ratio * (edge2_begin.x() - edge1_begin.x()) + edge1_begin.x();
    double y = k * x + b;
    Eigen::Vector3d new_point(x, y, 0);
    predict_line[j].push_back(new_point);
  }

  // 将中间点加入进去
  for (const auto& bound1 : edge1_) {
    double min_dis = std::numeric_limits<double>::max();
    for (size_t i = 1; i < edge2_.size(); ++i) {
      Eigen::Vector3d A = edge2_[i - 1];
      Eigen::Vector3d B = edge2_[i];
      Eigen::Vector3d P = bound1;

      // 计算A点和B点的方向向量
      Eigen::Vector3d AB = B - A;
      Eigen::Vector3d AP = P - A;
      // 计算AB模长
      double ABLengthSquared = AB.squaredNorm();
      if (ABLengthSquared == 0) {
        continue;
      }
      double t = AB.dot(AP) / ABLengthSquared;

      if (t < 0 || t > 1) {
        continue;
      }
      t = std::max(0.0, std::min(t, 1.0));
      Eigen::Vector3d C = A + t * AB;  // 点到线段的最近点
      // double dis = (P - C).norm();
      // uint32_t num_lines = uint32_t(dis / 3.5);

      // if (P.x() == C.x()) {
      //   for (double j = 0.0; j < num_lines + 1; ++j) {
      //     double ratio = j / num_lines;
      //     double x = P.x();
      //     double y = P.y() + (C.y() - P.y()) * ratio;
      //     Eigen::Vector3d new_point(x, y, 0);
      //     predict_line[j].push_back(new_point);
      //   }
      // }
      double k = (P.y() - C.y()) / (P.x() - C.x());
      double b = C.y() - k * C.x();
      for (double j = 0; j < lane_num + 1; ++j) {
        double ratio = j * 1.0 / lane_num;
        double x = ratio * (C.x() - P.x()) + P.x();
        double y = k * x + b;
        Eigen::Vector3d new_point(x, y, 0);
        predict_line[j].push_back(new_point);
      }
      break;
    }
  }
  // 将终点加入进去
  Eigen::Vector3d edge1_end = edge1_.back();
  Eigen::Vector3d edge2_end = edge2_.back();
  double k1 = (edge1_end.y() - edge2_end.y()) / (edge1_end.x() - edge2_end.x());
  double b1 = edge2_end.y() - k1 * edge2_end.x();
  for (uint32_t j = 0; j < lane_num + 1; ++j) {
    double ratio = j * 1.0 / lane_num;
    double x = ratio * (edge2_end.x() - edge1_end.x()) + edge1_end.x();
    double y = k1 * x + b1;
    Eigen::Vector3d new_point(x, y, 0);
    predict_line[j].push_back(new_point);
  }

  if (!predict_line.empty()) {
    for (const auto& it : predict_line) {
      predict_lanelines.emplace_back(std::make_pair(id, it));
      id -= 1;
    }
  }
}

void MapPrediction::AheadTopological(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        predict_lanelines,
    const std::shared_ptr<hozon::hdmap::Map>& topo_map_,
    const std::vector<std::string>& section_lane_id) {
  // 预测前方地图的拓扑关系
  // 存储预测的车道线的起点
  // std::lock_guard<std::mutex> lock(mtx_);
  if (!local_msg_ || !topo_map_ || predict_lanelines.empty()) {
    return;
  }

  if (section_lane_id.size() + 1 != predict_lanelines.size()) {
    return;
  }
  /*
  根据LOG判断section_lane_id的顺序依次是从右往左(boundary2-->boundary1)
  对拓扑关系进行优化操作：
  1.从section_lane_id找到对应的车道id
  2.从topo_map_中遍历id,找到左右边界lane，0-->左边界lane, 1-->右边界lane
  3.predict_lanelines中第一根新的起点与左lane的左线和右lane的右线进行比较求关联
  4.根据3中的关联关系依次创建new_lane，并赋予id和几何和左右拓扑关系，并加入local_msg中
  5.从topo_map_中找new_lane的所有前继id，在local_msg中赋予new_lane的前继关系
  6.从local_msg中找new_lane的所有前继id，并赋予后继关系
  */
  // 从bounary2-->boundary1
  for (int i = predict_lanelines.size() - 2; i >= 0; --i) {
    // 定义新车道
    hozon::hdmap::Lane new_lane;
    // 为新车道赋予Id
    new_lane.mutable_id()->set_id(
        section_lane_id[section_lane_id.size() - i - 1]);
    // 为新车道添加几何信息
    // 左
    for (const auto& point : predict_lanelines[i].second) {
      auto pt = new_lane.mutable_left_boundary()
                    ->mutable_curve()
                    ->add_segment()
                    ->mutable_line_segment()
                    ->add_point();
      pt->set_x(point.x());
      pt->set_y(point.y());
      pt->set_z(point.z());
    }
    // 右
    for (const auto& point : predict_lanelines[i + 1].second) {
      auto pt = new_lane.mutable_right_boundary()
                    ->mutable_curve()
                    ->add_segment()
                    ->mutable_line_segment()
                    ->add_point();
      pt->set_x(point.x());
      pt->set_y(point.y());
      pt->set_z(point.z());
    }
    // 查询新车道的所有前继id
    std::vector<std::string> pred;
    for (const auto& lane : topo_map_->lane()) {
      if (lane.id().id() == section_lane_id[section_lane_id.size() - i - 1] &&
          !lane.predecessor_id().empty()) {
        // 赋new_lane的前继id
        for (const auto& pred_id : lane.predecessor_id()) {
          new_lane.add_predecessor_id()->set_id(pred_id.id());
          pred.push_back(pred_id.id());
        }
      }
    }
    // 将new_lane加入到local_msg中
    local_msg_->add_lane()->CopyFrom(new_lane);
    // 将local_msg中对应的pred的后继指向new_lane
    uint32_t count = 0;
    for (auto& local_lane : local_msg_->lane()) {
      for (const auto& pd : pred) {
        if (local_lane.id().id() == pd) {
          local_msg_->mutable_lane(count)->add_successor_id()->set_id(
              new_lane.id().id());
        }
      }
      // 左右邻
      if (section_lane_id.size() - i - 1 > 0 &&
          local_lane.id().id() ==
              section_lane_id[section_lane_id.size() - i - 1]) {
        local_msg_->mutable_lane(count)
            ->add_right_neighbor_forward_lane_id()
            ->set_id(section_lane_id[section_lane_id.size() - i - 2]);
      }
      if (section_lane_id.size() - i - 1 > 0 &&
          local_lane.id().id() ==
              section_lane_id[section_lane_id.size() - i - 2]) {
        local_msg_->mutable_lane(count)
            ->add_left_neighbor_forward_lane_id()
            ->set_id(section_lane_id[section_lane_id.size() - i - 1]);
      }
      count += 1;
    }
  }

  // 补全预测车道之间的左右邻关系
  // AddAheadLeftRightTopo(local_msg);
}

std::shared_ptr<hozon::hdmap::Map> MapPrediction::GetPredictionMap() {
  std::lock_guard<std::mutex> lock_map(mtx_);
  return local_msg_;
}

void MapPrediction::AddAheadLeftRightTopo(hozon::hdmap::Map* local_msg_) {
  if (!local_msg_) {
    return;
  }

  for (auto& lane : local_msg_->lane()) {
    // 增添拓扑关系
    /*
    1 2 3
    4 5 6
    7 8 9
    */
    // 记录当前lane的左邻和后继和右邻
    std::string left_id;
    std::string successor_id;
    std::string right_id;
    if (!lane.left_neighbor_forward_lane_id().empty()) {
      left_id = lane.left_neighbor_forward_lane_id()[0].id();
    }
    if (!lane.successor_id().empty()) {
      successor_id = lane.successor_id()[0].id();
    }
    if (!lane.right_neighbor_forward_lane_id().empty()) {
      right_id = lane.right_neighbor_forward_lane_id()[0].id();
    }

    // left_id和right_id对应的车道后继id
    std::string left_successor_id_;
    std::string right_successor_id_;
    for (auto& lane_it : local_msg_->lane()) {
      if (lane_it.id().id() == left_id && !left_id.empty() &&
          !lane_it.successor_id().empty()) {
        left_successor_id_ = lane_it.successor_id()[0].id();
      }
      if (lane_it.id().id() == right_id && !right_id.empty() &&
          !lane_it.successor_id().empty()) {
        right_successor_id_ = lane_it.successor_id()[0].id();
      }
    }

    // 找到successor_id对应的车道
    int count = 0;
    for (auto& lane_it : local_msg_->lane()) {
      if (!successor_id.empty() && lane_it.id().id() == successor_id &&
          !left_successor_id_.empty()) {
        local_msg_->mutable_lane(count)
            ->add_left_neighbor_forward_lane_id()
            ->set_id(left_successor_id_);
      }
      if (!successor_id.empty() && lane_it.id().id() == successor_id &&
          !right_successor_id_.empty()) {
        local_msg_->mutable_lane(count)
            ->add_right_neighbor_forward_lane_id()
            ->set_id(right_successor_id_);
      }
      count += 1;
    }
  }
}

void MapPrediction::FitLaneCenterline(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        complete_lanelines,
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        all_center_lanelines) {
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>> LaneLines =
      complete_lanelines;
  // 使用lambda表达式和std::sort函数对数组进行排序
  std::sort(LaneLines.begin(), LaneLines.end(),
            [](const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& line1,
               const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& line2) {
              return (line1.second.front().x() < line2.second.front().x());
            });
  // 排序之后开始对两两车道线求车道中心线
  for (size_t i = 1; i < LaneLines.size(); ++i) {
    std::vector<Eigen::Vector3d> center_linepoints;
    for (const auto& it : LaneLines[i - 1].second) {
      for (size_t j = 1; j < LaneLines[i].second.size(); ++j) {
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
        center_linepoints.emplace_back((it + C) /
                                       2);  // 存入最近投影点的之间的投影点
        break;
      }
    }
    all_center_lanelines.emplace_back(std::make_pair(id, center_linepoints));
    id -= 1;
  }
}

void MapPrediction::Prediction() {
  pthread_setname_np(pthread_self(), "pred");
  while (!is_pred_proc_stop_) {
    usleep(FLAGS_pred_thread_interval * 1000);
    // 存取所有的车道id用于找取道路边界和预测前方车道线时使用
    std::vector<std::string> lane_id_;
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto& lane : local_msg_->lane()) {
      // 存储lane_id
      lane_id_.emplace_back(lane.id().id());
    }
    // 可视化
    // viz_map_.VizLocalMapLaneLine(localmap_lanelines, local_msg);

    if (!hq_map_server_) {
      HLOG_ERROR << "!!! nullptr hq_map_server_";
    }
    hozon::hdmap::LaneInfo::SampledWidth range;
    range.first = 150.;
    range.second = 2.;

    hozon::hdmap::Map topo_map;
    Eigen::Vector3d vehicle_pose_(location_->pos_gcj02().x(),
                                  location_->pos_gcj02().y(),
                                  location_->pos_gcj02().z());
    double x = vehicle_pose_.x();
    double y = vehicle_pose_.y();

    hozon::common::coordinate_convertor::GCS2UTM(51, &y, &x);
    hozon::common::PointENU enupos;
    enupos.set_x(y);
    enupos.set_y(x);
    enupos.set_z(0);

    int ret = hq_map_server_->GetLocalMap(enupos, range, &topo_map);

    if (ret != 0) {
      HLOG_ERROR << "get local map failed";
      return;
    }

    {
      // std::lock_guard<std::mutex> lock(mtx_);
      topo_map_ = std::make_shared<hozon::hdmap::Map>();
      topo_map_->CopyFrom(topo_map);
    }

    // 从这里拿到了150米的道路边界（前后150）
    std::vector<hozon::hdmap::RoadInfoConstPtr> roads;
    double distance = 150.;
    ret = hq_map_server_->GetRoads(enupos, distance, &roads);

    if (ret != 0) {
      HLOG_ERROR << "get road edge failed";
    }

    std::vector<hozon::hdmap::LaneInfoConstPtr> lanes;
    ret = hq_map_server_->GetLanes(enupos, distance, &lanes);

    if (ret != 0) {
      HLOG_ERROR << "get lanes failed";
    }
    // std::lock_guard<std::mutex> lock(mtx_);

    // 根据当前的road_id和topo_map_对local_msg中的车道信息进行补全
    // 找到最后的几个lane并存储
    std::vector<std::string> end_lane_id_;
    for (const auto& lane : local_msg_->lane()) {
      if (lane.successor_id().empty()) {
        end_lane_id_.emplace_back(lane.id().id());
      }
    }

    // 记录前方需要预测的车道id
    std::vector<std::string> end_lane_id_t = end_lane_id_;
    std::set<std::string> add_lane_id_;
    if (!lane_id_.empty() && !topo_map_->lane().empty()) {
      for (auto& end_id : end_lane_id_t) {
        for (const auto& lane : topo_map_->lane()) {
          if (lane.id().id() == end_id && !lane.successor_id().empty()) {
            for (const auto& succ_id : lane.successor_id()) {
              add_lane_id_.insert(succ_id.id());
              end_lane_id_t.push_back(succ_id.id());
            }
          }
        }
      }
    }

    // 根据topo_map找end_lane_id\lane_id_和add_lane_id对应的road id信息
    std::set<std::string> all_road_id_;
    std::set<std::string> curr_road_id_;
    std::set<std::string> add_road_id_;
    std::set<std::string> end_road_id_;
    if (!lanes.empty()) {
      for (const auto& lane : lanes) {
        if (!lane_id_.empty()) {
          for (const auto& it : lane_id_) {
            if (lane->id().id() == it) {
              all_road_id_.insert(lane->road_id().id());
              curr_road_id_.insert(lane->road_id().id());
            }
          }
        }
        if (!add_lane_id_.empty()) {
          for (const auto& it : add_lane_id_) {
            if (lane->id().id() == it) {
              all_road_id_.insert(lane->road_id().id());
              add_road_id_.insert(lane->road_id().id());
            }
          }
        }
        if (!end_lane_id_.empty()) {
          for (const auto& it : end_lane_id_) {
            if (lane->id().id() == it) {
              end_road_id_.insert(lane->road_id().id());
            }
          }
        }
      }
    }
    CompleteLaneline(end_lane_id_, end_road_id_, roads);
    viz_map_.VizLocalMapLaneLine(localmap_lanelines, local_msg_);

    PredictLeftRightLaneline(topo_map_, curr_road_id_, roads);

    // // 对已有的车道信息进行补全
    viz_map_.VizHqMapRoad(roads, all_road_id_, local_enu_center_);

    // 现在对预测的道路边界补充几何信息
    PredictAheadLaneLine(topo_map_, add_road_id_, roads);
    // double start_time = clock();

    // PredictLeftRightLaneline(topo_map_, curr_road_id_, roads);

    // double end_time = clock();
    // HLOG_ERROR << "==============PredictLeftRightLaneline run time: "
    //            << (end_time - start_time) / CLOCKS_PER_SEC;

    // 现在开始验证完整的local_msg
    Eigen::Vector3d pose(location_->pos_gcj02().x(), location_->pos_gcj02().y(),
                         location_->pos_gcj02().z());

    // 这里可视化拓扑关系的情况会报索引越界的错误（记得后面要修改）
    viz_map_.VizLocalMsg(local_msg_, pose);
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
