/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-05
 *****************************************************************************/
#include "modules/local_mapping/ops/lane/lane_op.h"

namespace hozon {
namespace mp {
namespace lm {

void LaneOp::Match(const Perception& cur_lane_lines, const LocalMap& local_map,
                   std::vector<LaneMatchInfo>* match_info) {
  if (local_map.lane_lines_.empty()) {
    for (const auto& cur_lane_line : cur_lane_lines.lane_lines_) {
      LaneMatchInfo match;
      match.frame_lane_line_ = std::make_shared<LaneLine>(cur_lane_line);
      match.update_type_ = ObjUpdateType::ADD_NEW;
      match_info->emplace_back(match);
    }
    return;
  }

  for (const auto& cur_lane_line : cur_lane_lines.lane_lines_) {
    LaneMatchInfo match;
    match.frame_lane_line_ = std::make_shared<LaneLine>(cur_lane_line);
    bool match_success = false;
    for (const auto& map_lane_line : local_map.lane_lines_) {
      if (cur_lane_line.track_id_ == map_lane_line.track_id_) {
        match.map_lane_line_ = std::make_shared<LaneLine>(map_lane_line);
        match_success = true;
      }
    }
    if (match_success) {
      match.update_type_ = ObjUpdateType::MERGE_OLD;
    } else {
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    match_info->emplace_back(match);
  }
}

void LaneOp::Match(const Perception& cur_lane_lines, const LocalMap& local_map,
                   std::vector<LaneMatchInfo>* match_info,
                   bool use_horizon_assoc_match) {
  if (local_map.lane_lines_.empty()) {
    for (const auto& cur_lane_line : cur_lane_lines.lane_lines_) {
      LaneMatchInfo match;
      match.frame_lane_line_ = std::make_shared<LaneLine>(cur_lane_line);
      match.update_type_ = ObjUpdateType::ADD_NEW;
      match_info->emplace_back(match);
    }
    return;
  }
  std::unordered_map<int, int> map_det_lm;
  if (use_horizon_assoc_match) {
    horizon_lane_assoc_ = std::make_shared<HorizonLaneAssoc>();
    map_det_lm = horizon_lane_assoc_->Process(cur_lane_lines.lane_lines_,
                                              local_map.lane_lines_);
  } else {
    lane_assoc_ = std::make_shared<LaneAssoc>(lane_assoc_options_);
    map_det_lm =
        lane_assoc_->Process(cur_lane_lines.lane_lines_, local_map.lane_lines_);
  }
  for (int i = 0; i < static_cast<int>(cur_lane_lines.lane_lines_.size());
       ++i) {
    if (!use_horizon_assoc_match) {
      lane_assoc_ = std::make_shared<LaneAssoc>(lane_assoc_options_);
      if (lane_assoc_->NeedDelete(i)) {
        continue;
      }
    }
    LaneMatchInfo match;
    match.frame_lane_line_ =
        std::make_shared<LaneLine>(cur_lane_lines.lane_lines_[i]);
    if (map_det_lm.find(i) != map_det_lm.end()) {
      match.map_lane_line_ =
          std::make_shared<LaneLine>(local_map.lane_lines_[map_det_lm[i]]);
      match.update_type_ = ObjUpdateType::MERGE_OLD;
      // HLOG_ERROR << "merge det " << i << " with lm " << map_det_lm[i];
    } else {
      match.update_type_ = ObjUpdateType::ADD_NEW;
    }
    match_info->emplace_back(match);
  }
}

bool LaneOp::MatchLeftRight(const LaneLine& query_lane_line,
                            const LaneLine& other_lane_line) {
  std::vector<cv::Point2f> cv_points;
  for (const auto& point : other_lane_line.points_) {
    cv::Point2f tmp_point = {static_cast<float>(point.x()),
                             static_cast<float>(point.y())};
    cv_points.emplace_back(tmp_point);
  }
  if (cv_points.empty()) {
    return false;
  }
  cv::flann::KDTreeIndexParams index_params(1);
  std::shared_ptr<cv::flann::Index> kdtree = std::make_shared<cv::flann::Index>(
      cv::Mat(cv_points).reshape(1), index_params);
  int num = 0;
  for (auto query_point : query_lane_line.points_) {
    if (query_point.x() > 50) {
      continue;
    }
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_dist(1);
    std::vector<float> query_points =
        std::vector<float>{static_cast<float>(query_point.x()),
                           static_cast<float>(query_point.y())};
    if (query_points.empty()) {
      continue;
    }
    kdtree->knnSearch(query_points, nearest_index, nearest_dist, 1,
                      cv::flann::SearchParams(-1));
    if (nearest_dist[0] > 1) {
      continue;
    }
    num++;
  }
  return num > 5;
}

void LaneOp::MergePointsLeftRight(LaneLine* query_lane_line,
                                  LaneLine* other_lane_line) {
  std::vector<Eigen::Vector3d> new_points;
  if (query_lane_line->points_[0].x() < other_lane_line->points_[0].x() &&
      query_lane_line->points_.back().x() <
          other_lane_line->points_.back().x()) {
    // HLOG_ERROR << "1";
    // HLOG_ERROR << "query_lane_line: ID startx endx"
    //            << query_lane_line->track_id_ << " "
    //            << query_lane_line->points_[0].x() << " "
    //            << query_lane_line->points_.back().x();
    // HLOG_ERROR << "other_lane_line: ID startx endx"
    //            << other_lane_line->track_id_ << " "
    //            << other_lane_line->points_[0].x() << " "
    //            << other_lane_line->points_.back().x();
    for (auto point : query_lane_line->points_) {
      if (point.x() > other_lane_line->points_[0].x() - 0.9) {
        break;
      }
      new_points.emplace_back(point);
    }
    std::copy(other_lane_line->points_.begin(), other_lane_line->points_.end(),
              std::back_inserter(new_points));
  } else if (query_lane_line->points_[0].x() >
                 other_lane_line->points_[0].x() &&
             query_lane_line->points_.back().x() >
                 other_lane_line->points_.back().x()) {
    // HLOG_ERROR << "2";
    // HLOG_ERROR << "query_lane_line: ID startx endx"
    //            << query_lane_line->track_id_ << " "
    //            << query_lane_line->points_[0].x() << " "
    //            << query_lane_line->points_.back().x();
    // HLOG_ERROR << "other_lane_line: ID startx endx"
    //            << other_lane_line->track_id_ << " "
    //            << other_lane_line->points_[0].x() << " "
    //            << other_lane_line->points_.back().x();
    for (auto point : other_lane_line->points_) {
      if (point.x() > query_lane_line->points_[0].x() - 0.9) {
        break;
      }
      new_points.emplace_back(point);
    }
    std::copy(query_lane_line->points_.begin(), query_lane_line->points_.end(),
              std::back_inserter(new_points));
  } else if (query_lane_line->points_[0].x() <
                 other_lane_line->points_[0].x() &&
             query_lane_line->points_.back().x() >
                 other_lane_line->points_.back().x()) {
    // HLOG_ERROR << "3";
    // HLOG_ERROR << "query_lane_line: ID startx endx"
    //            << query_lane_line->track_id_ << " "
    //            << query_lane_line->points_[0].x() << " "
    //            << query_lane_line->points_.back().x();
    // HLOG_ERROR << "other_lane_line: ID startx endx"
    //            << other_lane_line->track_id_ << " "
    //            << other_lane_line->points_[0].x() << " "
    //            << other_lane_line->points_.back().x();
    std::copy(query_lane_line->points_.begin(), query_lane_line->points_.end(),
              std::back_inserter(new_points));
  } else if (query_lane_line->points_[0].x() >
                 other_lane_line->points_[0].x() &&
             query_lane_line->points_.back().x() <
                 other_lane_line->points_.back().x()) {
    // HLOG_ERROR << "4";
    // HLOG_ERROR << "query_lane_line: ID startx endx"
    //            << query_lane_line->track_id_ << " "
    //            << query_lane_line->points_[0].x() << " "
    //            << query_lane_line->points_.back().x();
    // HLOG_ERROR << "other_lane_line: ID startx endx"
    //            << other_lane_line->track_id_ << " "
    //            << other_lane_line->points_[0].x() << " "
    //            << other_lane_line->points_.back().x();
    std::copy(other_lane_line->points_.begin(), other_lane_line->points_.end(),
              std::back_inserter(new_points));
  }

  if (query_lane_line->track_id_ < other_lane_line->track_id_) {
    other_lane_line->track_id_ = query_lane_line->track_id_;
    other_lane_line->lanepos_ = query_lane_line->lanepos_;
    other_lane_line->lanetype_ = query_lane_line->lanetype_;
    other_lane_line->points_ = new_points;
  } else {
    other_lane_line->points_ = new_points;
  }
  query_lane_line->need_delete_ = true;
}

void LaneOp::MergeMapLeftRight(LocalMap* local_map) {
  for (auto& lane_line : local_map->lane_lines_) {
    lane_line.need_merge_ = !lane_line.points_.empty();
    lane_line.need_delete_ = false;
  }
  for (auto& query_lane_line : local_map->lane_lines_) {
    if (!query_lane_line.need_merge_) {
      continue;
    }
    query_lane_line.need_merge_ = false;
    for (auto& other_lane_line : local_map->lane_lines_) {
      if (!other_lane_line.need_merge_) {
        continue;
      }
      if (!MatchLeftRight(query_lane_line, other_lane_line)) {
        continue;
      }
      MergePointsLeftRight(&query_lane_line, &other_lane_line);
    }
  }
  for (int i = 0; i < static_cast<int>(local_map->lane_lines_.size()); i++) {
    if (!local_map->lane_lines_[i].need_delete_) {
      continue;
    }
    local_map->lane_lines_.erase(local_map->lane_lines_.begin() + i);
    i--;
  }
}

bool LaneOp::MatchFrontBack(const LaneLine& query_lane_line,
                            const LaneLine& other_lane_line) {
  int n = static_cast<int>(other_lane_line.points_.size());
  if (n <= 1) {
    return false;
  }
  Eigen::MatrixXd A(n, 2);
  Eigen::VectorXd b(n);
  for (int j = 0; j < n; j++) {
    // 拟合直线
    A(j, 0) = other_lane_line.points_[j].x();
    A(j, 1) = 1.0;
    b(j) = other_lane_line.points_[j].y();
  }
  Eigen::Vector2d x = (A.transpose() * A).inverse() * A.transpose() * b;

  double sum_distance = 0;
  int count = 0;
  for (auto query_point : query_lane_line.points_) {
    if (query_point.x() > 50) {
      continue;
    }
    sum_distance += std::abs(x[0] * query_point.x() - query_point.y() + x[1]) /
                    std::sqrt(x[0] * x[0] + 1);
    count++;
  }
  return count > 0 && sum_distance / count <= 1;
}

bool LaneOp::MergePointsFrontBack(LaneLine* query_lane_line,
                                  LaneLine* other_lane_line) {
  std::vector<Eigen::Vector3d> new_points;
  if (other_lane_line->points_.front().x() >
      query_lane_line->points_.back().x()) {
    if (fabs(other_lane_line->points_.front().y() -
             query_lane_line->points_.back().y()) > 1) {
      return false;
    }
    for (const auto& point : query_lane_line->points_) {
      new_points.emplace_back(point);
    }
    double x_n = other_lane_line->points_.front().x();
    double y_n = other_lane_line->points_.front().y();
    double x_0 = query_lane_line->points_.back().x();
    double y_0 = query_lane_line->points_.back().y();
    double x = x_0;
    while (x < x_n) {
      double y = (y_n - y_0) / (x_n - x_0) * (x - x_0) + y_0;
      Eigen::Vector3d point = {x, y, 0.0};
      new_points.emplace_back(point);
      x++;
    }
    for (const auto& point : other_lane_line->points_) {
      new_points.emplace_back(point);
    }
  } else {
    if (fabs(query_lane_line->points_.front().y() -
             other_lane_line->points_.back().y()) > 1) {
      return false;
    }
    for (const auto& point : other_lane_line->points_) {
      new_points.emplace_back(point);
    }
    double x_n = query_lane_line->points_.front().x();
    double y_n = query_lane_line->points_.front().y();
    double x_0 = other_lane_line->points_.back().x();
    double y_0 = other_lane_line->points_.back().y();
    double x = x_0;
    while (x < x_n) {
      double y = (y_n - y_0) / (x_n - x_0) * (x - x_0) + y_0;
      Eigen::Vector3d point = {x, y, 0.0};
      new_points.emplace_back(point);
      x++;
    }
    for (const auto& point : query_lane_line->points_) {
      new_points.emplace_back(point);
    }
  }
  if (query_lane_line->track_id_ < other_lane_line->track_id_) {
    other_lane_line->track_id_ = query_lane_line->track_id_;
    other_lane_line->lanepos_ = query_lane_line->lanepos_;
    other_lane_line->lanetype_ = query_lane_line->lanetype_;
    other_lane_line->points_ = new_points;
  } else {
    other_lane_line->points_ = new_points;
  }
  query_lane_line->need_delete_ = true;
  return true;
}

void LaneOp::MergeMapFrontBack(LocalMap* local_map) {
  // 前后车道线合并
  for (auto& lane_line : local_map->lane_lines_) {
    lane_line.need_merge_ = !lane_line.points_.empty();
    lane_line.need_delete_ = false;
  }
  for (auto& query_lane_line : local_map->lane_lines_) {
    // if (query_lane.points_.back().x() < -50) continue;
    if (!query_lane_line.need_merge_) {
      continue;
    }
    query_lane_line.need_merge_ = false;
    for (auto& other_lane_line : local_map->lane_lines_) {
      if (!other_lane_line.need_merge_) {
        continue;
      }
      // HLOG_ERROR << "other_lane_line.c2_: " << other_lane_line.c2_;
      if ((query_lane_line.points_.front().x() >
           other_lane_line.points_.back().x() + 50) ||
          (other_lane_line.points_.front().x() >
           query_lane_line.points_.back().x() + 50) ||
          ((query_lane_line.points_.front().x() <
            other_lane_line.points_.back().x()) &&
           (query_lane_line.points_.back().x() >
            other_lane_line.points_.front().x())) ||
          (other_lane_line.c2_ > 0.001) ||
          query_lane_line.lanetype_ != other_lane_line.lanetype_) {
        continue;
      }
      if (!MatchFrontBack(query_lane_line, other_lane_line)) {
        continue;
      }
      if (!MergePointsFrontBack(&query_lane_line, &other_lane_line)) {
        continue;
      }
    }
  }
  for (int i = 0; i < static_cast<int>(local_map->lane_lines_.size()); i++) {
    if (!local_map->lane_lines_[i].need_delete_) {
      continue;
    }
    local_map->lane_lines_.erase(local_map->lane_lines_.begin() + i);
    i--;
  }
}

ConstDrDataPtr LaneOp::GetDrPoseForTime(const double& timestamp) {
  auto& local_data = LocalDataSingleton::GetInstance();

  ::std::list<::std::pair<double, ConstDrDataPtr>> dr_list;
  local_data.dr_buffer().get_all_messages(&dr_list);

  if (dr_list.size() <= 1) {
    HLOG_ERROR << "too few dr data, can't interpolate";
    return nullptr;
  }

  auto iter = dr_list.rbegin();
  for (; iter != dr_list.rend(); iter++) {
    if (iter->first < timestamp) {
      // HLOG_ERROR << "Dr time: " << std::setprecision(20) << iter->first;
      break;
    }
  }

  if (iter == dr_list.rbegin()) {
    if (timestamp - dr_list.back().second->timestamp > 0.2) {
      HLOG_ERROR << "Dr delay";
      return nullptr;
    }

    DrDataPtr dr_ptr = std::make_shared<DrData>();
    ConstDrDataPtr latest_data_ptr = dr_list.back().second;

    double delta_t = timestamp - latest_data_ptr->timestamp;
    dr_ptr->timestamp = timestamp;
    dr_ptr->pose =
        latest_data_ptr->pose +
        latest_data_ptr->quaternion * (latest_data_ptr->local_vel * delta_t);
    dr_ptr->quaternion = latest_data_ptr->quaternion;
    Eigen::Vector3d delta_ang = latest_data_ptr->local_omg * delta_t;
    if (delta_ang.norm() > 1e-12) {
      dr_ptr->quaternion =
          dr_ptr->quaternion * Eigen::Quaterniond(Eigen::AngleAxisd(
                                   delta_ang.norm(), delta_ang.normalized()));
      dr_ptr->quaternion = dr_ptr->quaternion.normalized();
    }

    dr_ptr->local_vel = latest_data_ptr->local_vel;
    dr_ptr->local_omg = latest_data_ptr->local_omg;

    return dr_ptr;
  }

  if (iter == dr_list.rend()) {
    HLOG_ERROR << "Dr lost";
    return nullptr;
  }

  auto pre = iter;
  auto next = --iter;
  double ratio = (timestamp - pre->first) / (next->first - pre->first);
  auto dr_pose_state = std::make_shared<DrData>(CommonUtil::Interpolate(
      ratio, *(pre->second), *(next->second), timestamp));
  return dr_pose_state;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
