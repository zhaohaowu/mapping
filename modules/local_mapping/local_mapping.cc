/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#include "modules/local_mapping/local_mapping.h"

#include <string>

#include "Eigen/src/Core/Matrix.h"
#include "modules/local_mapping/datalogger/load_data_singleton.h"
#include "modules/local_mapping/utils/data_convert.h"
#include "perception-base/base/utils/log.h"
namespace hozon {
namespace mp {
namespace lm {

namespace perception_base = hozon::perception::base;
namespace perception_lib = hozon::perception::lib;

LMapApp::LMapApp(const std::string& mapping_path,
                 const std::string& config_file) {
  YAML::Node config = YAML::LoadFile(config_file);
  use_rviz_ = config["use_rviz"].as<bool>();
  local_map_ptr_ = std::make_shared<LocalMap>();
  mmgr_ = std::make_shared<MapManager>();

  // 车道线后处理初始化
  lane_postprocessor_ = std::make_unique<environment::LanePostProcess>();
  environment::ProcessInitOption init_option;
  CHECK(lane_postprocessor_->Init(init_option));
  HLOG_DEBUG << "lane_postprocessor_ init successfully";

  // 路沿后处理初始化
  roadedge_postprocessor_ =
      std::make_unique<environment::RoadEdgePostProcess>();
  CHECK(roadedge_postprocessor_->Init(init_option));

  // 地面标识后处理初始化
  roadmark_postprocessor_ =
      std::make_unique<environment::RoadMarkPostProcess>();
  // CHECK(roadmark_postprocessor_->Init(init_option));

  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    // load city hdmap
    provider_ = std::make_shared<PriorProvider>();
    provider_->Init(mapping_path, config_file);
    auto map = provider_->GetPrior();
    if (map) {
      hdmap_ = std::make_shared<hozon::hdmap::HDMap>();
      hdmap_->LoadMapFromProto(*map);
    }
    stop_rviz_thread_.store(false);
    rviz_thread_ = std::thread(&LMapApp::RvizFunc, this);
  }
}

LMapApp::~LMapApp() {
  if (use_rviz_ && RVIZ_AGENT.Ok() && rviz_thread_.joinable()) {
    stop_rviz_thread_.store(true);
    rviz_thread_.join();
  }
}

void LMapApp::RvizFunc() {
  while (!stop_rviz_thread_.load()) {
    T_mutex_.lock();
    Sophus::SE3d T_W_V = T_W_V_;
    Sophus::SE3d T_G_V = T_G_V_;
    T_mutex_.unlock();
    localmap_mutex_.lock();
    LocalMap local_map = local_map_output_;
    localmap_mutex_.unlock();
    perception_mutex_.lock();
    Perception perception = perception_;
    perception_mutex_.unlock();
    auto sec = static_cast<uint64_t>(perception.timestamp_);
    auto nsec = static_cast<uint64_t>(
        (perception.timestamp_ - static_cast<double>(sec)) * 1e9);
    static int rviz_count = 0;
    rviz_count++;
    if (rviz_count >= 10 && ins_inited_) {
      rviz_count = 0;
      if (hdmap_) {
        RvizUtil::PubHdMapPoints(T_G_V, T_W_V, hdmap_, sec, nsec,
                                 "/localmap/hd_map_points");
      }
      RvizUtil::PubHqMapPoints(T_G_V, T_W_V, sec, nsec,
                               "/localmap/hq_map_points");
    }
    RvizUtil::PubPerLaneLine(T_W_V, perception.lane_lines_, sec, nsec,
                             "/localmap/per_lane_line");
    RvizUtil::PubPerRoadEdge(T_W_V, perception.road_edges_, sec, nsec,
                             "/localmap/per_road_edge");
    RvizUtil::PubPerStopLine(T_W_V, perception.stop_lines_, sec, nsec,
                             "/localmap/per_stop_line");
    RvizUtil::PubPerArrow(T_W_V, perception.arrows_, sec, nsec,
                          "/localmap/per_arrow");
    RvizUtil::PubPerZebraCrossing(T_W_V, perception.zebra_crossings_, sec, nsec,
                                  "/localmap/per_zebra_crossing");

    RvizUtil::PubMapLaneLine(T_W_V, local_map.lane_lines_, sec, nsec,
                             "/localmap/map_lane_line");
    RvizUtil::PubMapLaneLineOri(T_W_V, local_map.lane_lines_, sec, nsec,
                                "/localmap/map_lane_line_ori");
    RvizUtil::PubMapLaneLineMarker(T_W_V, local_map.lane_lines_, sec, nsec,
                                   "/localmap/map_lane_line/marker");
    RvizUtil::PubMapRoadEdge(T_W_V, local_map.road_edges_, sec, nsec,
                             "/localmap/map_road_edge");
    RvizUtil::PubMapRoadEdgeMarker(T_W_V, local_map.road_edges_, sec, nsec,
                                   "/localmap/map_road_edge/marker");
    RvizUtil::PubMapStopLine(T_W_V, local_map.stop_lines_, sec, nsec,
                             "/localmap/map_stop_line");
    RvizUtil::PubMapArrow(T_W_V, local_map.arrows_, sec, nsec,
                          "/localmap/map_arrow");
    RvizUtil::PubMapZebraCrossing(T_W_V, local_map.zebra_crossings_, sec, nsec,
                                  "/localmap/map_zebra_crossing");

    RvizUtil::PubImmatureMapLaneLine(T_W_V, local_map.lane_lines_, sec, nsec,
                                     "/localmap/immature_map_lane_line");
    RvizUtil::PubImmatureMapLaneLineMarker(
        T_W_V, local_map.lane_lines_, sec, nsec,
        "/localmap/immature_map_lane_line/marker");
    RvizUtil::PubImmatureMapZebraCrossing(
        T_W_V, local_map.zebra_crossings_, sec, nsec,
        "/localmap/immature_map_zebra_crossing");
    RvizUtil::PubImmatureMapZebraCrossingMarker(
        T_W_V, local_map.zebra_crossings_, sec, nsec,
        "/localmap/immature_map_zebra_crossing/marker");
    RvizUtil::PubImmatureMapStopLine(T_W_V, local_map.stop_lines_, sec, nsec,
                                     "/localmap/immature_map_stop_lines");
    RvizUtil::PubImmatureMapStopLineMarker(
        T_W_V, local_map.stop_lines_, sec, nsec,
        "/localmap/immature_map_stop_lines/marker");
    RvizUtil::PubImmatureMapArrow(T_W_V, local_map.arrows_, sec, nsec,
                                  "/localmap/immature_map_arrow");
    RvizUtil::PubImmatureMapArrowMarker(T_W_V, local_map.arrows_, sec, nsec,
                                        "/localmap/immature_map_arrow/marker");

    RvizUtil::PubMapLaneLineControl(T_W_V, local_map, sec, nsec,
                                    "/localmap/map_lane_line/control");
    RvizUtil::PubOdom(T_W_V, sec, nsec, "/localmap/odom");
    RvizUtil::PubTf(T_W_V, sec, nsec, "/localmap/tf");
    RvizUtil::PubPath(T_W_V, sec, nsec, "/localmap/path");
    usleep(100 * 1e3);
  }
}

void LMapApp::OnDr(const std::shared_ptr<hozon::perception::base::Location>&
                       latest_localization) {
  hozon::perception::lib::LocationManager::Instance()->Push(
      *latest_localization);
}

void LMapApp::OnLocalization(
    const std::shared_ptr<const Localization>& latest_localization) {
  Sophus::SE3d T_W_V_localization = Sophus::SE3d(
      latest_localization->quaternion_, latest_localization->position_);
  static double last_time = -1;
  Eigen::Vector3d trans_pose = T_W_V_localization.translation();
  Eigen::Quaterniond trans_quat = T_W_V_localization.so3().unit_quaternion();
  auto& local_data = LocalDataSingleton::GetInstance();
  HLOG_INFO << "latest_localization buffer size: "
            << local_data.dr_buffer().buffer_size();
  HLOG_INFO << "latest_localization timestamp: " << SET_PRECISION(20)
            << latest_localization->timestamp_;
  if (local_data.dr_buffer().is_empty() ||
      local_data.dr_buffer().back()->timestamp <
          latest_localization->timestamp_) {
    DrDataPtr dr_data_ptr = std::make_shared<DrData>();
    dr_data_ptr->timestamp = latest_localization->timestamp_;
    dr_data_ptr->pose = trans_pose;
    dr_data_ptr->quaternion = trans_quat;
    dr_data_ptr->local_omg = latest_localization->angular_vrf_;
    dr_data_ptr->local_vel = latest_localization->linear_vrf_;
    local_data.dr_buffer().push_new_message(latest_localization->timestamp_,
                                            dr_data_ptr);
    last_time = latest_localization->timestamp_;
  } else {
    HLOG_ERROR << "localization timestamp error";
  }
  if (!localization_inited_) {
    localization_inited_ = true;
  }
}

void LMapApp::OnIns(const std::shared_ptr<const InsData>& msg) {
  Sophus::SE3d T_G_V = Sophus::SE3d(msg->quaternion_, msg->position_);
  T_mutex_.lock();
  T_G_V_ = T_G_V;
  T_mutex_.unlock();
  if (!ins_inited_) {
    ins_inited_ = true;
  }
}

void LMapApp::ProcLaneLine(
    const std::shared_ptr<const Perception>& perception) {
  std::vector<LaneLineMatchInfo> lane_line_matches;
  mmgr_->MatchLaneLine(perception->lane_lines_, local_map_ptr_->lane_lines_,
                       &lane_line_matches);
  for (const auto& match : lane_line_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      mmgr_->AddNewLaneLine(local_map_ptr_.get(), match.per_lane_line_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldLaneLine(local_map_ptr_.get(), match.per_lane_line_,
                              match.map_lane_line_);
    }
  }
  mmgr_->DeleteOldLaneLinePoints(local_map_ptr_.get(), lane_line_matches);
}

void LMapApp::ProcRoadEdge(
    const std::shared_ptr<const Perception>& perception) {
  std::vector<RoadEdgeMatchInfo> road_edge_matches;
  mmgr_->MatchRoadEdge(perception->road_edges_, local_map_ptr_->road_edges_,
                       &road_edge_matches);
  for (const auto& match : road_edge_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      mmgr_->AddNewRoadEdge(local_map_ptr_.get(), match.per_road_edge_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldRoadEdge(local_map_ptr_.get(), match.per_road_edge_,
                              match.map_road_edge_);
    }
  }
}

// void LMapApp::PreProcArrow(const std::shared_ptr<Perception>& perception) {
//   int input_size = static_cast<int>(perception->arrows_.size());
//   ConstObjDataPtr msg;
//   if (LocalDataSingleton::GetInstance().GetObjByTimeStamp(
//           perception->timestamp_, &msg)) {
//     HLOG_INFO << "***msg-objs***" << msg->objs_.size();
//     perception->arrows_.erase(
//         std::remove_if(
//             perception->arrows_.begin(), perception->arrows_.end(),
//             [&](const Arrow& arrow) {
//               Eigen::Vector3d center = arrow.mid_point_;
//               Eigen::Vector3d size = {arrow.width_, arrow.length_, 0.0};
//               return CommonUtil::IsOcclusionByObstacle(center, size,
//                                                        msg->objs_);
//             }),
//         perception->arrows_.end());
//   }

//   int out_size = static_cast<int>(perception->arrows_.size());

//   if ((input_size - out_size) != 0) {
//     HLOG_INFO << "***delete input arrow size***" << input_size - out_size;
//   }
// }

void LMapApp::ProcArrow(const std::shared_ptr<const Perception>& perception) {
  std::vector<ArrowMatchInfo> arrow_matches;
  mmgr_->MatchArrow(perception->arrows_, local_map_ptr_->arrows_,
                    &arrow_matches);
  for (const auto& match : arrow_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      mmgr_->AddNewArrow(local_map_ptr_.get(), match.per_arrow_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldArrow(local_map_ptr_.get(), match.per_arrow_,
                           match.map_arrow_);
    }
  }
}

void LMapApp::ProcZebraCrossing(
    const std::shared_ptr<const Perception>& perception) {
  std::vector<ZebraCrossingMatchInfo> zebra_crossing_matches;
  mmgr_->MatchZebraCrossing(perception->zebra_crossings_,
                            local_map_ptr_->zebra_crossings_,
                            &zebra_crossing_matches);

  for (const auto& match : zebra_crossing_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      mmgr_->AddNewZebraCrossing(local_map_ptr_.get(),
                                 match.per_zebra_crossing_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldZebraCrossing(local_map_ptr_.get(),
                                   match.per_zebra_crossing_,
                                   match.map_zebra_crossing_);
    }
  }
}

void LMapApp::ProcStopLine(
    const std::shared_ptr<const Perception>& perception) {
  std::vector<StopLineMatchInfo> stop_line_matches;
  mmgr_->MatchStopLine(perception->stop_lines_, local_map_ptr_->stop_lines_,
                       &stop_line_matches);
  for (const auto& match : stop_line_matches) {
    if (match.update_type_ == ObjUpdateType::ADD_NEW) {
      mmgr_->AddNewStopLine(local_map_ptr_.get(), match.per_stop_line_);
    } else if (match.update_type_ == ObjUpdateType::MERGE_OLD) {
      mmgr_->MergeOldStopLine(local_map_ptr_.get(), match.per_stop_line_,
                              match.map_stop_line_);
    }
  }
}
bool LMapApp::DoPostProcess(
    hozon::perception::base::MeasurementFramePtr measurement_frame,
    hozon::perception::base::FusionFramePtr fusion_frame) {
  fusion_frame->header.timestamp = measurement_frame->header.timestamp;
  fusion_frame->header.sequence_num = measurement_frame->header.sequence_num;
  // 输入数据放入输入数据管理模块中。
  double ts = measurement_frame->header.timestamp;
  perception_base::LocationPtr cur_location =
      std::make_shared<perception_base::Location>();
  bool ret =
      perception_lib::LocationManager::Instance()->GetLocationByTimestamp(
          ts, cur_location.get());
  if (!ret) {
    HLOG_ERROR << "get location msg failed...";
    return false;
  }

  // 车道线后处理输入观测数据保存
  environment::InputDataSingleton* local_data_ =
      environment::InputDataSingleton::Instance();
  local_data_->dr_data_buffer_.push_new_message(ts, cur_location);
  local_data_->roadedges_buffer_.push_new_message(
      ts, measurement_frame->roadedges_measurement_->road_edges);
  local_data_->lanes_buffer_.push_new_message(
      ts, measurement_frame->lanelines_measurement_->lanelines);

  // 车道线后处理和结果转换
  HLOG_DEBUG << "do laneline postprocess...";
  if (!lane_postprocessor_) {
    HLOG_ERROR << "lane_postprocessor_ is nullptr";
    return -1;
  }
  lane_postprocessor_->Process(measurement_frame, fusion_frame);
  HLOG_DEBUG << "output track laneline nums trans to pb:"
             << fusion_frame->scene_->lane_lines->lanelines.size();

  // 路沿后处理和结果转换
  HLOG_DEBUG << "do roadedge_postprocess...";
  if (!roadedge_postprocessor_) {
    HLOG_ERROR << "roadedge_postprocessor_ is nullptr";
    return -1;
  }
  roadedge_postprocessor_->Process(measurement_frame, fusion_frame);
  HLOG_DEBUG << "output track road_edges nums trans to pb:"
             << fusion_frame->scene_->road_edges->road_edges.size();

  HLOG_DEBUG << "do roadmark_postprocess...";
  if (!roadmark_postprocessor_) {
    HLOG_ERROR << "roadmark_postprocessor_ is nullptr";
    return -1;
  }
  roadmark_postprocessor_->Process(measurement_frame, fusion_frame);

  HLOG_DEBUG << "output track stop_lines nums trans to pb:"
             << fusion_frame->scene_->stop_lines->stoplines.size();
  HLOG_DEBUG << "output track noparkings nums trans to pb:"
             << fusion_frame->scene_->no_parkings->noparkings.size();
  HLOG_DEBUG << "output track road_arrows nums trans to pb:"
             << fusion_frame->scene_->road_arrows->arrows.size();
  HLOG_DEBUG << "output track slow_downs nums trans to pb:"
             << fusion_frame->scene_->slow_downs->slow_downs.size();
  HLOG_DEBUG << "output track stoplines nums trans to pb:"
             << fusion_frame->scene_->stop_lines->stoplines.size();
  HLOG_DEBUG << "output track waitzones nums trans to pb:"
             << fusion_frame->scene_->wait_zones->waitzones.size();
  HLOG_DEBUG << "output track waitzones nums trans to pb:"
             << fusion_frame->scene_->zebra_crossings->crosswalks.size();

  return true;
}

void LMapApp::DoBuildMap(const std::shared_ptr<Perception>& perception) {
  ConstDrDataPtr perception_pose =
      LocalDataSingleton::GetInstance().GetDrPoseByTimeStamp(
          perception->timestamp_);
  if (perception_pose == nullptr) {
    HLOG_ERROR << "perception time is:"
               << std::to_string(perception->timestamp_);
    HLOG_ERROR << "perception_pose is nullptr";
    return;
  }
  cur_twv_ = Sophus::SE3d(perception_pose->quaternion, perception_pose->pose);
  Sophus::SE3d T_C_L = cur_twv_.inverse() * last_twv_;
  last_twv_ = cur_twv_;
  mmgr_->UpdateLocalMap(local_map_ptr_.get(), T_C_L);
  local_map_ptr_->timestamp_ = perception->timestamp_;

  ProcLaneLine(perception);
  ProcRoadEdge(perception);
  ProcStopLine(perception);
  ProcZebraCrossing(perception);

  // PreProcArrow(perception);
  ProcArrow(perception);

  mmgr_->UpdateLaneLinepos(local_map_ptr_.get());
  mmgr_->UpdateRoadEdgepos(local_map_ptr_.get());
  mmgr_->MergeLaneLine(local_map_ptr_.get());
  mmgr_->MergeRoadEdge(local_map_ptr_.get());
  mmgr_->MergeZebraCrossing(local_map_ptr_.get());
  mmgr_->MergeArrow(local_map_ptr_.get());
  mmgr_->MergeStopLine(local_map_ptr_.get());

  CommonUtil::FitLaneLines(&local_map_ptr_->lane_lines_);
  CommonUtil::FitRoadEdges(&local_map_ptr_->road_edges_);
  mmgr_->CutLocalMap(local_map_ptr_.get(), 80, 80);
  if (use_rviz_ && RVIZ_AGENT.Ok()) {
    localmap_mutex_.lock();
    local_map_output_ = *local_map_ptr_;
    localmap_mutex_.unlock();
    perception_mutex_.lock();
    perception_ = *perception;
    perception_mutex_.unlock();
    T_mutex_.lock();
    T_W_V_ = cur_twv_;
    T_mutex_.unlock();
  }
  if (!perception_inited_) {
    perception_inited_ = true;
  }
}

bool LMapApp::FetchLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map) {
  if (!localization_inited_) {
    HLOG_ERROR << "==========localization_inited_ failed=======";
    return false;
  }
  if (!perception_inited_) {
    HLOG_ERROR << "==========perception_inited_ failed=======";
    return false;
  }
  local_map->mutable_header()->set_seq(seq_++);
  local_map->set_init_timestamp(local_map_ptr_->timestamp_);
  local_map->mutable_header()->set_gnss_stamp(local_map_ptr_->timestamp_);
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  local_map->mutable_header()->set_publish_stamp(
      static_cast<double>(tp.time_since_epoch().count()) * 1.0e-9);
  local_map->mutable_header()->set_data_stamp(local_map_ptr_->timestamp_);

  DataConvert::ConvertMultiLaneLinesToPb(local_map_ptr_->lane_lines_,
                                         local_map);
  DataConvert::ConvertMultiRoadEdgesToPb(local_map_ptr_->road_edges_,
                                         local_map);
  DataConvert::ConvertMultiStopLinesToPb(local_map_ptr_->stop_lines_,
                                         local_map);
  DataConvert::ConvertMultiArrowsToPb(local_map_ptr_->arrows_, local_map);
  DataConvert::ConvertMultiZebraCrossingsToPb(local_map_ptr_->zebra_crossings_,
                                              local_map);
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
