/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei/zhaohaowu
 *Date: 2023-09-01
 *****************************************************************************/
#include "modules/local_mapping/core/map_worker.h"

#include <memory>
#include <string>

#include "modules/local_mapping/lib/datalogger/map_manager.h"
#include "modules/local_mapping/lib/datalogger/pose_manager.h"

namespace hozon {
namespace mp {
namespace lm {

bool MapWorker::Init() {
  // 车道线pipeline模块初始化
  laneline_mapping_pl_ptr_ = std::make_unique<LaneLineMappingPipeline>();
  CHECK(laneline_mapping_pl_ptr_ != nullptr);
  CHECK(laneline_mapping_pl_ptr_->Init());
  HLOG_DEBUG << "init  tracker name: " << laneline_mapping_pl_ptr_->Name();

  // 路沿pipeline模块初始化
  roadedge_mapping_pl_ptr_ = std::make_unique<RoadEdgeMappingPipeline>();
  CHECK(roadedge_mapping_pl_ptr_ != nullptr);
  CHECK(roadedge_mapping_pl_ptr_->Init());
  HLOG_DEBUG << "init tracker name: " << roadedge_mapping_pl_ptr_->Name();

  // 路面箭头pipeline模块初始化
  arrow_mapping_pl_ptr_ = std::make_unique<ArrowMappingPipeline>();
  CHECK(arrow_mapping_pl_ptr_ != nullptr);
  CHECK(arrow_mapping_pl_ptr_->Init());
  HLOG_DEBUG << "init  tracker name: " << arrow_mapping_pl_ptr_->Name();

  // 停止线pipeline模块初始化
  stopline_mapping_pl_ptr_ = std::make_unique<StopLineMappingPipeline>();
  CHECK(stopline_mapping_pl_ptr_ != nullptr);
  CHECK(stopline_mapping_pl_ptr_->Init());
  HLOG_DEBUG << "init  tracker name: " << stopline_mapping_pl_ptr_->Name();

  // 斑马线pipeline模块初始化
  zebracrossing_mapping_pl_ptr_ =
      std::make_unique<ZebraCrossingMappingPipeline>();
  CHECK(zebracrossing_mapping_pl_ptr_ != nullptr);
  CHECK(zebracrossing_mapping_pl_ptr_->Init());
  HLOG_DEBUG << "init  tracker name: " << zebracrossing_mapping_pl_ptr_->Name();

  return true;
}

bool MapWorker::Process(const MeasurementFrameConstPtr& measurement_frame_ptr) {
  if (POSE_MANAGER->IsStaticState()) {
    LocalMapFramePtr last_localmap_data = MAP_MANAGER->GetWriteLocalMap();
    if (!static_copy_map_once_) {
      LocalMapFramePtr read_localmap_data = MAP_MANAGER->GetLocalMap();
      DeepCopy(read_localmap_data, last_localmap_data);  // 确保读和写的数据一致
      static_copy_map_once_ = !static_copy_map_once_;
    }
    last_localmap_data->header.timestamp =
        measurement_frame_ptr->header.timestamp;
    last_localmap_data->header.sequence_num =
        measurement_frame_ptr->header.sequence_num;
    MAP_MANAGER->SwapLocalMap();
    return true;
  }
  static_copy_map_once_ = false;
  // 1. 清空地图数据
  auto local_map_ptr_ = MAP_MANAGER->GetWriteLocalMap();
  ClearLocalMap(local_map_ptr_);
  // 更新一下数据面时间和序列号
  local_map_ptr_->header.timestamp = measurement_frame_ptr->header.timestamp;
  local_map_ptr_->header.sequence_num =
      measurement_frame_ptr->header.sequence_num;

  ProcessOption option;
  option.timestamp = measurement_frame_ptr->header.timestamp;
  // 2. 车道线的建图过程
  HLOG_DEBUG << "start do laneline mapping process...";
  laneline_mapping_pl_ptr_->Process(option, measurement_frame_ptr,
                                    local_map_ptr_);
  HLOG_DEBUG << "finish do laneline mapping process...";
  // // 3. 路沿的建图过程
  HLOG_DEBUG << "start do roadedge mapping process...";
  roadedge_mapping_pl_ptr_->Process(option, measurement_frame_ptr,
                                    local_map_ptr_);
  HLOG_DEBUG << "finish do roadedge mapping process...";
  // // 4. 斑马线的建图过程
  HLOG_DEBUG << "start do zebracrossing mapping process...";
  zebracrossing_mapping_pl_ptr_->Process(option, measurement_frame_ptr,
                                         local_map_ptr_);
  HLOG_DEBUG << "finish do zebracrossing mapping process...";
  // 5. 停止线的建图过程
  HLOG_DEBUG << "start do stopline mapping process...";
  stopline_mapping_pl_ptr_->Process(option, measurement_frame_ptr,
                                    local_map_ptr_);
  HLOG_DEBUG << "finish do stopline mapping process...";
  // 6. 路面箭头的建图过程
  HLOG_DEBUG << "start do arrow mapping process...";
  arrow_mapping_pl_ptr_->Process(option, measurement_frame_ptr, local_map_ptr_);
  HLOG_DEBUG << "finish do arrow mapping process...";
  CutLocalMap(local_map_ptr_, 80, 80);

  // 7. 将历史数据存放到单例司数据管理器中（方便去拿历史数据）。
  MAP_MANAGER->SwapLocalMap();
  return true;
}

void MapWorker::ClearLocalMap(const LocalMapFramePtr& local_map_ptr_) {
  if (!local_map_ptr_->lane_lines_ptr) {
    local_map_ptr_->lane_lines_ptr->lanelines.clear();
  }

  if (!local_map_ptr_->road_edges_ptr) {
    local_map_ptr_->road_edges_ptr->road_edges.clear();
  }

  if (!local_map_ptr_->road_arrows_ptr) {
    local_map_ptr_->road_arrows_ptr->arrows.clear();
  }

  if (!local_map_ptr_->stop_lines_ptr) {
    local_map_ptr_->stop_lines_ptr->stoplines.clear();
  }

  if (!local_map_ptr_->zebra_crossings_ptr) {
    local_map_ptr_->zebra_crossings_ptr->zebra_crossings.clear();
  }
}

void MapWorker::CutLocalMap(std::shared_ptr<LocalMapFrame> local_map_ptr,
                            const double& length_x, const double& length_y) {
  // 车道线目标范围裁剪
  for (auto& lane_line_ptr : local_map_ptr->lane_lines_ptr->lanelines) {
    auto& vehicle_points = lane_line_ptr->vehicle_points;
    vehicle_points.erase(
        std::remove_if(vehicle_points.begin(), vehicle_points.end(),
                       [&](Eigen::Vector3d& point) {
                         return (point.x() < -length_x ||
                                 point.x() > length_x ||
                                 point.y() < -length_y || point.y() > length_y);
                       }),
        vehicle_points.end());
  }

  // 路沿目标范围裁剪
  for (auto& roadedge_ptr : local_map_ptr->road_edges_ptr->road_edges) {
    auto& vehicle_points = roadedge_ptr->vehicle_points;
    vehicle_points.erase(
        std::remove_if(vehicle_points.begin(), vehicle_points.end(),
                       [&](Eigen::Vector3d& point) {
                         return (point.x() < -length_x ||
                                 point.x() > length_x ||
                                 point.y() < -length_y || point.y() > length_y);
                       }),
        vehicle_points.end());
  }

  // 删除目标区域外停止线
  auto& stoplines_ptr = local_map_ptr->stop_lines_ptr->stoplines;
  stoplines_ptr.erase(
      std::remove_if(stoplines_ptr.begin(), stoplines_ptr.end(),
                     [&](const StopLinePtr& stop_line_ptr) {
                       return stop_line_ptr->center_point.x() < -length_x ||
                              stop_line_ptr->center_point.x() > length_x ||
                              stop_line_ptr->center_point.y() < -length_y ||
                              stop_line_ptr->center_point.y() > length_y;
                     }),
      stoplines_ptr.end());

  // 删除目标区域外箭头
  auto& arrows_ptr = local_map_ptr->road_arrows_ptr->arrows;
  arrows_ptr.erase(
      std::remove_if(arrows_ptr.begin(), arrows_ptr.end(),
                     [&](const ArrowPtr& arrow_ptr) {
                       return arrow_ptr->center_point.x() < -length_x ||
                              arrow_ptr->center_point.x() > length_x ||
                              arrow_ptr->center_point.y() < -length_y ||
                              arrow_ptr->center_point.y() > length_y;
                     }),
      arrows_ptr.end());

  // 删除目标区域外斑马线
  auto& zebracrossings_ptr =
      local_map_ptr->zebra_crossings_ptr->zebra_crossings;
  zebracrossings_ptr.erase(
      std::remove_if(
          zebracrossings_ptr.begin(), zebracrossings_ptr.end(),
          [&](const ZebraCrossingPtr& zebra_crossing_ptr) {
            return zebra_crossing_ptr->center_point.x() < -length_x ||
                   zebra_crossing_ptr->center_point.x() > length_x ||
                   zebra_crossing_ptr->center_point.y() < -length_y ||
                   zebra_crossing_ptr->center_point.y() > length_y;
          }),
      zebracrossings_ptr.end());
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
