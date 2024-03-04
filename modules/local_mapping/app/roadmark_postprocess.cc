/*================================================================
*   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
*   file       ：lane_post_process.h
*   author     ：Chenanmeng
*   date       ：2023.02.28
================================================================*/
#include "modules/local_mapping/app/roadmark_postprocess.h"

#include "depend/perception-base/base/scene/arrow.h"
#include "depend/perception-base/base/scene/noparking.h"
#include "depend/perception-base/base/scene/slowdown.h"
#include "depend/perception-base/base/scene/waitzone.h"
#include "depend/perception-base/base/scene/zebra_crossing.h"
#include "depend/perception-base/base/utils/log.h"

namespace hozon {
namespace mp {
namespace environment {

bool RoadMarkPostProcess::Process(
    const hozon::perception::base::MeasurementFramePtr measurement_ptr,
    hozon::perception::base::FusionFramePtr fusion_ptr) {
  HLOG_DEBUG << "Start RoadMarkPostProcess Working...";
  if (!fusion_ptr->scene_) {
    fusion_ptr->scene_ = std::make_shared<hozon::perception::base::Scene>();
  }

  if (!fusion_ptr->scene_->road_arrows) {
    fusion_ptr->scene_->road_arrows =
        std::make_shared<hozon::perception::base::Arrows>();
  }

  if (!fusion_ptr->scene_->no_parkings) {
    fusion_ptr->scene_->no_parkings =
        std::make_shared<hozon::perception::base::NoParkings>();
  }

  if (!fusion_ptr->scene_->slow_downs) {
    fusion_ptr->scene_->slow_downs =
        std::make_shared<hozon::perception::base::SlowDowns>();
  }

  if (!fusion_ptr->scene_->zebra_crossings) {
    fusion_ptr->scene_->zebra_crossings =
        std::make_shared<hozon::perception::base::ZebraCrossings>();
  }

  if (!fusion_ptr->scene_->stop_lines) {
    fusion_ptr->scene_->stop_lines =
        std::make_shared<hozon::perception::base::StopLines>();
  }

  if (!fusion_ptr->scene_->wait_zones) {
    fusion_ptr->scene_->wait_zones =
        std::make_shared<hozon::perception::base::WaitZones>();
  }

  ProcessArrows(measurement_ptr->arrows_measurement_,
                fusion_ptr->scene_->road_arrows);

  ProcessNoParkings(measurement_ptr->noparkings_measurement_,
                    fusion_ptr->scene_->no_parkings);

  ProcessSlowDowns(measurement_ptr->slowdowns_measurement_,
                   fusion_ptr->scene_->slow_downs);

  ProcessWaitZones(measurement_ptr->waitzones_measurement_,
                   fusion_ptr->scene_->wait_zones);

  ProcessZebraCrossings(measurement_ptr->zebracrossings_measurement_,
                        fusion_ptr->scene_->zebra_crossings);

  ProcessStopLines(measurement_ptr->stoplines_measurement_,
                   fusion_ptr->scene_->stop_lines);

  HLOG_DEBUG << "End RoadMarkPostProcess Working...";
  return true;
}

bool RoadMarkPostProcess::ProcessNoParkings(
    const hozon::perception::base::NoparkingsMeasurementPtr detect_noparkings,
    hozon::perception::base::NoParkingsPtr track_noparkings) {
  track_noparkings->noparkings.clear();

  if (!detect_noparkings) {
    return true;
  }

  for (const auto& detect_noparking : detect_noparkings->noparkings) {
    std::shared_ptr<hozon::perception::base::NoParking> noparking_ptr =
        std::make_shared<hozon::perception::base::NoParking>();
    noparking_ptr->id = detect_noparking->id;
    noparking_ptr->confidence = detect_noparking->confidence;
    // waitzone_ptr->heading = detect_zone->heading;
    noparking_ptr->point_set_2d = detect_noparking->point_set_2d;
    noparking_ptr->point_set_3d = detect_noparking->point_set_3d;
    track_noparkings->noparkings.push_back(noparking_ptr);
  }

  return true;
}

bool RoadMarkPostProcess::ProcessArrows(
    const hozon::perception::base::ArrowsMeasurementPtr detect_arrows,
    hozon::perception::base::ArrowsPtr track_arrows) {
  track_arrows->arrows.clear();
  if (!detect_arrows) {
    return true;
  }

  for (const auto& detect_arrow : detect_arrows->arrows) {
    std::shared_ptr<hozon::perception::base::Arrow> arrow_ptr =
        std::make_shared<hozon::perception::base::Arrow>();
    arrow_ptr->id = detect_arrow->id;
    arrow_ptr->type = detect_arrow->type;
    arrow_ptr->confidence = detect_arrow->confidence;
    arrow_ptr->heading = detect_arrow->heading;
    arrow_ptr->point_set_2d = detect_arrow->point_set_2d;
    arrow_ptr->point_set_3d = detect_arrow->point_set_3d;
    track_arrows->arrows.push_back(arrow_ptr);
  }

  return true;
}

bool RoadMarkPostProcess::ProcessSlowDowns(
    const hozon::perception::base::SlowdownsMeasurementPtr detect_slowdowns,
    hozon::perception::base::SlowDownsPtr track_slowdowns) {
  track_slowdowns->slow_downs.clear();
  if (!detect_slowdowns) {
    return true;
  }
  for (const auto& detect_slowdown : detect_slowdowns->slowdowns) {
    std::shared_ptr<hozon::perception::base::SlowDown> slowdown_ptr =
        std::make_shared<hozon::perception::base::SlowDown>();
    slowdown_ptr->id = detect_slowdown->id;
    slowdown_ptr->confidence = detect_slowdown->confidence;
    // slowdown_ptr->heading = detect_zone->heading;
    slowdown_ptr->point_set_2d = detect_slowdown->point_set_2d;
    slowdown_ptr->point_set_3d = detect_slowdown->point_set_3d;
    track_slowdowns->slow_downs.push_back(slowdown_ptr);
  }

  return true;
}

bool RoadMarkPostProcess::ProcessWaitZones(
    const hozon::perception::base::WaitZonesMeasurementPtr detect_waitzones,
    hozon::perception::base::WaitZonesPtr track_waitzones) {
  track_waitzones->waitzones.clear();

  if (!detect_waitzones) {
    return true;
  }
  for (const auto& detect_zone : detect_waitzones->waitzones) {
    std::shared_ptr<hozon::perception::base::WaitZone> waitzone_ptr =
        std::make_shared<hozon::perception::base::WaitZone>();
    waitzone_ptr->id = detect_zone->id;
    waitzone_ptr->confidence = detect_zone->confidence;
    // waitzone_ptr->heading = detect_zone->heading;
    waitzone_ptr->point_set_2d = detect_zone->point_set_2d;
    waitzone_ptr->point_set_3d = detect_zone->point_set_3d;
    track_waitzones->waitzones.push_back(waitzone_ptr);
  }

  return true;
}

bool RoadMarkPostProcess::ProcessStopLines(
    const hozon::perception::base::StopLinesMeasurementPtr detect_stoplines,
    hozon::perception::base::StopLinesPtr track_stoplines) {
  track_stoplines->stoplines.clear();
  if (!detect_stoplines) {
    return true;
  }

  for (const auto& detect_stopline : detect_stoplines->stoplines) {
    std::shared_ptr<hozon::perception::base::StopLine> stopline_ptr =
        std::make_shared<hozon::perception::base::StopLine>();
    stopline_ptr->id = detect_stopline->id;
    stopline_ptr->confidence = detect_stopline->confidence;
    stopline_ptr->point_set_2d = detect_stopline->point_set_2d;
    stopline_ptr->point_set_3d = detect_stopline->point_set_3d;
    track_stoplines->stoplines.push_back(stopline_ptr);
  }

  return true;
}

bool RoadMarkPostProcess::ProcessZebraCrossings(
    const hozon::perception::base::ZebraCrossingsMeasurementPtr
        detect_zebra_crossings,
    hozon::perception::base::ZebraCrossingsPtr track_zebra_crossings) {
  if (!detect_zebra_crossings) {
    return true;
  }

  for (const auto& detect_crosswalks : detect_zebra_crossings->crosswalks) {
    std::shared_ptr<hozon::perception::base::ZebraCrossing> zebracrossing_ptr =
        std::make_shared<hozon::perception::base::ZebraCrossing>();
    zebracrossing_ptr->id = detect_crosswalks->id;
    zebracrossing_ptr->confidence = detect_crosswalks->confidence;
    zebracrossing_ptr->heading = detect_crosswalks->heading;
    zebracrossing_ptr->point_set_2d = detect_crosswalks->point_set_2d;
    zebracrossing_ptr->point_set_3d = detect_crosswalks->point_set_3d;
    track_zebra_crossings->crosswalks.push_back(zebracrossing_ptr);
  }

  return true;
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
