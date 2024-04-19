/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： st_location.h
 *   author     ： wangmeng
 *   date       ： 2024.04
 ******************************************************************************/

#pragma once

#include <adf-lite/include/base.h>

#include <memory>
#include <string>

#include "depend/map/hdmap/hdmap_common.h"
#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "depend/perception-lib/lib/fault_manager/fault_manager.h"
#include "depend/perception-lib/lib/health_manager/health_manager.h"
#include "depend/proto/soc/sensor_gnss.pb.h"
#include "modules/location/pose_estimation/lib/pose_estimation.h"
#include "modules/map_fusion/include/map_fusion/map_service/global_hd_map.h"
#include "onboard/onboard_lite/phm_comment_lite/proto/running_mode.pb.h"

#include "localization/localization_manager.hpp"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::mp::loc::MapMatching;
using hozon::netaos::adf_lite::Bundle;

class STLocation : public hozon::netaos::adf_lite::Executor {
 public:
  STLocation() = default;
  ~STLocation() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
//   void RegistMessageType() const;
//   void RegistProcessFunc();

//   int32_t OnChassis(Bundle* input);
//   int32_t OnImuIns(Bundle* input);
//   int32_t OnGnss(Bundle* input);
//   int32_t OnPerception(Bundle* input);
//   int32_t OnRunningMode(Bundle* input);
//   int32_t OnLocOutput(Bundle* input);
//   bool GetLocalMapData(const Eigen::Vector3d& vehicle_position,
//                        const Eigen::Matrix3d& vehicle_rotation,
//                        const Eigen::Vector3d& ref_point,
//                        std::shared_ptr<senseAD::localization::RoadStructure>);
//   bool TransStOutput2LocOutput(
//       senseAD::localization::NavStateInfo& nav_output,
//       senseAD::localization::OdomStateInfo& odo_output,
//       std::shared_ptr<hozon::localization::Localization>& loc_output);

//  private:
  std::unique_ptr<senseAD::localization::LocalizationManager>
      localization_manager_;
//   hozon::mp::loc::Map<hozon::hdmap::Map> mhd_map_;
//   std::shared_ptr<senseAD::localization::RoadStructure> latest_local_map_data_{
//       nullptr};
};

REGISTER_ADF_CLASS(STLocation, STLocation);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
