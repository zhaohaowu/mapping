/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_config_lite.h
 *   author     ： xuliang
 *   date       ： 2023.11
 ******************************************************************************/
#pragma once

#include <gflags/gflags.h>
#include <gflags/gflags_declare.h>

#include "common/configs/config_gflags.h"

DECLARE_bool(orin_viz);
DECLARE_string(orin_viz_addr);
DECLARE_bool(topo_rviz);
DECLARE_bool(viz_odom_map_in_local);
DECLARE_bool(output_hd_map);
DECLARE_double(service_update_interval);
DECLARE_bool(road_recognition_rviz);
static const std::string kWorkModeFusionMap = "FusionMap";  // NOLINT
static const std::string kWorkModePercepMap = "PercepMap";  // NOLINT
static const std::string kWorkModeFusionAndPercepMap = "FusionMap+PercepMap";  // NOLINT
DECLARE_string(work_mode);
