/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_config_lite.cc
 *   author     ： xuliang
 *   date       ： 2023.11
 ******************************************************************************/

#include <gflags/gflags.h>

#include "onboard/onboard_lite/map_fusion/map_fusion_config_lite.h"

DEFINE_bool(orin_viz, false, "if use rviz");
DEFINE_string(orin_viz_addr, "tcp://10.6.73.235:9100",
              "RvizAgent's working address");
