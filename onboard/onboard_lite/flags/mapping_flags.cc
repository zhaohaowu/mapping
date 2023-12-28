/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-12-28
 *****************************************************************************/
#pragma once
#include "onboard/onboard_lite/flags/mapping_flags.h"
#include "gflags/gflags.h"

namespace hozon {
namespace perception {
namespace common_onboard {

DEFINE_bool(enable_phm_option, false, "default off phm");

}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
