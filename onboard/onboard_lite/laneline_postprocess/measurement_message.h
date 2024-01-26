/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-09-16
 *****************************************************************************/
#include <memory>

#include "adf-lite/include/base.h"
#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "perception-base/base/frame/fusion_frame.h"
#include "perception-base/base/frame/measurement_frame.h"

namespace hozon {
namespace mp {
namespace common_onboard {

using namespace hozon::perception;

struct MeasurementMessage : hozon::netaos::adf_lite::BaseData {
  std::shared_ptr<base::MeasurementFrame> measurement_frame;
};

struct FusionMessage : netaos::adf_lite::BaseData {
  std::shared_ptr<base::FusionFrame> fusion_frame;
};

}  //  namespace common_onboard
}  // namespace mp
}  //  namespace hozon
