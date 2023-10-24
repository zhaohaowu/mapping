/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#include "modules/local_mapping/datalogger/load_data_singleton.h"

namespace hozon {
namespace mp {
namespace lm {

void LocalDataSingleton::Init() {
  dr_data_buffer_.set_capacity(1000);
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
