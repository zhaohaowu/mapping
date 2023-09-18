/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "modules/local_mapping/lib/datalogger/data_buffer.h"
#include "modules/local_mapping/lib/ops/association/association.h"
#include "modules/local_mapping/lib/types/common.h"

namespace hozon {
namespace mp {
namespace lm {

class LocalDataSingleton {
 public:
  static LocalDataSingleton& GetInstance() {
    static LocalDataSingleton local_data;
    return local_data;
  }

  void Init();

 private:
  LocalDataSingleton(const LocalDataSingleton&) = delete;
  LocalDataSingleton& operator=(const LocalDataSingleton&) = delete;
  LocalDataSingleton() {}

 public:
  /** @brief sensor data buffer. */
  MessageBuffer<ConstDrDataPtr> dr_data_buffer_;
  MessageBuffer<ConstLanesPtr> lanes_buffer_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
