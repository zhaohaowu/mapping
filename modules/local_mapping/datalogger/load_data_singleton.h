/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "modules/local_mapping/datalogger/data_buffer.h"
#include "modules/local_mapping/ops/association/association.h"
#include "modules/local_mapping/types/types.h"

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
  MessageBuffer<ConstDrDataPtr>& dr_buffer() { return dr_data_buffer_; }
  LocalDataSingleton(const LocalDataSingleton&) = delete;
  LocalDataSingleton& operator=(const LocalDataSingleton&) = delete;
  ~LocalDataSingleton() = default;

 private:
  LocalDataSingleton() : dr_data_buffer_(1000) {}

  /** @brief sensor data buffer. */
  MessageBuffer<ConstDrDataPtr> dr_data_buffer_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
