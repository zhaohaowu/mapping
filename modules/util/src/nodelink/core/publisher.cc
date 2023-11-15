/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： publisher.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#include "util/nodelink/core/publisher.h"

#include "util/nodelink/core/pub_worker.h"

namespace hozon {
namespace mp {

std::string Publisher::Topic() { return topic_; }

void Publisher::Pub(void* data, size_t size) {
  if (!worker_) {
    return;
  }
  if ((data == nullptr) || size <= 0) {
    return;
  }

  worker_->AddData(topic_, data, size);
}

}  // namespace mp
}  // namespace hozon
