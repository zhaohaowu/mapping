/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： publisher.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#pragma once

#include <memory>
#include <string>

namespace hozon {
namespace mp {

class PubWorker;

class Publisher {
 public:
  Publisher(const std::string& topic, std::shared_ptr<PubWorker> worker)
      : topic_(topic), worker_(worker) {}

  Publisher(const Publisher& rhs) : topic_(rhs.topic_), worker_(rhs.worker_) {}

  Publisher& operator=(const Publisher& rhs) {
    topic_ = rhs.topic_;
    worker_ = rhs.worker_;
    return *this;
  }

  std::string Topic();

  void Pub(void* data, size_t size);

  static std::shared_ptr<Publisher> Create(const std::string& topic,
                                           std::shared_ptr<PubWorker> worker) {
    return std::make_shared<Publisher>(topic, worker);
  }

 private:
  std::string topic_;
  std::shared_ptr<PubWorker> worker_;
};

using PublisherPtr = std::shared_ptr<Publisher>;

}  // namespace mp
}  // namespace hozon
