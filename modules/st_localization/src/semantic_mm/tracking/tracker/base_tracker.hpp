/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <atomic>

#include "localization/data_type/semantic_type.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class BaseTracker {
 public:
  DEFINE_SMART_PTR(BaseTracker)
  DEFINE_PTR_CONTAINER(BaseTracker)

  BaseTracker() = default;
  explicit BaseTracker(const id_t& id) : id_(id) {}

  virtual ~BaseTracker() {}

  void SetId(const id_t& id) { id_ = id; }
  id_t GetId() const { return id_; }

 protected:
  static std::atomic<id_t> next_id_;
  id_t id_;  // global unique id
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
