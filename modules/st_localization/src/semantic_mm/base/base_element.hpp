/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "localization/data_type/base.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class BaseElement {
 public:
  DEFINE_SMART_PTR(BaseElement)
  DEFINE_PTR_CONTAINER(BaseElement)

  BaseElement() = default;
  explicit BaseElement(const id_t& id) : id_(id) {}

  virtual ~BaseElement() {}

  void SetId(const id_t& id) { id_ = id; }
  id_t GetId() const { return id_; }

 protected:
  id_t id_{0};  // element unique id
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
