/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@senseauto.com>
 */

#pragma once

#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

template <typename Type>
class SortedContainer {
 public:
  using T_ptr = std::shared_ptr<Type>;

 private:
  using MapContainerT = std::map<double, T_ptr>;
  MapContainerT MetaList_;
  T_ptr invalid_ = nullptr;
  static constexpr uint64_t INVALID_TIME = 0;

 public:
  using iterator_T = typename MapContainerT::iterator;

  // default constructor
  SortedContainer() {
    MetaList_.clear();
    invalid_.reset(new Type());
    invalid_->t_ns = INVALID_TIME;
  }

  inline T_ptr& GetInvalid() { return invalid_; }

  inline void clear() { MetaList_.clear(); }

  inline typename MapContainerT::size_type size() { return MetaList_.size(); }

  inline bool empty() { return MetaList_.empty(); }

  // get the iterator at the end of the container
  inline iterator_T IteratorEnd() { return MetaList_.end(); }

  // get the iterator at the beginning of the container
  inline iterator_T IteratorBegin() {
    if (empty() == true) {
      return IteratorEnd();
    }
    return MetaList_.begin();
  }

  // get the iterator before the beginning of the container
  inline iterator_T IteratorBeforeBegin() {
    iterator_T iter = IteratorBegin();
    if (iter == IteratorEnd()) {
      // if the container is empty
      return iter;
    }
    return (--iter);
  }

  // insert a value into the container strictly sorted by time
  // return the new added iterator position
  inline iterator_T insert(const T_ptr& value) {
    std::pair<iterator_T, bool> iter = MetaList_.insert(
        std::pair<double, T_ptr>(NanoSecToSec(value->t_ns), value));
    if (!iter.second) {  // has key equivalent to existed, replace it
      MetaList_[NanoSecToSec(value->t_ns)] = value;
    }
    return iter.first;
  }

  // get the iterator at the specific time
  inline iterator_T GetIteratorAt(const double& time) {
    iterator_T iter = MetaList_.find(time);
    if (iter == IteratorEnd()) {
      LC_LERROR(COMMON) << "can't find specific time value: " << time;
    }
    return iter;
  }

  // get the iterator at the closest time before specific time
  inline iterator_T GetIteratorClosestBefore(const double& time) {
    if (empty() == true) {
      return IteratorEnd();
    }
    iterator_T iter = MetaList_.lower_bound(time);
    // if all the elements time are lareger than specific time
    // iter points to the first elements in map
    return (--iter);
  }

  // get the iterator at the closest time after of specific time
  inline iterator_T GetIteratorClosestAfter(const double& time) {
    if (empty() == true) {
      return IteratorEnd();
    }
    iterator_T iter = MetaList_.upper_bound(time);
    // if all the elements time are smaller than specific time
    // iter points to end
    return iter;
  }

  // get the iterator closest to the specific time
  inline iterator_T GetIteratorClosest(const double& time) {
    iterator_T iter_at = MetaList_.find(time);
    if (iter_at != IteratorEnd()) {
      return iter_at;
    }

    iterator_T iter_before = GetIteratorClosestBefore(time);
    iterator_T iter_after = GetIteratorClosestAfter(time);
    if (iter_before == IteratorEnd() && iter_after == IteratorEnd()) {
      LC_LERROR(COMMON) << "empty container";
      return IteratorEnd();
    }

    // this may happen that
    // 1. container all elements are larger than specific time
    // 2. the first element is smaller than specific time
    if (iter_before == IteratorBeforeBegin()) {
      return IteratorBegin();
    }
    // this may happen that
    // container all elements are smaller than specific time
    if (iter_after == IteratorEnd()) {
      return (--iter_after);
    }
    if (std::fabs(iter_after->first - time) <
        std::fabs(iter_before->first - time)) {
      return iter_after;
    } else {
      return iter_before;
    }
  }

  // get the latest element in the container
  inline T_ptr& GetLast() {
    if (empty() == true) {
      return GetInvalid();
    }
    iterator_T iter_end = IteratorEnd();
    return (--iter_end)->second;
  }

  // get the first element in the container
  inline T_ptr& GetFirst() {
    if (empty() == true) {
      return GetInvalid();
    }
    return IteratorBegin()->second;
  }

  inline T_ptr& GetClosestBefore(const double& time) {
    if (empty() == true) {
      return GetInvalid();
    }
    iterator_T iter = MetaList_.lower_bound(time);
    if (iter == MetaList_.begin()) {
      // may happen that first elements are larger than specific time
      if (iter->first > time) {
        return GetInvalid();
      }
      return iter->second;
    }
    --iter;
    return iter->second;
  }

  inline T_ptr& GetClosestAfter(const double& time) {
    if (empty() == true) {
      return GetInvalid();
    }
    iterator_T iter = MetaList_.upper_bound(time);
    if (iter == MetaList_.end()) {
      return GetInvalid();
    }
    return iter->second;
  }

  inline double NanoSecToSec(uint64_t t_ns) {
    return static_cast<double>(t_ns) * 1.0e-9;
  }

  // clear all the elements having a time older than the time age
  inline void ClearOlderThan(const double& age) {
    T_ptr& newest = GetLast();
    T_ptr& oldest = GetFirst();
    if (newest == GetInvalid() || oldest == GetInvalid()) {
      LC_LERROR(COMMON) << "container is empty";
      return;
    }

    double erase_end_time = NanoSecToSec(newest->t_ns) - age;
    double oldest_time_time = NanoSecToSec(oldest->t_ns);
    if (erase_end_time < oldest_time_time) {
      return;
    } else {
      iterator_T iter = GetIteratorClosest(erase_end_time);
      if (iter == IteratorEnd()) {
        LC_LERROR(COMMON) << "can't drop container elements";
        return;
      }
      MetaList_.erase(IteratorBegin(), iter);
    }
  }

  // echo elements for test
  inline std::string EchoAllElements() {
    std::stringstream ss;
    ss << std::setprecision(10) << std::fixed;
    for (iterator_T iter = IteratorBegin(); iter != IteratorEnd(); ++iter) {
      ss << iter->second->t_ns * 1.0e-9 << " ";
    }
    ss << std::endl;
    return ss.str();
  }
};

}  // namespace localization
}  // namespace senseAD
