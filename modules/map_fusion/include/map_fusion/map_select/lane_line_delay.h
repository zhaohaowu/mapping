/*
 * Copyright (c) hozon auto Co., Ltd. 2021-2022. All rights reserved.
 */
#pragma once
#include <deque>

namespace hozon {
namespace mp {
namespace mf {
/**
 * @class
 * 默认延迟一个周期，可以加参数直接得到某几个周期的延迟，内部保存当前时刻和延迟周期数的数据
 * 所以deque数据的大小size = 延迟周期数+1
 */
template <typename T>
class Delay {
 public:
  Delay();
  explicit Delay(int size);
  ~Delay() = default;
  /**
   * @brief
   * 该函数将当前的数据存入，并返回上一周期的数据，初始时返回输入的数据.其中
   * 数据大小最少为2个，即size_ >= 2,默认存储数据大小为2
   */
  T Deal(const T& input);
  /**
   * @brief
   * 该函数将当前的数据存入，然后返回要求的延迟周期的数，如果要求的延迟周期数
   * 大于现有的周期数，那么返回延迟最大周期的数
   */
  T Deal(const T& input, int delay_period);
  /**
   * @return 返回除当前所有数据的平均值
   */
  T GetAverageValue();
  /**
   * @description: 该函数将当前数据存入， 
   * @param {T&} input
   * @return {*}
   */
  T GetAverageValue(const T& input);
  /**
   * @return
   * 返回输入要求的延迟数，如果数字大于现有的周期数，那么返回延迟最大周期的数
   * GetDelay(0)返回最近周期的数据
   */
  T GetDelay(int num);

  /**
   * @brief
   * 清除已经存储的数据
   */
  void Clear() { deque_data_.clear(); }

  /**
   * @brief
   * 返回已经存储的数据的大小
   */
  int GetDataSize() { return deque_data_.size(); }

 private:
  std::deque<T> deque_data_{};
  int size_;
};

template <typename T>
Delay<T>::Delay() : size_(2) {}

template <typename T>
Delay<T>::Delay(int size) {
  if (size < 2) {
    size_ = 2;
  } else {
    size_ = size;
  }
}

template <typename T>
T Delay<T>::Deal(const T& input) {
  if (deque_data_.empty()) {
    deque_data_.push_front(input);
    return deque_data_.back();
  }
  if (deque_data_.size() < size_) {
    deque_data_.push_front(input);
    return deque_data_.at(1);
  }
  deque_data_.pop_back();
  deque_data_.push_front(input);
  return deque_data_.at(1);
}

template <typename T>
T Delay<T>::Deal(const T& input, int delay_period) {
  if (deque_data_.size() < size_) {
    deque_data_.push_front(input);
  } else {
    deque_data_.pop_back();
    deque_data_.push_front(input);
  }
  if (delay_period > (deque_data_.size() - 1)) {
    return deque_data_.back();
  }
  if (delay_period < 0) {
    return deque_data_.front();
  }
  return deque_data_.at(delay_period);
}

template <typename T>
T Delay<T>::GetAverageValue() {
  T sum{0};
  if (deque_data_.empty()) {
    return sum;
  }
  for (auto& val : deque_data_) {
    sum += val;
  }
  return sum / deque_data_.size();
}

template <typename T>
T Delay<T>::GetAverageValue(const T& input) {
  deque_data_.push_front(input);
  if (deque_data_.size() > size_) {
    deque_data_.pop_back();
  }

  T sum{0};
  if (deque_data_.empty()) {
    return sum;
  }
  for (auto& val : deque_data_) {
    sum += val;
  }
  return sum / deque_data_.size();
}

template <typename T>
T Delay<T>::GetDelay(int num) {
  if (deque_data_.empty()) {
    T fal{};
    return fal;
  }
  if (num < 1) {
    return deque_data_.front();
  }
  if (num > (deque_data_.size() - 1)) {
    return deque_data_.back();
  }
  return deque_data_.at(num);
}
}  // namespace mf
}  // namespace mp
}  // namespace hozon
