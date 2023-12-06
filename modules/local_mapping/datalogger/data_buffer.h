/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <list>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

#include "modules/local_mapping/utils/common.h"

namespace hozon {
namespace mp {
namespace lm {

template <class MessageType>
class MessageBuffer {
 public:
  using ListIterator =
      typename ::std::list<::std::pair<double, MessageType>>::iterator;

 public:
  explicit MessageBuffer(int capacity);

  bool push_new_message(const double& timestamp, const MessageType& msg);
  bool pop_oldest_message();
  bool pop_oldest_message(MessageType* msg);
  bool pop_latest_message();
  bool pop_latest_message(MessageType* msg);
  bool get_message(const double& timestamp, MessageType* msg);
  bool get_latest_message(MessageType* msg);
  bool get_message_before(const double& timestamp, MessageType* msg);

  /** @brief Get all messages after timestamp in msg buffer.
   * If the message in timestamp doesn't exist in msg buffer,
   * return all message in buffer. */
  void get_messages_after(
      const double& timestamp,
      ::std::vector<::std::pair<double, MessageType>>& msgs);  // NOLINT
  void clear();

  void set_capacity(unsigned int capacity);
  unsigned int get_capacity();
  void get_all_messages(
      ::std::list<::std::pair<double, MessageType>>* msg_list);

  bool is_empty();
  bool is_full();
  unsigned int buffer_size();

  MessageType front();
  MessageType second();
  MessageType back();
  void get_messages_around(const double timestamp,
                           MessageType& before,  // NOLINT
                           MessageType& after);  // NOLINT

 private:
  ::std::map<double, ListIterator> _msg_map;
  ::std::list<::std::pair<double, MessageType>> _msg_list;

  ::std::mutex _buffer_mutex;
  unsigned int _capacity;
};

template <class MessageType>
class IntepolableMessageBuffer : public MessageBuffer<MessageType> {
 public:
  using ListIterator =
      typename ::std::list<::std::pair<double, MessageType>>::iterator;

 public:
  explicit IntepolableMessageBuffer(int capacity);

  bool query_message(const double& timestamp, MessageType* msg,
                     double timeout_s = 0.05);

 private:
  bool wait_message_buffer_ok(
      const double& timestamp, ::std::map<double, ListIterator>* msg_map,
      ::std::list<::std::pair<double, MessageType>>* msg_list,
      double timeout_ms);
};

// ==============MessageBuffer==================
template <class MessageType>
MessageBuffer<MessageType>::MessageBuffer(int capacity) : _capacity(capacity) {}

template <class MessageType>
bool MessageBuffer<MessageType>::push_new_message(const double& timestamp,
                                                  const MessageType& msg) {
  if (_capacity == 0) {
    // ::std::cerr << "The buffer capacity is 0." << ::std::endl;
    // LOCAL_LOG_ERROR_FORMAT("The buffer capacity is 0.");
    return false;
  }

  _buffer_mutex.lock();
  auto found_iter = _msg_map.find(timestamp);
  if (found_iter != _msg_map.end()) {
    _buffer_mutex.unlock();
    // LOCAL_LOG_ERROR_FORMAT("The msg has existed in buffer.");
    return false;
  }

  ::std::pair<double, MessageType> msg_pair = ::std::make_pair(timestamp, msg);
  if (_msg_list.size() < _capacity) {
    _msg_list.push_back(msg_pair);
  } else {
    _msg_map.erase(_msg_list.begin()->first);
    _msg_list.pop_front();
    _msg_list.push_back(msg_pair);
  }

  ListIterator iter = _msg_list.end();
  _msg_map[timestamp] = (--iter);
  _buffer_mutex.unlock();
  return true;
}

template <class MessageType>
bool MessageBuffer<MessageType>::pop_oldest_message(MessageType* msg) {
  _buffer_mutex.lock();
  if (_msg_list.empty()) {
    _buffer_mutex.unlock();
    // ::std::cerr << "The buffer is empty." << ::std::endl;
    // LOCAL_LOG_ERROR_FORMAT("The buffer is empty.");
    return false;
  }

  *msg = _msg_list.begin()->second;
  _msg_map.erase(_msg_list.begin()->first);
  _msg_list.pop_front();
  _buffer_mutex.unlock();

  return true;
}

template <class MessageType>
bool MessageBuffer<MessageType>::pop_oldest_message() {
  _buffer_mutex.lock();
  if (_msg_list.empty()) {
    _buffer_mutex.unlock();
    // LOCAL_LOG_ERROR_FORMAT("The buffer is empty.");
    return false;
  }

  _msg_map.erase(_msg_list.begin()->first);
  _msg_list.pop_front();
  _buffer_mutex.unlock();

  return true;
}

template <class MessageType>
bool MessageBuffer<MessageType>::pop_latest_message(MessageType* msg) {
  _buffer_mutex.lock();
  if (_msg_list.empty()) {
    _buffer_mutex.unlock();
    // ::std::cerr << "The buffer is empty." << ::std::endl;
    // LOCAL_LOG_ERROR_FORMAT("The buffer is empty.");
    return false;
  }

  *msg = _msg_list.back().second;

  _msg_map.erase(_msg_list.back().first);
  _msg_list.pop_back();
  _buffer_mutex.unlock();

  return true;
}

template <class MessageType>
bool MessageBuffer<MessageType>::pop_latest_message() {
  _buffer_mutex.lock();
  if (_msg_list.empty()) {
    _buffer_mutex.unlock();
    // LOCAL_LOG_ERROR_FORMAT("The buffer is empty.");
    return false;
  }

  _msg_map.erase(_msg_list.back().first);
  _msg_list.pop_back();
  _buffer_mutex.unlock();

  return true;
}

template <class MessageType>
bool MessageBuffer<MessageType>::get_message_before(const double& timestamp,
                                                    MessageType* msg) {
  _buffer_mutex.lock();
  if (_msg_list.empty()) {
    _buffer_mutex.unlock();
    // ::std::cerr << "The buffer is empty." << ::std::endl;
    // LOCAL_LOG_ERROR_FORMAT("The buffer is empty.");
    return false;
  }

  ::std::list<::std::pair<double, MessageType>> msg_list;
  ::std::copy(this->_msg_list.begin(), this->_msg_list.end(),
              ::std::back_inserter(msg_list));
  _buffer_mutex.unlock();

  for (ListIterator iter = msg_list.end(); iter != msg_list.begin();) {
    --iter;
    if (iter->first <= timestamp) {
      *msg = iter->second;
      return true;
    }
  }

  return false;
}

template <class MessageType>
bool MessageBuffer<MessageType>::get_message(const double& timestamp,
                                             MessageType* msg) {
  _buffer_mutex.lock();
  auto found_iter = _msg_map.find(timestamp);
  if (found_iter != _msg_map.end()) {
    *msg = found_iter->second->second;
    _buffer_mutex.unlock();
    return true;
  }
  _buffer_mutex.unlock();
  return false;
}

template <class MessageType>
bool MessageBuffer<MessageType>::get_latest_message(MessageType* msg) {
  _buffer_mutex.lock();
  if (_msg_list.empty()) {
    _buffer_mutex.unlock();
    // ::std::cerr << "The buffer is empty." << ::std::endl;
    // LOCAL_LOG_ERROR_FORMAT("The buffer is empty.");
    return false;
  }

  *msg = _msg_list.back().second;
  _buffer_mutex.unlock();

  return true;
}

template <class MessageType>
void MessageBuffer<MessageType>::get_messages_after(
    const double& timestamp,
    ::std::vector<::std::pair<double, MessageType>>& msgs) {  // NOLINT
  _buffer_mutex.lock();
  auto found_iter = _msg_map.find(timestamp);
  if (found_iter != _msg_map.end()) {
    ListIterator begin_iter = found_iter->second;
    ++begin_iter;
    ::std::copy(begin_iter, this->_msg_list.end(), ::std::back_inserter(msgs));
  } else {
    ::std::copy(this->_msg_list.begin(), this->_msg_list.end(),
                ::std::back_inserter(msgs));
  }
  _buffer_mutex.unlock();
}

template <class MessageType>
void MessageBuffer<MessageType>::clear() {
  _buffer_mutex.lock();
  _msg_list.clear();
  _msg_map.clear();
  _buffer_mutex.unlock();
}

template <class MessageType>
void MessageBuffer<MessageType>::set_capacity(unsigned int capacity) {
  _capacity = capacity;
}

template <class MessageType>
unsigned int MessageBuffer<MessageType>::get_capacity() {
  return _capacity;
}

template <class MessageType>
void MessageBuffer<MessageType>::get_all_messages(
    ::std::list<::std::pair<double, MessageType>>* msg_list) {
  _buffer_mutex.lock();
  msg_list->clear();
  ::std::copy(_msg_list.begin(), _msg_list.end(),
              ::std::back_inserter(*msg_list));
  _buffer_mutex.unlock();
}

template <class MessageType>
bool MessageBuffer<MessageType>::is_empty() {
  bool flag = true;
  _buffer_mutex.lock();
  flag = _msg_list.empty();
  _buffer_mutex.unlock();
  return flag;
}

template <class MessageType>
bool MessageBuffer<MessageType>::is_full() {
  unsigned int size = 0;
  _buffer_mutex.lock();
  size = _msg_list.size();
  _buffer_mutex.unlock();
  return (size == _capacity);
}

template <class MessageType>
unsigned int MessageBuffer<MessageType>::buffer_size() {
  unsigned int size = 0;
  _buffer_mutex.lock();
  size = _msg_list.size();
  _buffer_mutex.unlock();

  return size;
}

template <class MessageType>
MessageType MessageBuffer<MessageType>::front() {
  MessageType msg;
  if (buffer_size() == 0) {
    return msg;
  }
  _buffer_mutex.lock();
  msg = _msg_list.front().second;
  _buffer_mutex.unlock();
  return msg;
}

template <class MessageType>
MessageType MessageBuffer<MessageType>::second() {
  MessageType msg;
  if (buffer_size() < 2) {
    return msg;
  }
  _buffer_mutex.lock();

  msg = std::next(_msg_list.begin(), 1)->second;

  _buffer_mutex.unlock();
  return msg;
}

template <class MessageType>
MessageType MessageBuffer<MessageType>::back() {
  MessageType msg;
  if (buffer_size() == 0) {
    return msg;
  }

  _buffer_mutex.lock();
  msg = _msg_list.back().second;
  _buffer_mutex.unlock();
  return msg;
}

// ==============IntepolableMessageBuffer==================
template <class MessageType>
IntepolableMessageBuffer<MessageType>::IntepolableMessageBuffer(int capacity)
    : MessageBuffer<MessageType>(capacity) {}

template <class MessageType>
bool IntepolableMessageBuffer<MessageType>::query_message(
    const double& timestamp, MessageType* msg, double timeout_s) {
  ::std::map<double, ListIterator> msg_map_tem;
  ::std::list<::std::pair<double, MessageType>> msg_list_tem;

  if (!wait_message_buffer_ok(timestamp, &msg_map_tem, &msg_list_tem,
                              timeout_s * 1000)) {
    return false;
  }

  ListIterator begin_iter = msg_list_tem.begin();
  if (begin_iter->first > timestamp) {
    return false;
  }

  auto found_iter = msg_map_tem.find(timestamp);
  if (found_iter != msg_map_tem.end()) {
    *msg = found_iter->second->second;
    return true;
  }

  ListIterator after_iter;
  ListIterator before_iter;
  for (ListIterator iter = msg_list_tem.end(); iter != msg_list_tem.begin();) {
    --iter;
    if (iter->first > timestamp) {
      after_iter = iter;
    } else {
      before_iter = iter;

      double delta_time = after_iter->first - before_iter->first;
      // if (delete_time > 0.1) {
      //     return false;
      // }

      if (::std::abs(delta_time) < 1e-9) {
        // LOCAL_LOG_ERROR_FORMAT("Delta_time is too small.");
        return false;
      }
      double scale = (timestamp - before_iter->first) / delta_time;
      *msg = CommonUtil::Interpolate(scale, before_iter->second,
                                     after_iter->second, timestamp);
      break;
    }
  }
  return true;
}

template <class MessageType>
bool IntepolableMessageBuffer<MessageType>::wait_message_buffer_ok(
    const double& timestamp, ::std::map<double, ListIterator>* msg_map,
    ::std::list<::std::pair<double, MessageType>>* msg_list,
    double timeout_ms) {
  double sum_time = 0.0;

  this->_buffer_mutex.lock();
  bool empty = this->_msg_list.empty();
  this->_buffer_mutex.unlock();

  while (!empty) {
    this->_buffer_mutex.lock();
    if (this->_msg_list.back().first >= timestamp) {
      ::std::copy(this->_msg_list.begin(), this->_msg_list.end(),
                  ::std::back_inserter(*msg_list));
      *msg_map = this->_msg_map;
      this->_buffer_mutex.unlock();
      return true;
    }
    this->_buffer_mutex.unlock();

    if (sum_time >= timeout_ms) {
      // LOCAL_LOG_INFO_FORMAT("Waiting time is out.");
      return false;
    }

    // microseconds
    usleep(2000);
    sum_time += 2;
  }

  return false;
}

template <class MessageType>
void MessageBuffer<MessageType>::get_messages_around(
    const double timestamp, MessageType& before,  // NOLINT
    MessageType& after) {                         // NOLINT
  _buffer_mutex.lock();
  if (_msg_list.empty()) {
    before = nullptr;
    after = nullptr;
    _buffer_mutex.unlock();
    return;
  }

  auto found_iter = _msg_map.find(timestamp);
  if (found_iter != _msg_map.end()) {
    before = found_iter->second->second;
    after = found_iter->second->second;
    _buffer_mutex.unlock();
    return;
  }

  auto iter = _msg_list.rbegin();
  for (; iter != _msg_list.rend(); iter++) {
    if (iter->first < timestamp) {
      break;
    }
  }

  if (iter == _msg_list.rbegin()) {
    before = nullptr;
    if (timestamp - _msg_list.back().second->timestamp > 0.2) {
      after = nullptr;
      _buffer_mutex.unlock();
      return;
    }
    after = _msg_list.back().second;
    _buffer_mutex.unlock();
    return;
  }

  if (iter == _msg_list.rend()) {
    before = nullptr;
    after = nullptr;
    _buffer_mutex.unlock();
    return;
  }

  auto pre = iter;
  auto nex = --iter;
  before = pre->second;
  after = nex->second;
  _buffer_mutex.unlock();
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
