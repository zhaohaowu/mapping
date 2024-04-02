/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： id_generator.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

class IdGenerator {
 public:
  std::string Gen(const std::string& prefix) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (max_ids_.find(prefix) == max_ids_.end()) {
      max_ids_.insert({prefix, 0});
      return prefix + std::to_string(0);
    }

    max_ids_[prefix] += 1;
    return prefix + std::to_string(max_ids_[prefix]);
  }

 private:
  std::map<std::string, uint64_t> max_ids_;
  std::mutex mtx_;

 private:
  IdGenerator() = default;
  ~IdGenerator() = default;
  IdGenerator(const IdGenerator&);
  IdGenerator& operator=(const IdGenerator&);

 public:
  static IdGenerator& Instance() {
    static IdGenerator instance;
    return instance;
  }
};
#define ID_GENERATOR hozon::mp::rm::IdGenerator::Instance()

/// 把字符串id中末尾的数值提取出来，数值以split_symbol中的之一进行分割；
/// 比如输入：boundary_12345，提取出其中的12345；
template <typename T>
T RetrieveNumInId(const std::string& id,
                  const std::string& split_symbol = " ,;/|_") {
  std::vector<std::string> split_str;
  boost::split(split_str, id, boost::is_any_of(split_symbol),
               boost::token_compress_on);
  if (split_str.empty()) {
    return 0;
  }

  T num = 0;
  std::string last = split_str.back();
  try {
    num = boost::lexical_cast<T>(last);
  } catch (boost::bad_lexical_cast& e) {
    HLOG_ERROR << "lexical cast " << last << " failed";
  }

  return num;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
