/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： loader.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace hozon {
namespace mp {

class Loader {
 public:
  Loader() = default;
  ~Loader() = default;

  Loader(const Loader&) = delete;
  Loader& operator=(const Loader&) = delete;

  static std::shared_ptr<Loader> Create() { return std::make_shared<Loader>(); }

  int Load(const std::vector<std::string>& lib_paths);

  void Term();

 private:
  void* LoadLib(const std::string& lib_path);
  // key: lib's full canonical path
  // value: lib handle from dlopen
  std::map<std::string, void*> lib_handles_;
};

using LoaderPtr = std::shared_ptr<Loader>;

}  // namespace mp
}  // namespace hozon
