/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： factory.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>

namespace hozon {
namespace mp {

class Node;
using CreateNodeFn = std::function<std::shared_ptr<Node>()>;

class Factory {
 public:
  std::shared_ptr<Node> CreateNodeByName(const std::string& name) {
    std::map<std::string, CreateNodeFn>::const_iterator iter;
    iter = create_map_.find(name);
    if (iter == create_map_.end()) {
      return nullptr;
    } else {
      return iter->second();
    }
  }

  void RegisterNode(const std::string& name, CreateNodeFn creator) {
    create_map_.insert(std::pair<std::string, CreateNodeFn>(name, creator));
  }

 private:
  std::map<std::string, CreateNodeFn> create_map_;

  // declare singleton
 private:
  Factory() {}
  ~Factory() {}

  Factory(const Factory&);
  Factory& operator=(const Factory&);

 public:
  static Factory& Instance() {
    static Factory instance;
    return instance;
  }
};

#define REGISTER(name) REGISTER_INTERNAL(name, __COUNTER__)

#define REGISTER_INTERNAL(name, id)                                      \
  namespace {                                                            \
  std::shared_ptr<Node> Creator##name() {                                \
    auto ptr = std::make_shared<name>();                                 \
    return ptr;                                                          \
  }                                                                      \
  struct ProxyType##name##id {                                           \
    ProxyType##name##id() {                                              \
      hozon::mp::Factory::Instance().RegisterNode(#name, Creator##name); \
    }                                                                    \
  };                                                                     \
  static ProxyType##name##id g_register_class_##name##id;                \
  }  // namespace

}  // namespace mp
}  // namespace hozon
