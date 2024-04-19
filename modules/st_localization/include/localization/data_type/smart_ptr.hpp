/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Chen Longquan <chenlongquan1@sensetime.com>
 */
#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

namespace senseAD {
namespace localization {

#define DEFINE_PTR(...)                     \
  using Ptr = std::shared_ptr<__VA_ARGS__>; \
  using ConstPtr = std::shared_ptr<const __VA_ARGS__>;

#define DEFINE_UNIQUE_PTR(...)                    \
  using UniquePtr = std::unique_ptr<__VA_ARGS__>; \
  using ConstUniquePtr = std::unique_ptr<const __VA_ARGS__>;

#define DEFINE_WEAK_PTR(...)                  \
  using WeakPtr = std::weak_ptr<__VA_ARGS__>; \
  using ConstWeakPtr = std::weak_ptr<const __VA_ARGS__>;

#define DEFINE_SMART_PTR(...)    \
  DEFINE_PTR(__VA_ARGS__)        \
  DEFINE_UNIQUE_PTR(__VA_ARGS__) \
  DEFINE_WEAK_PTR(__VA_ARGS__)

#define DEFINE_PTR_CONTAINER(...)                             \
  using PtrVec = std::vector<__VA_ARGS__::Ptr>;               \
  using CPtrVec = std::vector<__VA_ARGS__::ConstPtr>;         \
  using PtrUMap = std::unordered_map<id_t, __VA_ARGS__::Ptr>; \
  using CPtrUMap = std::unordered_map<id_t, __VA_ARGS__::ConstPtr>;

}  // namespace localization
}  // namespace senseAD
