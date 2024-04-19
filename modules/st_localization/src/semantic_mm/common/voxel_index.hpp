
/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <utility>

#include "localization/data_type/base.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {
namespace smm {

// class holds the voxel index in 2-dimonsions
class VoxelIndex {
 public:
  VoxelIndex() {}
  VoxelIndex(int x, int y) : x_idx(x), y_idx(y) {}
  VoxelIndex(const VoxelIndex& rhs) : x_idx(rhs.x_idx), y_idx(rhs.y_idx) {}
  ~VoxelIndex() {}

  bool operator==(const VoxelIndex& rhs) const {
    return (x_idx == rhs.x_idx) && (y_idx == rhs.y_idx);
  }

 public:
  int x_idx;  // x-direction index
  int y_idx;  // y-direction index
};

// class holds the hash struct of voxel index
class VoxelIndexHash {
 public:
  size_t operator()(const VoxelIndex& idx) const {
    size_t seed = 0;
    hash_combine(&seed, idx.x_idx);
    hash_combine(&seed, idx.y_idx);
    return seed;
  }
  template <class T>
  void hash_combine(size_t* seed, const T& v) const {
    std::hash<T> hasher;
    *seed ^= hasher(v) + 0x9e3779b9 + (*seed << 6) + (*seed >> 2);
  }
};

// class holds the converter of voxel index with point3d
class VoxelIndexConverter {
 public:
  DEFINE_SMART_PTR(VoxelIndexConverter)

 public:
  explicit VoxelIndexConverter(double res) : res_x_(res), res_y_(res) {}
  VoxelIndexConverter(double res_x, double res_y)
      : res_x_(res_x), res_y_(res_y) {}
  ~VoxelIndexConverter() {}

  // @brief: converter methods
  VoxelIndex PointInnerToVoxelIndex(const Point2D_t& point) {
    return VoxelIndex(Index(point.x, res_x_), Index(point.y, res_y_));
  }
  VoxelIndex PointEigenToVoxelIndex(const Eigen::Vector2d& vec) {
    return VoxelIndex(Index(vec(0), res_x_), Index(vec(1), res_y_));
  }

  Point2D_t VoxelIndexToPointInner(const VoxelIndex& index) {
    return Point2D_t(Center(index.x_idx, res_x_), Center(index.y_idx, res_y_));
  }
  Eigen::Vector2d VoxelIndexToPointEigen(const VoxelIndex& index) {
    return Eigen::Vector2d(Center(index.x_idx, res_x_),
                           Center(index.y_idx, res_y_));
  }

 private:
  static inline int Index(double d, double res) {
    double base = d + 0.5 * res;
    return base < 0 ? static_cast<int>((base / res) - 1)
                    : static_cast<int>(base / res);
  }
  static inline double Center(int idx, double res) { return (idx * res); }

 private:
  double res_x_;  // voxel x resolution
  double res_y_;  // voxel y resolution
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
