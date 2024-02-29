/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： rviz_agent_client.h
 *   author     ： taoshaoyuan
 *   date       ： 2022.10
 ******************************************************************************/

#pragma once

#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>
// auto generated
#include "adsfi_proto/viz/geometry_msgs.pb.h"
#include "adsfi_proto/viz/nav_msgs.pb.h"
#include "adsfi_proto/viz/sensor_msgs.pb.h"
#include "adsfi_proto/viz/tf2_msgs.pb.h"
#include "adsfi_proto/viz/visualization_msgs.pb.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/nodelink/core/context.h"
#include "modules/util/include/util/nodelink/core/sub_worker.h"
#include "modules/util/include/util/rviz_agent/msg_alias.h"

namespace hozon {
namespace mp {
namespace util {

class RvizAgentClient {
 public:
  int Init(const std::vector<std::string>& addrs);

  // TODO(AAA): change these Registers to templates
  int Register(ImageCallback callback) {
    if (img_cbk_ != nullptr) {
      HLOG_ERROR << "image callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    img_cbk_ = std::move(callback);
    return 0;
  }

  int Register(OdometryCallback callback) {
    if (odom_cbk_ != nullptr) {
      HLOG_ERROR << "odometry callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    odom_cbk_ = std::move(callback);
    return 0;
  }

  int Register(PathCallback callback) {
    if (path_cbk_ != nullptr) {
      HLOG_ERROR << "path callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    path_cbk_ = std::move(callback);
    return 0;
  }

  int Register(TransformStampedCallback callback) {
    if (tf_cbk_ != nullptr) {
      HLOG_ERROR << "tf callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    tf_cbk_ = std::move(callback);
    return 0;
  }

  int Register(MarkerCallback callback) {
    if (marker_cbk_ != nullptr) {
      HLOG_ERROR << "marker callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    marker_cbk_ = std::move(callback);
    return 0;
  }

  int Register(MarkerArrayCallback callback) {
    if (marker_array_cbk_ != nullptr) {
      HLOG_ERROR << "marker array callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    marker_array_cbk_ = std::move(callback);
    return 0;
  }

  int Register(TwistStampedCallback callback) {
    if (twist_cbk_ != nullptr) {
      HLOG_ERROR << "twist stamped callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    twist_cbk_ = std::move(callback);
    return 0;
  }

  int Register(PolygonStampedCallback callback) {
    if (polygon_cbk_ != nullptr) {
      HLOG_ERROR << "polygon stamped callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    polygon_cbk_ = std::move(callback);
    return 0;
  }

  int Register(PointCloudCallback callback) {
    if (pt_cloud_cbk_ != nullptr) {
      HLOG_ERROR << "point cloud callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    pt_cloud_cbk_ = std::move(callback);
    return 0;
  }

  int Register(PointCloud2Callback callback) {
    if (pt_cloud2_cbk_ != nullptr) {
      HLOG_ERROR << "point cloud2 callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    pt_cloud2_cbk_ = std::move(callback);
    return 0;
  }

  int Register(PoseArrayCallback callback) {
    if (pose_array_cbk_ != nullptr) {
      HLOG_ERROR << "pose array callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    pose_array_cbk_ = std::move(callback);
    return 0;
  }

  int Register(OccupancyGridCallback callback) {
    if (occupancy_grid_cbk_ != nullptr) {
      HLOG_ERROR << "occupancy grid callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    occupancy_grid_cbk_ = std::move(callback);
    return 0;
  }

  int Register(LocalizationCallback callback) {
    if (localization_cbk_ != nullptr) {
      HLOG_ERROR << "localization callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    localization_cbk_ = std::move(callback);
    return 0;
  }

  int Register(HafNodeInfoCallback callback) {
    if (haf_node_info_cbk_ != nullptr) {
      HLOG_ERROR << "HafNodeInfo callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    haf_node_info_cbk_ = std::move(callback);
    return 0;
  }

  int Register(ImuInsCallback callback) {
    if (imu_ins_cbk_ != nullptr) {
      HLOG_ERROR << "ImuIns callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    imu_ins_cbk_ = std::move(callback);
    return 0;
  }

  int Register(DeadReckoningCallback callback) {
    if (dead_reckoning_cbk_ != nullptr) {
      HLOG_ERROR << "DeadReckoning callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    dead_reckoning_cbk_ = std::move(callback);
    return 0;
  }

  int Register(ChassisCallback callback) {
    if (chassis_cbk_ != nullptr) {
      HLOG_ERROR << "Chassis callback already registered";
      return -1;
    }
    std::lock_guard<std::mutex> lock(mtx_);
    chassis_cbk_ = std::move(callback);
    return 0;
  }

  /*
    template <typename ProtoType>
    typename Checker<ProtoType>::Int
    Register(std::function<void (const std::string&,
    std::shared_ptr<ProtoType>)> callback) { std::string type_alias =
    GetTypeAlias<ProtoType>(); std::lock_guard<std::mutex> lock(data_mtx_); if
    (type_alias == kCompressedImage) { img_cbk_ = callback; } else if
    (type_alias == kOdometry) { odom_cbk_ = callback; } else if (type_alias ==
    kPath) { path_cbk_ = callback; } else if (type_alias == kTransformStamped) {
        tf_cbk_ = callback;
      } else if (type_alias == kMarker) {
        marker_cbk_ = callback;
      } else if (type_alias == kMarkerArray) {
        marker_array_cbk_ = callback;
      }
      return 0;
    }
  */
  template <typename ProtoType>
  typename Checker<ProtoType>::Void UnRegister() {
    std::string type_alias = GetTypeAlias<ProtoType>();
    std::lock_guard<std::mutex> lock(mtx_);
    if (type_alias == kCompressedImage) {
      img_cbk_ = nullptr;
    } else if (type_alias == kOdometry) {
      odom_cbk_ = nullptr;
    } else if (type_alias == kPath) {
      path_cbk_ = nullptr;
    } else if (type_alias == kTransformStamped) {
      tf_cbk_ = nullptr;
    } else if (type_alias == kMarker) {
      marker_cbk_ = nullptr;
    } else if (type_alias == kMarkerArray) {
      marker_array_cbk_ = nullptr;
    } else if (type_alias == kTwistStamped) {
      twist_cbk_ = nullptr;
    } else if (type_alias == kPolygonStamped) {
      polygon_cbk_ = nullptr;
    } else if (type_alias == kPointCloud) {
      pt_cloud_cbk_ = nullptr;
    } else if (type_alias == kPointCloud2) {
      pt_cloud2_cbk_ = nullptr;
    } else if (type_alias == kPoseArray) {
      pose_array_cbk_ = nullptr;
    } else if (type_alias == kOccupancyGrid) {
      occupancy_grid_cbk_ = nullptr;
    } else if (type_alias == kLocalization) {
      localization_cbk_ = nullptr;
    } else if (type_alias == kHafNodeInfo) {
      haf_node_info_cbk_ = nullptr;
    } else if (type_alias == kImuIns) {
      imu_ins_cbk_ = nullptr;
    } else if (type_alias == kDeadReckoning) {
      dead_reckoning_cbk_ = nullptr;
    } else if (type_alias == kChassis) {
      chassis_cbk_ = nullptr;
    }
  }

  template <typename ProtoType>
  typename Checker<ProtoType>::Bool Registered() {
    bool ret = false;
    std::string type_alias = GetTypeAlias<ProtoType>();
    std::lock_guard<std::mutex> lock(mtx_);
    if (type_alias == kCompressedImage && img_cbk_) {
      ret = true;
    } else if (type_alias == kOdometry && odom_cbk_) {
      ret = true;
    } else if (type_alias == kPath && path_cbk_) {
      ret = true;
    } else if (type_alias == kTransformStamped && tf_cbk_) {
      ret = true;
    } else if (type_alias == kMarker && marker_cbk_) {
      ret = true;
    } else if (type_alias == kMarkerArray && marker_array_cbk_) {
      ret = true;
    } else if (type_alias == kTwistStamped && twist_cbk_) {
      ret = true;
    } else if (type_alias == kPolygonStamped && polygon_cbk_) {
      ret = true;
    } else if (type_alias == kPointCloud && pt_cloud_cbk_) {
      ret = true;
    } else if (type_alias == kPointCloud2 && pt_cloud2_cbk_) {
      ret = true;
    } else if (type_alias == kPoseArray && pose_array_cbk_) {
      ret = true;
    } else if (type_alias == kOccupancyGrid && occupancy_grid_cbk_) {
      ret = true;
    } else if (type_alias == kLocalization && localization_cbk_) {
      ret = true;
    } else if (type_alias == kHafNodeInfo && haf_node_info_cbk_) {
      ret = true;
    } else if (type_alias == kImuIns && imu_ins_cbk_) {
      ret = true;
    } else if (type_alias == kDeadReckoning && dead_reckoning_cbk_) {
      ret = true;
    } else if (type_alias == kChassis && chassis_cbk_) {
      ret = true;
    }
    return ret;
  }

  void Term();

 private:
  ImageCallback img_cbk_ = nullptr;
  OdometryCallback odom_cbk_ = nullptr;
  PathCallback path_cbk_ = nullptr;
  TransformStampedCallback tf_cbk_ = nullptr;
  MarkerCallback marker_cbk_ = nullptr;
  MarkerArrayCallback marker_array_cbk_ = nullptr;
  TwistStampedCallback twist_cbk_ = nullptr;
  PolygonStampedCallback polygon_cbk_ = nullptr;
  PointCloudCallback pt_cloud_cbk_ = nullptr;
  PointCloud2Callback pt_cloud2_cbk_ = nullptr;
  PoseArrayCallback pose_array_cbk_ = nullptr;
  OccupancyGridCallback occupancy_grid_cbk_ = nullptr;
  LocalizationCallback localization_cbk_ = nullptr;
  HafNodeInfoCallback haf_node_info_cbk_ = nullptr;
  ImuInsCallback imu_ins_cbk_ = nullptr;
  DeadReckoningCallback dead_reckoning_cbk_ = nullptr;
  ChassisCallback chassis_cbk_ = nullptr;
  std::shared_ptr<SubWorker> sub_ = nullptr;
  // <topic, type_alias>
  std::map<std::string, std::string> reg_msgs_;
  std::mutex mtx_;
  std::atomic_bool running_ = {false};
  static bool CheckAddr(const std::string& addr);
  void ProcCtrlMsg(void* data, size_t size);
  void ProcCommonMsg(const std::string& topic, void* data, size_t size);
  void CallbackForAllTopics(const std::string& topic, void* data, size_t size);

  // declare singleton
 private:
  RvizAgentClient() = default;
  ~RvizAgentClient() noexcept;
  RvizAgentClient(const RvizAgentClient&);
  RvizAgentClient& operator=(const RvizAgentClient&);

 public:
  static RvizAgentClient& Instance() {
    static RvizAgentClient instance;
    return instance;
  }
};

}  // namespace util
}  // namespace mp
}  // namespace hozon
