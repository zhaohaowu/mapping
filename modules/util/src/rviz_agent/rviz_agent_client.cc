/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： rviz_agent_client.cc
 *   author     ： taoshaoyuan
 *   date       ： 2022.10
 ******************************************************************************/

#include "util/rviz_agent/rviz_agent_client.h"

#include <fstream>

#include "util/nodelink/core.h"

namespace hozon {
namespace mp {
namespace util {

RvizAgentClient::~RvizAgentClient() noexcept { Term(); }

int RvizAgentClient::Init(const std::vector<std::string>& addrs) {
  if (running_.load()) {
    HLOG_WARN << "RvizAgentClient already started";
    return -1;
  }

  for (const auto& a : addrs) {
    if (!CheckAddr(a)) {
      HLOG_ERROR << "Invalid addr " << a;
      return -1;
    }
  }

  void* context = Context::Instance().Get();
  sub_ = SubWorker::Create(context, addrs);
  auto cbk = [this](auto&& PH1, auto&& PH2, auto&& PH3) {
    CallbackForAllTopics(std::forward<decltype(PH1)>(PH1),
                         std::forward<decltype(PH2)>(PH2),
                         std::forward<decltype(PH3)>(PH3));
  };
  sub_->RegAll(cbk);
  if (sub_->Init() < 0) {
    HLOG_ERROR << "SubWorker Init failed";
    return -1;
  }

  running_.store(true);
  return 0;
}

bool RvizAgentClient::CheckAddr(const std::string& addr) {
  if (addr.empty()) {
    return false;
  }

  if (addr.rfind("tcp://", 0) != 0 && addr.rfind("ipc:///", 0) != 0 &&
      addr.rfind("inproc://", 0) != 0) {
    return false;
  }

  return true;
}

void RvizAgentClient::ProcCtrlMsg(void* data, size_t size) {
  adsfi_proto::viz::MsgInfoArray infos;
  infos.ParseFromArray(data, static_cast<int>(size));

  for (const auto& it : infos.msgs()) {
    reg_msgs_[it.topic()] = it.alias();
  }
  HLOG_INFO << "recv ctrl msg:\n" << infos.DebugString();
  std::string msg_str;
  for (const auto& msg : reg_msgs_) {
    msg_str += msg.first;
    msg_str += " : ";
    msg_str += msg.second;
    msg_str += "\n";
  }
  HLOG_INFO << "all registered msg:\n" << msg_str;
}

void RvizAgentClient::ProcCommonMsg(const std::string& topic, void* data,
                                    size_t size) {
  const std::string& type = reg_msgs_[topic];
  if (type == kCompressedImage && img_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::CompressedImage>();
    proto->ParseFromArray(data, static_cast<int>(size));
    img_cbk_(topic, proto);
  } else if (type == kOdometry && odom_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::Odometry>();
    proto->ParseFromArray(data, static_cast<int>(size));
    odom_cbk_(topic, proto);
  } else if (type == kPath && path_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::Path>();
    proto->ParseFromArray(data, static_cast<int>(size));
    path_cbk_(topic, proto);
  } else if (type == kTransformStamped && tf_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::TransformStamped>();
    proto->ParseFromArray(data, static_cast<int>(size));
    tf_cbk_(topic, proto);
  } else if (type == kMarker && marker_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::Marker>();
    proto->ParseFromArray(data, static_cast<int>(size));
    marker_cbk_(topic, proto);
  } else if (type == kMarkerArray && marker_array_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::MarkerArray>();
    proto->ParseFromArray(data, static_cast<int>(size));
    marker_array_cbk_(topic, proto);
  } else if (type == kTwistStamped && twist_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::TwistStamped>();
    proto->ParseFromArray(data, static_cast<int>(size));
    twist_cbk_(topic, proto);
  } else if (type == kPolygonStamped && polygon_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::PolygonStamped>();
    proto->ParseFromArray(data, static_cast<int>(size));
    polygon_cbk_(topic, proto);
  } else if (type == kPointCloud && pt_cloud_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::PointCloud>();
    proto->ParseFromArray(data, static_cast<int>(size));
    pt_cloud_cbk_(topic, proto);
  } else if (type == kPointCloud2 && pt_cloud2_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::PointCloud2>();
    proto->ParseFromArray(data, static_cast<int>(size));
    pt_cloud2_cbk_(topic, proto);
  } else if (type == kPoseArray && pose_array_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::PoseArray>();
    proto->ParseFromArray(data, static_cast<int>(size));
    pose_array_cbk_(topic, proto);
  } else if (type == kOccupancyGrid && occupancy_grid_cbk_ != nullptr) {
    auto proto = std::make_shared<adsfi_proto::viz::OccupancyGrid>();
    proto->ParseFromArray(data, static_cast<int>(size));
    occupancy_grid_cbk_(topic, proto);
  }
}

void RvizAgentClient::CallbackForAllTopics(const std::string& topic, void* data,
                                           size_t size) {
  if (topic.empty() || (data == nullptr) || (size == 0U)) {
    HLOG_ERROR << "Invalid callback data";
    return;
  }

  std::lock_guard<std::mutex> lock(mtx_);

  if (topic == kCtrlTopic) {
    ProcCtrlMsg(data, size);
    return;
  }

  ProcCommonMsg(topic, data, size);
}

void RvizAgentClient::Term() {
  running_.store(false);

  std::lock_guard<std::mutex> lock(mtx_);

  if (sub_) {
    sub_->Term();
    sub_ = nullptr;
  }

  img_cbk_ = nullptr;
  odom_cbk_ = nullptr;
  path_cbk_ = nullptr;
  tf_cbk_ = nullptr;
  marker_cbk_ = nullptr;
  marker_array_cbk_ = nullptr;
  twist_cbk_ = nullptr;
  polygon_cbk_ = nullptr;
  pt_cloud_cbk_ = nullptr;
  pt_cloud2_cbk_ = nullptr;
  pose_array_cbk_ = nullptr;
  occupancy_grid_cbk_ = nullptr;
  reg_msgs_.clear();
}

}  // namespace util
}  // namespace mp
}  // namespace hozon
