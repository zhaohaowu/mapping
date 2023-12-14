/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： rviz_bridge.cc
 *   author     ： taoshaoyuan
 *   date       ： 2022.6
 ******************************************************************************/

#include <gflags/gflags.h>
#include <ros/ros.h>

// auto generated
#include <adsfi_proto/viz/geometry_msgs.pb.h>
#include <adsfi_proto/viz/nav_msgs.pb.h>
#include <adsfi_proto/viz/sensor_msgs.pb.h>
#include <adsfi_proto/viz/tf2_msgs.pb.h>
#include <adsfi_proto/viz/visualization_msgs.pb.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <csignal>
#include <filesystem>

#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/rviz_agent/msg_alias.h"
#include "modules/util/include/util/rviz_agent/rviz_agent_client.h"
#include "type_converter.h"  // NOLINT

namespace sfs = std::filesystem;
using namespace hozon::mp::util;  // NOLINT
// using namespace hozon::localization::tools;

class PubManager {
 public:
  explicit PubManager(const ros::NodeHandle& nh) : nh_(nh) {}

  ~PubManager() {
    for (auto& it : all_) {
      for (auto& itt : it.second) {
        itt.second.shutdown();
      }
    }
  }

  const ros::Publisher& GetPub(const std::string& type_alias,
                               const std::string& topic) {
    if (all_.find(type_alias) == all_.end()) {
      all_.insert({type_alias, std::map<std::string, ros::Publisher>()});
    }
    auto& type_pubs = all_[type_alias];
    if (type_pubs.find(topic) == type_pubs.end()) {
      ros::Publisher pub;
      if (type_alias == kCompressedImage) {
        pub = nh_.advertise<sensor_msgs::CompressedImage>(topic, 10);
      } else if (type_alias == kOdometry) {
        pub = nh_.advertise<nav_msgs::Odometry>(topic, 10);
      } else if (type_alias == kPath) {
        pub = nh_.advertise<nav_msgs::Path>(topic, 10);
      } else if (type_alias == kMarker) {
        pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
      } else if (type_alias == kMarkerArray) {
        pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
      } else if (type_alias == kTwistStamped) {
        pub = nh_.advertise<geometry_msgs::TwistStamped>(topic, 10);
      } else if (type_alias == kPolygonStamped) {
        pub = nh_.advertise<geometry_msgs::PolygonStamped>(topic, 10);
      } else if (type_alias == kPointCloud) {
        pub = nh_.advertise<sensor_msgs::PointCloud>(topic, 10);
      } else if (type_alias == kPointCloud2) {
        pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10);
      } else if (type_alias == kPoseArray) {
        pub = nh_.advertise<geometry_msgs::PoseArray>(topic, 10);
      } else if (type_alias == kOccupancyGrid) {
        pub = nh_.advertise<nav_msgs::OccupancyGrid>(topic, 10);
      }
      type_pubs.insert({topic, pub});
    }
    return all_[type_alias][topic];
  }

 private:
  ros::NodeHandle nh_;
  std::map<std::string, std::map<std::string, ros::Publisher>> all_;
};

class RvizBridge {
 public:
  explicit RvizBridge(const ros::NodeHandle& nh) : nh_(nh), pub_mng_(nh_) {}

  ~RvizBridge() { Shutdown(); }

  int Init(const std::vector<std::string>& addrs) {
    auto image_cbk = std::bind(&RvizBridge::OnImage, this,
                               std::placeholders::_1, std::placeholders::_2);
    int ret = RvizAgentClient::Instance().Register(image_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register image callback failed";
      return -1;
    }

    auto odom_cbk = std::bind(&RvizBridge::OnOdometry, this,
                              std::placeholders::_1, std::placeholders::_2);
    ret = RvizAgentClient::Instance().Register(odom_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register odom callback failed";
      return -1;
    }

    auto path_cbk = std::bind(&RvizBridge::OnPath, this, std::placeholders::_1,
                              std::placeholders::_2);
    ret = RvizAgentClient::Instance().Register(path_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register path callback failed";
      return -1;
    }

    auto tf_cbk = std::bind(&RvizBridge::OnTransformStamped, this,
                            std::placeholders::_1, std::placeholders::_2);
    ret = RvizAgentClient::Instance().Register(tf_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register tf callback failed";
      return -1;
    }

    auto marker_cbk = std::bind(&RvizBridge::OnMarker, this,
                                std::placeholders::_1, std::placeholders::_2);
    ret = RvizAgentClient::Instance().Register(marker_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register marker callback failed";
      return -1;
    }
    auto marker_array_cbk =
        std::bind(&RvizBridge::OnMarkerArray, this, std::placeholders::_1,
                  std::placeholders::_2);
    ret = RvizAgentClient::Instance().Register(marker_array_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register marker array callback failed";
      return -1;
    }
    auto twist_cbk = std::bind(&RvizBridge::OnTwist, this,
                               std::placeholders::_1, std::placeholders::_2);
    ret = RvizAgentClient::Instance().Register(twist_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register twist callback failed";
      return -1;
    }
    auto polygon_cbk = std::bind(&RvizBridge::OnPolygon, this,
                                 std::placeholders::_1, std::placeholders::_2);
    ret = RvizAgentClient::Instance().Register(polygon_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register polygon callback failed";
      return -1;
    }
    auto pt_cloud_cbk = std::bind(&RvizBridge::OnPointCloud, this,
                                  std::placeholders::_1, std::placeholders::_2);
    ret = RvizAgentClient::Instance().Register(pt_cloud_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register point cloud callback failed";
      return -1;
    }
    auto pt_cloud2_cbk =
        std::bind(&RvizBridge::OnPointCloud2, this, std::placeholders::_1,
                  std::placeholders::_2);
    ret = RvizAgentClient::Instance().Register(pt_cloud2_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register point cloud2 callback failed";
      return -1;
    }
    auto pose_array_cbk =
        std::bind(&RvizBridge::OnPoseArray, this, std::placeholders::_1,
                  std::placeholders::_2);
    ret = RvizAgentClient::Instance().Register(pose_array_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register pose array callback failed";
      return -1;
    }
    auto occupancy_grid_cbk =
        std::bind(&RvizBridge::OnOccupancyGrid, this, std::placeholders::_1,
                  std::placeholders::_2);
    ret = RvizAgentClient::Instance().Register(occupancy_grid_cbk);
    if (ret < 0) {
      HLOG_ERROR << "Register occupancy grid callback failed";
      return -1;
    }

    std::string addrs_str;
    for (const auto& s : addrs) {
      addrs_str += s;
      addrs_str += ", ";
    }
    HLOG_INFO << "Connect to addrs: " << addrs_str;
    ret = RvizAgentClient::Instance().Init(addrs);
    if (ret < 0) {
      HLOG_ERROR << "Start RvizAgentClient failed";
      return -1;
    }

    return 0;
  }

  void Shutdown() { RvizAgentClient::Instance().Term(); }

 private:
  ros::NodeHandle nh_;
  PubManager pub_mng_;

  void OnImage(const std::string& topic,
               std::shared_ptr<adsfi_proto::viz::CompressedImage> proto) {
    sensor_msgs::CompressedImage ros;
    TypeConverter::Convert(*proto, &ros);
    //! 由于rviz默认接收的压缩图像的topic必须以"/compressed"结尾，
    //! 这里检查原始topic，如果原始topic不是以"/compressed"结尾就加上
    std::string act_topic = topic;
    std::string suffix = "/compressed";
    if (topic.length() < suffix.length() ||
        topic.compare(topic.length() - suffix.length(), suffix.length(),
                      suffix) != 0) {
      act_topic = topic + suffix;
    }
    pub_mng_.GetPub(kCompressedImage, act_topic).publish(ros);
  }

  void OnOdometry(const std::string& topic,
                  std::shared_ptr<adsfi_proto::viz::Odometry> proto) {
    nav_msgs::Odometry ros;
    TypeConverter::Convert(*proto, &ros);
    pub_mng_.GetPub(kOdometry, topic).publish(ros);
  }

  void OnPath(const std::string& topic,
              std::shared_ptr<adsfi_proto::viz::Path> proto) {
    nav_msgs::Path ros;
    TypeConverter::Convert(*proto, &ros);
    pub_mng_.GetPub(kPath, topic).publish(ros);
  }

  void OnTransformStamped(
      const std::string& topic,
      std::shared_ptr<adsfi_proto::viz::TransformStamped> proto) {
    geometry_msgs::TransformStamped ros;
    TypeConverter::Convert(*proto, &ros);
    //! 注意：这里使用TransformBroadcaster来发布tf，默认topic都是"/tf"
    static tf2_ros::TransformBroadcaster tf_broad;
    tf_broad.sendTransform(ros);
    //! 这里为了方便，把tf来源的原始topic打出来
    HLOG_INFO << "tf " << ros.header.frame_id << " to " << ros.child_frame_id
              << " is from raw topic " << topic;
  }

  void OnMarker(const std::string& topic,
                std::shared_ptr<adsfi_proto::viz::Marker> proto) {
    visualization_msgs::Marker ros;
    TypeConverter::Convert(*proto, &ros);
    pub_mng_.GetPub(kMarker, topic).publish(ros);
  }

  void OnMarkerArray(const std::string& topic,
                     std::shared_ptr<adsfi_proto::viz::MarkerArray> proto) {
    visualization_msgs::MarkerArray ros;
    TypeConverter::Convert(*proto, &ros);
    pub_mng_.GetPub(kMarkerArray, topic).publish(ros);
  }

  void OnTwist(const std::string& topic,
               std::shared_ptr<adsfi_proto::viz::TwistStamped> proto) {
    geometry_msgs::TwistStamped ros;
    TypeConverter::Convert(*proto, &ros);
    pub_mng_.GetPub(kTwistStamped, topic).publish(ros);
  }

  void OnPolygon(const std::string& topic,
                 std::shared_ptr<adsfi_proto::viz::PolygonStamped> proto) {
    geometry_msgs::PolygonStamped ros;
    TypeConverter::Convert(*proto, &ros);
    pub_mng_.GetPub(kPolygonStamped, topic).publish(ros);
  }

  void OnPointCloud(const std::string& topic,
                    std::shared_ptr<adsfi_proto::viz::PointCloud> proto) {
    sensor_msgs::PointCloud ros;
    TypeConverter::Convert(*proto, &ros);
    pub_mng_.GetPub(kPointCloud, topic).publish(ros);
  }

  void OnPointCloud2(const std::string& topic,
                     std::shared_ptr<adsfi_proto::viz::PointCloud2> proto) {
    sensor_msgs::PointCloud2 ros;
    TypeConverter::Convert(*proto, &ros);
    pub_mng_.GetPub(kPointCloud2, topic).publish(ros);
  }

  void OnPoseArray(const std::string& topic,
                   std::shared_ptr<adsfi_proto::viz::PoseArray> proto) {
    geometry_msgs::PoseArray ros;
    TypeConverter::Convert(*proto, &ros);
    pub_mng_.GetPub(kPoseArray, topic).publish(ros);
  }

  void OnOccupancyGrid(const std::string& topic,
                       std::shared_ptr<adsfi_proto::viz::OccupancyGrid> proto) {
    nav_msgs::OccupancyGrid ros;
    TypeConverter::Convert(*proto, &ros);
    pub_mng_.GetPub(kOccupancyGrid, topic).publish(ros);
  }
};

RvizBridge* g_rviz_bridge = nullptr;
void RegisterRvizBridge(RvizBridge* bridge) { g_rviz_bridge = bridge; }

void SigIntHandler(int sig) {
  HLOG_ERROR << "recv ctrl-c, now shutdown";
  if (g_rviz_bridge) {
    g_rviz_bridge->Shutdown();
  }
  ros::shutdown();
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  ros::init(argc, argv, "rviz_bridge", ros::init_options::NoSigintHandler);

  const std::string log_path("./log");
  sfs::path log_dir(log_path);
  if (!sfs::exists(log_dir) && !sfs::create_directory(log_dir)) {
    std::cerr << "Create log dir failed!" << std::endl;
    return -1;
  }

  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = log_path;
  FLAGS_stderrthreshold = 0;
  FLAGS_minloglevel = 0;

  ros::NodeHandle nh;
  std::vector<std::string> viz_addrs;
  ros::param::get("/rviz_bridge/viz_addrs", viz_addrs);

  RvizBridge bridge(nh);
  if (bridge.Init(viz_addrs) < 0) {
    HLOG_ERROR << "Init RvizBridge failed!";
    return -1;
  }

  RegisterRvizBridge(&bridge);
  signal(SIGINT, SigIntHandler);

  ros::spin();

  google::ShutdownGoogleLogging();

  return 0;
}
