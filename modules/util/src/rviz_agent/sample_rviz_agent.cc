/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： sample_rviz_agent.cc
 *   author     ： taoshaoyuan
 *   date       ： 2022.10
 ******************************************************************************/

#include <gflags/gflags.h>

#include <Eigen/Geometry>
#include <random>

#include <opencv2/opencv.hpp>

#include "util/rviz_agent/rviz_agent.h"

// NOLINTBEGIN
DEFINE_string(viz_addr, "ipc:///tmp/sample_rviz_agent",
              "RvizAgent working address, may like "
              "ipc:///tmp/sample_rviz_agent or "
              "inproc://sample_rviz_agent or "
              "tcp://127.0.0.1:9100");
// NOLINTEND

using namespace hozon::mp::util; // NOLINT
using namespace std::chrono_literals; // NOLINT

bool g_stop = false;
void SigIntHandler(int sig) { g_stop = true; }

void GenImage(adsfi_proto::viz::CompressedImage* img);
void GenOdom(adsfi_proto::viz::Odometry* odom);
void GenPath(const adsfi_proto::viz::Odometry& odom,
             adsfi_proto::viz::Path* path);
void GenTf(const adsfi_proto::viz::Odometry& odom,
           adsfi_proto::viz::TransformStamped* tf);
void GenMarker(adsfi_proto::viz::Marker* marker);
void GenMarkerArray(adsfi_proto::viz::MarkerArray* marker_array);
void GenTwist(adsfi_proto::viz::TwistStamped* twist);
void GenPolygon(adsfi_proto::viz::PolygonStamped* polygon);
void GenPointCloud2(adsfi_proto::viz::PointCloud2* point_cloud);

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);

  signal(SIGINT, SigIntHandler);

  /*
   * STEP 1. 启动RvizAgent:
   *   (1) 使用Publish()前必须先启动;
   *   (2) 多次启动时只有第一次有效;
   *   (3) 建议放在程序入口处启动.
   */
  int ret = RvizAgent::Instance().Init(FLAGS_viz_addr);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent Init failed";
    return -1;
  }

  const std::string kImageTopic = "/sample_image";
  const std::string kOdomTopic = "/sample_odom";
  const std::string kPathTopic = "/sample_path";
  const std::string kTfTopic = "/sample_tf";
  const std::string kMarkerTopic = "/sample_marker";
  const std::string kMarkerArrayTopic = "/sample_marker_array";
  const std::string kTwistTopic = "/sample_twist";
  const std::string kPolygonTopic = "/sample_polygon";
  const std::string kPointCloud2Topic = "/sample_point_cloud2";

  /*
   * STEP 2. 注册要发送的消息的类型及topic:
   * (1) Publish()前必须先注册;
   * (2) 类型必须是proto类型，当前支持的有:
   *     - adsfi_proto::viz::CompressedImage
   *     - adsfi_proto::viz::Odometry
   *     - adsfi_proto::viz::Path
   *     - adsfi_proto::viz::TransformStamped
   *     - adsfi_proto::viz::Marker
   *     - adsfi_proto::viz::MarkerArray
   *     - adsfi_proto::viz::TwistStamped
   *     - adsfi_proto::viz::PolygonStamped
   *     - adsfi_proto::viz::PointCloud2
   *   分别对应rviz支持的ros msg类型:
   *     - sensor_msgs::CompressedImage
   *     - nav_msgs::Odometry
   *     - nav_msgs::Path
   *     - geometry_msgs::TransformStamped
   *     - visualization_msgs::Marker
   *     - visualization_msgs::MarkerArray
   *     - geometry_msgs::TwistStamped
   *     - geometry_msgs::PolygonStamped
   *     - sensor_msgs::PointCloud2
   * (3) 可以注册同类型的多个消息, 但要保证每个消息的topic是唯一的;
   * (4) 除tf消息外, 所有指定的topic, 都对应最终rviz能收到的ros msg的topic;
   * (5) 由于rviz默认只接收topic为"/tf"的tf消息,
   * 所以所有tf消息最终都转为了"/tf", 这里指定 的topic是用于区分来源; (6)
   * 取消注册某个topic: RvizAgent::Instance().UnRegister(topic); (7)
   * 查询某个topic是否已注册: bool registered =
   * RvizAgent::Instance().Registered(topic);
   */
  ret = RvizAgent::Instance().Register<adsfi_proto::viz::CompressedImage>(
      kImageTopic);
  if (ret < 0) {
    HLOG_ERROR << "register image error";
    return -1;
  }

  ret = RvizAgent::Instance().Register<adsfi_proto::viz::Odometry>(kOdomTopic);
  if (ret < 0) {
    HLOG_ERROR << "register odometry error";
    return -1;
  }

  ret = RvizAgent::Instance().Register<adsfi_proto::viz::Path>(kPathTopic);
  if (ret < 0) {
    HLOG_ERROR << "register path error";
    return -1;
  }

  ret = RvizAgent::Instance().Register<adsfi_proto::viz::TransformStamped>(
      kTfTopic);
  if (ret < 0) {
    HLOG_ERROR << "register tf error";
    return -1;
  }

  ret = RvizAgent::Instance().Register<adsfi_proto::viz::Marker>(kMarkerTopic);
  if (ret < 0) {
    HLOG_ERROR << "register marker error";
    return -1;
  }

  ret = RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
      kMarkerArrayTopic);
  if (ret < 0) {
    HLOG_ERROR << "register marker array error";
    return -1;
  }

  ret = RvizAgent::Instance().Register<adsfi_proto::viz::TwistStamped>(
      kTwistTopic);
  if (ret < 0) {
    HLOG_ERROR << "register twist error";
    return -1;
  }

  ret = RvizAgent::Instance().Register<adsfi_proto::viz::PolygonStamped>(
      kPolygonTopic);
  if (ret < 0) {
    HLOG_ERROR << "register polygon error";
    return -1;
  }

  ret = RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud2>(
      kPointCloud2Topic);
  if (ret < 0) {
    HLOG_ERROR << "register point cloud2 error";
    return -1;
  }

  while (!g_stop) {
    /*
     * STEP 3. 发布消息:
     * (1) topic必须已经Register过;
     * (2) 可以传入proto类型，也可以传入proto类型的std::shared_ptr;
     */
    adsfi_proto::viz::CompressedImage img;
    img.Clear();
    GenImage(&img);
    ret = RvizAgent::Instance().Publish(kImageTopic, img);
    if (ret < 0) {
      HLOG_ERROR << "pub image error";
      break;
    }

    adsfi_proto::viz::Odometry odom;
    odom.Clear();
    GenOdom(&odom);
    RvizAgent::Instance().Publish(kOdomTopic, odom);
    if (ret < 0) {
      HLOG_ERROR << "pub odom error";
      break;
    }

    adsfi_proto::viz::Path path;
    path.Clear();
    GenPath(odom, &path);
    RvizAgent::Instance().Publish(kPathTopic, path);
    if (ret < 0) {
      HLOG_ERROR << "pub path error";
      break;
    }

    adsfi_proto::viz::TransformStamped tf;
    tf.Clear();
    GenTf(odom, &tf);
    RvizAgent::Instance().Publish(kTfTopic, tf);
    if (ret < 0) {
      HLOG_ERROR << "pub tf error";
      break;
    }

    adsfi_proto::viz::Marker marker;
    marker.Clear();
    GenMarker(&marker);
    RvizAgent::Instance().Publish(kMarkerTopic, marker);
    if (ret < 0) {
      HLOG_ERROR << "pub marker error";
      break;
    }

    adsfi_proto::viz::MarkerArray marker_array;
    marker_array.Clear();
    GenMarkerArray(&marker_array);
    RvizAgent::Instance().Publish(kMarkerArrayTopic, marker_array);
    if (ret < 0) {
      HLOG_ERROR << "pub marker array error";
      break;
    }

    adsfi_proto::viz::TwistStamped twist;
    twist.Clear();
    GenTwist(&twist);
    RvizAgent::Instance().Publish(kTwistTopic, twist);
    if (ret < 0) {
      HLOG_ERROR << "pub twist error";
      break;
    }

    adsfi_proto::viz::PolygonStamped polygon;
    polygon.Clear();
    GenPolygon(&polygon);
    RvizAgent::Instance().Publish(kPolygonTopic, polygon);
    if (ret < 0) {
      HLOG_ERROR << "pub polygon error";
      break;
    }

    adsfi_proto::viz::PointCloud2 point_cloud;
    point_cloud.Clear();
    GenPointCloud2(&point_cloud);
    RvizAgent::Instance().Publish(kPointCloud2Topic, point_cloud);
    if (ret < 0) {
      HLOG_ERROR << "pub point cloud2 error";
      break;
    }

    std::this_thread::sleep_for(100ms);
  }

  /*
   * STEP 4. 关闭RvizAgent:
   * (1) 程序结束时关闭RvizAgent;
   * (2) 可以主动执行, 也可以不主动执行(会在析构时自动执行);
   */
  RvizAgent::Instance().Term();
}

void GenImage(adsfi_proto::viz::CompressedImage* img) {
  static int seq = 0;

  img->mutable_header()->set_frameid("camera");
  img->mutable_header()->set_seq(seq);
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  img->mutable_header()->mutable_timestamp()->set_sec(ts.tv_sec);
  img->mutable_header()->mutable_timestamp()->set_nsec(ts.tv_nsec);
  img->set_format("jpeg");

  cv::Scalar bgr(255, 0, 0);
  cv::Mat mat(1080, 1920, CV_8UC3, bgr);
  cv::putText(mat, "camera_" + std::to_string(seq), cv::Point(0, 1079),
              cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0, 255, 0), 2);
  std::vector<int> param(2);
  param[0] = cv::IMWRITE_JPEG_QUALITY;
  param[1] = 95;
  std::vector<uchar> buf;
  cv::imencode(".jpg", mat, buf, param);
  for (unsigned char i : buf) {
    img->mutable_data()->push_back(i);
  }

  seq++;
}

struct Pose {
  double x = 0.;
  double y = 0.;
  double z = 0.;
  double roll = 0.;
  double pitch = 0.;
  double yaw = 0.;
};

void GenPose(double x, Pose* pose) {
  // y = -(x-100)^2/100+100
  // y' = -2x/100+2
  pose->x = x;
  pose->y = -(x - 100) * (x - 100) / 100 + 100;
  pose->z = 0;
  double slope = -2 * x / 100 + 2;
  pose->yaw = atan(slope);
  pose->pitch = 0;
  pose->roll = 0;
}

void GenOdom(adsfi_proto::viz::Odometry* odom) {
  static int seq = 0;

  static Pose curr_pose;
  static double x = 0.;

  odom->mutable_header()->set_frameid("local_enu");
  odom->mutable_header()->set_seq(seq++);
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  odom->mutable_header()->mutable_timestamp()->set_sec(ts.tv_sec);
  odom->mutable_header()->mutable_timestamp()->set_nsec(ts.tv_nsec);

  x += 0.1;
  GenPose(x, curr_pose);
  odom->mutable_pose()->mutable_pose()->mutable_position()->set_x(curr_pose.x);
  odom->mutable_pose()->mutable_pose()->mutable_position()->set_y(curr_pose.y);
  odom->mutable_pose()->mutable_pose()->mutable_position()->set_z(curr_pose.z);

  Eigen::AngleAxisd roll(curr_pose.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(curr_pose.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(curr_pose.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yaw * roll * pitch;

  odom->mutable_pose()->mutable_pose()->mutable_orientation()->set_x(q.x());
  odom->mutable_pose()->mutable_pose()->mutable_orientation()->set_y(q.y());
  odom->mutable_pose()->mutable_pose()->mutable_orientation()->set_z(q.z());
  odom->mutable_pose()->mutable_pose()->mutable_orientation()->set_w(q.w());
  odom->set_child_frame_id("vehicle");
  for (int i = 0; i < 36; ++i) {
    odom->mutable_pose()->add_covariance(0);
  }

  odom->mutable_twist()->mutable_twist()->mutable_linear()->set_x(seq * 0.1);
  odom->mutable_twist()->mutable_twist()->mutable_linear()->set_y(seq * 0.1);
  odom->mutable_twist()->mutable_twist()->mutable_linear()->set_z(seq * 0.1);
  odom->mutable_twist()->mutable_twist()->mutable_angular()->set_x(seq * 0.01);
  odom->mutable_twist()->mutable_twist()->mutable_angular()->set_y(seq * 0.01);
  odom->mutable_twist()->mutable_twist()->mutable_angular()->set_z(seq * 0.01);
  for (int i = 0; i < 36; ++i) {
    odom->mutable_twist()->add_covariance(0);
  }

  seq++;
}

void GenPath(const adsfi_proto::viz::Odometry& odom,
             adsfi_proto::viz::Path* path) {
  static int seq = 0;
  static adsfi_proto::viz::Path last_path;

  last_path.mutable_header()->set_frameid("local_enu");
  last_path.mutable_header()->set_seq(seq);
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  last_path.mutable_header()->mutable_timestamp()->set_sec(ts.tv_sec);
  last_path.mutable_header()->mutable_timestamp()->set_nsec(ts.tv_nsec);

  if (last_path.poses().size() >= 100) {
    last_path.mutable_poses()->DeleteSubrange(0, 1);
  }

  auto pose = last_path.mutable_poses()->Add();
  pose->mutable_header()->set_frameid("local_enu");
  pose->mutable_header()->set_seq(seq);
  pose->mutable_header()->mutable_timestamp()->set_sec(ts.tv_sec);
  pose->mutable_header()->mutable_timestamp()->set_nsec(ts.tv_nsec);
  auto p = odom.pose().pose().position();
  pose->mutable_pose()->mutable_position()->set_x(p.x());
  pose->mutable_pose()->mutable_position()->set_y(p.y());
  pose->mutable_pose()->mutable_position()->set_z(p.z());

  auto q = odom.pose().pose().orientation();
  pose->mutable_pose()->mutable_orientation()->set_x(q.x());
  pose->mutable_pose()->mutable_orientation()->set_y(q.y());
  pose->mutable_pose()->mutable_orientation()->set_z(q.z());
  pose->mutable_pose()->mutable_orientation()->set_w(q.w());

  path->CopyFrom(last_path);

  seq++;
}

void GenTf(const adsfi_proto::viz::Odometry& odom,
           adsfi_proto::viz::TransformStamped* tf) {
  static int seq = 0;

  tf->mutable_header()->set_frameid("local_enu");
  tf->mutable_header()->set_seq(seq);
  tf->mutable_header()->mutable_timestamp()->set_sec(
      odom.header().timestamp().sec());
  tf->mutable_header()->mutable_timestamp()->set_nsec(
      odom.header().timestamp().nsec());
  tf->set_child_frame_id("vehicle");
  auto p = odom.pose().pose().position();
  tf->mutable_transform()->mutable_translation()->set_x(p.x());
  tf->mutable_transform()->mutable_translation()->set_y(p.y());
  tf->mutable_transform()->mutable_translation()->set_z(p.z());

  auto q = odom.pose().pose().orientation();
  tf->mutable_transform()->mutable_rotation()->set_x(q.x());
  tf->mutable_transform()->mutable_rotation()->set_y(q.y());
  tf->mutable_transform()->mutable_rotation()->set_z(q.z());
  tf->mutable_transform()->mutable_rotation()->set_w(q.w());

  seq++;
}

void GenMarker(adsfi_proto::viz::Marker* marker) {
  static int seq = 0;

  marker->mutable_header()->set_frameid("vehicle");
  marker->mutable_header()->set_seq(seq);
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  marker->mutable_header()->mutable_timestamp()->set_sec(ts.tv_sec);
  marker->mutable_header()->mutable_timestamp()->set_nsec(ts.tv_nsec);
  marker->set_ns("sample_marker");
  marker->set_id(0);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
  marker->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  marker->mutable_scale()->set_x(0.2);
  marker->mutable_color()->set_r(1.0);
  marker->mutable_color()->set_g(1.0);
  marker->mutable_color()->set_b(1.0);
  marker->mutable_color()->set_a(1.0);

  auto pt1 = marker->add_points();
  pt1->set_x(10.);
  pt1->set_y(10.);
  pt1->set_z(0.);
  auto pt2 = marker->add_points();
  pt2->set_x(10.);
  pt2->set_y(-10.);
  pt2->set_z(0.);

  seq++;
}

void GenMarkerArray(adsfi_proto::viz::MarkerArray* marker_array) {
  static int seq = 0;
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  auto m0 = marker_array->add_markers();
  m0->mutable_header()->set_frameid("vehicle");
  m0->mutable_header()->set_seq(seq);
  m0->mutable_header()->mutable_timestamp()->set_sec(ts.tv_sec);
  m0->mutable_header()->mutable_timestamp()->set_nsec(ts.tv_nsec);
  m0->set_ns("sample_marker_array");
  m0->set_id(0);
  m0->set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
  m0->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  m0->mutable_pose()->mutable_orientation()->set_x(0.);
  m0->mutable_pose()->mutable_orientation()->set_y(0.);
  m0->mutable_pose()->mutable_orientation()->set_z(0.);
  m0->mutable_pose()->mutable_orientation()->set_w(1.);
  m0->mutable_scale()->set_x(0.2);
  m0->mutable_color()->set_r(1.0);
  m0->mutable_color()->set_g(1.0);
  m0->mutable_color()->set_b(0.0);
  m0->mutable_color()->set_a(1.0);

  auto pt1 = m0->add_points();
  pt1->set_x(5.);
  pt1->set_y(2.5);
  pt1->set_z(0.);
  auto pt2 = m0->add_points();
  pt2->set_x(25.);
  pt2->set_y(2.5);
  pt2->set_z(0.);

  auto m1 = marker_array->add_markers();
  m1->mutable_header()->set_frameid("vehicle");
  m1->mutable_header()->set_seq(seq);
  m1->mutable_header()->mutable_timestamp()->set_sec(ts.tv_sec);
  m1->mutable_header()->mutable_timestamp()->set_nsec(ts.tv_nsec);
  m1->set_ns("sample_marker_array");
  m1->set_id(1);
  m1->set_type(adsfi_proto::viz::MarkerType::LINE_LIST);
  m1->set_action(adsfi_proto::viz::MarkerAction::MODIFY);
  m1->mutable_pose()->mutable_orientation()->set_x(0.);
  m1->mutable_pose()->mutable_orientation()->set_y(0.);
  m1->mutable_pose()->mutable_orientation()->set_z(0.);
  m1->mutable_pose()->mutable_orientation()->set_w(1.);
  m1->mutable_scale()->set_x(0.2);
  m1->mutable_color()->set_r(1.0);
  m1->mutable_color()->set_g(1.0);
  m1->mutable_color()->set_b(0.0);
  m1->mutable_color()->set_a(1.0);

  pt1 = m1->add_points();
  pt1->set_x(5.);
  pt1->set_y(-2.5);
  pt1->set_z(0.);
  pt2 = m1->add_points();
  pt2->set_x(25.);
  pt2->set_y(-2.5);
  pt2->set_z(0.);

  seq++;
}

void GenTwist(adsfi_proto::viz::TwistStamped* twist) {
  static int seq = 0;

  twist->mutable_header()->set_frameid("vehicle");
  twist->mutable_header()->set_seq(seq);
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  twist->mutable_header()->mutable_timestamp()->set_sec(ts.tv_sec);
  twist->mutable_header()->mutable_timestamp()->set_nsec(ts.tv_nsec);
  twist->mutable_twist()->mutable_linear()->set_x(2.);
  twist->mutable_twist()->mutable_linear()->set_y(0.);
  twist->mutable_twist()->mutable_linear()->set_z(0.);
  twist->mutable_twist()->mutable_angular()->set_x(0.);
  twist->mutable_twist()->mutable_angular()->set_y(0.);
  twist->mutable_twist()->mutable_angular()->set_z(0.2);

  seq++;
}

struct Point2D {
  double x;
  double y;
};

void GenArrow(std::vector<Point2D>* pts) {
  pts->clear();
  Point2D pt1 = {0, 0.15};
  Point2D pt2 = {0, -0.15};
  Point2D pt3 = {3.6, -0.15};
  Point2D pt4 = {3.6, -0.45};
  Point2D pt5 = {6, 0};
  Point2D pt6 = {3.6, 0.45};
  Point2D pt7 = {3.6, 0.15};
  pts->push_back(pt1);
  pts->push_back(pt2);
  pts->push_back(pt3);
  pts->push_back(pt4);
  pts->push_back(pt5);
  pts->push_back(pt6);
  pts->push_back(pt7);
}
void GenPolygon(adsfi_proto::viz::PolygonStamped* polygon) {
  static int seq = 0;

  polygon->mutable_header()->set_frameid("vehicle");
  polygon->mutable_header()->set_seq(seq);
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  polygon->mutable_header()->mutable_timestamp()->set_sec(ts.tv_sec);
  polygon->mutable_header()->mutable_timestamp()->set_nsec(ts.tv_nsec);

  std::vector<Point2D> pts;
  GenArrow(pts);
  for (auto it : pts) {
    auto ppt = polygon->mutable_polygon()->add_points();
    ppt->set_x(it.x + 10);
    ppt->set_y(it.y);
    ppt->set_z(0.);
  }

  seq++;
}

struct PointXYZI {
  float x;
  float y;
  float z;
  unsigned char intensity;
};
void GenPointCloud2(adsfi_proto::viz::PointCloud2* point_cloud) {
  static int seq = 0;

  point_cloud->mutable_header()->set_frameid("vehicle");
  point_cloud->mutable_header()->set_seq(seq);
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  point_cloud->mutable_header()->mutable_timestamp()->set_sec(ts.tv_sec);
  point_cloud->mutable_header()->mutable_timestamp()->set_nsec(ts.tv_nsec);

  auto fx = point_cloud->add_fields();
  fx->set_name("x");
  fx->set_offset(0);
  fx->set_datatype(adsfi_proto::viz::PointField::FLOAT32);
  fx->set_count(1);
  auto fy = point_cloud->add_fields();
  fy->set_name("y");
  fy->set_offset(4);
  fy->set_datatype(adsfi_proto::viz::PointField::FLOAT32);
  fy->set_count(1);
  auto fz = point_cloud->add_fields();
  fz->set_name("z");
  fz->set_offset(8);
  fz->set_datatype(adsfi_proto::viz::PointField::FLOAT32);
  fz->set_count(1);
  auto fi = point_cloud->add_fields();
  fi->set_name("intensity");
  fi->set_offset(12);
  fi->set_datatype(adsfi_proto::viz::PointField::UINT8);
  fi->set_count(1);

  int pts_num = 1000;
  std::random_device r;
  std::default_random_engine e1(r());
  std::uniform_real_distribution<float> uniform_dist_x(10, 30);
  std::uniform_real_distribution<float> uniform_dist_y_l(2, 2.2);
  std::uniform_real_distribution<float> uniform_dist_y_r(-2, -2.2);

  std::vector<PointXYZI> data;
  for (int i = 0; i < pts_num; ++i) {
    float x = uniform_dist_x(e1);
    float ly = uniform_dist_y_l(e1);
    float ry = uniform_dist_y_r(e1);
    unsigned char li = 1;
    unsigned char ri = 10;
    PointXYZI lpt = {x, ly, 0., li};
    PointXYZI rpt = {x, ry, 0., ri};
    data.push_back(lpt);
    data.push_back(rpt);
  }
  point_cloud->set_height(1);
  point_cloud->set_width(data.size());
  char* ptr = reinterpret_cast<char*>(data.data());
  int bytes_num = data.size() * sizeof(PointXYZI);
  for (int i = 0; i < bytes_num; ++i) {
    point_cloud->mutable_data()->push_back(ptr[i]);
  }

  point_cloud->set_is_bigendian(false);
  point_cloud->set_point_step(sizeof(PointXYZI));
  point_cloud->set_row_step(bytes_num);
  point_cloud->set_is_dense(true);

  seq++;
}
