/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： type_converter.cc
 *   author     ： taoshaoyuan
 *   date       ： 2022.6
 ******************************************************************************/

#include "type_converter.h"

#include "modules/util/include/util/temp_log.h"

namespace hozon {
namespace mp {
namespace util {

void TypeConverter::Convert(const adsfi_proto::hz_Adsfi::AlgHeader &proto,
                            std_msgs::Header &ros) {
  ros.frame_id = proto.frameid();
  ros.seq = proto.seq();
  ros.stamp.sec = proto.timestamp().sec();
  ros.stamp.nsec = proto.timestamp().nsec();
}

void TypeConverter::Convert(const adsfi_proto::viz::CompressedImage &proto,
                            sensor_msgs::CompressedImage &ros) {
  Convert(proto.header(), ros.header);
  ros.format = proto.format();
  for (char i : proto.data()) {
    ros.data.push_back(i);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::Odometry &proto,
                            nav_msgs::Odometry &ros) {
  Convert(proto.header(), ros.header);
  ros.child_frame_id = proto.child_frame_id();
  ros.pose.pose.position.x = proto.pose().pose().position().x();
  ros.pose.pose.position.y = proto.pose().pose().position().y();
  ros.pose.pose.position.z = proto.pose().pose().position().z();
  ros.pose.pose.orientation.x = proto.pose().pose().orientation().x();
  ros.pose.pose.orientation.y = proto.pose().pose().orientation().y();
  ros.pose.pose.orientation.z = proto.pose().pose().orientation().z();
  ros.pose.pose.orientation.w = proto.pose().pose().orientation().w();
  if (proto.pose().covariance_size() != ros.pose.covariance.size()) {
    HLOG_ERROR << "covariance size doesn't match!";
    return;
  }
  for (int i = 0; i != proto.pose().covariance_size(); ++i) {
    ros.pose.covariance[i] = proto.pose().covariance(i);
  }
  ros.twist.twist.linear.x = proto.twist().twist().linear().x();
  ros.twist.twist.linear.y = proto.twist().twist().linear().y();
  ros.twist.twist.linear.z = proto.twist().twist().linear().z();
  ros.twist.twist.angular.x = proto.twist().twist().angular().x();
  ros.twist.twist.angular.y = proto.twist().twist().angular().y();
  ros.twist.twist.angular.z = proto.twist().twist().angular().z();
  for (int i = 0; i != proto.twist().covariance_size(); ++i) {
    ros.twist.covariance[i] = proto.twist().covariance(i);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::Path &proto,
                            nav_msgs::Path &ros) {
  Convert(proto.header(), ros.header);
  for (int i = 0; i != proto.poses_size(); ++i) {
    geometry_msgs::PoseStamped pose;
    Convert(proto.poses(i).header(), pose.header);
    pose.pose.position.x = proto.poses(i).pose().position().x();
    pose.pose.position.y = proto.poses(i).pose().position().y();
    pose.pose.position.z = proto.poses(i).pose().position().z();
    pose.pose.orientation.x = proto.poses(i).pose().orientation().x();
    pose.pose.orientation.y = proto.poses(i).pose().orientation().y();
    pose.pose.orientation.z = proto.poses(i).pose().orientation().z();
    pose.pose.orientation.w = proto.poses(i).pose().orientation().w();
    ros.poses.push_back(pose);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::TransformStamped &proto,
                            geometry_msgs::TransformStamped &ros) {
  Convert(proto.header(), ros.header);
  ros.child_frame_id = proto.child_frame_id();
  ros.transform.translation.x = proto.transform().translation().x();
  ros.transform.translation.y = proto.transform().translation().y();
  ros.transform.translation.z = proto.transform().translation().z();
  ros.transform.rotation.x = proto.transform().rotation().x();
  ros.transform.rotation.y = proto.transform().rotation().y();
  ros.transform.rotation.z = proto.transform().rotation().z();
  ros.transform.rotation.w = proto.transform().rotation().w();
}

void TypeConverter::Convert(const adsfi_proto::viz::Marker &proto,
                            visualization_msgs::Marker &ros) {
  Convert(proto.header(), ros.header);
  ros.ns = proto.ns();
  ros.id = proto.id();
  ros.type = proto.type();
  ros.action = proto.action();
  ros.pose.position.x = proto.pose().position().x();
  ros.pose.position.y = proto.pose().position().y();
  ros.pose.position.z = proto.pose().position().z();
  ros.pose.orientation.x = proto.pose().orientation().x();
  ros.pose.orientation.y = proto.pose().orientation().y();
  ros.pose.orientation.z = proto.pose().orientation().z();
  ros.pose.orientation.w = proto.pose().orientation().w();
  ros.scale.x = proto.scale().x();
  ros.scale.y = proto.scale().y();
  ros.scale.z = proto.scale().z();
  ros.color.r = proto.color().r();
  ros.color.g = proto.color().g();
  ros.color.b = proto.color().b();
  ros.color.a = proto.color().a();
  ros.lifetime.sec = proto.lifetime().sec();
  ros.lifetime.nsec = proto.lifetime().nsec();
  ros.frame_locked = proto.frame_locked();

  for (int i = 0; i != proto.points_size(); ++i) {
    geometry_msgs::Point rp;
    rp.x = proto.points(i).x();
    rp.y = proto.points(i).y();
    rp.z = proto.points(i).z();
    ros.points.push_back(rp);
  }

  for (int i = 0; i != proto.colors_size(); ++i) {
    std_msgs::ColorRGBA rc;
    rc.r = proto.colors(i).r();
    rc.g = proto.colors(i).g();
    rc.b = proto.colors(i).b();
    rc.a = proto.colors(i).a();
    ros.colors.push_back(rc);
  }

  ros.text = proto.text();

  ros.mesh_resource = proto.mesh_resource();
  ros.mesh_use_embedded_materials = proto.mesh_use_embedded_materials();
}

void TypeConverter::Convert(const adsfi_proto::viz::MarkerArray &proto,
                            visualization_msgs::MarkerArray &ros) {
  for (int i = 0; i != proto.markers_size(); ++i) {
    visualization_msgs::Marker rm;
    Convert(proto.markers(i), rm);
    ros.markers.push_back(rm);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::TwistStamped &proto,
                            geometry_msgs::TwistStamped &ros) {
  Convert(proto.header(), ros.header);
  ros.twist.linear.x = proto.twist().linear().x();
  ros.twist.linear.y = proto.twist().linear().y();
  ros.twist.linear.z = proto.twist().linear().z();
  ros.twist.angular.x = proto.twist().angular().x();
  ros.twist.angular.y = proto.twist().angular().y();
  ros.twist.angular.z = proto.twist().angular().z();
}

void TypeConverter::Convert(const adsfi_proto::viz::PolygonStamped &proto,
                            geometry_msgs::PolygonStamped &ros) {
  Convert(proto.header(), ros.header);
  for (int i = 0; i != proto.polygon().points_size(); ++i) {
    geometry_msgs::Point32 rpt;
    rpt.x = proto.polygon().points(i).x();
    rpt.y = proto.polygon().points(i).y();
    rpt.z = proto.polygon().points(i).z();
    ros.polygon.points.push_back(rpt);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::PointCloud &proto,
                            sensor_msgs::PointCloud &ros) {
  Convert(proto.header(), ros.header);

  for (const auto &p : proto.points()) {
    geometry_msgs::Point32 points;
    points.x = p.x();
    points.y = p.y();
    points.z = p.z();
    ros.points.push_back(points);
  }

  for (const auto &cn : proto.channels()) {
    sensor_msgs::ChannelFloat32 channels;
    channels.name = cn.name();
    for (const auto &values : cn.values()) {
      channels.values.push_back(values);
    }
    ros.channels.push_back(channels);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::PointCloud2 &proto,
                            sensor_msgs::PointCloud2 &ros) {
  Convert(proto.header(), ros.header);
  ros.height = proto.height();
  ros.width = proto.width();

  for (const auto &ppf : proto.fields()) {
    sensor_msgs::PointField rpf;
    rpf.name = ppf.name();
    rpf.offset = ppf.offset();
    rpf.datatype = ppf.datatype();
    rpf.count = ppf.count();
    ros.fields.push_back(rpf);
  }

  ros.is_bigendian = proto.is_bigendian();
  ros.point_step = proto.point_step();
  ros.row_step = proto.row_step();
  for (char i : proto.data()) {
    ros.data.push_back(i);
  }
  ros.is_dense = proto.is_dense();
}

void TypeConverter::Convert(const adsfi_proto::viz::PoseArray &proto,
                            geometry_msgs::PoseArray &ros) {
  Convert(proto.header(), ros.header);

  for (const auto &pose : proto.poses()) {
    geometry_msgs::Pose p;
    p.position.x = pose.position().x();
    p.position.y = pose.position().y();
    p.position.z = pose.position().z();
    p.orientation.w = pose.orientation().w();
    p.orientation.x = pose.orientation().x();
    p.orientation.y = pose.orientation().y();
    p.orientation.z = pose.orientation().z();
    ros.poses.emplace_back(p);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::OccupancyGrid &proto,
                            nav_msgs::OccupancyGrid &ros) {
  Convert(proto.header(), ros.header);

  ros.info.map_load_time.sec = proto.info().map_load_time().sec();
  ros.info.map_load_time.nsec = proto.info().map_load_time().nsec();
  ros.info.resolution = proto.info().resolution();
  ros.info.width = proto.info().width();
  ros.info.height = proto.info().height();
  ros.info.origin.position.x = proto.info().origin().position().x();
  ros.info.origin.position.y = proto.info().origin().position().y();
  ros.info.origin.position.z = proto.info().origin().position().z();
  ros.info.origin.orientation.w = proto.info().origin().orientation().w();
  ros.info.origin.orientation.x = proto.info().origin().orientation().x();
  ros.info.origin.orientation.y = proto.info().origin().orientation().y();
  ros.info.origin.orientation.z = proto.info().origin().orientation().z();
  for (char i : proto.data()) {
    ros.data.emplace_back(i);
  }
}

}  // namespace util
}  // namespace mp
}  // namespace hozon