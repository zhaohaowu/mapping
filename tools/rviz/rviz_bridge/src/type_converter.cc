/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： type_converter.cc
 *   author     ： taoshaoyuan
 *   date       ： 2022.6
 ******************************************************************************/

#include "type_converter.h"  // NOLINT

#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace util {

void SplitStamp(double secs, uint32_t* sec, uint32_t* nsec) {
  if (sec == nullptr || nsec == nullptr) {
    return;
  }
  if (secs < 0) {
    return;
  }
  auto s = static_cast<uint32_t>(secs);
  auto ns = static_cast<uint32_t>((secs - s) * 1e9);
  *sec = s;
  *nsec = ns;
}

void TypeConverter::Convert(const adsfi_proto::hz_Adsfi::AlgHeader& proto,
                            std_msgs::Header* ros) {
  ros->frame_id = proto.frameid();
  ros->seq = proto.seq();
  ros->stamp.sec = proto.timestamp().sec();
  ros->stamp.nsec = proto.timestamp().nsec();
}

void TypeConverter::Convert(const hozon::common::Header& proto,
                            std_msgs::Header* ros) {
  ros->frame_id = proto.frame_id();
  ros->seq = proto.seq();
  uint32_t sec = 0;
  uint32_t nsec = 0;
  SplitStamp(proto.data_stamp(), &sec, &nsec);
  ros->stamp.sec = sec;
  ros->stamp.nsec = nsec;
}

void TypeConverter::Convert(const hozon::common::Header& proto,
                            rviz_msgs::Header* ros) {
  ros->seq = proto.seq();
  ros->frame_id = proto.frame_id();
  ros->publish_stamp = proto.publish_stamp();
  ros->gnss_stamp = proto.gnss_stamp();
  ros->data_stamp = proto.data_stamp();
  ros->sensor_stamp.lidar_stamp = proto.sensor_stamp().lidar_stamp();
  ros->sensor_stamp.radar_stamp = proto.sensor_stamp().radar_stamp();
  ros->sensor_stamp.uss_stamp = proto.sensor_stamp().uss_stamp();
  ros->sensor_stamp.chassis_stamp = proto.sensor_stamp().chassis_stamp();
  ros->sensor_stamp.camera_stamp = proto.sensor_stamp().camera_stamp();
  ros->sensor_stamp.imuins_stamp = proto.sensor_stamp().imuins_stamp();
  ros->sensor_stamp.gnss_stamp = proto.sensor_stamp().gnss_stamp();
}

void TypeConverter::Convert(const adsfi_proto::viz::CompressedImage& proto,
                            sensor_msgs::CompressedImage* ros) {
  Convert(proto.header(), &ros->header);
  ros->format = proto.format();
  for (char i : proto.data()) {
    ros->data.push_back(i);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::Odometry& proto,
                            nav_msgs::Odometry* ros) {
  Convert(proto.header(), &ros->header);
  ros->child_frame_id = proto.child_frame_id();
  ConvertXYZ(proto.pose().pose().position(), &ros->pose.pose.position);
  ConvertWXYZ(proto.pose().pose().orientation(), &ros->pose.pose.orientation);
  if (proto.pose().covariance_size() != ros->pose.covariance.size()) {
    HLOG_ERROR << "covariance size doesn't match!";
    return;
  }
  for (int i = 0; i != proto.pose().covariance_size(); ++i) {
    ros->pose.covariance[i] = proto.pose().covariance(i);
  }
  ConvertXYZ(proto.twist().twist().linear(), &ros->twist.twist.linear);
  ConvertXYZ(proto.twist().twist().angular(), &ros->twist.twist.angular);
  for (int i = 0; i != proto.twist().covariance_size(); ++i) {
    ros->twist.covariance[i] = proto.twist().covariance(i);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::Path& proto,
                            nav_msgs::Path* ros) {
  Convert(proto.header(), &ros->header);
  for (int i = 0; i != proto.poses_size(); ++i) {
    geometry_msgs::PoseStamped pose;
    Convert(proto.poses(i).header(), &pose.header);
    ConvertXYZ(proto.poses(i).pose().position(), &pose.pose.position);
    ConvertWXYZ(proto.poses(i).pose().orientation(), &pose.pose.orientation);
    ros->poses.push_back(pose);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::TransformStamped& proto,
                            geometry_msgs::TransformStamped* ros) {
  Convert(proto.header(), &ros->header);
  ros->child_frame_id = proto.child_frame_id();
  ConvertXYZ(proto.transform().translation(), &ros->transform.translation);
  ConvertWXYZ(proto.transform().rotation(), &ros->transform.rotation);
}

void TypeConverter::Convert(const adsfi_proto::viz::Marker& proto,
                            visualization_msgs::Marker* ros) {
  Convert(proto.header(), &ros->header);
  ros->ns = proto.ns();
  ros->id = proto.id();
  ros->type = proto.type();
  ros->action = proto.action();
  ConvertXYZ(proto.pose().position(), &ros->pose.position);
  ConvertWXYZ(proto.pose().orientation(), &ros->pose.orientation);
  ConvertXYZ(proto.scale(), &ros->scale);
  ros->color.r = proto.color().r();
  ros->color.g = proto.color().g();
  ros->color.b = proto.color().b();
  ros->color.a = proto.color().a();
  ros->lifetime.sec = proto.lifetime().sec();
  ros->lifetime.nsec = proto.lifetime().nsec();
  ros->frame_locked = proto.frame_locked();

  for (int i = 0; i != proto.points_size(); ++i) {
    geometry_msgs::Point rp;
    ConvertXYZ(proto.points(i), &rp);
    ros->points.push_back(rp);
  }

  for (int i = 0; i != proto.colors_size(); ++i) {
    std_msgs::ColorRGBA rc;
    rc.r = proto.colors(i).r();
    rc.g = proto.colors(i).g();
    rc.b = proto.colors(i).b();
    rc.a = proto.colors(i).a();
    ros->colors.push_back(rc);
  }

  ros->text = proto.text();

  ros->mesh_resource = proto.mesh_resource();
  ros->mesh_use_embedded_materials = proto.mesh_use_embedded_materials();
}

void TypeConverter::Convert(const adsfi_proto::viz::MarkerArray& proto,
                            visualization_msgs::MarkerArray* ros) {
  for (int i = 0; i != proto.markers_size(); ++i) {
    visualization_msgs::Marker rm;
    Convert(proto.markers(i), &rm);
    ros->markers.push_back(rm);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::TwistStamped& proto,
                            geometry_msgs::TwistStamped* ros) {
  Convert(proto.header(), &ros->header);
  ConvertXYZ(proto.twist().linear(), &ros->twist.linear);
  ConvertXYZ(proto.twist().angular(), &ros->twist.angular);
}

void TypeConverter::Convert(const adsfi_proto::viz::PolygonStamped& proto,
                            geometry_msgs::PolygonStamped* ros) {
  Convert(proto.header(), &ros->header);
  for (int i = 0; i != proto.polygon().points_size(); ++i) {
    geometry_msgs::Point32 rpt;
    ConvertXYZ(proto.polygon().points(i), &rpt);
    ros->polygon.points.push_back(rpt);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::PointCloud& proto,
                            sensor_msgs::PointCloud* ros) {
  Convert(proto.header(), &ros->header);

  for (const auto& p : proto.points()) {
    geometry_msgs::Point32 points;
    ConvertXYZ(p, &points);
    ros->points.push_back(points);
  }

  for (const auto& cn : proto.channels()) {
    sensor_msgs::ChannelFloat32 channels;
    channels.name = cn.name();
    for (const auto& values : cn.values()) {
      channels.values.push_back(values);
    }
    ros->channels.push_back(channels);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::PointCloud2& proto,
                            sensor_msgs::PointCloud2* ros) {
  Convert(proto.header(), &ros->header);
  ros->height = proto.height();
  ros->width = proto.width();

  for (const auto& ppf : proto.fields()) {
    sensor_msgs::PointField rpf;
    rpf.name = ppf.name();
    rpf.offset = ppf.offset();
    rpf.datatype = ppf.datatype();
    rpf.count = ppf.count();
    ros->fields.push_back(rpf);
  }

  ros->is_bigendian = proto.is_bigendian();
  ros->point_step = proto.point_step();
  ros->row_step = proto.row_step();
  for (char i : proto.data()) {
    ros->data.push_back(i);
  }
  ros->is_dense = proto.is_dense();
}

void TypeConverter::Convert(const adsfi_proto::viz::PoseArray& proto,
                            geometry_msgs::PoseArray* ros) {
  Convert(proto.header(), &ros->header);

  for (const auto& pose : proto.poses()) {
    geometry_msgs::Pose p;
    ConvertXYZ(pose.position(), &p.position);
    ConvertWXYZ(pose.orientation(), &p.orientation);
    ros->poses.emplace_back(p);
  }
}

void TypeConverter::Convert(const adsfi_proto::viz::OccupancyGrid& proto,
                            nav_msgs::OccupancyGrid* ros) {
  Convert(proto.header(), &ros->header);

  ros->info.map_load_time.sec = proto.info().map_load_time().sec();
  ros->info.map_load_time.nsec = proto.info().map_load_time().nsec();
  ros->info.resolution = proto.info().resolution();
  ros->info.width = proto.info().width();
  ros->info.height = proto.info().height();
  ConvertXYZ(proto.info().origin().position(), &ros->info.origin.position);
  ConvertWXYZ(proto.info().origin().orientation(),
              &ros->info.origin.orientation);
  for (char i : proto.data()) {
    ros->data.emplace_back(i);
  }
}

void TypeConverter::Convert(const hozon::common::Pose& proto,
                            rviz_msgs::Pose* ros) {
  ConvertXYZ(proto.position(), &ros->position);
  ConvertWXYZ(proto.quaternion(), &ros->quaternion);
  ConvertXYZ(proto.euler_angle(), &ros->euler_angle);
  ConvertXYZ(proto.rotation_vrf(), &ros->rotation_vrf);

  ros->heading = proto.heading();

  ConvertXYZ(proto.linear_velocity_vrf(), &ros->linear_velocity_vrf);
  ConvertXYZ(proto.linear_acceleration_vrf(), &ros->linear_acceleration_vrf);
  ConvertXYZ(proto.angular_velocity_vrf(), &ros->angular_velocity_vrf);
  ConvertXYZ(proto.euler_angles(), &ros->euler_angles);
  ConvertXYZ(proto.wgs(), &ros->wgs);
  ConvertXYZ(proto.gcj02(), &ros->gcj02);
  ConvertXYZ(proto.pos_utm_01(), &ros->pos_utm_01);
  ConvertXYZ(proto.pos_utm_02(), &ros->pos_utm_02);

  ros->utm_zone_01 = proto.utm_zone_01();
  ros->utm_zone_02 = proto.utm_zone_02();
  ros->using_utm_zone = proto.using_utm_zone();
  ros->heading_gcs = proto.heading_gcs();

  ConvertXYZ(proto.linear_acceleration_raw_vrf(),
             &ros->linear_acceleration_raw_vrf);
  ConvertXYZ(proto.linear_velocity(), &ros->linear_velocity);
  ConvertXYZ(proto.linear_acceleration(), &ros->linear_acceleration);
  ConvertXYZ(proto.angular_velocity(), &ros->angular_velocity);
  ConvertXYZ(proto.local_pose(), &ros->local_pose);
  ConvertXYZ(proto.angular_velocity_raw_vrf(), &ros->angular_velocity_raw_vrf);
  ConvertXYZ(proto.euler_angles_local(), &ros->euler_angles_local);

  ros->local_heading = proto.local_heading();
}

void TypeConverter::Convert(const hozon::localization::Localization& proto,
                            rviz_msgs::Localization* ros) {
  Convert(proto.header(), &ros->header);
  Convert(proto.header(), &ros->proto_header);

  Convert(proto.pose(), &ros->pose);
  ros->measurement_time = proto.measurement_time();
  ros->rtk_status = proto.rtk_status();
  ros->location_state = proto.location_state();
  Convert(proto.pose_dr(), &ros->pose_dr);
  Convert(proto.pose_local(), &ros->pose_local);
  ros->laneid = proto.laneid();
}
void TypeConverter::Convert(const hozon::localization::HafNodeInfo& proto,
                            rviz_msgs::HafNodeInfo* ros) {
  Convert(proto.header(), &ros->header);
  Convert(proto.header(), &ros->proto_header);

  ros->type = proto.type();
  ConvertXYZ(proto.pos_wgs(), &ros->pos_wgs);
  ConvertXYZ(proto.attitude(), &ros->attitude);
  ConvertWXYZ(proto.quaternion(), &ros->quaternion);
  ConvertXYZ(proto.linear_velocity(), &ros->linear_velocity);

  ros->sys_status = proto.sys_status();
  ros->gps_status = proto.gps_status();
  ros->heading = proto.heading();

  ConvertXYZ(proto.pos_gcj02(), &ros->pos_gcj02);
  ConvertXYZ(proto.angular_velocity(), &ros->angular_velocity);
  ConvertXYZ(proto.linear_acceleration(), &ros->linear_acceleration);
}

void TypeConverter::Convert(const hozon::soc::ImuInfo& proto,
                            rviz_msgs::ImuInfo* ros) {
  ConvertXYZ(proto.angular_velocity(), &ros->angular_velocity);
  ConvertXYZ(proto.linear_acceleration(), &ros->linear_acceleration);
  ConvertXYZ(proto.imuvb_angular_velocity(), &ros->imuVB_angular_velocity);
  ConvertXYZ(proto.imuvb_linear_acceleration(),
             &ros->imuVB_linear_acceleration);

  ros->imu_status = proto.imu_status();
  ros->temperature = proto.temperature();
}
void TypeConverter::Convert(const hozon::soc::InsInfo& proto,
                            rviz_msgs::InsInfo* ros) {
  ros->latitude = proto.latitude();
  ros->longitude = proto.longitude();
  ros->altitude = proto.altitude();

  ConvertXYZ(proto.attitude(), &ros->attitude);
  ConvertXYZ(proto.linear_velocity(), &ros->linear_velocity);
  ConvertXYZ(proto.augular_velocity(), &ros->augular_velocity);
  ConvertXYZ(proto.linear_acceleration(), &ros->linear_acceleration);

  ros->heading = proto.heading();
  ros->sys_status = proto.sys_status();
  ros->gps_status = proto.gps_status();
}

void TypeConverter::Convert(const hozon::soc::ImuIns& proto,
                            rviz_msgs::ImuIns* ros) {
  Convert(proto.header(), &ros->header);
  Convert(proto.header(), &ros->proto_header);

  ros->gps_week = proto.gps_week();
  ros->gps_sec = proto.gps_sec();

  Convert(proto.imu_info(), &ros->imu_info);
  Convert(proto.ins_info(), &ros->ins_info);
}

void TypeConverter::Convert(const hozon::dead_reckoning::DeadReckoning& proto,
                            rviz_msgs::DeadReckoning* ros) {
  Convert(proto.header(), &ros->header);
  Convert(proto.header(), &ros->proto_header);

  ros->gnss_timestamp = proto.gnss_timestamp();
  ConvertXYZ(proto.mounting_error(), &ros->mounting_error);

  Convert(proto.pose().pose_wgs(), &ros->pose.pose_wgs);
  Convert(proto.pose().pose_local(), &ros->pose.pose_local);
  Convert(proto.pose().pose_gcj02(), &ros->pose.pose_gcj02);
  Convert(proto.pose().pose_utm01(), &ros->pose.pose_utm01);
  Convert(proto.pose().pose_utm02(), &ros->pose.pose_utm02);

  ros->pose.utm_zone_id01 = proto.pose().utm_zone_id01();
  ros->pose.utm_zone_id02 = proto.pose().utm_zone_id02();

  if (proto.pose().std().size() != ros->pose.std.size()) {
    HLOG_ERROR << "covariance size doesn't match!";
  } else {
    for (int i = 0; i != ros->pose.std.size(); ++i) {
      ros->pose.std[i] = proto.pose().std(i);
    }
  }

  ConvertXYZ(proto.velocity().twist_vrf().linear_vrf(),
             &ros->velocity.twist_vrf.linear_vrf);
  ConvertXYZ(proto.velocity().twist_vrf().angular_raw_vrf(),
             &ros->velocity.twist_vrf.angular_raw_vrf);
  ConvertXYZ(proto.velocity().twist_vrf().angular_vrf(),
             &ros->velocity.twist_vrf.angular_vrf);

  if (proto.velocity().std().size() != ros->velocity.std.size()) {
    HLOG_ERROR << "covariance size doesn't match!";
  } else {
    for (int i = 0; i != ros->velocity.std.size(); ++i) {
      ros->velocity.std[i] = proto.velocity().std(i);
    }
  }

  ConvertXYZ(proto.acceleration().linear_vrf().linear_raw_vrf(),
             &ros->acceleration.linear_vrf.linear_raw_vrf);
  ConvertXYZ(proto.acceleration().linear_vrf().linear_vrf(),
             &ros->acceleration.linear_vrf.linear_vrf);
  ConvertXYZ(proto.acceleration().linear_vrf().angular_vrf(),
             &ros->acceleration.linear_vrf.angular_vrf);

  if (proto.acceleration().std().size() != ros->acceleration.std.size()) {
    HLOG_ERROR << "covariance size doesn't match!";
  } else {
    for (int i = 0; i != ros->acceleration.std.size(); ++i) {
      ros->acceleration.std[i] = proto.acceleration().std(i);
    }
  }

  ros->dr_status = proto.dr_status();
}

void TypeConverter::Convert(const hozon::soc::Chassis& proto,
                            rviz_msgs::Chassis* ros) {
  Convert(proto.header(), &ros->header);
  Convert(proto.header(), &ros->proto_header);

  ros->wheel_speed.is_wheel_spd_rr_valid =
      proto.wheel_speed().is_wheel_spd_rr_valid();
  ros->wheel_speed.wheel_direction_rr =
      proto.wheel_speed().wheel_direction_rr();
  ros->wheel_speed.wheel_spd_rr = proto.wheel_speed().wheel_spd_rr();
  ros->wheel_speed.is_wheel_spd_rl_valid =
      proto.wheel_speed().is_wheel_spd_rl_valid();
  ros->wheel_speed.wheel_direction_rl =
      proto.wheel_speed().wheel_direction_rl();
  ros->wheel_speed.wheel_spd_rl = proto.wheel_speed().wheel_spd_rl();
  ros->wheel_speed.is_wheel_spd_fr_valid =
      proto.wheel_speed().is_wheel_spd_fr_valid();
  ros->wheel_speed.wheel_direction_fr =
      proto.wheel_speed().wheel_direction_fr();
  ros->wheel_speed.wheel_spd_fr = proto.wheel_speed().wheel_spd_fr();
  ros->wheel_speed.is_wheel_spd_fl_valid =
      proto.wheel_speed().is_wheel_spd_fl_valid();
  ros->wheel_speed.wheel_direction_fl =
      proto.wheel_speed().wheel_direction_fl();
  ros->wheel_speed.wheel_spd_fl = proto.wheel_speed().wheel_spd_fl();

  ros->wheel_counter.is_wheel_cnt_rr_valid =
      proto.wheel_counter().is_wheel_cnt_rr_valid();
  ros->wheel_counter.wheel_counter_rr =
      proto.wheel_counter().wheel_counter_rr();
  ros->wheel_counter.is_wheel_cnt_rl_valid =
      proto.wheel_counter().is_wheel_cnt_rl_valid();
  ros->wheel_counter.wheel_counter_rl =
      proto.wheel_counter().wheel_counter_rl();
  ros->wheel_counter.is_wheel_cnt_fr_valid =
      proto.wheel_counter().is_wheel_cnt_fr_valid();
  ros->wheel_counter.wheel_counter_fr =
      proto.wheel_counter().wheel_counter_fr();
  ros->wheel_counter.is_wheel_cnt_fl_valid =
      proto.wheel_counter().is_wheel_cnt_fl_valid();
  ros->wheel_counter.wheel_counter_fl =
      proto.wheel_counter().wheel_counter_fl();

  ros->gear_location = proto.gear_location();
}

}  // namespace util
}  // namespace mp
}  // namespace hozon
