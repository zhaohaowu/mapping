// /********************************************************
//  * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
//  * Licensed Hozon
//  * Author: egui
//  *******************************************************/

// #include "modules/local_mapping/data_mapping/data_mapping.h"
// #include "perception-base/base/object/object.h"
// #include "perception-base/base/object/object_types.h"

// namespace hozon {
// namespace mp {

// namespace lm {
// namespace data_mapping {

// using hozon::perception::PerceptionObstacle;
// using hozon::perception::SensorStatus;
// using hozon::perception::base::ObjectSubType;
// using hozon::perception::base::ObjectType;

// void CvtTypeObjectToPb(const perception_base::ObjectPtr& obj,
//                        NetaPerceptionObstacle* pb_obj);

// void CvtPb2TypeObject(const NetaPerceptionObstacle& pb_obj,
//                       perception_base::ObjectPtr& obj);  // NOLINT

// bool CvtPbObjectToFrame(const NetaPerceptionObstaclesPtr& pb_obj,
//                         perception_base::FramePtr framemsg) {
//   return true;
// }

// bool DataMapping::CvtPb2MultiObjects(
//     const hozon::perception::PerceptionObstacles& pb_objects,
//     std::vector<perception_base::ObjectPtr>& objects_msg) {  // NOLINT
//   for (auto& item : pb_objects.detected_obstacle()) {
//     perception_base::ObjectPtr objptr =
//         std::make_shared<perception_base::Object>();
//     if (!CvtPb2Object(item, objptr)) {
//       return false;
//     }
//     objects_msg.push_back(objptr);
//   }
//   return true;
// }

// bool DataMapping::CvtPb2Object(const NetaPerceptionObstacle& pb_obj,
//                                perception_base::ObjectPtr& object_msg) {
//   object_msg->id = pb_obj.id();
//   object_msg->confidence = pb_obj.type_confidence();

//   // object_msg->center.x() = pb_obj.position().x();
//   // object_msg->center.y() = pb_obj.position().y();
//   // object_msg->center.z() = pb_obj.position().z();

//   object_msg->velocity.x() = pb_obj.velocity().x();
//   object_msg->velocity.y() = pb_obj.velocity().y();
//   object_msg->velocity.z() = pb_obj.velocity().z();

//   object_msg->size.x() = pb_obj.length();
//   object_msg->size.y() = pb_obj.width();
//   object_msg->size.z() = pb_obj.height();

//   object_msg->theta = pb_obj.theta();

//   for (auto& item : pb_obj.polygon_point()) {
//     perception_base::PointF pt;
//     pt.x = item.x();
//     pt.y = item.y();
//     pt.z = item.z();
//     object_msg->polygon.push_back(pt);
//   }

//   object_msg->acceleration.x() = pb_obj.acceleration().x();
//   object_msg->acceleration.y() = pb_obj.acceleration().y();
//   object_msg->acceleration.z() = pb_obj.acceleration().z();

//   object_msg->center.x() = pb_obj.center().x();
//   object_msg->center.y() = pb_obj.center().y();
//   object_msg->center.z() = pb_obj.center().z();

//   for (const auto& item : pb_obj.type_probs()) {
//     object_msg->type_probs.push_back(item);
//   }

//   CvtPb2TypeObject(pb_obj, object_msg);

//   return true;
// }

// bool DataMapping::CvtObjectToPb(const perception_base::ObjectPtr& object_msg,
//                                 NetaPerceptionObstacle* pb_object) {
//   if (nullptr == object_msg || nullptr == pb_object) {
//     HLOG_ERROR << "object msg  or pb_object is nullptr.";
//     return false;
//   }
//   // id
//   pb_object->set_id(object_msg->id);
//   // position
//   pb_object->mutable_position()->set_x(object_msg->center.x());
//   pb_object->mutable_position()->set_y(object_msg->center.y());
//   pb_object->mutable_position()->set_z(object_msg->center.z());
//   // velocity
//   pb_object->mutable_velocity()->set_x(object_msg->velocity.x());
//   pb_object->mutable_velocity()->set_y(object_msg->velocity.y());
//   pb_object->mutable_velocity()->set_z(object_msg->velocity.z());
//   // theta
//   pb_object->set_theta(object_msg->theta);
//   // l,w,h
//   pb_object->set_length(object_msg->size.x());
//   pb_object->set_width(object_msg->size.y());
//   pb_object->set_height(object_msg->size.z());
//   // polygon point
//   uint32_t polygon_point_size = object_msg->polygon.size();
//   for (size_t j = 0; j < polygon_point_size; j++) {
//     auto pb_polygon_point = pb_object->add_polygon_point();
//     pb_polygon_point->set_x(object_msg->polygon[j].x);
//     pb_polygon_point->set_y(object_msg->polygon[j].y);
//     pb_polygon_point->set_z(object_msg->polygon[j].z);
//   }
//   // tracking_time
//   pb_object->set_tracking_time(object_msg->tracking_time);
//   pb_object->set_latest_tracked_time(object_msg->latest_tracked_time);
//   // type, sub_type
//   CvtTypeObjectToPb(object_msg, pb_object);
//   // timestamp
//   // point_cloud
//   // type_confidence
//   pb_object->set_type_confidence(object_msg->confidence);
//   // confidence_type
//   // drops
//   // accelerataion
//   pb_object->mutable_acceleration()->set_x(object_msg->acceleration.x());
//   pb_object->mutable_acceleration()->set_y(object_msg->acceleration.y());
//   pb_object->mutable_acceleration()->set_z(object_msg->acceleration.z());
//   // anchor_point
//   // bbox2d
//   // measurements
//   // height_above_ground
//   // position_covariance
//   // velocity_covariance
//   // acceleration_covariance
//   // light_status
//   if (pb_object->type() == PerceptionObstacle::VEHICLE) {
//     auto pb_light_status = pb_object->mutable_light_status();
//     auto& light_status = object_msg->light_status;
//     // pb_light_status->set_brake_visible(light_status.brake_visible);
//     // pb_light_status->set_brake_switch_on(light_status.brake_switch_on);
//     //
//     pb_light_status->set_left_turn_visible(light_status.left_turn_visible);
//     //
//     pb_light_status->set_left_turn_switch_on(light_status.left_turn_switch_on);
//     //
//     pb_light_status->set_right_turn_visible(light_status.right_turn_visible);
//     // pb_light_status->set_right_turn_switch_on(
//     //     light_status.right_turn_switch_on);
//   }
//   // source
//   // v2x_info
//   // motion_type
//   // maintenance_type
//   // orientation

//   // orientation_st_dev
//   // existence_probability
//   // pb_object->set_existence_probability(object_msg->confidence);
//   // creation_time
//   // position_flu
//   // theta_flu
//   // velocity_flu
//   // acceleartion_flu
//   // current_detect_sensor
//   // history_detect_sensor
//   // history_motion_type
//   // car_near_side
//   // track_id
//   pb_object->set_track_id(object_msg->id);
//   // type_probs
//   for (size_t j = 0; j < object_msg->type_probs.size(); j++) {
//     pb_object->add_type_probs(object_msg->type_probs[j]);
//   }
//   // sub_type_probs
//   for (size_t j = 0; j < object_msg->sub_type_probs.size(); j++) {
//     pb_object->set_sub_type_probs(j, object_msg->sub_type_probs[j]);
//   }
//   // track_state
//   // track_age
//   // lost_age
//   // center
//   pb_object->mutable_center()->set_x(object_msg->center.x());
//   pb_object->mutable_center()->set_y(object_msg->center.y());
//   pb_object->mutable_center()->set_z(object_msg->center.z());
//   // center_std_dev
//   // center_uncertainty
//   // size_std_dev
//   // theta_std_dev
//   // feature
//   // velocity_converge
//   // velocity_confidence
//   // parse_type
//   // parse_type_probs
//   // latest_tracked_time
//   // is_back_ground
//   // drop_num
//   // is_truncation
//   // truncated_prob
//   // is_occlusion
//   // occluded_prob
//   // is_onroad
//   // onroad_prob
//   // is_sprinkler
//   // sprinkler_prob
//   // track_object_assigned_cost
//   // lidar_supplement
//   // contour_points

//   return true;
// }

// void CvtTypeObjectToPb(const perception_base::ObjectPtr& obj,
//                        NetaPerceptionObstacle* pb_obj) {
//   if (pb_obj == nullptr || obj == nullptr) {
//     HLOG_ERROR << "input data is nullptr.";
//     return;
//   }
//   // type
//   switch (obj->type) {
//     case ObjectType::UNKNOWN:
//       pb_obj->set_type(PerceptionObstacle::UNKNOWN);
//       break;
//     case ObjectType::UNKNOWN_UNMOVABLE:
//       pb_obj->set_type(PerceptionObstacle::UNKNOWN_UNMOVABLE);
//       break;
//     case ObjectType::UNKNOWN_MOVABLE:
//       pb_obj->set_type(PerceptionObstacle::UNKNOWN_MOVABLE);
//       break;
//     case ObjectType::PEDESTRIAN:
//       pb_obj->set_type(PerceptionObstacle::PEDESTRIAN);
//       break;
//     case ObjectType::BICYCLE:
//       pb_obj->set_type(PerceptionObstacle::BICYCLE);
//       break;
//     case ObjectType::VEHICLE:
//       pb_obj->set_type(PerceptionObstacle::VEHICLE);
//       break;
//     case ObjectType::CYCLIST:
//       pb_obj->set_type(PerceptionObstacle::CYCLIST);
//       break;
//     case ObjectType::STATIC_OBSTACLE:
//       pb_obj->set_type(PerceptionObstacle::STATIC_OBSTACLE);
//       break;
//     case ObjectType::TRANSPORT_ELEMENT:
//       pb_obj->set_type(PerceptionObstacle::TRANSPORT_ELEMENT);
//       break;
//     case ObjectType::ANIMAL:
//       pb_obj->set_type(PerceptionObstacle::ANIMAL);
//       break;
//   }

//   // sub_type
//   switch (obj->sub_type) {
//     case ObjectSubType::UNKNOWN:
//       // 处理 UNKNOWN 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_UNKNOWN);
//       break;
//     case ObjectSubType::UNKNOWN_MOVABLE:
//       // 处理 UNKNOWN_MOVABLE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_UNKNOWN_MOVABLE);
//       break;
//     case ObjectSubType::UNKNOWN_UNMOVABLE:
//       // 处理 UNKNOWN_UNMOVABLE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_UNKNOWN_UNMOVABLE);
//       break;
//     case ObjectSubType::PEDESTRIAN:
//       // 处理 PEDESTRIAN 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_PEDESTRIAN);
//       break;
//     case ObjectSubType::BUGGY:
//       // 处理 BUGGY 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_BUGGY);
//       break;
//     case ObjectSubType::BICYCLE:
//       // 处理 BICYCLE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_BICYCLE);
//       break;
//     case ObjectSubType::ELETRICBICYCLE:
//       // 处理 ELETRICBICYCLE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_ELETRICBICYCLE);
//       break;
//     case ObjectSubType::MOTORCYCLE:
//       // 处理 MOTORCYCLE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_MOTORCYCLE);
//       break;
//     case ObjectSubType::TRICYCLE:
//       // 处理 TRICYCLE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_TRICYCLE);
//       break;
//     case ObjectSubType::HANDCAR:
//       // 处理 HANDCAR 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_HANDCAR);
//       break;
//     case ObjectSubType::CAR:
//       // 处理 CAR 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_CAR);
//       break;
//     case ObjectSubType::VAN:
//       // 处理 VAN 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_VAN);
//       break;
//     case ObjectSubType::TRUCK:
//       // 处理 TRUCK 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_TRUCK);
//       break;
//     case ObjectSubType::BIG_TRUCK:
//       // 处理 BIG_TRUCK 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_TRUCK);
//       break;
//     case ObjectSubType::BUS:
//       // 处理 BUS 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_BUS);
//       break;
//     case ObjectSubType::MIDDLE_BUS:
//       // 处理 MIDDLE_BUS 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_BUS);
//       break;
//     case ObjectSubType::MINIBUS:
//       // 处理 MINIBUS 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_MINIBUS);
//       break;
//     case ObjectSubType::PICKUP:
//       // 处理 PICKUP 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_PICKUP);
//       break;
//     case ObjectSubType::AMBULANCE:
//       // 处理 AMBULANCE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_AMBULANCE);
//       break;
//     case ObjectSubType::POLICECAR:
//       // 处理 POLICECAR 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_POLICECAR);
//       break;
//     case ObjectSubType::FIRE_ENGINE:
//       // 处理 FIRE_ENGINE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_UNKNOWN);
//       break;
//     case ObjectSubType::CYCLIST:
//       // 处理 CYCLIST 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_CYCLIST);
//       break;
//     case ObjectSubType::MOTORCYCLIST:
//       // 处理 MOTORCYCLIST 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_MOTORCYCLIST);
//       break;
//     case ObjectSubType::TRICYCLIST:
//       // 处理 TRICYCLIST 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_TRICYCLIST);
//       break;
//     case ObjectSubType::EBICYCLIST:
//       // 处理 EBICYCLIST 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_EBICYCLIST);
//       break;
//     case ObjectSubType::TRAFFICCONE:
//       // 处理 TRAFFICCONE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_TRAFFICCONE);
//       break;
//     case ObjectSubType::SPEEDBUMP:
//       // 处理 SPEEDBUMP 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_SPEEDBUMP);
//       break;
//     case ObjectSubType::FENCE:
//       // 处理 FENCE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_FENCE);
//       break;
//     case ObjectSubType::BARRIER_PARKING_LEVER:
//       // 处理 BARRIER_PARKING_LEVER 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_BARRIER_PARKING_LEVER);
//       break;
//     case ObjectSubType::WATERHORSE:
//       // 处理 WATERHORSE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_WATERHORSE);
//       break;
//     case ObjectSubType::CRASHBARRELS:
//       // 处理 CRASHBARRELS 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_CRASHBARRELS);
//       break;
//     case ObjectSubType::SIGNBOARD:
//       // 处理 SIGNBOARD 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_SIGNBOARD);
//       break;
//     case ObjectSubType::WARNINGTRIANGLE:
//       // 处理 WARNINGTRIANGLE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_WARNINGTRIANGLE);
//       break;
//     case ObjectSubType::STONEBLOCK:
//       // 处理 STONEBLOCK 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_STONEBLOCK);
//       break;
//     // case ObjectSubType::PARKINGSIGN:
//     //   // 处理 PARKINGSIGN 类型,泊车牌分两个子类，金属或者塑料材质
//     //   pb_obj->set_sub_type(PerceptionObstacle::ST_METALPARKINGSIGN);
//     //   break;
//     case ObjectSubType::FIREHYDRANT:
//       // 处理 FIREHYDRANT 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_FIREHYDRANT);
//       break;
//     case ObjectSubType::WHEELSTOP:
//       // 处理 WHEELSTOP 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_WHEELSTOP);
//       break;
//     case ObjectSubType::LOCKER:
//       // 处理 LOCKER 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_LOCKER);
//       break;
//     case ObjectSubType::TRASH:
//       // 处理 TRASH 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_TRASH);
//       break;
//     case ObjectSubType::PILLAR:
//       // 处理 PILLAR 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_PILLAR);
//       break;
//     case ObjectSubType::BARRIER_GAT:
//       // 处理 BARRIER_GAT 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_BARRIER_GAT);
//       break;
//     case ObjectSubType::BARRIER_SIGN:
//       // 处理 BARRIER_SIGN 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_UNKNOWN);
//       break;
//     case ObjectSubType::BARRIER_TRIANGLE:
//       // 处理 BARRIER_TRIANGLE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_WARNINGTRIANGLE);
//       break;
//     case ObjectSubType::STOP:
//       // 处理 STOP 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_STOP);
//       break;
//     case ObjectSubType::SLOWYIELD:
//       // 处理 SLOWYIELD 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_SLOWYIELD);
//       break;
//     case ObjectSubType::NOPASS:
//       // 处理 NOPASS 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_NOPASS);
//       break;
//     case ObjectSubType::NOENTRY:
//       // 处理 NOENTRY 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_NOENTRY);
//       break;
//     case ObjectSubType::NOTURNINGLEFT:
//       // 处理 NOTURNINGLEFT 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_NOTURNINGLEFT);
//       break;
//     case ObjectSubType::NOTURNINGRIGHT:
//       // 处理 NOTURNINGRIGHT 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_NOTURNINGRIGHT);
//       break;
//     case ObjectSubType::NOGOINGSTRAIGHT:
//       // 处理 NOGOINGSTRAIGHT 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_NOGOINGSTRAIGHT);
//       break;
//     case ObjectSubType::NOTURNINGAROUND:
//       // 处理 NOTURNINGAROUND 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_NOTURNINGAROUND);
//       break;
//     case ObjectSubType::NOOVERTAKING:
//       // 处理 NOOVERTAKING 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_NOOVERTAKING);
//       break;
//     case ObjectSubType::REMOVENOOVERTAKING:
//       // 处理 REMOVENOOVERTAKING 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_REMOVENOOVERTAKING);
//       break;
//     case ObjectSubType::NOPARKING:
//       // 处理 NOPARKING 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_NOPARKING);
//       break;
//     case ObjectSubType::NOHONKING:
//       // 处理 NOHONKING 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_NOHONKING);
//       break;
//     case ObjectSubType::SPEEDLIMITLIFTED:
//       // 处理 SPEEDLIMITLIFTED 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_SPEEDLIMITLIFTED);
//       break;
//     case ObjectSubType::SPEEDRELEASELIMITLIFTED:
//       // 处理 SPEEDRELEASELIMITLIFTED 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_SPEEDRELEASELIMITLIFTED);
//       break;
//     case ObjectSubType::TRAFFICLIGHT:
//       // 处理 TRAFFICLIGHT 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_TRAFFICLIGHT);
//       break;
//     case ObjectSubType::BAN:
//       // 处理 BAN 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_BAN);
//       break;
//     case ObjectSubType::D_ARROW:
//       // 处理 D_ARROW 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_D_ARROW);
//       break;
//     case ObjectSubType::L_ARROW:
//       // 处理 L_ARROW 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_L_ARROW);
//       break;
//     case ObjectSubType::R_ARROW:
//       // 处理 R_ARROW 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_R_ARROW);
//       break;
//     case ObjectSubType::A_ARROW:
//       // 处理 A_ARROW 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_A_ARROW);
//       break;
//     case ObjectSubType::ZEBRA:
//       // 处理 ZEBRA 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_ZEBRA);
//       break;
//     case ObjectSubType::STOP_LINE:
//       // 处理 STOP_LINE 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_STOP_LINE);
//       break;
//     case ObjectSubType::CAT:
//       // 处理 CAT 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_CAT);
//       break;
//     case ObjectSubType::DOG:
//       // 处理 DOG 类型
//       pb_obj->set_sub_type(PerceptionObstacle::ST_DOG);
//       break;
//     default:
//       // 处理未知类型
//       break;
//   }
// }

// void CvtPb2TypeObject(const NetaPerceptionObstacle& pb_obj,
//                       perception_base::ObjectPtr& obj) {  // NOLINT
//   // type
//   switch (pb_obj.type()) {
//     case PerceptionObstacle::UNKNOWN:
//       obj->type = ObjectType::UNKNOWN;
//       break;
//     case PerceptionObstacle::UNKNOWN_UNMOVABLE:
//       obj->type = ObjectType::UNKNOWN_UNMOVABLE;
//       break;
//     case PerceptionObstacle::UNKNOWN_MOVABLE:
//       obj->type = ObjectType::UNKNOWN_MOVABLE;
//       break;
//     case PerceptionObstacle::PEDESTRIAN:
//       obj->type = ObjectType::PEDESTRIAN;
//       break;
//     case PerceptionObstacle::BICYCLE:
//       obj->type = ObjectType::BICYCLE;
//       break;
//     case PerceptionObstacle::VEHICLE:
//       obj->type = ObjectType::VEHICLE;
//       break;
//     case PerceptionObstacle::CYCLIST:
//       obj->type = ObjectType::CYCLIST;
//       break;
//     case PerceptionObstacle::STATIC_OBSTACLE:
//       obj->type = ObjectType::STATIC_OBSTACLE;
//       break;
//     case PerceptionObstacle::TRANSPORT_ELEMENT:
//       obj->type = ObjectType::TRANSPORT_ELEMENT;
//       break;
//     case PerceptionObstacle::ANIMAL:
//       obj->type = ObjectType::ANIMAL;
//       break;
//   }

//   // sub_type
//   switch (pb_obj.sub_type()) {
//     case PerceptionObstacle::ST_UNKNOWN:
//       // 处理 UNKNOWN 类型
//       obj->sub_type = ObjectSubType::UNKNOWN;
//       break;
//     case PerceptionObstacle::ST_UNKNOWN_MOVABLE:
//       // 处理 UNKNOWN_MOVABLE 类型
//       obj->sub_type = ObjectSubType::UNKNOWN_MOVABLE;
//       break;
//     case PerceptionObstacle::ST_UNKNOWN_UNMOVABLE:
//       // 处理 UNKNOWN_UNMOVABLE 类型
//       obj->sub_type = ObjectSubType::UNKNOWN_UNMOVABLE;
//       break;
//     case PerceptionObstacle::ST_PEDESTRIAN:
//       // 处理 PEDESTRIAN 类型
//       obj->sub_type = ObjectSubType::PEDESTRIAN;
//       break;
//     case PerceptionObstacle::ST_BUGGY:
//       // 处理 BUGGY 类型
//       obj->sub_type = ObjectSubType::BUGGY;
//       break;
//     case PerceptionObstacle::ST_BICYCLE:
//       // 处理 BICYCLE 类型
//       obj->sub_type = ObjectSubType::BICYCLE;
//       break;
//     case PerceptionObstacle::ST_ELETRICBICYCLE:
//       // 处理 ELETRICBICYCLE 类型
//       obj->sub_type = ObjectSubType::ELETRICBICYCLE;
//       break;
//     case PerceptionObstacle::ST_MOTORCYCLE:
//       // 处理 MOTORCYCLE 类型
//       obj->sub_type = ObjectSubType::MOTORCYCLE;
//       break;
//     case PerceptionObstacle::ST_TRICYCLE:
//       // 处理 TRICYCLE 类型
//       obj->sub_type = ObjectSubType::TRICYCLE;
//       break;
//     case PerceptionObstacle::ST_HANDCAR:
//       // 处理 HANDCAR 类型
//       obj->sub_type = ObjectSubType::HANDCAR;
//       break;
//     case PerceptionObstacle::ST_CAR:
//       // 处理 CAR 类型
//       obj->sub_type = ObjectSubType::CAR;
//       break;
//     case PerceptionObstacle::ST_VAN:
//       // 处理 VAN 类型
//       obj->sub_type = ObjectSubType::VAN;
//       break;
//     case PerceptionObstacle::ST_TRUCK:
//       // 处理 TRUCK 类型
//       obj->sub_type = ObjectSubType::TRUCK;
//       break;
//     case PerceptionObstacle::ST_BUS:
//       // 处理 BUS 类型
//       obj->sub_type = ObjectSubType::BUS;
//       break;
//     case PerceptionObstacle::ST_MINIBUS:
//       // 处理 MINIBUS 类型
//       obj->sub_type = ObjectSubType::MINIBUS;
//       break;
//     case PerceptionObstacle::ST_PICKUP:
//       // 处理 PICKUP 类型
//       obj->sub_type = ObjectSubType::PICKUP;
//       break;
//     case PerceptionObstacle::ST_AMBULANCE:
//       // 处理 AMBULANCE 类型
//       obj->sub_type = ObjectSubType::AMBULANCE;
//       break;
//     case PerceptionObstacle::ST_POLICECAR:
//       // 处理 POLICECAR 类型
//       obj->sub_type = ObjectSubType::POLICECAR;
//       break;
//     // case PerceptionObstacle::ST_UNKNOWN:
//     //   // 处理 FIRE_ENGINE 类型
//     //   obj->sub_type = ObjectSubType::FIRE_ENGINE;
//     //   break;
//     case PerceptionObstacle::ST_CYCLIST:
//       // 处理 CYCLIST 类型
//       obj->sub_type = ObjectSubType::CYCLIST;
//       break;
//     case PerceptionObstacle::ST_MOTORCYCLIST:
//       // 处理 MOTORCYCLIST 类型
//       obj->sub_type = ObjectSubType::MOTORCYCLIST;
//       break;
//     case PerceptionObstacle::ST_TRICYCLIST:
//       // 处理 TRICYCLIST 类型
//       obj->sub_type = ObjectSubType::TRICYCLIST;
//       break;
//     case PerceptionObstacle::ST_EBICYCLIST:
//       // 处理 EBICYCLIST 类型
//       obj->sub_type = ObjectSubType::EBICYCLIST;
//       break;
//     case PerceptionObstacle::ST_TRAFFICCONE:
//       // 处理 TRAFFICCONE 类型
//       obj->sub_type = ObjectSubType::TRAFFICCONE;
//       break;
//     case PerceptionObstacle::ST_SPEEDBUMP:
//       // 处理 SPEEDBUMP 类型
//       obj->sub_type = ObjectSubType::SPEEDBUMP;
//       break;
//     case PerceptionObstacle::ST_FENCE:
//       // 处理 FENCE 类型
//       obj->sub_type = ObjectSubType::FENCE;
//       break;
//     case PerceptionObstacle::ST_BARRIER_PARKING_LEVER:
//       // 处理 BARRIER_PARKING_LEVER 类型
//       obj->sub_type = ObjectSubType::BARRIER_PARKING_LEVER;
//       break;
//     case PerceptionObstacle::ST_WATERHORSE:
//       // 处理 WATERHORSE 类型
//       obj->sub_type = ObjectSubType::WATERHORSE;
//       break;
//     case PerceptionObstacle::ST_CRASHBARRELS:
//       // 处理 CRASHBARRELS 类型
//       obj->sub_type = ObjectSubType::CRASHBARRELS;
//       break;
//     case PerceptionObstacle::ST_SIGNBOARD:
//       // 处理 SIGNBOARD 类型
//       obj->sub_type = ObjectSubType::SIGNBOARD;
//       break;
//     case PerceptionObstacle::ST_WARNINGTRIANGLE:
//       // 处理 WARNINGTRIANGLE 类型
//       obj->sub_type = ObjectSubType::WARNINGTRIANGLE;
//       break;
//     case PerceptionObstacle::ST_STONEBLOCK:
//       // 处理 STONEBLOCK 类型
//       obj->sub_type = ObjectSubType::STONEBLOCK;
//       break;
//     // case PerceptionObstacle::ST_METALPARKINGSIGN:
//     //   // 处理 PARKINGSIGN 类型,泊车牌分两个子类，金属或者塑料材质
//     //   obj->sub_type = ObjectSubType::PARKINGSIGN;
//     //   break;
//     case PerceptionObstacle::ST_FIREHYDRANT:
//       // 处理 FIREHYDRANT 类型
//       obj->sub_type = ObjectSubType::FIREHYDRANT;
//       break;
//     case PerceptionObstacle::ST_WHEELSTOP:
//       // 处理 WHEELSTOP 类型
//       obj->sub_type = ObjectSubType::WHEELSTOP;
//       break;
//     case PerceptionObstacle::ST_LOCKER:
//       // 处理 LOCKER 类型
//       obj->sub_type = ObjectSubType::LOCKER;
//       break;
//     case PerceptionObstacle::ST_TRASH:
//       // 处理 TRASH 类型
//       obj->sub_type = ObjectSubType::TRASH;
//       break;
//     case PerceptionObstacle::ST_PILLAR:
//       // 处理 PILLAR 类型
//       obj->sub_type = ObjectSubType::PILLAR;
//       break;
//     case PerceptionObstacle::ST_BARRIER_GAT:
//       // 处理 BARRIER_GAT 类型
//       obj->sub_type = ObjectSubType::BARRIER_GAT;
//       break;
//     // case PerceptionObstacle::ST_UNKNOWN:
//     //   // 处理 BARRIER_SIGN 类型
//     //   obj->sub_type = ObjectSubType::BARRIER_SIGN;
//     //   break;
//     // case PerceptionObstacle::ST_WARNINGTRIANGLE:
//     //   // 处理 BARRIER_TRIANGLE 类型
//     //   obj->sub_type = ObjectSubType::BARRIER_TRIANGLE;
//     //   break;
//     case PerceptionObstacle::ST_STOP:
//       // 处理 STOP 类型
//       obj->sub_type = ObjectSubType::STOP;
//       break;
//     case PerceptionObstacle::ST_SLOWYIELD:
//       // 处理 SLOWYIELD 类型
//       obj->sub_type = ObjectSubType::SLOWYIELD;
//       break;
//     case PerceptionObstacle::ST_NOPASS:
//       // 处理 NOPASS 类型
//       obj->sub_type = ObjectSubType::NOPASS;
//       break;
//     case PerceptionObstacle::ST_NOENTRY:
//       // 处理 NOENTRY 类型
//       obj->sub_type = ObjectSubType::NOENTRY;
//       break;
//     case PerceptionObstacle::ST_NOTURNINGLEFT:
//       // 处理 NOTURNINGLEFT 类型
//       obj->sub_type = ObjectSubType::NOTURNINGLEFT;
//       break;
//     case PerceptionObstacle::ST_NOTURNINGRIGHT:
//       // 处理 NOTURNINGRIGHT 类型
//       obj->sub_type = ObjectSubType::NOTURNINGRIGHT;
//       break;
//     case PerceptionObstacle::ST_NOGOINGSTRAIGHT:
//       // 处理 NOGOINGSTRAIGHT 类型
//       obj->sub_type = ObjectSubType::NOGOINGSTRAIGHT;
//       break;
//     case PerceptionObstacle::ST_NOTURNINGAROUND:
//       // 处理 NOTURNINGAROUND 类型
//       obj->sub_type = ObjectSubType::NOTURNINGAROUND;
//       break;
//     case PerceptionObstacle::ST_NOOVERTAKING:
//       // 处理 NOOVERTAKING 类型
//       obj->sub_type = ObjectSubType::NOOVERTAKING;
//       break;
//     case PerceptionObstacle::ST_REMOVENOOVERTAKING:
//       // 处理 REMOVENOOVERTAKING 类型
//       obj->sub_type = ObjectSubType::REMOVENOOVERTAKING;
//       break;
//     case PerceptionObstacle::ST_NOPARKING:
//       // 处理 NOPARKING 类型
//       obj->sub_type = ObjectSubType::NOPARKING;
//       break;
//     case PerceptionObstacle::ST_NOHONKING:
//       // 处理 NOHONKING 类型
//       obj->sub_type = ObjectSubType::NOHONKING;
//       break;
//     case PerceptionObstacle::ST_SPEEDLIMITLIFTED:
//       // 处理 SPEEDLIMITLIFTED 类型
//       obj->sub_type = ObjectSubType::SPEEDLIMITLIFTED;
//       break;
//     case PerceptionObstacle::ST_SPEEDRELEASELIMITLIFTED:
//       // 处理 SPEEDRELEASELIMITLIFTED 类型
//       obj->sub_type = ObjectSubType::SPEEDRELEASELIMITLIFTED;
//       break;
//     case PerceptionObstacle::ST_TRAFFICLIGHT:
//       // 处理 TRAFFICLIGHT 类型
//       obj->sub_type = ObjectSubType::TRAFFICLIGHT;
//       break;
//     case PerceptionObstacle::ST_BAN:
//       // 处理 BAN 类型
//       obj->sub_type = ObjectSubType::BAN;
//       break;
//     case PerceptionObstacle::ST_D_ARROW:
//       // 处理 D_ARROW 类型
//       obj->sub_type = ObjectSubType::D_ARROW;
//       break;
//     case PerceptionObstacle::ST_L_ARROW:
//       // 处理 L_ARROW 类型
//       obj->sub_type = ObjectSubType::L_ARROW;
//       break;
//     case PerceptionObstacle::ST_R_ARROW:
//       // 处理 R_ARROW 类型
//       obj->sub_type = ObjectSubType::R_ARROW;
//       break;
//     case PerceptionObstacle::ST_A_ARROW:
//       // 处理 A_ARROW 类型
//       obj->sub_type = ObjectSubType::A_ARROW;
//       break;
//     case PerceptionObstacle::ST_ZEBRA:
//       // 处理 ZEBRA 类型
//       obj->sub_type = ObjectSubType::ZEBRA;
//       break;
//     case PerceptionObstacle::ST_STOP_LINE:
//       // 处理 STOP_LINE 类型
//       obj->sub_type = ObjectSubType::STOP_LINE;
//       break;
//     case PerceptionObstacle::ST_CAT:
//       // 处理 CAT 类型
//       obj->sub_type = ObjectSubType::CAT;
//       break;
//     case PerceptionObstacle::ST_DOG:
//       // 处理 DOG 类型
//       obj->sub_type = ObjectSubType::DOG;
//       break;
//     default:
//       // 处理未知类型
//       break;
//   }
// }

// }  // namespace data_mapping
// }  // namespace lm
// }  // namespace mp
// }  //  namespace hozon
