// /********************************************************
//  * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
//  * Licensed Hozon
//  * Author: Hozon
//  *******************************************************/

// #include "modules/local_mapping/data_mapping/data_mapping.h"
// #include "perception-base/base/scene/waitzone.h"
// #include "proto/perception/transport_element.pb.h"

// namespace hozon {
// namespace mp {

// namespace lm {
// namespace data_mapping {

// static bool CvtWaitZoneToPb(const WaitZonePtr& waitzone_msg,
//                             hozon::perception::TurnWaitingZone* pb_object);

// static hozon::perception::WaitZoneType CvtWaitZoneType2Pb(ZoneType shape) {
//   switch (shape) {
//     case ZoneType::LEFTWAIT_ZONE:
//       return hozon::perception::WaitZoneType::LEFTWAIT_ZONE;
//     case ZoneType::STRAIGHTWAIT_ZONE:
//       return hozon::perception::WaitZoneType::STRAIGHTWAIT_ZONE;
//     default:
//       return hozon::perception::WaitZoneType::WAITZONETYPE_UNKNOWN;
//   }
// }

// bool DataMapping::CvtMultiWaitZonesToPb(
//     const std::vector<WaitZonePtr>& waitzone_msgs,
//     NetaTransportElementPtr pb_objects) {
//   if (waitzone_msgs.size() == 0) {
//     HLOG_DEBUG << "waitzone msg size is 0";
//     return true;
//   }
//   if (nullptr == pb_objects) {
//     HLOG_ERROR << "pb_objects is nullptr.";
//     return false;
//   }

//   uint32_t waitzone_size = waitzone_msgs.size();
//   for (size_t i = 0; i < waitzone_size; ++i) {
//     auto pb_waitzone = pb_objects->add_turn_waiting_zone();
//     if (!CvtWaitZoneToPb(waitzone_msgs[i], pb_waitzone)) {
//       HLOG_ERROR << "cvt waitzone to proto struct failed.";
//       return false;
//     }
//   }
//   return true;
// }

// bool DataMapping::CvtWaitZoneToPb(
//     const WaitZonePtr& waitzone_msg,
//     hozon::perception::TurnWaitingZone* pb_waitzone) {
//   if (nullptr == waitzone_msg || nullptr == pb_waitzone) {
//     HLOG_ERROR << "waitzone msg  or pb_waitzone is nullptr.";
//     return false;
//   }
//   pb_waitzone->set_track_id(waitzone_msg->id);
//   pb_waitzone->set_confidence(waitzone_msg->confidence);
//   pb_waitzone->set_type(CvtWaitZoneType2Pb(waitzone_msg->type));
//   if (waitzone_msg->point_set_3d.size() != 0) {
//     for (auto& item_pt : waitzone_msg->point_set_3d) {
//       auto arrow_points = pb_waitzone->mutable_points();
//       auto pb_pt = arrow_points->add_point();
//       pb_pt->set_x(item_pt.x);
//       pb_pt->set_y(item_pt.y);
//       pb_pt->set_z(item_pt.z);
//     }
//   } else if (waitzone_msg->point_set_2d.size() != 0) {
//     for (auto& item_pt : waitzone_msg->point_set_2d) {
//       auto arrow_points = pb_waitzone->mutable_points();
//       auto pb_pt = arrow_points->add_point();
//       pb_pt->set_x(item_pt.x);
//       pb_pt->set_y(item_pt.y);
//       pb_pt->set_z(0.0);
//       // HLOG_DEBUG << "============x: " << item_pt.x << " y:" << item_pt.y;
//     }
//   }

//   return true;
// }

// bool DataMapping::CvtWaitZoneMeasurementToPb(
//     const std::vector<WaitZoneMeasurementPtr>& waitzone_measure,
//     hozon::perception::TransportElement* transport_element) {
//   for (auto& item : waitzone_measure) {
//     auto pb_waitzone = transport_element->add_turn_waiting_zone();
//     pb_waitzone->set_track_id(item->id);
//     pb_waitzone->set_type(CvtWaitZoneType2Pb(item->type));
//     pb_waitzone->set_confidence(item->confidence);
//     if (item->point_set_3d.size() != 0) {
//       for (auto& item_pt : item->point_set_3d) {
//         auto arrow_points = pb_waitzone->mutable_points();
//         auto pb_pt = arrow_points->add_point();
//         pb_pt->set_x(item_pt.x);
//         pb_pt->set_y(item_pt.y);
//         pb_pt->set_z(item_pt.z);
//       }
//     } else if (item->point_set_2d.size() != 0) {
//       for (auto& item_pt : item->point_set_2d) {
//         auto arrow_points = pb_waitzone->mutable_points();
//         auto pb_pt = arrow_points->add_point();
//         pb_pt->set_x(item_pt.x);
//         pb_pt->set_y(item_pt.y);
//         pb_pt->set_z(0.0);
//       }
//     }
//   }
//   return true;
// }

// }  // namespace data_mapping
// }  // namespace lm
// }  // namespace mp
// }  //  namespace hozon
