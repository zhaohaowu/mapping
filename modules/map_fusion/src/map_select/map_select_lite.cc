/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_select_lite.cc
 *   author     ：
 *   date       ： 2024.01
 ******************************************************************************/

#include "map_fusion/map_select/map_select_lite.h"
#include <math.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "map_fusion/fusion_common/calc_util.h"
#include "modules/map_fusion/include/map_fusion/map_service/global_hd_map.h"
#include "modules/util/include/util/orin_trigger_manager.h"
#include "proto/perception/transport_element.pb.h"
#include "third_party/x86_2004/opencv/include/opencv4/opencv2/core/types.hpp"

namespace hozon {
namespace mp {
namespace mf {
namespace select {
constexpr uint64_t kLocationIsFault = 12;
constexpr uint64_t kLocationFaultStart = 101;
constexpr uint64_t kLocationFatalFault = 111;
constexpr uint64_t kLocationSeriouslFault = 121;
constexpr uint64_t kLocationNormalFault = 151;
constexpr int kErrStep = 21;
constexpr int kShiftStep = 11;
constexpr int kInitToShiftCount = 10;
constexpr int kInitToErrCount = 20;
constexpr int kToInitCount = 50;
constexpr int kErrToInitCount = 350;
constexpr int kShiftToErrCount = 3;
constexpr double kLocationXerrMin = 2.0;
constexpr double kLocationYerrMin = 0.6;
constexpr double kLanemarkerMinJumpErr = 1.5;
constexpr double kRiseTime = 0.5;
constexpr double kDownTime = 2.0;
constexpr double kMainLoopTime = 0.1;
constexpr double kMinDisToDownRamp = 1000;
constexpr double kMinSplitMergeDis = 200;
constexpr double kInitWithDiff = 0.25;
// lane map matching check
constexpr double kMaxLanemarkerTm = 2.1;
constexpr double kValLanemarkerTm = 1.1;
constexpr double kBetterLanemarkerTm = 0.6;
constexpr double kDeltaTm = 0.1;
constexpr double kCheckErr = 0.1;
constexpr double kValErr = 0.32;
constexpr double kMaxErr = 0.4;
constexpr double kMinDeltaTm = 0.3;

const double kSegmentationEpsilon = 0.2;
const std::size_t kLanePointMinSize = 2U;
constexpr double epslion = 1e-8;
std::string boolToString(bool value) { return value ? "true" : "false"; }

using hozon::navigation_hdmap::MapMsg;

bool ValidQuaternion(const hozon::common::Quaternion& quat);
bool ValidLla(const hozon::common::PointENU& lla);
bool SetProtoToBinaryFile(const google::protobuf::Message& message,
                          const std::string& file_name) {
  std::fstream output(file_name,
                      std::ios::out | std::ios::trunc | std::ios::binary);
  return message.SerializeToOstream(&output);
}

bool MapSelectLite::Init() {
  left_lanemarker_quality_debounce_.ResetTime(kRiseTime, kDownTime,
                                              kMainLoopTime);
  right_lanemarker_quality_debounce_.ResetTime(kRiseTime, kDownTime,
                                               kMainLoopTime);
  left_lanemarker_viewrange_debounce_.ResetTime(kRiseTime, kDownTime,
                                                kMainLoopTime);
  right_lanemarker_viewrange_debounce_.ResetTime(kRiseTime, kDownTime,
                                                 kMainLoopTime);
  is_in_main_road_debounce_.ResetTime(5.0, 0.0, kMainLoopTime);
  is_good_lanemarker_debounce_.ResetTime(1.0, 3.0, kMainLoopTime);
  is_good_init_lanemarker_debounce_.ResetTime(2.0, 0.0, kMainLoopTime);
  // is_in_overlaplane_debounce_.ResetTime(0.3, 0.0, 0.1);
  return true;
}
void MapSelectLite::Reset() {
  left_map_c0_ = kDfaultLaneWidth;
  right_map_c0_ = kDfaultLaneWidth;
  is_left_lane_change_ = false;
  is_right_lane_change_ = false;
  is_lane_change_ = false;
  map_lanechange_observer_.Reset();
  per_lanechange_observer_.Reset();
  double_left_lane_change_rise_decider_.Reset();
  double_right_lane_change_rise_decider_.Reset();
  left_match_state_ = 0;
  right_match_state_ = 0;
  left_match_state_count_ = 0;
  right_match_state_count_ = 0;
  left_check_dis_count_ = 0;
  right_check_dis_count_ = 0;
}

//! 处理逻辑参考：https://hozonauto.feishu.cn/docx/Vw9JdxiluoboMExiUzIcrnUWnmh
MapSelectResult MapSelectLite::Process(
    const std::shared_ptr<hozon::localization::Localization>& localization,
    const std::shared_ptr<hozon::hdmap::Map>& fusion_map,
    const std::shared_ptr<hozon::hdmap::Map>& perc_map,
    const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in) {
  bool fct_valid = CheckFctIn(fct_in);
  if (!fct_valid) {
    bool percep_available = PercepMapAvailable(perc_map, localization);
    if (percep_available) {
      HLOG_INFO << "fct invalid, percep available";
      return {MapMsg::PERCEP_MAP, true, 2};
    }
    HLOG_INFO << "fct invalid, percep unavailable";
    return {MapMsg::INVALID, false, 2};
  }

  if (!NnpSwitchOn(fct_in)) {
    bool percep_available = PercepMapAvailable(perc_map, localization);
    if (percep_available) {
      HLOG_INFO << "fct valid, nnp off, percep available";
      return {MapMsg::PERCEP_MAP, true, 0};
    }
    HLOG_INFO << "fct valid, nnp off, percep unavailable";
    return {MapMsg::INVALID, false, 0};
  }

  bool fusion_available =
      FusionMapAvailable(fusion_map, localization, localization);
  if (fusion_available) {
    HLOG_INFO << "fct valid, nnp on, fusion available";
    //! TBD：当前默认都是FUSION_NNP，不考虑FUSION_NCP
    return {MapMsg::FUSION_NNP_MAP, true, 0};
  }

  bool percep_available = PercepMapAvailable(perc_map, localization);
  if (percep_available) {
    HLOG_INFO << "fct valid, nnp on, fusion unavailable, percep available";
    return {MapMsg::PERCEP_MAP, true, 1};
  }
  HLOG_INFO << "fct valid, nnp on, fusion unavailable, percep unavailable";
  return {MapMsg::INVALID, false, 2};
}

MapSelectResult MapSelectLite::Process(
    const std::shared_ptr<hozon::localization::Localization>& localization,
    const std::shared_ptr<hozon::hdmap::Map>& fusion_map,
    const std::shared_ptr<hozon::hdmap::Map>& perc_map,
    const std::shared_ptr<const ::hozon::perception::TransportElement>&
        perception_msg,
    const std::shared_ptr<const ::hozon::dead_reckoning::DeadReckoning>& dr_msg,
    const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in) {
  // 第一步，直接根据HMI NNP主开关来选择相应的地图，为true 发送Fusion
  // map,为False 发送Perception Map
  // if (fusion_map == nullptr && perc_map == nullptr) {
  //   HLOG_ERROR << "input fusion_map and perc_map is null";

  //   return hozon::navigation_hdmap::MapMsg_MapType_INVALID;
  // }
  // else if (fusion_map == nullptr)
  // {
  //   HLOG_ERROR << "input fusion_map is null";
  //   return hozon::navigation_hdmap::MapMsg_MapType_PERCEP_MAP;
  // }else if (perc_map == nullptr)
  // {
  //   HLOG_ERROR << "input perc_map is null";
  //   return hozon::navigation_hdmap::MapMsg_MapType_FUSION_MAP;
  // }
  // if (!fct_in->has_fct_2_soc_tbd_u32_03()) {
  //   HLOG_ERROR << "input fct_in has no fct_2_soc_tbd_u32_03";
  //   return hozon::navigation_hdmap::MapMsg_MapType_PERCEP_MAP;
  // }
  // HLOG_ERROR << ">>>> fct_in->fct_2_soc_tbd_u32_03():"
  //            << fct_in->fct_2_soc_tbd_u32_03();
  // if (GetU32BitByIndex(fct_in->fct_2_soc_tbd_u32_03(), 28)) {
  //   HLOG_ERROR << ">>>>result: fusion_map";
  //   return  hozon::navigation_hdmap::MapMsg_MapType_FUSION_MAP;
  // }
  // return hozon::navigation_hdmap::MapMsg_MapType_PERCEP_MAP;

  // 第二步
  debug_.clear_origin_response();
  bool valid_fusion_map = CheckFusionMapMsg(fusion_map);
  bool valid_percep_map = CheckPercepMapMsg(perc_map);
  bool valid_localization_msg = CheckLocalizationMsg(localization);
  if ((!valid_fusion_map) && (!valid_percep_map)) {
    HLOG_ERROR << "input fusion_map and perc_map is null or empty";
    debug_.add_origin_response("fusion_map: null/empty  perc_map: null/empty");
    return {hozon::navigation_hdmap::MapMsg_MapType_INVALID, false, 2};
  }
  if (!valid_fusion_map) {
    HLOG_ERROR << "fusion_map: invalid, percep_map: valid";
    return {hozon::navigation_hdmap::MapMsg_MapType_PERCEP_MAP, true, 2};
  }
  if (!valid_percep_map) {
    HLOG_ERROR << "fusion_map: valid, percep_map: invalid";
    debug_.add_origin_response("input perc_map is null/empty");
    return {GetMapTypeByRoadType(), true, 0};
  }

  //! fusion_map无效 && percep_map无效： 返回 INVALID + invalid
  //! fusion_map有效 && percep_map无效：
  //!   - 自车在fusion_map车道内：返回 FUSION_XXX_MAP + valid
  //!   - 自车不在fusion_map车道内：返回 FUSION_XXX_MAP + invalid
  //! fusion_map无效 && percep_map有效：
  //!   - 自车在percep_map车道内：返回 PERCEP_MAP + valid
  //!   - 自车不在percep_map车道内：返回 PERCEP_MAP + invalid
  //! fusion_map有效 && percep_map有效：
  //!  根据之前定义的判断条件进行仲裁

  bool is_map_service = true;
  bool is_perception_msg = true;

  if (!GLOBAL_HD_MAP) {
    HLOG_ERROR << "GLOBAL_HD_MAP is nullptr";
    debug_.add_origin_response("GLOBAL_HD_MAP: nullptr");
    is_map_service = false;
  }

  if (!CheckPerceMsg(perception_msg)) {
    HLOG_ERROR << "input perception_msg is wrong";
    debug_.add_origin_response("input perception_msg is wrong");
    is_perception_msg = false;
  }
  if (!is_map_service && !is_perception_msg) {
    return {hozon::navigation_hdmap::MapMsg_MapType_INVALID, false, 2};
  } else if (!is_map_service) {
    return {hozon::navigation_hdmap::MapMsg_MapType_PERCEP_MAP, true, 2};
  } else if (!is_perception_msg) {
    return {GetMapTypeByRoadType(), true, 0};
  } else {
    if (!CheckFCTIN(fct_in) || !CheckDrMsg(dr_msg) || !valid_localization_msg) {
      return {hozon::navigation_hdmap::MapMsg_MapType_PERCEP_MAP, true, 2};
    }
  }

  // HLOG_ERROR << ">>>> fct_in->fct_2_soc_tbd_u32_03():"
  //            << fct_in->fct_2_soc_tbd_u32_03();
  // NNP主开关未打开：只下发percepmap地图
  if (!GetU32BitByIndex(fct_in->fct_2_soc_tbd_u32_03(), 28)) {
    HLOG_ERROR << "NNP switch is close";
    debug_.add_origin_response("NNP switch is close");
    return {hozon::navigation_hdmap::MapMsg_MapType_PERCEP_MAP, true, 0};
  }
  // NNP主开关打开：
  const auto nnp_sys_state = fct_in->fct_nnp_in().nnp_sysstate();
  const auto pilot_sys_state = fct_in->fct_nnp_in().npilot_state();

  const bool is_nnp_active =
      (nnp_sys_state == hozon::functionmanager::NNPSysState::NNPS_ACTIVE ||
       nnp_sys_state == hozon::functionmanager::NNPSysState::NNPS_OVERRIDE ||
       nnp_sys_state ==
           hozon::functionmanager::NNPSysState::NNPS_LAT_OVERRIDE ||
       nnp_sys_state == hozon::functionmanager::NNPSysState::NNPS_LON_OVERRIDE);

  const bool is_pilot_active =
      pilot_sys_state == hozon::functionmanager::FctToNnpInput::PILOT_SUSPEND ||
      pilot_sys_state == hozon::functionmanager::FctToNnpInput::PILOT_ACTIVE;

  auto arbitrate_result = ArbitrateMap(localization, fusion_map, perc_map,
                                       perception_msg, dr_msg, fct_in);
  HLOG_ERROR << ">>>>is_nnp_active: " << is_nnp_active
             << "is_pilot_active: " << is_pilot_active;
  hozon::navigation_hdmap::MapMsg_MapType map_type;

  if (is_nnp_active) {
    // NNP状态下
    debug_.add_origin_response("NNP is active");
    switch (arbitrate_result) {
      case -1:
        map_type = hozon::navigation_hdmap::MapMsg_MapType_PERCEP_MAP;
        break;
      case 0:
        map_type = GetMapTypeByRoadType();
        break;
      case 1:
        map_type = hozon::navigation_hdmap::MapMsg_MapType_PERCEP_MAP;
        break;
      case 2:
        map_type = hozon::navigation_hdmap::MapMsg_MapType_PERCEP_MAP;
        break;
      default:
        map_type = GetMapTypeByRoadType();
        break;
    }
  } else if (!is_nnp_active && is_pilot_active) {
    // PILOT状态下
    debug_.add_origin_response("pilot is active");
    map_type = GetMapTypeByRoadType();
  } else {
    debug_.add_origin_response("pilot and nnp is off");
    map_type = GetMapTypeByRoadType();
  }
  return {map_type, true, 0};
}
int32_t MapSelectLite::ArbitrateMap(
    const std::shared_ptr<hozon::localization::Localization>& localization,
    const std::shared_ptr<hozon::hdmap::Map>& fusion_map,
    const std::shared_ptr<hozon::hdmap::Map>& perc_map,
    const std::shared_ptr<const ::hozon::perception::TransportElement>&
        perception_msg,
    const std::shared_ptr<const ::hozon::dead_reckoning::DeadReckoning>& dr_msg,
    const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in) {
  bool not_in_main_road = true;
  // 定位故障处理
  int ori_location_err_state = LocationErrDecider(localization);

  // 不在主路判断
  hozon::common::PointENU utm_pos;
  utm_pos.set_x(localization->pose().pos_utm_01().x());
  utm_pos.set_y(localization->pose().pos_utm_01().y());
  utm_pos.set_z(0);
  bool is_main_road = true;
  bool is_ramp_road = false;
  bool is_continue_road = true;
  int predecessor_lane_id_size = 0;
  std::vector<hozon::hdmap::LaneInfoConstPtr> ego_lanes_ptr;
  hozon::hdmap::LaneInfoConstPtr ego_lane_ptr = nullptr;
  if (!GetEgoLanes(utm_pos, ego_lanes_ptr, ego_lane_ptr) ||
      ego_lane_ptr == nullptr) {
    HLOG_ERROR << "cannot get ego lane ptr";
    debug_.add_origin_response("cannot get ego lane ptr");
    return -1;
  }
  current_road_type_ = ego_lane_ptr->GetRoadType();
  for (const auto& lane_ptr : ego_lanes_ptr) {
    if (!lane_ptr->IsMainRoad()) {
      is_main_road = false;
    }
    if (lane_ptr->IsRampRoad()) {
      is_ramp_road = true;
    }
    if (lane_ptr->lane().has_lane_transition()) {
      if (lane_ptr->lane().lane_transition() !=
          hozon::hdmap::Lane_LaneTransition_CONTINUE) {
        is_continue_road = false;
      }
    } else {
      is_continue_road = false;
    }
    predecessor_lane_id_size =
        lane_ptr->lane().successor_id_size() >
                lane_ptr->lane().predecessor_id_size()
            ? std::max(predecessor_lane_id_size,
                       lane_ptr->lane().successor_id_size())
            : std::max(predecessor_lane_id_size,
                       lane_ptr->lane().predecessor_id_size());
  }
  not_in_main_road = !is_in_main_road_debounce_.DealDebounce(
      is_main_road && !is_ramp_road &&  // NOLINT
      is_continue_road && predecessor_lane_id_size < 2);
  // 感知车道线质量判断 is_bad_lanemarkers
  bool is_bad_lanemarkers = true;

  if (perception_msg->has_header() && perception_msg->lane_size() > 0) {
    is_bad_lanemarkers = DeciderPercpMsg(perception_msg, dr_msg);
  }
  bool match_prerequisite =
      !is_bad_lanemarkers && !not_in_main_road && ori_location_err_state == 0;
  bool map_checker_bl = FusionMapLaneChecker(utm_pos);
  // Fusion Map和感知车道线对比的折线检查
  // bool is_match = MatchPerceLaneAndFusionLane(
  //                     localization->pose(), ego_lane_ptr, match_prerequisite)
  //                     &&
  //                 map_checker_bl;
  bool is_match = map_checker_bl;
  HLOG_ERROR << " *****map_checker_bl******* = " << map_checker_bl
             << " , is_match: " << is_match;
  hozon::common::math::Vec2d point_utm;
  point_utm.set_x(localization->pose().pos_utm_01().x());
  point_utm.set_y(localization->pose().pos_utm_01().y());
  double s = 0.0;
  double lateral = 0.0;
  double left_width = 0.0;
  double right_width = 0.0;
  // 定位点在车道中心线左边l为正，右边l为负
  bool has_no_width_diff = false;
  double left_heading_diff{0.0};
  double right_heading_diff{0.0};
  std::pair<double, double> width_diff = std::make_pair(0.0, 0.0);

  if (ego_lane_ptr->GetProjection(point_utm, &s, &lateral)) {
    ego_lane_ptr->GetWidth(s, &left_width, &right_width);
    // const auto map_heading = ego_lane_ptr->Heading(s);
    // const auto map_veh_heading = map_heading -
    // localization->pose().heading();
    // HLOG_ERROR << "left_width =" << left_width << "right_width=" <<
    // right_width
    //            << "lateral=" << lateral
    //            << "x =" << localization->pose().pos_utm_01().x()
    //            << "y=" << localization->pose().pos_utm_01().y();
    left_map_c0_ = left_width - lateral;
    right_map_c0_ = -(right_width + lateral);
    if (!CalculateValidLocErrHasMap(&width_diff)) {
      HLOG_ERROR << "cannot get width_diff";
      return -1;
    }
    const auto is_lane_change_per =
        per_lanechange_observer_.Observer(ego_left_lane_, ego_right_lane_);
    hozon::perception::LaneInfo left_map_lane;
    hozon::perception::LaneInfo right_map_lane;
    auto left_lane_set =
        left_map_lane.mutable_lane_param()->add_cubic_curve_set();
    auto right_lane_set =
        right_map_lane.mutable_lane_param()->add_cubic_curve_set();
    left_lane_set->set_c0(static_cast<float>(left_map_c0_));
    left_map_lane.set_confidence(1.0);

    right_lane_set->set_c0(static_cast<float>(right_map_c0_));
    right_map_lane.set_confidence(1.0);
    const auto is_lane_change_map =
        map_lanechange_observer_.Observer(left_map_lane, right_map_lane);
    LanechangeRiseDecider(is_lane_change_per, is_lane_change_map);
    HLOG_INFO << "left_map_c0: " << left_map_c0_
              << " , right_map_c0: " << right_map_c0_
              << " , left_per lane_c0 : "
              << ego_left_lane_.lane_param().cubic_curve_set(0).c0()
              << " , right lane_c0: "
              << ego_right_lane_.lane_param().cubic_curve_set(0).c0();
    HLOG_INFO << "width_diff left: " << width_diff.first
              << " , width_diff right: " << width_diff.second;
    HLOG_INFO << " , is_lane_change_per left: " << is_lane_change_per.first
              << " , is_lane_change_per right: " << is_lane_change_per.second
              << " , is_lane_change_map left: " << is_lane_change_map.first
              << " , is_lane_change_map right: " << is_lane_change_map.second
              << " , is_lane_change: " << is_lane_change_;

  } else {
    has_no_width_diff = true;
  }
  HLOG_INFO << "ori_location_err_state = " << ori_location_err_state
            << ", is_bad_lanemarkers = " << is_bad_lanemarkers
            << ", has_no_width_diff = " << has_no_width_diff;
  if (is_lane_change_) {
    width_diff.first = width_diff_history_.first;
    width_diff.second = width_diff_history_.second;
  }
  HLOG_INFO << "left_width = " << left_width
            << ", right_width = " << right_width << ", s = " << s
            << ", l = " << lateral;
  HLOG_INFO << "left_c0 = "
            << ego_left_lane_.lane_param().cubic_curve_set(0).c0()
            << ", left_map_c0 = " << left_map_c0_ << ", right_c0 = "
            << ego_right_lane_.lane_param().cubic_curve_set(0).c0()
            << ", right_map_c0 = " << right_map_c0_;
  HLOG_INFO << "left_lane_err = " << width_diff.first
            << ", right_lane_err = " << width_diff.second;
  // 仅仅使用定位自己的故障进行降级
  if (only_using_locationself_err) {
    width_diff.first = 0.0;
    width_diff.second = 0.0;
    is_bad_lanemarkers = false;
    not_in_main_road = false;
    has_no_width_diff = false;
    left_map_c0_ = kDfaultLaneWidth;
    right_map_c0_ = kDfaultLaneWidth;
  }

  // 地图一直未消失的定位失效和恢复判断
  DealPerceptionLocErr(width_diff, is_bad_lanemarkers, has_no_width_diff,
                       not_in_main_road, is_match);
  HLOG_ERROR << "loc_err_state_  :" << loc_err_state_
             << "ori_location_err_state:" << ori_location_err_state
             << "  per_loc_err_state_ :" << per_loc_err_state_
             << " not_in_main_road:" << not_in_main_road;
  debug_.add_origin_response(
      "loc_err_state_:" + std::to_string(loc_err_state_) +
      "ori_location_err_state:" + std::to_string(ori_location_err_state) +
      "per_loc_err_state_:" + std::to_string(per_loc_err_state_) +
      " not_in_main_road:" + boolToString(not_in_main_road) +
      "is_bad_lanemarkers:" + boolToString(is_bad_lanemarkers));

  switch (loc_err_state_) {
    case 0:
      if (ori_location_err_state == 2 ||
          (ori_location_err_state == 1 &&
           (not_in_main_road || is_bad_lanemarkers)) ||
          per_loc_err_state_ == LocErrState::Err) {
        loc_err_state_ = 2;
      } else if (ori_location_err_state == 1 ||
                 per_loc_err_state_ == LocErrState::Shift) {
        loc_err_state_ = 1;
      }
      break;
    case 1:
      if (ori_location_err_state == 2 ||
          (ori_location_err_state == 1 &&
           (not_in_main_road || is_bad_lanemarkers)) ||
          per_loc_err_state_ == LocErrState::Err) {
        loc_err_state_ = 2;
      } else if (ori_location_err_state == 0 &&
                 per_loc_err_state_ == LocErrState::Loc_Init) {
        loc_err_state_ = 0;
      }
      break;
    case 2:
      if (ori_location_err_state == 0 &&
          per_loc_err_state_ == LocErrState::Loc_Init) {
        loc_err_state_ = 0;
      }
      break;
    default:
      break;
  }
  HLOG_ERROR << "---------loc_err_state----- " << loc_err_state_;

  // #ifdef ISORIN
  //   HLOG_ERROR << "Start to trigger dc";
  //   if (loc_err_state_ > 0) {
  //     GLOBAL_DC_TRIGGER.TriggerCollect(10001);
  //   }
  // #endif
  return loc_err_state_;
}
void MapSelectLite::LanechangeRiseDecider(
    const std::pair<bool, bool> perception, const std::pair<bool, bool> map) {
  is_left_lane_change_ = double_left_lane_change_rise_decider_.Decider(
      perception.first, map.first);
  is_right_lane_change_ = double_right_lane_change_rise_decider_.Decider(
      perception.second, map.second);
  is_lane_change_ = is_left_lane_change_ || is_right_lane_change_;
}

void MapSelectLite::DealPerceptionLocErr(
    const std::pair<double, double>& width_diff, bool is_bad_lanemarkers,
    bool has_no_width_diff, bool not_in_main_road, bool is_match) {
  // 地图一直未消失的定位失效和恢复判断
  switch (per_loc_err_state_) {
    case LocErrState::Loc_Init:
      DealLocInitState(width_diff, is_bad_lanemarkers, has_no_width_diff,
                       not_in_main_road, is_match);
      break;
    case LocErrState::Shift:
      DealLocShiftState(width_diff, is_bad_lanemarkers, has_no_width_diff,
                        not_in_main_road, is_match);
      break;
    case LocErrState::Err:
      if (is_bad_lanemarkers || has_no_width_diff || not_in_main_road) {
        err_state_count_++;
        width_same_count_ = 0;
      } else if (std::fabs(width_diff.first) < kInitWithDiff &&
                 std::fabs(width_diff.second) < kInitWithDiff && is_match) {
        width_same_count_++;
        err_state_count_ = 0;
      } else {
        width_same_count_ = 0;
        err_state_count_ = 0;
      }
      if (err_state_count_ > kErrToInitCount ||
          width_same_count_ > kToInitCount) {
        per_loc_err_state_ = LocErrState::Loc_Init;
        left_map_c0_ = kDfaultLaneWidth;
        right_map_c0_ = kDfaultLaneWidth;
        width_diff_count_ = 0;
        width_same_count_ = 0;
        err_state_count_ = 0;
      }
      break;
    default:
      break;
  }
}

void MapSelectLite::DealLocShiftState(
    const std::pair<double, double> width_diff, const bool is_bad_lanemarkers,
    const bool has_no_width_diff, const bool not_in_main_road,
    const bool is_match) {
  UNUSED(not_in_main_road);
  UNUSED(has_no_width_diff);
  // if ((is_bad_lanemarkers || err_not_main_road_) &&
  //     !is_change_mode_by_odd_type_)
  if (is_bad_lanemarkers || not_in_main_road) {
    width_diff_count_ += kErrStep;
    left_map_c0_ = kDfaultLaneWidth;
    right_map_c0_ = kDfaultLaneWidth;
  } else if (std::fabs(width_diff.first) < kInitWithDiff &&
             std::fabs(width_diff.second) < kInitWithDiff && is_match) {
    width_same_count_++;
    width_diff_count_ = 0;
  } else {
    width_diff_count_ = 0;
    width_same_count_ = 0;
  }
  if (width_diff_count_ > kShiftToErrCount) {
    per_loc_err_state_ = LocErrState::Err;
    width_diff_count_ = 0;
  }
  if (width_same_count_ > kToInitCount) {
    per_loc_err_state_ = LocErrState::Loc_Init;
    width_same_count_ = 0;
  }
}

void MapSelectLite::DealLocInitState(const std::pair<double, double> width_diff,
                                     const bool is_bad_lanemarkers,
                                     const bool has_no_width_diff,
                                     const bool not_in_main_road,
                                     const bool is_match) {
  const double kShiftWithDiff = 0.32;
  const double left_c0 = ego_left_lane_.lane_param().cubic_curve_set(0).c0();
  const double right_c0 = ego_right_lane_.lane_param().cubic_curve_set(0).c0();
  const double left_delta = left_c0 < 0.6 ? 0.2 : 0.0;
  const double right_delta = right_c0 > -0.6 ? 0.2 : 0.0;
  // const bool is_step_delta_err =
  //     std::fabs(width_diff.first - width_diff_history_.first) > 0.25 ||
  //     std::fabs(width_diff.second - width_diff_history_.second) > 0.25;
  const bool is_width_diff =
      (std::fabs(width_diff.first) >
           kShiftWithDiff + left_delta + left_lanemarker_c1_delta_ &&
       std::fabs(width_diff.second) > kInitWithDiff &&
       width_diff.first * width_diff.second > 0.0) ||
      (std::fabs(width_diff.first) > kInitWithDiff &&
       std::fabs(width_diff.second) >
           kShiftWithDiff + right_delta + right_lanemarker_c1_delta_ &&
       width_diff.first * width_diff.second > 0.0);
  // if (is_bad_lanemarkers || not_in_main_road || is_change_mode_by_odd_type_)
  // {
  if (is_bad_lanemarkers || not_in_main_road) {
    width_diff_count_ = 0;
    left_map_c0_ = kDfaultLaneWidth;
    right_map_c0_ = kDfaultLaneWidth;
  } else if (!is_match) {
    width_diff_count_ += kShiftStep;
  } else if (left_c0 < 0.1 || right_c0 > -0.1) {
    width_diff_count_ = 0;
  } else if ((std::fabs(width_diff.first) > kLanemarkerMinJumpErr &&
              std::fabs(width_diff.second) > kLanemarkerMinJumpErr) ||
             has_no_width_diff) {
    width_diff_count_ += kShiftStep;
    HLOG_ERROR << "1 width_diff_count: " << width_diff_count_;
  } else if (is_width_diff) {
    width_diff_count_++;
    HLOG_ERROR << "2 width_diff_count: " << width_diff_count_;
  } else {
    width_diff_count_ = 0;
  }

  if (width_diff_count_ > kInitToErrCount) {
    per_loc_err_state_ = LocErrState::Err;
    width_diff_count_ = 0;
  } else if (width_diff_count_ > kInitToShiftCount) {
    per_loc_err_state_ = LocErrState::Shift;
    width_diff_count_ = 0;
  }
}

bool MapSelectLite::CheckLocalizationMsg(
    const std::shared_ptr<hozon::localization::Localization>& localization) {
  if (localization == nullptr) {
    return false;
  }
  // if (last_loc_time_ != localization->header().data_stamp()) {
  //   last_loc_time_ = localization->header().data_stamp();
  // } else {
  //   HLOG_ERROR << "localization data time not update :"
  //              << localization->header().data_stamp();
  //   return false;
  // }
  if (!localization->pose_local().has_position()) {
    HLOG_ERROR << "localization has no pose_local";
    return false;
  }
  if (!localization->pose_local().position().has_x() ||
      !localization->pose_local().position().has_y() ||
      !localization->pose_local().has_local_heading()) {
    HLOG_ERROR << "localization msg pose_local empty";
    return false;
  }

  if (!localization->pose().has_pos_utm_01()) {
    HLOG_ERROR << "localization msg has no pos utm";
    return false;
  }
  if (!localization->pose().pos_utm_01().has_x() ||
      !localization->pose().pos_utm_01().has_y() ||
      !localization->pose().has_heading()) {
    HLOG_ERROR << "localization msg pos utm empty";
    return false;
  }
  if ((localization->pose().pos_utm_01().y() < 10) ||
      (localization->pose().pos_utm_01().x() < 10)) {
    HLOG_ERROR << "invalid pos utm " << localization->pose().pos_utm_01().x()
               << ", " << localization->pose().pos_utm_01().y();
    return false;
  }

  if (!localization->has_location_state()) {
    HLOG_ERROR << "localization msg has no location_state";
    return false;
  }
  if (0 == localization->location_state()) {
    HLOG_ERROR << "localization msg location_state 0";
    return false;
  }
  return true;
}

bool MapSelectLite::GetEgoLanes(
    hozon::common::PointENU local_pos,
    std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes_ptr,
    hozon::hdmap::LaneInfoConstPtr& lane_ptr) {
  if (GLOBAL_HD_MAP == nullptr || GLOBAL_HD_MAP->Empty()) {
    HLOG_ERROR << "GLOBAL_HD_MAP is null or empty";
    return false;
  }
  double range = 300.0;
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  std::vector<hozon::hdmap::LaneInfoConstPtr> all_lanes_ptr;
  int ret = GLOBAL_HD_MAP->GetLanes(local_pos, range, &all_lanes_ptr);
  if (ret != 0) {
    HLOG_ERROR << "get local map lanes failed";
    return false;
  }
  hozon::hdmap::LaneInfoConstPtr ego_lane_ptr = nullptr;
  int flag = GLOBAL_HD_MAP->GetNearestLane(local_pos, &ego_lane_ptr, &nearest_s,
                                           &nearest_l);
  if (flag != 0 || ego_lane_ptr == nullptr) {
    HLOG_ERROR << "get nearest lane failed";
    return false;
  }
  lane_ptr = ego_lane_ptr;
  if (ego_lane_ptr->lane().successor_id_size() != 1) {
    return false;
  }
  lanes_ptr.emplace_back(ego_lane_ptr);

  bool find_next_lane = true;
  while (find_next_lane) {
    if (lanes_ptr.empty() || lanes_ptr.back()->lane().successor_id().empty()) {
      break;
    }
    auto next_lane_id = lanes_ptr.back()->lane().successor_id().at(0).id();
    hozon::hdmap::LaneInfoConstPtr next_lane_ptr = nullptr;
    if (IsLaneInLanes(next_lane_id, all_lanes_ptr, next_lane_ptr)) {
      lanes_ptr.emplace_back(next_lane_ptr);
    } else {
      find_next_lane = false;
    }
  }
  return !(lanes_ptr.empty() || lane_ptr == nullptr);
}

bool MapSelectLite::IsLaneInLanes(
    const std::string& lane_id,
    const std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes,
    hozon::hdmap::LaneInfoConstPtr& lane_ptr) {
  for (const auto& lane : lanes) {
    if (lane->id().id() == lane_id) {
      lane_ptr = lane;
      return true;
    }
  }
  return false;
}
bool MapSelectLite::GetSuccessorLanes(
    const std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes,
    const std::string& lane_id, hozon::hdmap::LaneInfoConstPtr lane_ptr) {
  if (lanes.empty()) {
    return false;
  }
  for (const auto& lane : lanes) {
    if (lane->lane().id().id() == lane_id) {
      lane_ptr = lane;
      return true;
    }
  }
  return false;
}
bool SeemsEqual(const double x, const double y) {
  return (std::isinf(x) != std::isinf(y))
             ? false
             : (std::isinf(x) || fabs(x - y) <= epslion);
}
void MapSelectLite::AddMapPoints(
    std::vector<hozon::common::math::Vec2d>* const left_map_points,
    std::vector<hozon::common::math::Vec2d>* const right_map_points,
    const hozon::hdmap::LaneInfoConstPtr& lane,
    const hozon::common::Pose& pose) {
  RETURN_IF_NULL(lane);
  hozon::common::PointENU prev_p;
  prev_p.set_x(std::numeric_limits<double>::infinity());
  prev_p.set_y(std::numeric_limits<double>::infinity());
  int i = 0;
  for (const auto& seg : lane->lane().left_boundary().curve().segment()) {
    if (!seg.has_line_segment()) {
      continue;
    }

    for (const auto& p : seg.line_segment().point()) {
      if (!std::isinf(prev_p.x()) && !std::isinf(prev_p.y()) &&
          SeemsEqual(prev_p.x(), p.x()) && SeemsEqual(prev_p.y(), p.y())) {
        continue;
      }
      left_map_points->emplace_back(PointEarth2Bus({p.x(), p.y()}, pose));
      prev_p.set_x(p.x());
      prev_p.set_y(p.y());
    }
  }
  prev_p.set_x(std::numeric_limits<double>::infinity());
  prev_p.set_y(std::numeric_limits<double>::infinity());
  for (const auto& seg : lane->lane().right_boundary().curve().segment()) {
    if (!seg.has_line_segment()) {
      continue;
    }
    for (const auto& p : seg.line_segment().point()) {
      if (!std::isinf(prev_p.x()) && !std::isinf(prev_p.y()) &&
          SeemsEqual(prev_p.x(), p.x()) && SeemsEqual(prev_p.y(), p.y())) {
        continue;
      }
      right_map_points->emplace_back(PointEarth2Bus({p.x(), p.y()}, pose));
      prev_p.set_x(p.x());
      prev_p.set_y(p.y());
    }
  }
  RemoveDuplicates(left_map_points);
  RemoveDuplicates(right_map_points);
}

void MapSelectLite::RemoveDuplicates(
    std::vector<hozon::common::math::Vec2d>* points) {
  RETURN_IF_NULL(points);

  size_t points_size = points->size();

  if (points_size <= kLanePointMinSize) {
    return;
  }

  int count = 0;
  // 3个条件：
  // 1.第1个点和最后一个点必要
  // 2.中间点距离>limit
  // 3.最后两个点如果距离太近只要最后一个点
  for (size_t i = 0; i < points_size; ++i) {
    if (count == 0 || i == points_size - 1) {
      (*points)[count++] = points->at(i);
    } else {
      if (points->at(i).DistanceTo(points->at(count - 1)) >
          kSegmentationEpsilon) {
        (*points)[count++] = points->at(i);
      }
    }
  }
  if (count > 2) {
    if (points->at(count - 2).DistanceTo(points->at(count - 1)) <=
        kSegmentationEpsilon) {
      points->at(count - 2) = points->at(count - 1);
      --count;
    }
  }
  points->resize(count);
}

bool MapSelectLite::MatchPerceLaneAndFusionLane(
    const hozon::common::Pose& pose,
    const hozon::hdmap::LaneInfoConstPtr& curr_lane, bool match_prerequisite) {
  const auto speed = std::max(3.0, vehicle_spd_);
  double max_lanemarker_dis = speed * kMaxLanemarkerTm;
  double val_dis = speed * kValLanemarkerTm;
  double check_dis = speed * kBetterLanemarkerTm;
  double delta_dis = speed * kDeltaTm;

  std::vector<hozon::common::math::Vec2d> fusion_map_left_points;
  std::vector<hozon::common::math::Vec2d> fusion_map_right_points;
  double view_range = 0.0;
  double left_view_range = 0.0;
  double right_view_range = 0.0;

  // if (CheckPerceMsg(ego_left_lane_) && CheckPerceMsg(ego_right_lane_)) {
  //   left_view_range =
  //       ego_left_lane_.lane_param().cubic_curve_set(0).end_point_x();
  //   right_view_range =
  //       ego_right_lane_.lane_param().cubic_curve_set(0).end_point_x();
  //   view_range = std::max(left_view_range, right_view_range);
  // } else {
  //   ResetMatchState();
  //   return true;
  // }
  // const double start_s = pnc_map->GetADCWaypoint().s;
  double lane_length = curr_lane->total_length();
  // HLOG_INFO << "lane_length: " << lane_length << " , view_range: " <<
  // view_range
  //           << " , lane id: " << curr_lane->lane().id().id();
  AddMapPoints(&fusion_map_left_points, &fusion_map_right_points, curr_lane,
               pose);
  int add_lane_size = 1;
  auto next_lane = curr_lane;
  while (lane_length < view_range) {
    if (next_lane->lane().successor_id().empty()) {
      break;
    }
    if (next_lane->lane().successor_id().size() > 1) {
      ResetMatchState();
      return true;
    }

    auto lane_id = next_lane->lane().successor_id().at(0);
    HLOG_INFO << " success lane id: " << lane_id.id();
    next_lane = GLOBAL_HD_MAP->GetLaneById(lane_id);
    if (next_lane == nullptr) {
      break;
    }
    AddMapPoints(&fusion_map_left_points, &fusion_map_right_points, next_lane,
                 pose);
    lane_length += next_lane->total_length();
    HLOG_INFO << "add lane length:" << next_lane->total_length()
              << " , total lane length: " << lane_length;
    add_lane_size++;
  }
  HLOG_INFO << "left_point_size: " << fusion_map_left_points.size()
            << " , add_lane_size: " << add_lane_size;
  int i = 0;
  for (const auto point : fusion_map_left_points) {
    HLOG_INFO << "fusion_map_left_points[" << i << "].x " << point.x()
              << "  y:" << point.y();
    i++;
  }

  if (fusion_map_left_points.size() < 2 || fusion_map_right_points.size() < 2) {
    ResetMatchState();
    return true;
  }

  std::vector<hozon::common::math::Vec2d> left_bus_bound_points =
      InterPolateVec2dPoints(fusion_map_left_points, 0.0, view_range,
                             delta_dis);
  std::vector<hozon::common::math::Vec2d> right_bus_bound_points =
      InterPolateVec2dPoints(fusion_map_right_points, 0.0, view_range,
                             delta_dis);

  if (left_bus_bound_points.size() < 2 || right_bus_bound_points.size() < 2) {
    ResetMatchState();
    return true;
  }
  int z = 0;
  for (const auto point : left_bus_bound_points) {
    HLOG_INFO << "left_bus_bound_points[" << z << "].x " << point.x()
              << "  y:" << point.y();
    z++;
  }

  const auto left_dis = CheckerDistance(left_bus_bound_points, ego_left_lane_);
  const auto right_dis =
      CheckerDistance(right_bus_bound_points, ego_right_lane_);

  // speed * kMinDeltaTm 容忍车道线最多比需求的短0.3s
  HLOG_INFO << "speed: " << speed << "left_view_range:" << left_view_range
            << "left_bus_bound_points.back().x():"
            << left_bus_bound_points.back().x()
            << "right_view_range:" << right_view_range
            << "fusion_map_right_points.back().x():"
            << fusion_map_right_points.back().x();

  HLOG_INFO << "ego_left_lane_ end_point_x(): "
            << ego_left_lane_.lane_param().cubic_curve_set(0).end_point_x()
            << "start point x"
            << ego_left_lane_.lane_param().cubic_curve_set(0).start_point_x();
  const bool left_match_prerequisite =
      match_prerequisite && speed > 10.0 &&
      left_view_range + speed * kMinDeltaTm > max_lanemarker_dis &&
      left_bus_bound_points.back().x() > left_view_range;
  const bool right_match_prerequisite =
      match_prerequisite && speed > 10.0 &&
      right_view_range + speed * kMinDeltaTm > max_lanemarker_dis &&
      right_bus_bound_points.back().x() > right_view_range;
  HLOG_INFO << "left_match_prerequisite: " << left_match_prerequisite
            << " , right_match_prerequisite: " << right_match_prerequisite
            << " , match_prerequisite: " << match_prerequisite;

  // left match state
  if (left_match_state_ == 0) {
    if (left_match_prerequisite && std::get<0>(left_dis) > check_dis &&
        std::get<1>(left_dis) < max_lanemarker_dis) {
      left_match_state_ = 1;
    }
  } else if (left_match_state_ == 1) {
    left_match_state_count_++;
    if (std::get<0>(left_dis) < check_dis) {
      left_check_dis_count_++;
    }
    HLOG_INFO << " left_match_state_count: " << left_match_state_count_;
    if (!left_match_prerequisite || left_check_dis_count_ > 3 ||
        std::get<1>(left_dis) > max_lanemarker_dis ||
        std::get<1>(left_dis) <
            max_lanemarker_dis - (left_match_state_count_ + 3) * delta_dis) {
      left_check_dis_count_ = 0;
      left_match_state_count_ = 0;
      left_match_state_ = 0;
    } else if (left_match_prerequisite && left_check_dis_count_ <= 3 &&
               std::get<1>(left_dis) < val_dis && left_match_state_count_ > 3) {
      left_check_dis_count_ = 0;
      left_match_state_count_ = 0;
      left_match_state_ = 2;
    }
  } else if (left_match_state_ == 2) {
    left_match_state_count_++;
    if ((std::get<0>(left_dis) > check_dis &&
         std::get<1>(left_dis) > max_lanemarker_dis) ||
        !left_match_prerequisite || left_match_state_count_ > 50) {
      left_match_state_count_ = 0;
      left_match_state_ = 0;
    }
  }

  // right match state
  if (right_match_state_ == 0) {
    if (right_match_prerequisite && std::get<0>(right_dis) > check_dis &&
        std::get<1>(right_dis) < max_lanemarker_dis) {
      right_match_state_ = 1;
    }
  } else if (right_match_state_ == 1) {
    if (std::get<0>(right_dis) < check_dis) {
      right_check_dis_count_++;
    }
    right_match_state_count_++;
    HLOG_INFO << " right_match_state_count: " << right_match_state_count_;
    if (!right_match_prerequisite || right_check_dis_count_ > 3 ||
        std::get<1>(right_dis) > max_lanemarker_dis ||
        std::get<1>(right_dis) <
            max_lanemarker_dis - (right_match_state_count_ + 3) * delta_dis) {
      right_check_dis_count_ = 0;
      right_match_state_count_ = 0;
      right_match_state_ = 0;
    } else if (right_match_prerequisite && right_check_dis_count_ <= 3 &&
               std::get<1>(right_dis) < val_dis &&
               right_match_state_count_ > 3) {
      right_check_dis_count_ = 0;
      right_match_state_count_ = 0;
      right_match_state_ = 2;
    }
  } else if (right_match_state_ == 2) {
    right_match_state_count_++;
    if ((std::get<0>(right_dis) > check_dis &&
         std::get<1>(right_dis) > max_lanemarker_dis) ||
        !right_match_prerequisite || right_match_state_count_ > 50) {
      right_match_state_count_ = 0;
      right_match_state_ = 0;
    }
  }

  HLOG_INFO << "--------left_match_state: " << left_match_state_
            << " , right_match_state: " << right_match_state_;
  return left_match_state_ != 2 && right_match_state_ != 2;
}

void MapSelectLite::ResetMatchState() {
  left_match_state_ = 0;
  right_match_state_ = 0;
  left_match_state_count_ = 0;
  right_match_state_count_ = 0;
  left_check_dis_count_ = 0;
  right_check_dis_count_ = 0;
}

std::vector<hozon::common::math::Vec2d> MapSelectLite::InterPolateVec2dPoints(
    const std::vector<hozon::common::math::Vec2d>& map_points, double start_x,
    double end_x, double delta) {
  std::vector<hozon::common::math::Vec2d> out_points;
  for (uint64_t i = 0; i < map_points.size() - 1; i++) {
    auto bus_point_next = map_points[i + 1];
    if (map_points[i].x() < 0 && map_points[i + 1].x() > 0) {
      auto point = InterPolateVec2dPoint(map_points[i], map_points[i + 1],
                                         start_x - map_points[i].x());
      out_points.emplace_back(point);
      while (bus_point_next.x() - point.x() > 2 * delta && point.x() < end_x) {
        point = InterPolateVec2dPoint(point, bus_point_next, delta);
        out_points.emplace_back(point);
      }
    } else if (map_points[i].x() > 0) {
      auto point = map_points[i];
      out_points.emplace_back(point);
      while ((bus_point_next.x() - point.x()) > (2 * delta) &&
             point.x() < end_x) {
        point = InterPolateVec2dPoint(point, bus_point_next, delta);
        out_points.emplace_back(point);
      }
    }
  }
  return out_points;
}

hozon::common::math::Vec2d MapSelectLite::InterPolateVec2dPoint(
    const hozon::common::math::Vec2d& point_start,
    const hozon::common::math::Vec2d& point_end, double dis) {
  return {point_start.x() + dis,
          point_start.y() + dis / (point_end.x() - point_start.x()) *
                                (point_end.y() - point_start.y())};
}

bool MapSelectLite::FusionMapLaneChecker(
    const hozon::common::PointENU& local_pos) {
  // 150米范围内的lane  高精地图异常曲率
  double fusion_map_range = 150.0;
  bool is_abnormal_lane = false;
  std::vector<hozon::hdmap::LaneInfoConstPtr> fusion_lanes{};
  int flag =
      GLOBAL_HD_MAP->GetLanes(local_pos, fusion_map_range, &fusion_lanes);
  if (flag != 0) {
    HLOG_ERROR << "fusion map lane checker get local map lanes failed";
    return true;
  }
  for (const auto& fusion_lane : fusion_lanes) {
    if (fusion_lane->lane().curvature_type() ==
        hozon::hdmap::Lane_LaneCurvature_ABNORMAL) {
      return false;
    }
  }
  return true;
}

int MapSelectLite::LocationErrDecider(
    const std::shared_ptr<hozon::localization::Localization>& localization) {
  // 0:no err 1:normal err 2: serious err 3: fatal err
  int original_location_err_status = 0;
  is_loc_pose_jump_ = false;
  // TODO(zhangrui) 和下游抑制变道的接口
  not_change_lane_ = false;
  if (localization == nullptr) {
    is_location_init_ = false;
    history_pose_time_ = 0.0;
    original_location_err_status = 1;
    return original_location_err_status;
  }
  if (!localization->has_pose() || !localization->has_location_state() ||
      !localization->has_header() || localization->header().has_status()) {
    is_location_init_ = false;
    history_pose_time_ = 0.0;
    original_location_err_status = 1;
    return original_location_err_status;
  }
  uint64_t location_state = localization->location_state();
  // 低速下前方有障碍物时，屏蔽123和130故障
  // 因无法在mapping中获取车状态信息，此策略省去
  // 无法在mapping中获取FunctionManagerOut，无法添加依据odd
  // type赋值location_state，从而屏蔽定位故障
  // ADEBUG << "loc err: " << location_state;
  if (location_state >= kLocationFaultStart &&
      location_state < kLocationFatalFault) {
    original_location_err_status = 3;
  } else if (location_state >= kLocationFatalFault &&
             location_state < kLocationSeriouslFault) {
    original_location_err_status = 2;
  } else if (location_state >= kLocationSeriouslFault &&
             location_state < kLocationNormalFault) {
    original_location_err_status = 1;
  }
  original_location_err_status = original_location_err_debounce_.DealDebounce(
                                     original_location_err_status == 1)
                                     ? 1
                                     : 0;
  not_change_lane_ =
      original_location_err_status == 0 && location_state == kLocationIsFault;
  if (original_location_err_status > 0) {
    is_location_init_ = false;
    history_pose_time_ = 0.0;
    return original_location_err_status;
  }

  const auto pose = localization->pose();
  const bool is_same_utm_zone = localization_zone_id_ == pose.using_utm_zone();
  localization_zone_id_ = pose.using_utm_zone();

  // 定位pose跳变检测  TODO  需传给下游
  if (history_pose_time_ > kMainLoopTime) {
    double time_diff = localization->header().data_stamp() - history_pose_time_;
    double x_diff = std::fabs(history_location_point_.x - pose.position().x() +
                              pose.linear_velocity().x() * time_diff);
    double y_diff = std::fabs(history_location_point_.y - pose.position().y() +
                              pose.linear_velocity().y() * time_diff);
    double vehicle_pose_x =
        x_diff * std::cos(pose.heading()) + y_diff * std::sin(pose.heading());
    double vehicle_pose_y = -(-x_diff * std::sin(pose.heading()) +
                              y_diff * std::cos(pose.heading()));
    if (is_same_utm_zone && (vehicle_pose_x > kLocationXerrMin ||
                             vehicle_pose_y > kLocationYerrMin)) {
      is_loc_pose_jump_ = is_location_init_;
    } else {
      is_location_init_ = true;
    }

    // ADEBUG << "history_location_point x = " << history_location_point_.x()
    //        << ", y = " << history_location_point_.y()
    //        << ", now_location_point x = " << pose.position().x()
    //        << ", y = " << pose.position().y()
    //        << ", x_diff = " << pose.linear_velocity().x() * time_diff
    //        << ", pose_diff_x = "
    //        << history_location_point_.x() - pose.position().x()
    //        << ", pose_diff_y = "
    //        << history_location_point_.y() - pose.position().y()
    //        << ", y_diff = " << pose.linear_velocity().y() * time_diff
    //        << ", vehicle_x_err = " << vehicle_pose_x
    //        << ", vehicle_y_err = " << vehicle_pose_y;
  }
  history_pose_time_ = localization->header().data_stamp();
  history_location_point_.x = pose.position().x();
  history_location_point_.y = pose.position().y();
  return original_location_err_status;
}

bool MapSelectLite::DeciderPercpMsg(
    const std::shared_ptr<const ::hozon::perception::TransportElement>&
        perception_msg,
    const std::shared_ptr<const ::hozon::dead_reckoning::DeadReckoning>&
        dr_msg) {
  vehicle_spd_ =
      (dr_msg->has_velocity() && dr_msg->velocity().has_twist_vrf() &&
       dr_msg->velocity().twist_vrf().has_linear_vrf() &&
       dr_msg->velocity().twist_vrf().linear_vrf().has_x())
          ? dr_msg->velocity().twist_vrf().linear_vrf().x()
          : 0.0;
  const double good_quality_value = 0.6;
  const double min_lane_width = 3.2;
  const double max_lane_width = 4.5;
  const double lane_a0_coff_var_thd = 0.16;
  const double lane_a1_coff_var_thd = 0.00035;
  const double min_length_view_range = std::max(vehicle_spd_ * 1.5, 20.0);
  const double min_view_range_bad = std::min(10.0, std::max(vehicle_spd_, 1.0));
  const double max_heading_angle = 0.05;
  const double min_heading_angle = 0.01;
  const double k_lane_change_delta = 8.0;

  if (!SelectEgoLaneInfo(perception_msg)) {
    HLOG_ERROR << "Cannot get ego original perception lane";
    return false;
  }
  const auto ego_left_attribute =
      ego_left_lane_.lane_param().cubic_curve_set(0);
  const auto ego_right_attribute =
      ego_right_lane_.lane_param().cubic_curve_set(0);
  const auto left_c1 = std::fabs(ego_left_attribute.c1());
  const auto right_c1 = std::fabs(ego_right_attribute.c1());
  bool is_lanemarker_jump = false;
  is_lanemarker_jump =
      ((std::pow(left_lanemarker_a0_delay_.GetAverageValue() -
                     ego_left_attribute.c0(),
                 2) > lane_a0_coff_var_thd ||
        std::pow(left_lanemarker_a1_delay_.GetAverageValue() -
                     ego_left_attribute.c1(),
                 2) > lane_a1_coff_var_thd ||
        std::pow(right_lanemarker_a0_delay_.GetAverageValue() -
                     ego_right_attribute.c0(),
                 2) > lane_a0_coff_var_thd ||
        std::pow(right_lanemarker_a1_delay_.GetAverageValue() -
                     ego_right_attribute.c1(),
                 2) > lane_a1_coff_var_thd) &&
       (left_c1 < min_heading_angle && right_c1 < min_heading_angle)) ||
      (left_c1 > max_heading_angle || right_c1 > max_heading_angle);
  left_lanemarker_a0_delay_.Deal(ego_left_attribute.c0());
  left_lanemarker_a1_delay_.Deal(ego_left_attribute.c1());
  right_lanemarker_a0_delay_.Deal(ego_right_attribute.c0());
  right_lanemarker_a1_delay_.Deal(ego_right_attribute.c1());

  const auto good_left_lanemarker =
      left_lanemarker_quality_debounce_.DealDebounce(
          ego_left_lane_.confidence() > good_quality_value);
  const auto good_right_lanemarker =
      right_lanemarker_quality_debounce_.DealDebounce(
          ego_right_lane_.confidence() > good_quality_value);

  HLOG_ERROR << " ego_left_lane_.confidence() = " << ego_left_lane_.confidence()
             << "ego_right_lane_.confidence():" << ego_right_lane_.confidence()
             << "good_left_lanemarker:" << good_left_lanemarker
             << "good_right_lanemarker:" << good_right_lanemarker;
  const bool is_good_quality = ego_left_lane_.confidence() > 0.3 &&
                               ego_right_lane_.confidence() > 0.3 &&
                               good_left_lanemarker && good_right_lanemarker;

  const auto view_range_deb =
      (left_c1 < min_heading_angle && right_c1 < min_heading_angle)
          ? min_view_range_bad
          : min_length_view_range;
  const auto good_left_view_range =
      left_lanemarker_viewrange_debounce_.DealDebounce(
          ego_left_attribute.end_point_x() > view_range_deb);
  const auto good_right_view_range =
      right_lanemarker_viewrange_debounce_.DealDebounce(
          ego_right_attribute.end_point_x() > view_range_deb);
  const bool good_viewrange =
      ego_left_attribute.end_point_x() > min_view_range_bad &&
      ego_right_attribute.end_point_x() > min_view_range_bad &&
      (good_left_view_range || good_right_view_range);

  double lane_width = ego_left_attribute.c0() - ego_right_attribute.c0();

  double view_lane_width =
      CalculateLanemarkerY(min_view_range_bad, ego_left_attribute) -
      CalculateLanemarkerY(min_view_range_bad, ego_right_attribute);
  const bool is_suitable_lane_width = lane_width > min_lane_width &&
                                      lane_width < max_lane_width &&
                                      view_lane_width > min_lane_width - 0.2 &&
                                      view_lane_width < max_lane_width;
  HLOG_ERROR << " is_lanemarker_jump = " << is_lanemarker_jump
             << ", is_good_quality = " << is_good_quality
             << ", good_viewrange = " << good_viewrange
             << ", is_suitable_lane_width = " << is_suitable_lane_width;
  // 只使用quality和viewrange进行判断
  // 降级前判断严格些，不然会出现无车道线判断偏差过大的情况
  const bool is_good_lane = is_good_quality && good_viewrange &&
                            !is_lanemarker_jump && is_suitable_lane_width;
  const bool is_init_good_lane =
      is_good_init_lanemarker_debounce_.DealDebounce(is_good_lane);
  const bool is_break_good_lane =
      is_good_lanemarker_debounce_.DealDebounce(is_good_lane);
  left_lanemarker_c1_delta_ = left_c1 * k_lane_change_delta;
  right_lanemarker_c1_delta_ = right_c1 * k_lane_change_delta;

  return !(loc_err_state_ == 1 ? is_break_good_lane : is_init_good_lane);
}
bool MapSelectLite::SelectEgoLaneInfo(
    const std::shared_ptr<const ::hozon::perception::TransportElement>&
        perception_msg) {
  if (perception_msg->lane().empty()) {
    return false;
  }
  bool select_left = false;
  bool select_right = false;
  for (const auto& lane_info : perception_msg->lane()) {
    if (lane_info.lanepos() ==
        ::hozon::perception::LanePositionType::EGO_LEFT) {
      ego_left_lane_ = lane_info;
      select_left = true;
    }
    if (lane_info.lanepos() ==
        ::hozon::perception::LanePositionType::EGO_RIGHT) {
      ego_right_lane_ = lane_info;
      select_right = true;
    }

    if (select_left && select_right) {
      return true;
    }
  }
  return false;
}

double MapSelectLite::CalculateLanemarkerY(
    const double distance,
    const hozon::perception::LaneCubicCurve& lane_attribute) {
  double x2 = distance * distance;
  double x3 = x2 * distance;
  return lane_attribute.c0() + lane_attribute.c1() * distance +
         lane_attribute.c2() * x2 + lane_attribute.c3() * x3;
}

std::tuple<double, double, double> MapSelectLite::CheckerDistance(
    const std::vector<hozon::common::math::Vec2d>& map_bound_points,
    const ::hozon::perception::LaneInfo& lane_info) {
  int index = 0;
  double left_map_match_x = map_bound_points.back().x();
  bool is_check_match = true;
  bool is_check_diff = true;
  bool is_checker_recorve = true;
  double left_diff_x = map_bound_points.back().x();
  double left_check_start_x = map_bound_points.back().x();
  auto lane_attribute = lane_info.lane_param().cubic_curve_set(0);
  // double left_check_recorve_x = map_bound_points.back().x();
  for (uint64_t i = 0; i < map_bound_points.size() - 1; i++) {
    auto bus_point = map_bound_points[i];
    auto y = CalculateLanemarkerY(bus_point.x(), lane_attribute);
    double delta_y = std::fabs(bus_point.y() - y);
    // HLOG_INFO << " bus_bound_points[" << index << "] x: " << bus_point.x()
    //           << " , map_y: " << bus_point.y() << " ,lane_y: " << y
    //           << " , delta_y: " << delta_y;
    if (delta_y > kCheckErr && is_check_match) {
      is_check_match = false;
      left_map_match_x = bus_point.x();
    }
    if (delta_y > kValErr && is_check_diff) {
      is_check_diff = false;
      left_diff_x = bus_point.x();
    }
    if (delta_y > 0.3 && is_checker_recorve) {
      is_checker_recorve = false;
      // left_check_recorve_x = bus_point.x();
    }
    if (delta_y > kMaxErr) {
      left_check_start_x = bus_point.x();
      break;
    }
    index++;
  }
  // HLOG_INFO << "left_map_match_x: " << left_map_match_x
  //           << " , left_diff_x: " << left_diff_x
  //           << " , left_check_start_x: " << left_check_start_x;
  return {left_map_match_x, left_diff_x, left_check_start_x};
}

bool MapSelectLite::CheckPerceMsg(
    const std::shared_ptr<const ::hozon::perception::TransportElement>&
        perception_msg) {
  if (perception_msg == nullptr) {
    HLOG_ERROR << "perception msg is null";
    return false;
  }

  // //! TBD: by
  // taoshaoyuan，暂时直接返回true，即不对原始感知时间戳和内容进行检查. return
  // true;

  // if (last_perce_time_ == perception_msg->header().data_stamp()) {
  //   HLOG_ERROR << "perception msg not update data_time"
  //              << perception_msg->header().data_stamp();
  //   return false;
  // }
  // last_perce_time_ = perception_msg->header().data_stamp();

  for (const auto& lane_info : perception_msg->lane()) {
    if (!lane_info.has_track_id() || !lane_info.has_lanepos() ||
        !lane_info.has_lane_param()) {
      HLOG_ERROR << "original perception msg is wrong";
      return false;
    }
    if (lane_info.lane_param().cubic_curve_set().empty()) {
      HLOG_ERROR << "original perception msg is wrong";
      return false;
    }
    if (!lane_info.lane_param().cubic_curve_set(0).has_c0() ||
        !lane_info.lane_param().cubic_curve_set(0).has_c1() ||
        !lane_info.lane_param().cubic_curve_set(0).has_end_point_x()) {
      HLOG_ERROR << "original perception msg is wrong";
      return false;
    }
    if ((lane_info.lanepos() ==
             ::hozon::perception::LanePositionType::EGO_LEFT ||
         lane_info.lanepos() ==
             ::hozon::perception::LanePositionType::EGO_RIGHT) &&
        std::abs(lane_info.lane_param().cubic_curve_set(0).c0()) > 3.75) {
      HLOG_ERROR << "original perception cubic_curve_set msg is wrong";
      return false;
    }
  }
  return true;
}
bool MapSelectLite::CheckDrMsg(
    const std::shared_ptr<const ::hozon::dead_reckoning::DeadReckoning>&
        dr_msg) {
  if (dr_msg == nullptr) {
    HLOG_ERROR << "dr msg is null";
    return false;
  }
  //  if (last_dr_time_ == dr_msg->header().data_stamp()) {
  //    HLOG_ERROR << "dr msg is not update";
  //    return false;
  //  }
  last_dr_time_ = dr_msg->header().data_stamp();
  return true;
}

bool MapSelectLite::CheckFCTIN(
    const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in) {
  if (nullptr == fct_in) {
    HLOG_ERROR << "fct msg is null";
    return false;
  }
  //  if (last_fct_in_time_ == fct_in->header().publish_stamp()) {
  //    HLOG_ERROR << "fct in  msg is not update";
  //    return false;
  //  }

  last_fct_in_time_ = fct_in->header().publish_stamp();
  if (!fct_in->has_fct_2_soc_tbd_u32_03()) {
    HLOG_ERROR << "nnp turn off message not in fct in  msg";
    return false;
  }
  return true;
}
bool MapSelectLite::CheckFusionMapMsg(
    const std::shared_ptr<hozon::hdmap::Map>& fusion_map) {
  if (nullptr == fusion_map) {
    debug_.add_origin_response("fusion map msg is null");
    return false;
  }
  if (fusion_map->lane().empty() || fusion_map->road().empty()) {
    debug_.add_origin_response("fusion map msg has no lane");
    return false;
  }
  return true;
}

bool MapSelectLite::CheckPercepMapMsg(
    const std::shared_ptr<hozon::hdmap::Map>& percep_map) {
  if (nullptr == percep_map) {
    debug_.add_origin_response("percep map msg is null");
    return false;
  }
  // if (!fusion_map->lane()) {
  //   debug_.add_origin_response("fusion map msg has no lane");
  //   return false;
  // }
  if (percep_map->lane().empty()) {
    debug_.add_origin_response("percep map msg has no lane");
    return false;
  }
  return true;
}

bool MapSelectLite::CalculateValidLocErrHasMap(
    std::pair<double, double>* width_diff) {
  // 视觉车道线对地图的偏差，左正右负
  double left_err =
      ego_left_lane_.lane_param().cubic_curve_set(0).c0() - left_map_c0_;
  double right_err =
      ego_right_lane_.lane_param().cubic_curve_set(0).c0() - right_map_c0_;
  // HLOG_ERROR << "ego_left_lane_.lane_param().cubic_curve_set(0).c0():"
  //            << ego_left_lane_.lane_param().cubic_curve_set(0).c0()
  //            << "\nego_right_lane_.lane_param().cubic_curve_set(0).c0():"
  //  << ego_right_lane_.lane_param().cubic_curve_set(0).c0();
  *width_diff = std::make_pair(left_err, right_err);
  return true;
}

hozon::common::math::Vec2d MapSelectLite::PointEarth2Bus(
    const hozon::common::math::Vec2d& point, const hozon::common::Pose& pose) {
  hozon::common::math::Vec2d bus_point;
  double x = point.x() - pose.pos_utm_01().x();
  double y = point.y() - pose.pos_utm_01().y();
  bus_point.set_x(x * std::cos(pose.heading()) + y * std::sin(pose.heading()));
  bus_point.set_y(-x * std::sin(pose.heading()) + y * std::cos(pose.heading()));
  return bus_point;
}
bool MapSelectLite::GetU32BitByIndex(const uint32_t input,
                                     const uint32_t index) {
  return (input >> index) & 0x1;
}
hozon::navigation_hdmap::MapMsg_MapType MapSelectLite::GetMapTypeByRoadType() {
  HLOG_INFO << "road_type: "
            << hozon::hdmap::Road::Type_Name(current_road_type_);
  if (hozon::hdmap::Road::Type::Road_Type_HIGHWAY == current_road_type_ ||
      hozon::hdmap::Road::Type::Road_Type_CITY_HIGHWAY == current_road_type_) {
    return hozon::navigation_hdmap::MapMsg_MapType_FUSION_NNP_MAP;
  }
  return hozon::navigation_hdmap::MapMsg_MapType_FUSION_NCP_MAP;
}

bool MapSelectLite::NnpSwitchOn(
    const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in) {
  return GetU32BitByIndex(fct_in->fct_2_soc_tbd_u32_03(), 28);
}

bool MapSelectLite::CheckFctIn(
    const std::shared_ptr<hozon::functionmanager::FunctionManagerIn>& fct_in) {
  if (nullptr == fct_in) {
    HLOG_ERROR << "fct msg is null";
    return false;
  }

  if (!fct_in->has_fct_2_soc_tbd_u32_03()) {
    HLOG_ERROR << "nnp switch message not in fct in msg";
    return false;
  }
  return true;
}

bool MapSelectLite::CheckMapMsg(
    const std::shared_ptr<hozon::hdmap::Map>& map,
    const std::shared_ptr<hozon::localization::Localization>& localization,
    bool loc_state, bool is_fusion_map) {
  if (map == nullptr) {
    return false;
  }
  if (map->lane().empty()) {
    return false;
  }

  if (!is_fusion_map) {
    return true;
  }
  // 目前只针对FusionMap起作用
  if (loc_state) {
    if (!IsCarInLanes(map, localization)) {
      HLOG_ERROR << "Car is not in lanes scope"
                 << "is_fusion_map: " << is_fusion_map;
      return false;
    }
  }

  return true;
}

bool MapSelectLite::CheckGlobalLoc(
    const std::shared_ptr<hozon::localization::Localization>& loc) {
  if (loc == nullptr) {
    return false;
  }
  if (!loc->has_pose() || !ValidQuaternion(loc->pose().quaternion())) {
    return false;
  }
  if (!loc->pose().has_gcj02() || !ValidLla(loc->pose().gcj02())) {
    return false;
  }
  if (!loc->pose().has_heading()) {
    return false;
  }
  if (!loc->has_location_state()) {
    return false;
  }
  const auto loc_state = loc->location_state();

  std::set<uint32_t> normal_loc_states = {
      2,  // 组合导航+MM+DR
  };
  if (normal_loc_states.find(loc_state) == normal_loc_states.end()) {
    return false;
  }

  return true;
}

bool MapSelectLite::CheckLocalLoc(
    const std::shared_ptr<hozon::localization::Localization>& loc) {
  if (loc == nullptr) {
    return false;
  }
  if (!loc->has_pose_local() ||
      !ValidQuaternion(loc->pose_local().quaternion())) {
    return false;
  }
  if (!loc->pose_local().has_position()) {
    return false;
  }
  if (!loc->pose_local().has_local_heading()) {
    return false;
  }
  return true;
}

bool MapSelectLite::PercepMapAvailable(
    const std::shared_ptr<hozon::hdmap::Map>& map,
    const std::shared_ptr<hozon::localization::Localization>& local_loc) {
  bool valid_local_loc = CheckLocalLoc(local_loc);
  bool valid_percep_map = CheckMapMsg(map, local_loc, valid_local_loc, false);
  bool available = valid_local_loc && valid_percep_map;
  HLOG_INFO << "local_loc " << (valid_local_loc ? "valid" : "invalid")
            << ", percep_map " << (valid_percep_map ? "valid" : "invalid")
            << ", percep_map " << (available ? "available" : "unavailable");
  return available;
}

bool MapSelectLite::FusionMapAvailable(
    const std::shared_ptr<hozon::hdmap::Map>& map,
    const std::shared_ptr<hozon::localization::Localization>& global_loc,
    const std::shared_ptr<hozon::localization::Localization>& local_loc) {
  bool valid_local_loc = CheckLocalLoc(local_loc);
  bool valid_global_loc = CheckGlobalLoc(global_loc);
  bool valid_fusion_map =
      CheckMapMsg(map, local_loc, valid_local_loc && valid_global_loc, true);
  bool available = valid_local_loc && valid_global_loc && valid_fusion_map;
  HLOG_INFO << "local_loc " << (valid_local_loc ? "valid" : "invalid")
            << ", global_loc " << (valid_global_loc ? "valid" : "invalid")
            << ", fusion_map " << (valid_fusion_map ? "valid" : "invalid")
            << ", fusion_map " << (available ? "available" : "unavailable");
  return available;
}

bool ValidQuaternion(const hozon::common::Quaternion& quat) {
  if (!quat.has_w() || !quat.has_x() || !quat.has_y() || !quat.has_z()) {
    return false;
  }

  if (std::isnan(quat.w()) || std::isnan(quat.x()) || std::isnan(quat.y()) ||
      std::isnan(quat.z())) {
    return false;
  }
  if (std::isinf(quat.w()) || std::isinf(quat.x()) || std::isinf(quat.y()) ||
      std::isinf(quat.z())) {
    return false;
  }
  Eigen::Quaternionf q(quat.w(), quat.x(), quat.y(), quat.z());
  auto norm = q.norm();
  const float kEpsilon = 0.01;
  if (std::abs(norm - 1) > kEpsilon) {
    return false;
  }

  return true;
}

bool ValidLla(const hozon::common::PointENU& lla) {
  if (!lla.has_x() || !lla.has_y() || !lla.has_z()) {
    return false;
  }
  if (std::isnan(lla.x()) || std::isnan(lla.y()) || std::isnan(lla.z())) {
    return false;
  }
  if (std::isinf(lla.x()) || std::isinf(lla.y()) || std::isinf(lla.z())) {
    return false;
  }
  if (lla.x() < -85.0 || lla.x() > 85.0 || lla.y() < -180.0 ||
      lla.y() > 180.0) {
    return false;
  }
  return true;
}
bool MapSelectLite::IsCarInLanes(
    const std::shared_ptr<hozon::hdmap::Map>& map,
    const std::shared_ptr<hozon::localization::Localization>& localization) {
  lanes_lines_map_.clear();
  min_dist_value_ = FLT_MAX;
  for (const auto& lane : map->lane()) {
    if (!CheckLane(lane)) {
      HLOG_DEBUG << "something is wrong when check lane ";
      continue;
    }
    SelectLaneLine lane_line;
    lane_line.id = lane.id().id();
    std::vector<cv::Point2f> kdtree_points;
    std::vector<Eigen::Vector2d> line_points;

    for (const auto& segment : lane.central_curve().segment()) {
      for (const auto& point : segment.line_segment().point()) {
        kdtree_points.emplace_back(static_cast<float>(point.x()),
                                   static_cast<float>(point.y()));
        line_points.emplace_back(point.x(), point.y());
      }
    }

    // 构建当前lane的kdtree
    cv::flann::KDTreeIndexParams index_params(1);
    std::shared_ptr<cv::flann::Index> kdtree_ptr =
        std::make_shared<cv::flann::Index>(cv::Mat(kdtree_points).reshape(1),
                                           index_params);
    lane_line.lane_line_kdtree = kdtree_ptr;
    lane_line.lane_line_points = line_points;
    lanes_lines_map_[lane.id().id()] = lane_line;
  }

  std::vector<float> local_car_pos = std::vector<float>{
      static_cast<float>(localization->pose_local().position().x()),
      static_cast<float>(localization->pose_local().position().y())};
  Eigen::Vector2d car_pos = {localization->pose_local().position().x(),
                             localization->pose_local().position().y()};
  int dim = 1;
  float curr_dist_value = max_half_lane_dis_;
  for (const auto& lane_lines : lanes_lines_map_) {
    std::vector<int> nearest_index(dim);
    std::vector<float> nearest_dist(dim);
    lane_lines.second.lane_line_kdtree->knnSearch(local_car_pos, nearest_index,
                                                  nearest_dist, dim,
                                                  cv::flann::SearchParams(-1));

    auto nearest_dis_value = sqrt(nearest_dist[0]);

    lanes_lines_map_[lane_lines.first].min_dist_value = nearest_dis_value;
    lanes_lines_map_[lane_lines.first].nearest_index = nearest_index[0];
    if (nearest_dis_value < min_dist_value_) {
      min_dist_value_ = nearest_dis_value;
      min_dist_id_ = lane_lines.first;
      nearest_index_ = nearest_index[0];
    }
  }
  if (lanes_lines_map_.find(min_dist_id_) != lanes_lines_map_.end()) {
    auto line_points_size =
        lanes_lines_map_[min_dist_id_].lane_line_points.size();
    UpdateThresholdValue(map, &curr_dist_value);
    if ((min_dist_value_ > curr_dist_value) && (line_points_size > 1)) {
      int second_point_index = 0;
      if (nearest_index_ <= line_points_size - 2) {
        second_point_index = nearest_index_ + 1;
      } else {
        second_point_index = nearest_index_ - 1;
      }

      auto nearest_point =
          lanes_lines_map_[min_dist_id_].lane_line_points[nearest_index_];
      auto second_point =
          lanes_lines_map_[min_dist_id_].lane_line_points[second_point_index];

      min_dist_value_ = PointToVectorDist(nearest_point, second_point, car_pos);
    }
  }

  bool upate_lane_id = false;
  if (min_dist_value_ > curr_dist_value) {
    for (const auto& lane_lines : lanes_lines_map_) {
      auto lane_id = lane_lines.first;
      auto lane_nearest_index = lane_lines.second.nearest_index;
      int second_lane_index = 0;
      auto line_points_size = lanes_lines_map_[lane_id].lane_line_points.size();
      if (line_points_size < 2) {
        continue;
      }
      if (lane_nearest_index <= line_points_size - 2) {
        second_lane_index = lane_nearest_index + 1;
      } else {
        second_lane_index = lane_nearest_index - 1;
      }
      auto nearest_point =
          lanes_lines_map_[lane_id].lane_line_points[lane_nearest_index];
      auto second_point =
          lanes_lines_map_[lane_id].lane_line_points[second_lane_index];
      auto line_dist_value =
          PointToVectorDist(nearest_point, second_point, car_pos);
      if (min_dist_value_ > line_dist_value) {
        upate_lane_id = true;
        min_dist_value_ = line_dist_value;
        min_dist_id_ = lane_id;
      }
    }
  }

  if (upate_lane_id) {
    UpdateThresholdValue(map, &curr_dist_value);
  }

  if (min_dist_value_ <= curr_dist_value) {
    std::vector<Eigen::Vector2d> left_start_end_points;
    std::vector<Eigen::Vector2d> right_start_end_points;
    bool out_of_end_scope = false;
    if (UpdateSelfBoundaryPoints(map, min_dist_id_, &left_start_end_points,
                                 &right_start_end_points)) {
      if (PointInVectorSide(left_start_end_points.back(),
                            right_start_end_points.back(), car_pos) < 0) {
        HLOG_ERROR << "the car out the end point scope";
        out_of_end_scope = true;
        // return false;
      }
      if (PointInVectorSide(left_start_end_points.front(),
                            right_start_end_points.front(), car_pos) > 0) {
        HLOG_ERROR << "the car out the start point scope";
        return false;
      }
    }
    if (out_of_end_scope) {
      if ((left_start_end_points.size() == 3) &&
          (right_start_end_points.size() == 3)) {
        if (PointInVectorSide(left_start_end_points.at(1),
                              right_start_end_points.at(1), car_pos) > 0) {
          return true;
        }
      }
      return false;
    }
    return true;
  }
  HLOG_ERROR << "dis between car and the nearest cental line :"
             << min_dist_value_
             << "lanes_lines_  size:" << lanes_lines_map_.size()
             << "location_state:  " << localization->location_state()
             << "min dis lane id  " << min_dist_id_
             << "curr_dist_value :" << curr_dist_value;
  return false;
}

bool MapSelectLite::CheckLane(const hozon::hdmap::Lane& lane) {
  // if (lane.successor_id().empty() && lane.predecessor_id().empty()) {
  //   HLOG_ERROR << ">>>>1";
  //   return false;
  // }

  if (!lane.has_central_curve() || !lane.has_left_boundary() ||
      !lane.has_right_boundary() || !lane.has_id()) {
    HLOG_DEBUG << "lane has no boundary or id";
    return false;
  }
  if (!lane.left_boundary().has_curve() || !lane.right_boundary().has_curve()) {
    HLOG_DEBUG << "lane has no curve";
    return false;
  }
  if (lane.central_curve().segment().empty() ||
      lane.left_boundary().curve().segment().empty() ||
      lane.right_boundary().curve().segment().empty()) {
    HLOG_DEBUG << "the segment of lane is empty";
    return false;
  }
  for (const auto& segment : lane.central_curve().segment()) {
    if (!segment.has_line_segment()) {
      HLOG_DEBUG << "central line has no line segment";
      return false;
    }
    if (segment.line_segment().point().empty()) {
      HLOG_DEBUG << "central line point vec is empty";
      return false;
    }
    for (const auto& point : segment.line_segment().point()) {
      if (!point.has_x() || !point.has_y()) {
        HLOG_DEBUG << "central line point has no x or y";
        return false;
      }
    }
  }
  for (const auto& right_segment : lane.right_boundary().curve().segment()) {
    if (!right_segment.has_line_segment()) {
      HLOG_DEBUG << "right boundary has no line segment";
      return false;
    }
    if (right_segment.line_segment().point().empty()) {
      HLOG_DEBUG << "right boundary  point is empty";
      return false;
    }
    for (const auto& point : right_segment.line_segment().point()) {
      if (!point.has_x() || !point.has_y()) {
        HLOG_DEBUG << "the point of right boundary  has no x or y";
        return false;
      }
    }
  }
  for (const auto& left_segment : lane.left_boundary().curve().segment()) {
    if (!left_segment.has_line_segment()) {
      HLOG_DEBUG << "left boundary has no line segment";
      return false;
    }
    if (left_segment.line_segment().point().empty()) {
      HLOG_DEBUG << "left boundary  point is empty";
      return false;
    }
    for (const auto& point : left_segment.line_segment().point()) {
      if (!point.has_x() || !point.has_y()) {
        HLOG_DEBUG << "the point of left boundary  has no x or y";
        return false;
      }
    }
  }
  return true;
}

bool MapSelectLite::UpdateSelfBoundaryPoints(
    const std::shared_ptr<hozon::hdmap::Map>& map, std::string lane_id,
    std::vector<Eigen::Vector2d>* left_points,
    std::vector<Eigen::Vector2d>* right_points) {
  std::string suc_id;
  while (IsLaneInMap(map, lane_id).is_lane_in_map) {
    suc_id = lane_id;
    lane_id = IsLaneInMap(map, lane_id).predecessor_id;
  }

  left_boundary_points_.clear();
  right_boundary_points_.clear();
  while (IsLaneInMap(map, suc_id).is_lane_in_map) {
    auto suc_lane_info = IsLaneInMap(map, suc_id);
    suc_id = suc_lane_info.successor_id;
    if (CheckLane(suc_lane_info.lane)) {
      for (const auto& left_segment :
           suc_lane_info.lane.left_boundary().curve().segment()) {
        for (const auto& point : left_segment.line_segment().point()) {
          left_boundary_points_.emplace_back(point.x(), point.y());
        }
      }
      for (const auto& right_segment :
           suc_lane_info.lane.right_boundary().curve().segment()) {
        for (const auto& point : right_segment.line_segment().point()) {
          right_boundary_points_.emplace_back(point.x(), point.y());
        }
      }
    }
  }
  RemoveDuplicates(&left_boundary_points_);
  RemoveDuplicates(&right_boundary_points_);

  auto left_size = left_boundary_points_.size();
  auto right_size = right_boundary_points_.size();
  if ((left_size < 2) || (right_size < 2)) {
    return false;
  }
  left_points->emplace_back(left_boundary_points_.front().x(),
                            left_boundary_points_.front().y());
  right_points->emplace_back(right_boundary_points_.front().x(),
                             right_boundary_points_.front().y());
  if ((left_size > 5) && (right_size > 5)) {
    left_points->emplace_back(left_boundary_points_.at(left_size / 2).x(),
                              left_boundary_points_.at(left_size / 2).y());
    right_points->emplace_back(right_boundary_points_.at(left_size / 2).x(),
                               right_boundary_points_.at(left_size / 2).y());
  }
  left_points->emplace_back(left_boundary_points_.back().x(),
                            left_boundary_points_.back().y());
  right_points->emplace_back(right_boundary_points_.back().x(),
                             right_boundary_points_.back().y());
  return true;
}

bool MapSelectLite::UpdateThresholdValue(
    const std::shared_ptr<hozon::hdmap::Map>& map, float* dis_value) {
  for (const auto& lane : map->lane()) {
    if ((min_dist_id_ == lane.id().id()) &&
        (lanes_lines_map_.find(min_dist_id_) != lanes_lines_map_.end())) {
      auto index = lanes_lines_map_[min_dist_id_].nearest_index;
      auto points_size = lanes_lines_map_[min_dist_id_].lane_line_points.size();
      if ((index > points_size - 1) || (index < 0)) {
        break;
      }
      auto eigen_point = lanes_lines_map_[min_dist_id_].lane_line_points[index];
      hozon::common::math::Vec2d nearest_point{eigen_point.x(),
                                               eigen_point.y()};
      std::vector<hozon::common::math::Vec2d> lbundary_points;
      for (const auto& left_segment : lane.left_boundary().curve().segment()) {
        for (const auto& point : left_segment.line_segment().point()) {
          lbundary_points.emplace_back(point.x(), point.y());
        }
      }
      if (lbundary_points.size() > 1) {
        std::sort(lbundary_points.begin(), lbundary_points.end(),
                  [&](const auto& a, const auto& b) {
                    return a.DistanceTo(nearest_point) <
                           b.DistanceTo(nearest_point);
                  });
        Eigen::Vector2d p0 = {lbundary_points.at(0).x(),
                              lbundary_points.at(0).y()};
        Eigen::Vector2d p1 = {lbundary_points.at(1).x(),
                              lbundary_points.at(1).y()};
        auto vertical_dist = PointToVectorDist(p0, p1, eigen_point);

        // HLOG_ERROR << "vertical_dist: " << vertical_dist
        //            << "p0 x :" << lbundary_points.at(0).x()
        //            << "p0 y:" << lbundary_points.at(0).y()
        //            << "   p1 x: " << lbundary_points.at(1).x()
        //            << "   p1 y: " << lbundary_points.at(1).y()
        //            << "  eigen_point x: " << eigen_point.x()
        //            << "  eigen_point y: " << eigen_point.y();
        float update_dist = (vertical_dist < max_half_lane_dis_)
                                ? vertical_dist
                                : max_half_lane_dis_;

        *dis_value = (update_dist > min_half_lane_dis_) ? update_dist
                                                        : min_half_lane_dis_;
        return true;
      }
    }
  }
  return false;
}

LaneInMapInfo MapSelectLite::IsLaneInMap(
    const std::shared_ptr<hozon::hdmap::Map>& map, std::string lane_id) {
  LaneInMapInfo lane_info;
  lane_info.is_lane_in_map = false;
  for (const auto& lane : map->lane()) {
    if (lane_id == lane.id().id()) {
      lane_info.is_lane_in_map = true;
      if (!lane.predecessor_id().empty()) {
        lane_info.predecessor_id = lane.predecessor_id()[0].id();
      }
      if (!lane.successor_id().empty()) {
        lane_info.successor_id = lane.successor_id()[0].id();
      }
      lane_info.lane = lane;
      break;
    }
  }
  return lane_info;
}
}  // namespace select
}  // namespace mf
}  // namespace mp
}  // namespace hozon
