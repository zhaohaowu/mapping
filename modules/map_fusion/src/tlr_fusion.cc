/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： tlr_fusion.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.12
 ******************************************************************************/

#include "map_fusion/tlr_fusion.h"

#include "base/utils/log.h"
#include "util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

hozon::hdmap::JunctionPassable::JunctionPassableType ColorToPassable(
    const hozon::perception::Color& color) {
  hozon::hdmap::JunctionPassable::JunctionPassableType ret =
      hozon::hdmap::JunctionPassable_JunctionPassableType_JUNCTION_RED;
  switch (color) {
    case hozon::perception::GREEN:
      ret = hozon::hdmap::JunctionPassable_JunctionPassableType_JUNCTION_GREEN;
      break;
    case hozon::perception::RED:
      ret = hozon::hdmap::JunctionPassable_JunctionPassableType_JUNCTION_RED;
      break;
    case hozon::perception::YELLOW:
      ret = hozon::hdmap::JunctionPassable_JunctionPassableType_JUNCTION_YELLOW;
      break;
    default:
      break;
  }
  return ret;
}

int TlrFusion::Proc(
    const std::shared_ptr<hozon::perception::TrafficLightDetection>& tlr,
    hozon::hdmap::JunctionPassable* junc_passable) {
  if (tlr == nullptr || junc_passable == nullptr) {
    HLOG_ERROR << "input nullptr tlr or junc_passable";
    return -1;
  }

  if (!tlr->has_header()) {
    HLOG_ERROR << "no header in input tlr";
    return -1;
  }

  junc_passable->mutable_header()->CopyFrom(tlr->header());

  if (!tlr->has_passable_info()) {
    HLOG_DEBUG << "no passable_info in input tlr";
    return 0;
  }

  if (!tlr->has_contain_lights() || !tlr->contain_lights()) {
    HLOG_DEBUG << "no contain_lights in input tlr";
    return 0;
  }

  if (tlr->passable_info().has_turn_around()) {
    if (tlr->passable_info().turn_around() != hozon::perception::NONE &&
        tlr->passable_info().turn_around() !=
            hozon::perception::NONE_NO_CROSSROAD) {
      auto left_u_turn = junc_passable->add_direction_passable_infos();
      left_u_turn->set_direction(
          hozon::hdmap::JunctionPassable::JUNCTION_LEFT_U_TURN);
      left_u_turn->set_passable_type(
          ColorToPassable(tlr->passable_info().turn_around()));
    }
  }

  if (tlr->passable_info().has_turn_left()) {
    if (tlr->passable_info().turn_left() != hozon::perception::NONE &&
        tlr->passable_info().turn_left() !=
            hozon::perception::NONE_NO_CROSSROAD) {
      auto left_turn = junc_passable->add_direction_passable_infos();
      left_turn->set_direction(
          hozon::hdmap::JunctionPassable::JUNCTION_LEFT_TURN);
      left_turn->set_passable_type(
          ColorToPassable(tlr->passable_info().turn_left()));
    }
  }

  if (tlr->passable_info().has_straight()) {
    if (tlr->passable_info().straight() != hozon::perception::NONE &&
        tlr->passable_info().straight() !=
            hozon::perception::NONE_NO_CROSSROAD) {
      auto forward = junc_passable->add_direction_passable_infos();
      forward->set_direction(hozon::hdmap::JunctionPassable::JUNCTION_FORWARD);
      forward->set_passable_type(
          ColorToPassable(tlr->passable_info().straight()));
    }
  }

  if (tlr->passable_info().has_turn_right()) {
    if (tlr->passable_info().turn_right() != hozon::perception::NONE &&
        tlr->passable_info().turn_right() !=
            hozon::perception::NONE_NO_CROSSROAD) {
      auto right_turn = junc_passable->add_direction_passable_infos();
      right_turn->set_direction(
          hozon::hdmap::JunctionPassable::JUNCTION_RIGHT_TURN);
      right_turn->set_passable_type(
          ColorToPassable(tlr->passable_info().turn_right()));
    }
  }

  return 0;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
