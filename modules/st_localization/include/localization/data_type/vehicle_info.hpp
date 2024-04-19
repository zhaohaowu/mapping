/*
 * Copyright (C) 2018 by SenseTime Group Limited. All rights reserved.
 * Zhao Jinan <zhaojinan@sensetime.com>
 */
#pragma once

#include <string>

namespace senseAD {
namespace localization {

typedef struct StructCanbusDataType {
  typedef enum {
    GEAR_NONE = 0,
    GEAR_PARK = 1,
    GEAR_REVERSE = 2,
    GEAR_NEUTRAL = 3,
    GEAR_DRIVE = 4,
    GEAR_LOW = 5
  } GEAR;

  typedef enum {
    TURN_SIGNAL_NONE = 0,
    TURN_SIGNAL_LEFT = 1,
    TURN_SIGNAL_RIGHT = 2
  } TURN_SIGNAL;

  GEAR gear;
  TURN_SIGNAL turn_signal;
  // real throttle percent of range [0.0, 100.0]
  float throttle;
  // real brake percent of range [0.0, 100.0]
  float brake;
  // real steering angular in radius per second [-8.203, 8.203], anticlockwise
  // is positive
  float steering_angle = 0.0;
  float steering_torque;
  float vehicle_speed = 0.0;
  // real wheel speed in radius per second [0, 655.35]
  // name: wheel_speed_[front|rear][left|right]
  float wheel_speed_fl;
  float wheel_speed_fr;
  float wheel_speed_rl;
  float wheel_speed_rr;
  // real acceleration of three directions in meter per seconds squard
  // [-327.68, 327.68]
  float acceleration_lat = 0.0;
  float acceleration_long = 0.0;
  float acceleration_vert;
  // real roll angular rates in radius per second [-6.5536, 6.5534]
  float angular_roll_rate;
  // real yaw angular rates in radius per second [-6.5536, 6.5534],
  // anticlockwise is positive
  float angular_yaw_rate;
  // real suspension height in millimeter [-200.391424, 200.391424]
  // name: suspension_[front|rear][left|right]
  float suspension_fl;
  float suspension_fr;
  float suspension_rl;
  float suspension_rr;
  // real tire pressure in [kPa]
  // name: tire_pressure_[front|rear][left|right]
  float tire_pressure_fl;
  float tire_pressure_fr;
  float tire_pressure_rl;
  float tire_pressure_rr;
  // real fuel level in percentage [0, 100]
  float fuel_level;
} VehicleInfo;

inline std::string GearStr(const VehicleInfo::GEAR gear) {
  switch (gear) {
    case VehicleInfo::GEAR_NONE:
      return "NONE";
    case VehicleInfo::GEAR_PARK:
      return "PARK";
    case VehicleInfo::GEAR_REVERSE:
      return "REVERSE";
    case VehicleInfo::GEAR_NEUTRAL:
      return "NEUTRAL";
    case VehicleInfo::GEAR_DRIVE:
      return "DRIVE";
    case VehicleInfo::GEAR_LOW:
      return "LOW";
    default:
      return "UNKNOWN";
  }
}

inline std::string GearStr(const VehicleInfo& info) {
  return GearStr(info.gear);
}

typedef struct StructVehicleStat {
  std::string VehicleType;
  // auto drive status, 0: manual, 1: auto, 2: error
  uint8_t AutoCtrlStat;
  // Acc status, 0: manual, 1: auto
  uint8_t AccCtrlStat;
  // Brk status, 0: manual, 1: auto
  uint8_t BrkCtrlStat;
  // Str status, 0: manual, 1: auto
  uint8_t StrCtrlStat;
  // gear, 0: null, 1: P, 2: R, 3: N, 4: D, 5: L
  uint8_t GearAct;
  uint8_t GearCmd;
  // door status, false: closed, true: open
  bool DoorFL;
  bool DoorFR;
  bool DoorRL;
  bool DoorRR;
  bool DoorTrunk;
} VehicleStat;

}  // namespace localization
}  // namespace senseAD
