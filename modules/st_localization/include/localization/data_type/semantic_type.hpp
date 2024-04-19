/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Chen Longquan <chenlongquan1@senseauto.com>
 * Zhao Ming <zhaoming@senseauto.com>
 */
#pragma once

#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

using id_t = int64_t;

enum class CoordinateType {
  VEHICLE = 0,    // vehicle coordinate
  ENU = 2,        // ENU coordinate
  WGS = 3,        // WGS coordinate, xyz -> Longitude,Latitude,height
  LOCAL = 4,      // local coordinate
  UTM = 5,        // UTM coordinate
  Curvature = 6,  // curvature coordinate
  Image = 7,      // image coordinate
  GCJ02 = 8,      // GCJ02 coordinate, xyz -> Longitude,Latitude,height
};

static inline CoordinateType GetCoordTypeEnum(uint8_t value) {
  if (value == 0) {
    return CoordinateType::VEHICLE;
  } else if (value == 2) {
    return CoordinateType::ENU;
  } else if (value == 3) {
    return CoordinateType::WGS;
  } else if (value == 4) {
    return CoordinateType::LOCAL;
  } else if (value == 5) {
    return CoordinateType::UTM;
  } else if (value == 6) {
    return CoordinateType::Curvature;
  } else if (value == 7) {
    return CoordinateType::Image;
  } else if (value == 8) {
    return CoordinateType::GCJ02;
  } else {
    LC_LERROR(LOCALIZATION) << "unsupported coordinate type.";
    return CoordinateType::VEHICLE;
  }
}

///////////////////////// semantic related type ////////////////////////////////

struct EnumClassHash {
  template <typename T>
  std::size_t operator()(T t) const {
    return static_cast<std::size_t>(t);
  }
};

// SemanticType
enum class SemanticType {
  NONE = 0,
  LaneLine = 1,
  StopLine = 2,
  Pole = 3,
  TrafficSign = 5
};

// Color
enum class Color {
  NONE = 0,
  White = 1,
  Yellow = 2,
  Orange = 3,
  Blue = 4,
  Green = 5,
  Gray = 6,
  LeftGrayRightYellow = 7,
  LeftYellowRightWhite = 8,
  Other = 255,
};

// LineType
enum class LineType {
  Unknown = 0,
  Centerline = 1,
  LaneMarking = 2,
  Guardrail = 3,
  Fence = 4,
  Curb = 5,
  Wall = 6,
  Concrete = 7,
  Nature = 8,
  Canopy = 9,
  Virtual = 10,
  Clift = 11,
  Ditch = 12,
  Other = 255,
};

// LineStyle
enum class LineStyle {
  Unknown = 0,
  SingleSolid = 1,
  SingleDashed = 2,
  DoubleSolid = 3,
  DoubleDashed = 4,
  LeftSolidRightDashed = 5,
  RightSolidLeftDashed = 6,
  ShortDashed = 7,
  LongDashed = 8,
  FishBone = 9,
  ShadedArea = 10,
  LaneVirtualMarking = 11,
  IntersectionVirualMarking = 12,
  CurbVirtualMarking = 13,
  UnclosedRoad = 14,
  RoadVirtualLine = 15,
  Other = 255,
};

inline static bool IsRoadSideLine(LineType type) {
  return type == LineType::Curb || type == LineType::Concrete ||
         type == LineType::Fence || type == LineType::Guardrail ||
         type == LineType::Other;
}

inline static bool IsLaneMarkingLine(LineType type) {
  return type == LineType::LaneMarking;
}

inline static bool IsDashedLine(LineStyle style) {
  return style == LineStyle::SingleDashed || style == LineStyle::DoubleDashed ||
         style == LineStyle::LeftSolidRightDashed ||
         style == LineStyle::RightSolidLeftDashed ||
         style == LineStyle::ShortDashed || style == LineStyle::LongDashed;
}

enum class PoleType {
  Unknown = 0,
  Gantry = 1,
  SignPost = 2,
  Signal = 3,
  Other = 255,
};

enum class TrafficSignType {
  Unknown = 0,
  RoadWorks = 1,
  Stop = 2,
  OvertakingProhibited = 3,
  EndOfProhibitionOnOvertaking = 4,
  ChildrenAndSchoolZone = 5,
  MinSpeedLimit = 6,
  MaxSpeedLimit = 7,
  EndOfSpeedLimit = 8,
  NoEntrance = 9,
  AllSpeedLimitCancel = 10,
  NoParkingSign = 11,
  StartOfHighway = 12,
  EndOfHighway = 13,
  LeftCurve = 14,
  RightCurve = 15,
  SeriesCurves = 16,
  Others = 17,
};

//////////////////////////// route related type ////////////////////////////////

enum class LinkType {
  Unknown = 0,           // 未知类型
  Continue = 1,          // 正常 基本直行  等同于Straight
  Split = 2,             // 分裂
  Merge = 3,             // 合并
  WideStepByStep = 4,    // 渐宽车道
  NarrowStepByStep = 5,  // 渐窄车道
  OpeningAndEnding = 6,  // 渐宽后渐窄（最窄处不足以车辆通行）
  EndingAndOpening = 7,  // 渐窄后渐宽（最窄处不足以车辆通行）
  TurnLeft = 8,
  TurnRight = 9,
  TurnLeftAround = 10,
  TurnRightAround = 11,
  TurnLeftArea = 12,  // 左转待转区
  Other = 255,
};

enum class LaneType {
  Unknown = 0,                // 未知车道类型
  Normal = 1,                 // 常规车道（无明显特征）
  Express = 2,                // 快速车道
  Slow = 3,                   // 慢速车道
  Emergency = 4,              // 应急车道
  EmergencyParkingStrip = 5,  // 紧急停车带
  Climbing = 6,               // 爬坡车道
  Escape = 7,                 // 避险车道
  Entry = 8,                  // 驶入车道（进入高速主路的车道）
                              // 通常后续会紧接着出现加速车道
  Exit = 9,                   // 驶出车道（离开高速/主路的车道）
                              // 通常之前会紧接着出现减速车道
  Accelerate = 10,            // 加速车道
  Decelerate = 11,            // 减速车道
  Ramp = 12,                  // 无特征类型匝道
  RampAbord = 13,             // 上行匝道（驶入高速的匝道）
  RampAshore = 14,            // 下行匝道（驶离高速的匝道）
  RampJCT = 15,               // 连结匝道（连接高速与高速）
  TollBooth = 16,             // 收费站车道
  NonDriveway = 17,          // 不可行驶车道（可紧急情况下占用）
  DedicatedBus = 18,         // 公交专用车道
  DedicatedCustom = 19,      // 其他专用车道
  Reversible = 20,           // 潮汐车道
  Variable = 21,             // 可变车道
  DrivableShoulder = 22,     // 可行驶露肩
  DisDrivableShoulder = 23,  // 不可行驶露肩
  Bicycle = 24,              // 非机动车道
  Overtaking = 25,           // 超车道
  HOV = 26,                  // HOV车道
  UTurn = 27,                // 掉头车道
  Intersection = 28,         // 路口车道
  BicycleOrVehicle = 29,     // 机非混合车道
  SideWalk = 30,             // 人行道
  Motorcycle = 31,           // 摩托车道
  ETC = 32,                  // ETC车道
  CheckPoint = 33,           // 检查站车道
  IsolationStrip = 34,       // 隔离带车道
  DiversionStrip = 35,       // 导流带车道
  CustomsSupervision = 36,   // 海关监管车道
  Parking = 37,              // 停车道
  Temporary = 38,            // 临时车道
  TurnLeftWait = 39,         // 左转待转车道
  StraightWait = 40,         // 直行待行车道
  LeftAccelerate = 41,       // 左侧加速车道
  RightAccelerate = 42,      // 右侧加速车道
  LeftDecelerate = 43,       // 左侧减速车道
  RightDecelerate = 44,      // 右侧减速车道
  Service = 45,              // 服务区车道
  Curb = 46,                 // 路缘带
  RailBus = 47,              // 有轨（公交）车道
  BusStop = 48,              // 公交车停靠车道
  Roundabout = 49,           // 环岛车道
  Invalid = 50,              // 无效车道
  Wide = 51,                 // 宽车道
  Virtual = 52,              // 虚拟

  // 复合类型车道
  ParkingAndBicycleOrVehicle = 53,
  // 停车道 + 机非混合车道
  ServiceAndRamp = 54,     // 服务器车道 + 匝道
  BusStopAndBicycle = 55,  // 公交车停靠车道 + 非机动车道
  BusStopAndBicycleOrVehicle = 56,
  // 公交车停靠车道 + 机非混合车道
  BusStopAndDedicatedBus = 57,
  // 公交车停靠车道 + 公交车道
  Other = 255,  // 其他
};

enum class TransType {
  Unknown = 0,           // 未知类型
  Continue = 1,          // 正常 基本直行
  Split = 2,             // 分裂
  Merge = 3,             // 合并
  WideStepByStep = 4,    // 渐宽车道
  NarrowStepByStep = 5,  // 渐窄车道
  OpeningAndEnding = 6,  // 渐宽后渐窄（最窄处不足以车辆通行）
  EndingAndOpening = 7,  // 渐窄后渐宽（最窄处不足以车辆通行）
  Other = 255,
};

enum class SectionClass {
  Unknown = 0,          // 未知
  Expressway = 1,       // 高速
  Freeway = 2,          // 城快
  Regular = 3,          // 城市常规道路
  National = 4,         // 国道
  Provincial = 5,       // 省道
  Major = 6,            // 主要道路
  Minor = 7,            // 次要道路
  Normal = 8,           // 普通道路
  County = 9,           // 县道
  Village = 10,         // 乡道
  CountyInternal = 11,  // 县乡村内部道路
  Other = 255,          // 其他
};

enum class SectionType {
  Unknown = 0,       // 未知道路
  Main = 1,          // 主路
  Auxiliary = 2,     // 辅路
  EntryRamp = 3,     // 入口匝道
  ExitRamp = 4,      // 出口匝道
  Ramp = 5,          // 常规匝道
  JCT = 6,           // 连接高速与高速的联通匝道
  Service = 7,       // 高速/城块服务区内部道路
  Intersection = 8,  // 城区路口
  Roundabout = 9,    // 城区环岛道路
  Pedestrian = 10,   // 城区人行道路
  Parallel = 11,     // 平行道路，分叉后又会合并
  TollBooth = 12,    // 收费站（包括收费口、收费口前/后）
  Tunnel = 13,       // 隧道
  Bridge = 14,       // 桥梁
  Viaduct = 15,      // 高架桥
  TollGate = 16,     // 收费站
  TollEntry = 17,    // 收费站入口
  TollExit = 18,     // 收费站出口
  Breakup = 19,      // 断头路
  MultipleCarriageWay = 20,       // 多车道道路？or 双向行车道？
  SingleCarriageWay = 21,         // 单车道道路？or 单向行车道？
  CrossLink = 22,                 // 交叉点内
  SlipRoad = 23,                  // 引路
  SideRoad = 24,                  // 辅路
  SlipAndJCT = 25,                // 引路 + JCT
  TurnRightLineA = 26,            // 右转车道A
  TurnRightLineB = 27,            // 右转车道B
  TurnLeftLineA = 28,             // 左转车道A
  TurnLeftLineB = 29,             // 左转车道B
  TurnLeftRightLine = 30,         // 左右转车道
  ServiceAndSlipRoad = 31,        // 服务区 + 引路
  ServiceAndJCT = 32,             // 服务区+JCT
  ServiceAndSlipRoadAndJCT = 33,  // 服务区+引路+JCT
  Other = 255,                    // 其他
};

}  // namespace localization
}  // namespace senseAD
