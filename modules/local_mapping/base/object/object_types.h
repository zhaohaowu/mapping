/*================================================================
*   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
*   file       ：object_types.h
*   author     ：WuXiaopeng
*   date       ：2021.11.11
================================================================*/

#pragma once

#include <map>
#include <string>

namespace hozon {
namespace mp {
namespace lm {

// @brief general object type
enum class ObjectType {
  UNKNOWN = 0,
  UNKNOWN_UNMOVABLE = 1,
  UNKNOWN_MOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  VEHICLE = 5,
  CYCLIST = 6,            // 骑行者
  STATIC_OBSTACLE = 7,    // 静态障碍物
  TRANSPORT_ELEMENT = 8,  // 交通元素
  ANIMAL = 9,             // 动物
};

// @brief fine-grained object types
enum class ObjectSubType {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 100,
  UNKNOWN_UNMOVABLE = 200,

  PEDESTRIAN = 300,  // 人

  BUGGY = 400,           // 婴儿车，童车
  BICYCLE = 401,         // 自行车
  ELETRICBICYCLE = 402,  // 电动自行车
  MOTORCYCLE = 403,      // 摩托车
  TRICYCLE = 404,        // 三轮车
  HANDCAR = 405,         // 手推车 CART = 2

  CAR = 500,
  VAN = 501,         // 厢式货车
  TRUCK = 502,       // 卡车，中型，
  BIG_TRUCK = 503,   // 大卡车
  BUS = 504,         // 公交车
  MIDDLE_BUS = 505,  // 中型客车
  MINIBUS = 506,     // 面包车，MPV
  PICKUP = 507,      // 皮卡
  AMBULANCE = 508,   // 救护车
  POLICECAR = 509,   // 警车
  FIRE_ENGINE = 510,
  SPECIAL_CAR = 511,  // 特种车

  CYCLIST = 600,       // 骑自行车的人
  MOTORCYCLIST = 601,  // 骑摩托车的人
  TRICYCLIST = 602,    // 骑三轮车
  EBICYCLIST = 603,    // 骑电动自行车的人

  TRAFFICCONE = 700,            // 交通锥桶
  SPEEDBUMP = 701,              // 减速带
  FENCE = 702,                  // 篱笆，栅栏，护栏
  BARRIER_PARKING_LEVER = 703,  // 停车杆
  WATERHORSE = 704,             // 水马  BARRIER_WATER
  CRASHBARRELS = 705,           // 防撞桶  // BARRIER_BUCKET
  SIGNBOARD = 706,              // 施工牌
  WARNINGTRIANGLE = 707,        // 三角警示牌
  STONEBLOCK = 708,             // 石墩（圆,椭圆,球形)
  COLUMN = 709,                 // 交通立柱
  PAPER_BOX = 710,              // 纸箱
  BARRIER = 711,                // 路障
  PARKINGSIGN = 712,            // 泊车牌
  FIREHYDRANT = 713,            // 消防栓
  WHEELSTOP = 714,              // 轮挡  BARRIER_WHEEL_STOPPER
  LOCKER = 715,                 // 地锁  BARRIER_PARKING_LOCK
  TRASH = 716,                  // 垃圾桶
  PILLAR = 717,                 // 车库大柱子 BARRIER_PILLAR
  BARRIER_GAT = 718,            // 闸机

  BARRIER_SIGN = 800,
  BARRIER_TRIANGLE = 801,         // 三角警示牌
  STOP = 802,                     // 停车让行
  SLOWYIELD = 803,                // 减速让行
  NOPASS = 804,                   // 禁止通行
  NOENTRY = 805,                  // 禁止驶入
  NOTURNINGLEFT = 806,            // 禁止向左转弯
  NOTURNINGRIGHT = 807,           // 禁止向右转弯
  NOGOINGSTRAIGHT = 808,          // 禁止直行
  NOTURNINGAROUND = 809,          // 禁止掉头
  NOOVERTAKING = 810,             // 禁止超车
  REMOVENOOVERTAKING = 811,       // 解除禁止超车
  NOPARKING = 812,                // 禁止车辆停放
  NOHONKING = 813,                // 禁止鸣喇叭
  SPEEDLIMITLIFTED = 814,         // 限速标志
  SPEEDRELEASELIMITLIFTED = 815,  // 解除限速标志
  TRAFFICLIGHT = 816,             // 交通灯
  BAN = 817,                      // 禁停牌
  D_ARROW = 818,                  // 直行箭头
  L_ARROW = 819,                  // 左转箭头
  R_ARROW = 820,                  // 右转箭头
  A_ARROW = 821,                  // 掉头箭头
  ZEBRA = 822,                    // 斑马线
  STOP_LINE = 823,                // 停止线

  CAT = 900,
  DOG = 901,

  USS = 1000,  // 超声波障碍物
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
