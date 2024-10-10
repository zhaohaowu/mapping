/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_service_fault.h
 *   author     ： mashaoping
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once

namespace hozon {
namespace mp {
namespace mf {

enum MapServiceFault {
  // 无故障
  MS_NO_ERROR = -1,
  // 定位输入错误
  MS_INPUT_LOC_ERROR = 0,
  // ehp初始化失败
  MS_EHP_INIT_ERROR = 1,
  // 地图数据空间不足
  MS_MAP_DATA_ROM_ERROR = 2,
  // UUID错误(位数不够或为空)
  MS_UUID_ERROR = 3,
  // 高德sdk内部故障
  MS_SDK_INNER_ERROR = 4,
  // 地图数据存储路径异常
  MS_MAP_DATA_PATH_ERROR = 5,
  // 地图数据路径读写权限异常
  MS_PATH_RW_ERROR = 6,
  // 高德sdk未激活或激活失效
  MS_SDK_ACTIVE_ERROR = 7,
  // 高德sdk初始化失败
  MS_SDK_INIT_FAIL = 8
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
