/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： percep_obj_manager.h
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <memory>

#include <boost/circular_buffer.hpp>

#include "depend/proto/perception/perception_obstacle.pb.h"
#include "perception-base/base/utils/macros.h"

namespace hozon {
namespace mp {
namespace mf {

class PercepObjManager {
 public:
  ~PercepObjManager() = default;
  bool Init();
  bool PushObjects(
      const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg);

  boost::circular_buffer<std::shared_ptr<Object>> GetHistoryObjs() const;
  boost::circular_buffer<std::shared_ptr<Object>> GetInverseHistoryObjs() const;

 private:
  // 类初始化相关
  bool inited_ = false;
  std::mutex mutex_;
  boost::circular_buffer<std::shared_ptr<Object>> history_objs_;
  int history_objs_size_ = 10;
  boost::circular_buffer<std::shared_ptr<Object>> inverse_history_objs_;
  int inverse_history_objs_size_ = 20;
  // get instance by Instance()
  DECLARE_SINGLETON_PERCEPTION(PercepObjManager)
};

#define OBJECT_MANAGER PercepObjManager::Instance()

}  // namespace mf
}  // namespace mp
}  // namespace hozon
