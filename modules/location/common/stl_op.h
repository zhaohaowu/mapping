/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： stl_op.h
 *   author     ： lilanxing
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once

namespace hozon {
namespace mp {
namespace loc {
namespace cm {

template <typename T>
void ShrinkQueue(T* const queue, uint32_t capacity) {
  if (!queue) {
    return;
  }
  while (queue->size() > capacity) {
    queue->pop_front();
  }
}

}  // namespace cm
}  // namespace loc
}  // namespace mp
}  // namespace hozon
