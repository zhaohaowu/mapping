/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： data_verify.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

namespace hozon {
namespace mp {
namespace loc {
namespace cm {

template <typename T>
bool HasValidHeader(const T& pb) {
  // seq not used on orin
  // if (!pb.has_header() || !pb.header().has_seq() ||
  //     !pb.header().has_gnss_stamp()) {
  //   return false;
  // }
  // if (pb.header().seq() < 0 || pb.header().gnss_stamp() < 1e-3) {
  //   return false;
  // }
  if (!pb.has_header()) {
    return false;
  }
  // if (!pb.has_header() || !pb.header().has_publish_stamp()) {
  //   return false;
  // }
  // if (pb.header().publish_stamp() < 1e-3) {
  //   return false;
  // }
  return true;
}

}  // namespace cm
}  // namespace loc
}  // namespace mp
}  // namespace hozon
