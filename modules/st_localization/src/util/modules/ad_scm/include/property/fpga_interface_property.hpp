/**
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Zhu Haibo <zhuhaibo@sensetime.com>
 */
#pragma once

#include <string>

namespace senseAD {
namespace fpga {

class NetworkProperty {
 public:
    std::string control_connection_ip_addr;
    std::string data_connection_ip_addr;
};

class HostProperty {
 public:
    NetworkProperty network_property;
};

}  // namespace fpga
}  // namespace senseAD
