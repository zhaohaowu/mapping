/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-15
 *****************************************************************************/
#include "modules/local_mapping/utils/fetch_hq.h"

namespace hozon {
namespace mp {
namespace lm {

int PriorProvider::Init(const std::string& map_file_path) {
  std::ifstream map_stream(map_file_path, std::ios::in | std::ios::binary);
  if (!map_stream) {
    HLOG_ERROR << "Failed to open " << map_file_path;
    map_stream.close();
    return -1;
  }

  prior_ = std::make_shared<hozon::hdmap::Map>();
  if (!prior_->ParseFromIstream(&map_stream)) {
    HLOG_ERROR << "Parse map proto failed";
    map_stream.close();
    return -1;
  }

  return 0;
}

std::shared_ptr<hozon::hdmap::Map> PriorProvider::GetPrior() { return prior_; }

}  // namespace lm
}  // namespace mp
}  // namespace hozon
