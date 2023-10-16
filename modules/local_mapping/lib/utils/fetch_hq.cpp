/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-15
 *****************************************************************************/
#include "modules/local_mapping/lib/utils/fetch_hq.h"

#include <yaml-cpp/yaml.h>

#include <fstream>

#include "modules/util/include/util/temp_log.h"

namespace hozon {
namespace mp {
namespace lm {

int PriorProvider::Init(const std::string& conf) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(conf);
  } catch (YAML::BadFile& ex) {
    HLOG_ERROR << "load yaml " << conf << " failed: " << ex.what();
    return -1;
  } catch (YAML::ParserException& ex) {
    HLOG_ERROR << "parse yaml " << conf << " failed: " << ex.what();
    return -1;
  }
  std::string key = "map_file";
  if (!root[key]) {
    HLOG_ERROR << key << " not exist in " << conf;
    return -1;
  }

  const auto map_file = root[key].as<std::string>();
  std::ifstream map_stream(map_file, std::ios::in | std::ios::binary);
  if (!map_stream) {
    HLOG_ERROR << "Failed to open " << map_file;
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
