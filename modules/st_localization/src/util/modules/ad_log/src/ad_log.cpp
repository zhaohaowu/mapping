
/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Guo Zhichong <guozhichong@sensetime.com>
 */

#include <string>

#include "ad_log/ad_log.hpp"

namespace senseAD {

void LoggingHelper::InitLogging(const std::string log_dir,
                                const char* prog_name,
                                const std::string symbol_link_file_basename) {
    LoggingHelper::Instance()->Init(log_dir, prog_name,
                                    symbol_link_file_basename);
}
LoggingHelper* LoggingHelper::Instance() {
    static LoggingHelper helper;
    return &helper;
}

void LoggingHelper::Init(const std::string log_dir,
                         const char* prog_name,
                         const std::string symbol_link_file_basename) {}

}  // namespace senseAD
