/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#pragma once

#include <fstream>
#include <string>

#include "common/path_util.hpp"
#include "localization/common/log.hpp"
#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {

template <typename SerialData>
class EvaluatorBase {
 public:
  EvaluatorBase() = default;
  virtual ~EvaluatorBase() { output_stream_.close(); }

  adLocStatus_t Init(const std::string& results_save_directory,
                     const std::string& setting_type,
                     const std::string& testcase_id) {
    if (!FileSystem::IsDir(results_save_directory) &&
        false == FileSystem::MakeDirs(results_save_directory)) {
      LC_LERROR(EVAL) << "cannot create results save directory: "
                      << results_save_directory;
      return LOC_LOCALIZATION_ERROR;
    }

    if (output_stream_.is_open()) output_stream_.close();

    // open file
    std::string result_file_name = testcase_id + "-" + setting_type + ".bin";
    std::string output_file =
        JOIN_PATH(results_save_directory, result_file_name);
    output_stream_.open(output_file,
                        std::ios::trunc | std::ios::out | std::ios::binary);

    if (!output_stream_.is_open()) {
      LC_LERROR(EVAL) << "evaluator cannot open file: " << output_file;
      return LOC_LOCALIZATION_ERROR;
    }

    return LOC_SUCCESS;
  }

  virtual adLocStatus_t WriteResult(double timestamp,
                                    const SerialData& data) = 0;

 protected:
  std::ofstream output_stream_;
};

}  // namespace localization
}  // namespace senseAD
