/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * WEI Zhengyuan<weizhengyuan@sensetime.com>
 */
#pragma once

#include <sys/stat.h>  // stat
#include <unistd.h>

#include <string>
#include <vector>

namespace senseAD {
namespace localization {

#define JOIN_PATH(root, path) \
  (root.back() == '/' ? root + path : root + '/' + path)

class FileSystem {
 public:
  /*
   * @brief check the "file"
   * @param path[in] specify the "file" path
   * @return int flag to specify the status of "file".
   *  -1 means invalid;
   *   0 means Neither file nor directory
   *   1 means it's a regular file
   *   2 menas it's a directory
   */
  static int FileStatus(const std::string& path);
  /*
   * @brief check if a regular file
   * @param path[in] specify the "file" path
   * @return true if it's a regular file
   */
  static bool IsFile(const std::string& path) { return 1 == FileStatus(path); }
  /*
   * @brief check if a directory
   * @param path[in] specify the "file" path
   * @return true if it's a regular directory
   */
  static bool IsDir(const std::string& path) { return 2 == FileStatus(path); }

  /*
   * @brief Get the current working directory
   * @return the current working directory
   */
  static std::string CurrentDir();

  /*
   * @brief call 'mkdir -p' to create directory(ies)
   * @return true if succeed
   */
  static bool MakeDirs(const std::string& dir);
};

}  // namespace localization
}  // namespace senseAD
