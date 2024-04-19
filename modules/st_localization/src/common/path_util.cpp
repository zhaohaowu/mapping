/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * WEI Zhengyuan<weizhengyuan@sensetime.com>
 * GUO Zhichong<guozhichong@sensetime.com>
 */
#include "common/path_util.hpp"
#include <dirent.h>  // DIR/opendir/dirent/readdir

#include <cstdlib>  // getenv

namespace senseAD {
namespace localization {

int FileSystem::FileStatus(const std::string& path) {
  struct stat sb;
  if (stat(path.c_str(), &sb) == -1) {
    return -1;
  }
  switch (sb.st_mode & S_IFMT) {
    case S_IFDIR:
      return 2;
    case S_IFREG:
      return 1;
    default:
      return 0;
  }
  return 0;
}

std::string FileSystem::CurrentDir() {
  char buff[1024];
  ssize_t len = readlink("/proc/self/exe", buff, sizeof(buff) - 1);
  if (len != -1) {
    buff[len] = '\0';
    std::string temp(buff);
    return temp.substr(0, temp.find_last_of('/'));
  }
  return std::string("");
}

bool FileSystem::MakeDirs(const std::string& dir) {
  // A fastest version implemented first
  std::string command = std::string("mkdir -p ") + dir;
  int nerrno = system(command.c_str());
  return 0 == nerrno;
}

}  // namespace localization
}  // namespace senseAD
