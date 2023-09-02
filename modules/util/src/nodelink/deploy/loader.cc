/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： loader.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.01
 ******************************************************************************/

#include "util/nodelink/deploy/loader.h"

#include <dlfcn.h>

#include <filesystem>

#include "util/temp_log.h"

namespace hozon {
namespace mp {

namespace fs = std::filesystem;

int Loader::Load(const std::vector<std::string>& lib_paths) {
  for (const auto& p : lib_paths) {
    fs::path path(p);
    if (!fs::exists(p)) {
      HLOG_ERROR << p << " not exist";
      return -1;
    }

    fs::path canon_path = fs::canonical(path);
    if (lib_handles_.find(canon_path.string()) == lib_handles_.end()) {
      void* handler = dlopen(canon_path.c_str(), RTLD_NOLOAD);
      if (handler != nullptr) {
        HLOG_INFO << canon_path.string() << " already loaded, skip";
        continue;
      }

      handler = dlopen(canon_path.c_str(), RTLD_LAZY);
      if (handler == nullptr) {
        HLOG_ERROR << "dlopen " << canon_path.string() << " failed";
        const char* err = dlerror();
        if (err) {
          HLOG_ERROR << "dlerror: " << std::string(err);
        }
        return -1;
      }
      HLOG_INFO << canon_path.string() << " loaded";
      lib_handles_[canon_path.string()] = handler;
    }
  }
  return 0;
}

void Loader::Term() {
  //! TBD: 按理说这里应该把每个lib handle都dlclose,
  //! 但由于每个Node注册时都生成了一个 全局变量, 因此这里如果直接dlclose,
  //! 全局变量是最后析构的, 那么那些全局变量会有问题, 因此这里选择什么都不做,
  //! 依赖操作系统在进程结束时关闭各个lib. 后面想想有没有更好的办法.
}

}  // namespace mp
}  // namespace hozon