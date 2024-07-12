/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2024-07-08
 *****************************************************************************/
#include "adf/include/log.h"
#include "gflags/gflags.h"
#include "gtest/gtest.h"

extern "C" void RegisterPhmCommentTests();
extern "C" void RegisterDrTests();
extern "C" void Registerlaneline_postprocess_tests();
extern "C" void Registerlocal_mapping_tests();
extern "C" void Registerfusion_center_tests();
extern "C" void Registerins_fusion_tests();
extern "C" void Registerpose_estimation_tests();
extern "C" void Registercoord_adapter_tests();
extern "C" void Registermap_fusion_tests();


int main(int argc, char* argv[]) {
  hozon::netaos::log::InitLogging("mal_ut", "mal_ut",
                                  hozon::netaos::log::LogLevel::kInfo,
                                  HZ_LOG2FILE, "./", 10, (20));
  hozon::netaos::adf::NodeLogger::GetInstance().CreateLogger(
      "mal_ut", "mal_ut test", hozon::netaos::log::LogLevel::kInfo);

  const char* var = std::getenv("ADFLITE_ROOT_PATH");
  if (var == nullptr) {
    std::cerr << "can not get ADFLITE_ROOT_PATH" << std::endl;
    return -1;
  }
  std::string work_root = std::string(var);
  std::string gflag_file =
      work_root + "/runtime_service/mapping/conf/lite/mapping_config.flags";
  gflags::ReadFromFlagsFile(gflag_file, "", false);

  RegisterPhmCommentTests();
  RegisterDrTests();
  Registerlaneline_postprocess_tests();
  Registerlocal_mapping_tests();
  Registerfusion_center_tests();
  Registerins_fusion_tests();
  Registerpose_estimation_tests();
  Registercoord_adapter_tests();
  Registermap_fusion_tests();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
