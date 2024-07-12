/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: hozon
 *Date: 2024-07-08
 *****************************************************************************/
#include "phm_comment_lite/phm_unit_test.h"

namespace hozon {
namespace perception {
namespace common_onboard {

TEST_F(PhmCommentTest, phm_function_check) {
  HLOG_INFO << " phm_init gtest start ";
  // 1.check Init InitFault InitHealth
  bool res = phm_comment_->Init();
  EXPECT_TRUE(res);
  HLOG_INFO << " res " << res;

  // 2.check GetFilePath
  std::string path = "yaml_file";
  phm_comment_->GetFilePath(path);
  res = (path.find("config.yaml") != std::string::npos);
  std::cout << " contain config.yaml res " << res << std::endl;
  EXPECT_TRUE(res);
  HLOG_INFO << " file_path path " << path;

  // 3.check FaultReport
  res = phm_comment_->FaultReport(5020, 3, 1, 3, 1000);
  std::cout << " FaultReport res " << res << std::endl;
  EXPECT_TRUE(res);

  // 4.check ReportCheckPointId
  res = phm_comment_->ReportCheckPointId(10);
  std::cout << " ReportCheckPointId res " << res << std::endl;
  EXPECT_TRUE(res);

  // 5.check ResetFault
  phm_comment_->ResetFault();
  int size_num = phm_comment_->faultmap_.size();
  std::cout << " faultmap_ size " << size_num << std::endl;
  EXPECT_EQ(size_num, 0);

  // 6.check comment Name
  std::string comment_name = phm_comment_->Name();
  std::cout << " comment_name " << comment_name << std::endl;
  res = (comment_name == "MappingComponent");
  EXPECT_TRUE(res);
}

}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
