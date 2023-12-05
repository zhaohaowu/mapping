/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： Bwlabel.h
 *   author     ： Nihongjie
 *   date       ： 2023.12
 ******************************************************************************/
#pragma once
#include <math.h>

#include <vector>

class BwLbel {
 public:
  BwLbel();
  ~BwLbel();
  std::vector<std::vector<int>> bwlabel(
      const std::vector<std::vector<int>>& frechet_compare_mat);
  std::vector<std::vector<int>> number_of_group(
      const std::vector<std::vector<int>>& frechet_compare_mat,
      std::vector<std::vector<int>>& frechet_group_mat);
  void group_mat_process(
      std::vector<std::vector<int>>& number_of_group,
      const std::vector<std::vector<int>>& frechet_group_mat);
  void group_row_process(const std::vector<int>& previous_row,
                         const std::vector<int>& curr_row,
                         std::vector<std::vector<int>>& number_of_group);
  std::vector<int> findNeighbor(size_t index,
                                const std::vector<int>& neighbor_row);
  void group_num_change(std::vector<std::vector<int>>& number_of_group,
                        std::vector<std::vector<int>>& frechet_group_mat);
  void num_trace(std::vector<std::vector<int>>& number_of_group, int pre_num,
                 int tar_num);
  void unique_vector(std::vector<int>& num_array);
  void num_change(std::vector<std::vector<int>>& number_of_group,
                  int origin_num, int target_num);
};
