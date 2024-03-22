// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: simple_lane_tracker.cc
// @brief: container for filter
#pragma once

#include <cstring>
#include <iostream>

namespace hozon {
namespace mp {
namespace environment {

void met_lld_AddMat(double* A, double* B, double* C, int num_r, int num_c);

void met_lld_TransposeMat(double* A, int num_r, int num_c, double* AT);

void met_lld_Mult(double* A, int num_row_A, int num_col_A, double* X,
                  int num_row_X, int num_col_X, double* B);

void met_lld_fitPoly(double* x_data, double* y_data, int x_size, int prevOrder,
                     double poly[4]);

void met_lld_TransposeMat_Float(float* A, int num_r, int num_c, float* AT);

void met_lld_fitPolyThirdDegree(double* x_data, double* y_data, int x_size,
                                double poly[4]);

void met_lld_fitPolySecondDegree(double* x_data, double* y_data, int x_size,
                                 double poly[3]);

void met_lld_fitPolySecondDegree(float* x_data, float* y_data, int x_size,
                                 float poly[3]);

void met_lld_InvMat_Float(float* invA, float* A, int n);

void met_lld_fitPolyFirstDegree(double* x_data, double* y_data, int x_size,
                                double poly[3]);

void met_lld_InvMat(double* invA, double* A, int n);

void met_lld_Mult_Float(float* A, int num_row_A, int num_col_A, float* X,
                        int num_row_X, int num_col_X, float* B);

}  // namespace environment
}  // namespace mp
}  // namespace hozon
