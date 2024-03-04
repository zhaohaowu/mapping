// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: simple_lane_tracker.cc
// @brief: container for filter

#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/data_fusion/util.h"

namespace hozon {
namespace mp {
namespace environment {

void met_lld_fitPolyThirdDegree(double* x_data, double* y_data, int x_size,
                                double poly[4]) {
  int i;
  double H_data[512 * 4];
  double H_data_T[512 * 4];
  double H_data_multi[16], H_data_multi_inv[16];
  double temp[512 * 4];
  for (i = 0; i < x_size; i++) {
    H_data[0 + 4 * i] = 1.0F;
    H_data[1 + 4 * i] = x_data[i];
    H_data[2 + 4 * i] = x_data[i] * x_data[i];
    H_data[3 + 4 * i] = x_data[i] * x_data[i] * x_data[i];
  }
  met_lld_TransposeMat(H_data, x_size, 4, H_data_T);
  met_lld_Mult(H_data_T, 4, x_size, H_data, x_size, 4, H_data_multi);
  met_lld_InvMat(H_data_multi_inv, H_data_multi, 4);
  met_lld_Mult(H_data_multi_inv, 4, 4, H_data_T, 4, x_size, temp);
  met_lld_Mult(temp, 4, x_size, y_data, x_size, 1, poly);
  if (poly[3] > 1 || poly[2] > 1) printf("error\n");
}

void met_lld_fitPolySecondDegree(double* x_data, double* y_data, int x_size,
                                 double poly[3]) {
  int i;
  double H_data[512 * 3];
  double H_data_T[512 * 3];
  double H_data_multi[9], H_data_multi_inv[9];
  double temp[512 * 3];
  for (i = 0; i < x_size; i++) {
    H_data[0 + 3 * i] = 1.0F;
    H_data[1 + 3 * i] = x_data[i];
    H_data[2 + 3 * i] = x_data[i] * x_data[i];
  }
  met_lld_TransposeMat(H_data, x_size, 3, H_data_T);
  met_lld_Mult(H_data_T, 3, x_size, H_data, x_size, 3, H_data_multi);
  met_lld_InvMat(H_data_multi_inv, H_data_multi, 3);
  met_lld_Mult(H_data_multi_inv, 3, 3, H_data_T, 3, x_size, temp);
  met_lld_Mult(temp, 3, x_size, y_data, x_size, 1, poly);
}

void met_lld_fitPolySecondDegree(float* x_data, float* y_data, int x_size,
                                 float poly[3]) {
  int i;
  float H_data[512 * 3];
  float H_data_T[512 * 3];
  float H_data_multi[9], H_data_multi_inv[9];
  float temp[512 * 3];
  for (i = 0; i < x_size; i++) {
    H_data[0 + 3 * i] = 1.0F;
    H_data[1 + 3 * i] = x_data[i];
    H_data[2 + 3 * i] = x_data[i] * x_data[i];
  }
  met_lld_TransposeMat_Float(H_data, x_size, 3, H_data_T);
  met_lld_Mult_Float(H_data_T, 3, x_size, H_data, x_size, 3, H_data_multi);
  met_lld_InvMat_Float(H_data_multi_inv, H_data_multi, 3);
  met_lld_Mult_Float(H_data_multi_inv, 3, 3, H_data_T, 3, x_size, temp);
  met_lld_Mult_Float(temp, 3, x_size, y_data, x_size, 1, poly);
}

void met_lld_InvMat_Float(float* invA, float* A, int n) {
  int i, j, k;
  double sum, x;
  for (i = 0; i < n * n; i++) {
    invA[i] = A[i];
  }
  for (i = 0; i < n; i++) {
    j = i * n + i;
    if ((invA[j] < 1e-8f) && (invA[j] > -1e-8f)) {
      invA[j] = 1e-8f;
    }
  }
  if (n <= 1) return;
  for (i = 1; i < n; i++) {
    invA[i] /= invA[0]; /* normalize row 0 */
  }
  for (i = 1; i < n; i++) {
    for (j = i; j < n; j++) { /* do a column of L */
      sum = 0.0;
      for (k = 0; k < i; k++) {
        sum += invA[j * n + k] * invA[k * n + i];
      }
      invA[j * n + i] -= sum;
    }
    if (i == n - 1) continue;
    for (j = i + 1; j < n; j++) { /* do a row of U */
      sum = 0.0;
      for (k = 0; k < i; k++) {
        sum += invA[i * n + k] * invA[k * n + j];
      }
      invA[i * n + j] = (invA[i * n + j] - sum) / invA[i * n + i];
    }
  }
  for (i = 0; i < n; i++) /* invert L */ {
    for (j = i; j < n; j++) {
      x = 1.0;
      if (i != j) {
        x = 0.0;
        for (k = i; k < j; k++) {
          x -= invA[j * n + k] * invA[k * n + i];
        }
      }
      invA[j * n + i] = x / invA[j * n + j];
    }
  }
  for (i = 0; i < n; i++) /* invert U */ {
    for (j = i; j < n; j++) {
      if (i == j) continue;
      sum = 0.0;
      for (k = i; k < j; k++) {
        sum += invA[k * n + j] * ((i == k) ? 1.0f : invA[i * n + k]);
      }
      invA[i * n + j] = -sum;
    }
  }
  for (i = 0; i < n; i++) /* final inversion */ {
    for (j = 0; j < n; j++) {
      sum = 0.0;
      for (k = ((i > j) ? i : j); k < n; k++) {
        sum += ((j == k) ? 1.0f : invA[j * n + k]) * invA[k * n + i];
      }
      invA[j * n + i] = sum;
    }
  }
}

void met_lld_fitPolyFirstDegree(double* x_data, double* y_data, int x_size,
                                double poly[3]) {
  int i;
  double H_data[512 * 2];
  double H_data_T[512 * 2];
  double H_data_multi[4], H_data_multi_inv[4];
  double temp[512 * 2];
  for (i = 0; i < x_size; i++) {
    H_data[0 + 2 * i] = 1.0F;
    H_data[1 + 2 * i] = x_data[i];
  }
  met_lld_TransposeMat(H_data, x_size, 2, H_data_T);
  met_lld_Mult(H_data_T, 2, x_size, H_data, x_size, 2, H_data_multi);
  met_lld_InvMat(H_data_multi_inv, H_data_multi, 2);
  met_lld_Mult(H_data_multi_inv, 2, 2, H_data_T, 2, x_size, temp);
  met_lld_Mult(temp, 2, x_size, y_data, x_size, 1, poly);
}

void met_lld_InvMat(double* invA, double* A, int n) {
  int i, j, k;
  double sum, x;

  /*  Copy the input matrix to output matrix */
  for (i = 0; i < n * n; i++) {
    invA[i] = A[i];
  }

  /* Add small value to diagonal if diagonal is zero */
  for (i = 0; i < n; i++) {
    j = i * n + i;
    if ((invA[j] < 1e-8f) && (invA[j] > -1e-8f)) {
      invA[j] = 1e-8f;
    }
  }

  /* Matrix size must be larger than one */
  if (n <= 1) return;

  for (i = 1; i < n; i++) {
    invA[i] /= invA[0]; /* normalize row 0 */
  }

  for (i = 1; i < n; i++) {
    for (j = i; j < n; j++) { /* do a column of L */
      sum = 0.0;
      for (k = 0; k < i; k++) {
        sum += invA[j * n + k] * invA[k * n + i];
      }
      invA[j * n + i] -= sum;
    }
    if (i == n - 1) continue;
    for (j = i + 1; j < n; j++) { /* do a row of U */
      sum = 0.0;
      for (k = 0; k < i; k++) {
        sum += invA[i * n + k] * invA[k * n + j];
      }
      invA[i * n + j] = (invA[i * n + j] - sum) / invA[i * n + i];
    }
  }
  for (i = 0; i < n; i++) /* invert L */ {
    for (j = i; j < n; j++) {
      x = 1.0;
      if (i != j) {
        x = 0.0;
        for (k = i; k < j; k++) {
          x -= invA[j * n + k] * invA[k * n + i];
        }
      }
      invA[j * n + i] = x / invA[j * n + j];
    }
  }
  for (i = 0; i < n; i++) /* invert U */ {
    for (j = i; j < n; j++) {
      if (i == j) continue;
      sum = 0.0;
      for (k = i; k < j; k++) {
        sum += invA[k * n + j] * ((i == k) ? 1.0f : invA[i * n + k]);
      }
      invA[i * n + j] = -sum;
    }
  }
  for (i = 0; i < n; i++) /* final inversion */ {
    for (j = 0; j < n; j++) {
      sum = 0.0;
      for (k = ((i > j) ? i : j); k < n; k++) {
        sum += ((j == k) ? 1.0f : invA[j * n + k]) * invA[k * n + i];
      }
      invA[j * n + i] = sum;
    }
  }
}

void met_lld_Mult_Float(float* A, int num_row_A, int num_col_A, float* X,
                        int num_row_X, int num_col_X, float* B) {
  int m = num_row_A, n = num_col_A, p = num_col_X;
  int i, j, k;
  float lsum;
  for (i = 0; i < m; i++) {
    for (j = 0; j < p; j++) {
      lsum = 0;
      for (k = 0; k < n; k++) {
        lsum += A[i * n + k] * X[k * p + j];
      }
      B[i * p + j] = lsum;
    }
  }
}

void met_lld_TransposeMat_Float(float* A, int num_r, int num_c, float* AT) {
  int iter_r, iter_c, indexA, indexAT;
  for (iter_r = 0; iter_r < num_r; iter_r++) {
    for (iter_c = 0; iter_c < num_c; iter_c++) {
      indexA = iter_c + iter_r * num_c;
      indexAT = iter_r + iter_c * num_r;
      AT[indexAT] = A[indexA];
    }
  }
}

void met_lld_fitPoly(double* x_data, double* y_data, int x_size, int prevOrder,
                     double poly[4]) {
  double length;
  double coeff[4];
  const float k_min_length_threshold = 0.01;          // 0.1*0.1
  const float k_first_order_length_threshold = 100;   // 10*10
  const float k_second_order_length_threshold = 625;  // 25*25
  const float k_length_sample_threshold = 0.04;
  length = (x_data[x_size - 1] - x_data[0]) * (x_data[x_size - 1] - x_data[0]) +
           (y_data[x_size - 1] - y_data[0]) * (y_data[x_size - 1] - y_data[0]);
  memset(coeff, 0, sizeof(double) * 4);
  if (prevOrder == 0) {
    if (length > k_second_order_length_threshold) {
      met_lld_fitPolyThirdDegree(x_data, y_data, x_size, coeff);
      poly[0] = static_cast<float>(coeff[0]);
      poly[1] = static_cast<float>(coeff[1]);
      poly[2] = static_cast<float>(coeff[2]);
      poly[3] = static_cast<float>(coeff[3]);
    } else if (length > k_first_order_length_threshold) {
      met_lld_fitPolySecondDegree(x_data, y_data, x_size, coeff);
      poly[0] = static_cast<float>(coeff[0]);
      poly[1] = static_cast<float>(coeff[1]);
      poly[2] = static_cast<float>(coeff[2]);
      poly[3] = 0.0f;
    } else if (length > k_min_length_threshold) {
      met_lld_fitPolyFirstDegree(x_data, y_data, x_size, coeff);
      poly[0] = static_cast<float>(coeff[0]);
      poly[1] = static_cast<float>(coeff[1]);
      poly[2] = 0.0f;
      poly[3] = 0.0f;
    } else {
      poly[0] = -1e5;
      poly[1] = -1e5;
      poly[2] = -1e5;
      poly[3] = -1e5;
    }
  } else {
    switch (prevOrder - 1) {
      case 3:
        met_lld_fitPolyThirdDegree(x_data, y_data, x_size, coeff);
        poly[0] = static_cast<float>(coeff[0]);
        poly[1] = static_cast<float>(coeff[1]);
        poly[2] = static_cast<float>(coeff[2]);
        poly[3] = static_cast<float>(coeff[3]);
        break;
      case 2:
        met_lld_fitPolySecondDegree(x_data, y_data, x_size, coeff);
        poly[0] = static_cast<float>(coeff[0]);
        poly[1] = static_cast<float>(coeff[1]);
        poly[2] = static_cast<float>(coeff[2]);
        poly[3] = 0.0f;
        break;
      case 1:
        met_lld_fitPolyFirstDegree(x_data, y_data, x_size, coeff);
        poly[0] = static_cast<float>(coeff[0]);
        poly[1] = static_cast<float>(coeff[1]);
        poly[2] = 0.0f;
        poly[3] = 0.0f;
        break;
      default:
        printf("Error previous order parameter!\n");
        break;
    }
  }
}

void met_lld_Mult(double* A, int num_row_A, int num_col_A, double* X,
                  int num_row_X, int num_col_X, double* B) {
  int m = num_row_A, n = num_col_A, p = num_col_X;
  int i, j, k;
  double lsum;
  for (i = 0; i < m; i++) {
    for (j = 0; j < p; j++) {
      lsum = 0;
      for (k = 0; k < n; k++) {
        lsum += A[i * n + k] * X[k * p + j];
      }
      B[i * p + j] = lsum;
    }
  }
}

void met_lld_TransposeMat(double* A, int num_r, int num_c, double* AT) {
  int iter_r, iter_c, indexA, indexAT;
  for (iter_r = 0; iter_r < num_r; iter_r++) {
    for (iter_c = 0; iter_c < num_c; iter_c++) {
      indexA = iter_c + iter_r * num_c;
      indexAT = iter_r + iter_c * num_r;
      AT[indexAT] = A[indexA];
    }
  }
}

void met_lld_AddMat(double* A, double* B, double* C, int num_r, int num_c) {
  int iter_r, iter_c, index;
  for (iter_r = 0; iter_r < num_r; iter_r++) {
    for (iter_c = 0; iter_c < num_c; iter_c++) {
      index = iter_c + iter_r * num_c;
      C[index] = A[index] + B[index];
    }
  }
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
