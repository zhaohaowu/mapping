// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: lane_point_filter.h
// @brief: filter 3d points

#pragma once

#include <math.h>
#include <deque>
#include <vector>

namespace hozon {
namespace mp {
namespace environment {
void BuffterworthCoefficients(const double ts, const double cutoff_freq,
                              std::vector<double> *denominators,
                              std::vector<double> *numerators);
void FirstOrderCoefficients(const double ts, const double settling_time,
                            const double dead_time,
                            std::vector<double> *denominators,
                            std::vector<double> *numerators);
class ButterworthFilter {
 public:
  ButterworthFilter() = default;

  /**
   * @brief Initializes a ButterworthFilter with given denominators and
   * numerators.
   * @param denominators The denominators of the ButterworthFilter.
   * @param numerators The numerators of the ButterworthFilter.
   */
  ButterworthFilter(const std::vector<double> &denominators,
                    const std::vector<double> &numerators);

  /**
   * @brief Default destructor.
   */
  ~ButterworthFilter() = default;

  /**
   * @brief Processes a new measurement with the filter.
   * @param x_insert The new input to be processed by the filter.
   */
  double Filter(const double x_insert);
  /**
   * @desc: Filter by the input x_insert
   * Input: new value of x_insert
   * Remove x[n - 1], insert x_insert into x[0]
   * Remove y[n - 1],
   * Solve den[0] * y + den[1] * y[0] + ... + den[n - 1]*y[n - 2] =
   *   num[0] * x[0] + num[1] * x[1] + ... + num[n - 1] * x[n - 1] for y
   * Insert y into y[0]
   * Output: y[0]
   */

  /**
   * @brief set denominators and numerators
   * @param denominators The denominators of filter
   * @param numerators The numerators of filter
   */
  void set_coefficients(const std::vector<double> &denominators,
                        const std::vector<double> &numerators);

  /**
   * @brief set filter deadzone
   * @param deadzone The value of deadzone
   */
  void set_dead_zone(const double deadzone);

  /**
   * @brief re-set the x_values_ and y_values_
   * @param deadzone The value of deadzone
   */
  void reset_values();

  /**
   * @brief get denominators
   * @return vector<double> The denominators of filter
   */
  const std::vector<double> &denominators() const;

  /**
   * @brief get numerators
   * @return vector<double> The numerators of filter
   */
  const std::vector<double> &numerators() const;

  /**
   * @brief get dead_zone
   * @return double The dead_zone
   */
  double dead_zone() const;

  /**
   * @brief get inputs of the filter
   * @return deque<double> The queue of inputs of filter
   */
  const std::deque<double> &inputs_queue() const;

  /**
   * @brief get outputs of the filter
   * @return deque<double> The queue of outputs of filter
   */
  const std::deque<double> &outputs_queue() const;

 private:
  /**
   * @desc: Update the last-filtered value,
   *        if the difference is less than dead_zone_
   */
  double UpdateLast(const double input);

  /**
   * @desc: Compute the inner product of values[coeff_start : coeff_end] and
   *        coefficients[coeff_start : coeff_end]
   */
  double ComputeValue(const std::deque<double> &values,
                      const std::vector<double> &coefficients,
                      const std::size_t coeff_start,
                      const std::size_t coeff_end);

  // Front is latest, back is oldest.
  std::deque<double> x_values_;

  // Front is latest, back is oldest.
  std::deque<double> y_values_;

  // Coefficients with y values
  std::vector<double> denominators_;

  // Coefficients with x values
  std::vector<double> numerators_;

  // threshold of updating last-filtered value
  double dead_zone_ = 0.0;

  // last-filtered value
  double last_ = 0.0;

  const double Epsilon_ = 1.0e-6;
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
