// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: lane_point_filter.h
// @brief: filter 3d points
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/data_fusion/digital_filter_util.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "perception-base/base/utils/log.h"

namespace hozon {
namespace mp {
namespace environment {

// Buffterworth coffecient parameters compute and set
void BuffterworthCoefficients(const double ts, const double cutoff_freq,
                              std::vector<double>* denominators,
                              std::vector<double>* numerators) {
  denominators->clear();
  numerators->clear();
  denominators->reserve(3);
  numerators->reserve(3);

  double wa = 2.0 * M_PI * cutoff_freq;  // Analog frequency in rad/s
  double alpha = wa * ts / 2.0;          // tan(Wd/2), Wd is discrete frequency
  double alpha_sqr = alpha * alpha;
  double tmp_term = std::sqrt(2.0) * alpha + alpha_sqr;
  double gain = alpha_sqr / (1.0 + tmp_term);

  denominators->push_back(1.0);
  denominators->push_back(2.0 * (alpha_sqr - 1.0) / (1.0 + tmp_term));
  denominators->push_back((1.0 - std::sqrt(2.0) * alpha + alpha_sqr) /
                          (1.0 + tmp_term));

  numerators->push_back(gain);
  numerators->push_back(2.0 * gain);
  numerators->push_back(gain);
}

// firstOrder Butterworth filter coefficients
void FirstOrderCoefficients(const double ts, const double settling_time,
                            const double dead_time,
                            std::vector<double>* denominators,
                            std::vector<double>* numerators) {
  // sanity check
  if (ts <= 0.0 || settling_time < 0.0 || dead_time < 0.0) {
    HLOG_ERROR << "time cannot be negative";
    return;
  }

  const size_t k_d = static_cast<size_t>(dead_time / ts);
  double a_term;

  denominators->clear();
  numerators->clear();
  denominators->reserve(2);
  numerators->reserve(k_d + 1);  // size depends on dead-time

  if (settling_time == 0.0) {
    a_term = 0.0;
  } else {
    a_term = exp(-1 * ts / settling_time);
  }

  denominators->push_back(1.0);
  denominators->push_back(-a_term);
  numerators->insert(numerators->end(), k_d, 0.0);
  numerators->push_back(1 - a_term);
}

ButterworthFilter::ButterworthFilter(const std::vector<double>& denominators,
                                     const std::vector<double>& numerators) {
  set_coefficients(denominators, numerators);
}

void ButterworthFilter::set_coefficients(
    const std::vector<double>& denominators,
    const std::vector<double>& numerators) {
  numerators_ = numerators;
  x_values_.resize(numerators_.size(), 0.0);
  denominators_ = denominators;
  y_values_.resize(denominators_.size(), 0.0);
}

void ButterworthFilter::set_dead_zone(const double deadzone) {
  dead_zone_ = std::fabs(deadzone);
  HLOG_DEBUG << "Setting digital filter dead zone = " << dead_zone_;
}

double ButterworthFilter::Filter(const double x_insert) {
  if (denominators_.empty() || numerators_.empty()) {
    HLOG_ERROR << "Empty denominators or numerators";
    return 0.0;
  }

  x_values_.pop_back();
  x_values_.push_front(x_insert);
  const double xside =
      ComputeValue(x_values_, numerators_, 0, numerators_.size() - 1);

  y_values_.pop_back();
  const double yside =
      ComputeValue(y_values_, denominators_, 1, denominators_.size() - 1);

  double y_insert = 0.0;
  if (std::fabs(denominators_.front()) > Epsilon_) {
    y_insert = (xside - yside) / denominators_.front();
  }
  y_values_.push_front(y_insert);

  return UpdateLast(y_insert);
}

void ButterworthFilter::reset_values() {
  std::fill(x_values_.begin(), x_values_.end(), 0.0);
  std::fill(y_values_.begin(), y_values_.end(), 0.0);
}

double ButterworthFilter::UpdateLast(const double input) {
  const double diff = std::fabs(input - last_);
  // dead_zone_ is the change value
  if (diff < dead_zone_) {
    return last_;
  }
  last_ = input;
  return input;
}

double ButterworthFilter::ComputeValue(const std::deque<double>& values,
                                       const std::vector<double>& coefficients,
                                       const std::size_t coeff_start,
                                       const std::size_t coeff_end) {
  CHECK(coeff_start <= coeff_end && coeff_end < coefficients.size());
  CHECK((coeff_end - coeff_start + 1) == values.size());

  double sum = 0.0;
  auto i = coeff_start;
  for (const auto value : values) {
    sum += value * coefficients[i];
    ++i;
  }
  return sum;
}

const std::vector<double>& ButterworthFilter::denominators() const {
  return denominators_;
}

const std::vector<double>& ButterworthFilter::numerators() const {
  return numerators_;
}

double ButterworthFilter::dead_zone() const { return dead_zone_; }

const std::deque<double>& ButterworthFilter::inputs_queue() const {
  return x_values_;
}

const std::deque<double>& ButterworthFilter::outputs_queue() const {
  return y_values_;
}
}  // namespace environment
}  // namespace mp
}  // namespace hozon
