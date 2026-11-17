// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

namespace emcl2
{
// Durations use a monotonic clock supplied by the caller. Only real sensor updates call observe.
class RecoveryHealth
{
public:
  double lost_threshold = 0.3, recovered_threshold = 0.7, lost_hold = 1.0, recovered_hold = 2.0;
  bool lost = false, recovered = false;
  void invalidate() {low_since_ = high_since_ = -1; recovered = false;}
  void observe(double alpha, double time)
  {
    recovered = false;
    if (!std::isfinite(alpha)) {invalidate();return;}
    if (alpha < lost_threshold) {
      if (low_since_ < 0) {low_since_ = time;}
      if (time - low_since_ >= lost_hold) {lost = true;}
    } else {low_since_ = -1;}
    if (alpha >= recovered_threshold) {
      if (high_since_ < 0) {high_since_ = time;}
      if (time - high_since_ >= recovered_hold) {lost = false;recovered = true;}
    } else {high_since_ = -1;}
  }

private:
  double low_since_ = -1, high_since_ = -1;
};

inline bool validGnssCov(double xx, double xy, double yx, double yy)
{
  return std::isfinite(xx) && std::isfinite(xy) && std::isfinite(yx) && std::isfinite(yy) &&
         xx >= 0 && yy >= 0 &&
         std::abs(xy - yx) <= 1e-8 * std::max({1.0, std::abs(xy), std::abs(yx)}) &&
         xy * xy <= xx * yy + 1e-10 * std::max(1.0, xx * yy);
}
// Position is the antenna location. Each sampled heading supplies its own lever-arm correction.
template<class Particle, class Generator>
void replaceGnssParticles(
  std::vector<Particle> & particles, double x, double y, double xx,
  double xy, double yy, double sigma_floor, double offset_x, double offset_y,
  Generator & rng)
{
  if (!validGnssCov(xx, xy, xy, yy) || !std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(sigma_floor) || sigma_floor <= 0 || !std::isfinite(offset_x) ||
    !std::isfinite(offset_y) || particles.empty())
  {
    throw std::invalid_argument("invalid GNSS particle distribution");
  }
  xx += sigma_floor * sigma_floor;
  yy += sigma_floor * sigma_floor;
  const double a = std::sqrt(xx), b = xy / a, c = std::sqrt(std::max(0.0, yy - b * b));
  std::normal_distribution<double> normal(0, 1);
  std::uniform_real_distribution<double> heading(-3.141592653589793, 3.141592653589793);
  for (auto & p:particles) {
    const double theta = heading(rng), u = normal(rng), v = normal(rng);
    p.p_.t_ = theta;
    p.p_.x_ = x + a * u - (std::cos(theta) * offset_x - std::sin(theta) * offset_y);
    p.p_.y_ = y + b * u + c * v - (std::sin(theta) * offset_x + std::cos(theta) * offset_y);
    p.w_ = 1.0 / particles.size();
  }
}
}
