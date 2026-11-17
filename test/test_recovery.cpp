// SPDX-License-Identifier: Apache-2.0
#include "emcl2/Recovery.h"
#include <gtest/gtest.h>
TEST(Health, SustainedLowAndHigh) {
  emcl2::RecoveryHealth h;
  h.observe(.1, 1);h.observe(.1, 1.9);EXPECT_FALSE(h.lost);
  h.observe(.1, 2.1);EXPECT_TRUE(h.lost);
  h.observe(.9, 3);h.observe(.9, 4.9);EXPECT_TRUE(h.lost);
  h.observe(.9, 5.1);EXPECT_FALSE(h.lost);EXPECT_TRUE(h.recovered);
}
TEST(Health, InterruptionBreaksContinuity) {
  emcl2::RecoveryHealth h;h.observe(.1, 1);h.invalidate();h.observe(.1, 3);EXPECT_FALSE(h.lost);
  h.observe(.1, 4.1);EXPECT_TRUE(h.lost);
  h.observe(.9, 5);h.invalidate();h.observe(.9, 8);EXPECT_FALSE(h.recovered);
}
struct Particle {struct Pose {double x_, y_, t_;} p_;double w_;};
TEST(Particles, CorrelatedDistributionAndLeverArm) {
  std::vector<Particle> p(30000);std::mt19937 rng(42);
  emcl2::replaceGnssParticles(p, 10, 20, 4, 1, 9, .1, 1, 0, rng);
  double x = 0, y = 0, xx = 0, xy = 0, yy = 0, w = 0;
  for (const auto & q:p) {
    const double a = q.p_.x_ + std::cos(q.p_.t_) - 10, b = q.p_.y_ + std::sin(q.p_.t_) - 20;
    x += a;y += b;xx += a * a;xy += a * b;yy += b * b;w += q.w_;
  }
  EXPECT_NEAR(x / p.size(), 0, .05);EXPECT_NEAR(y / p.size(), 0, .08);
  EXPECT_NEAR(xx / p.size(), 4.01, .15);EXPECT_NEAR(xy / p.size(), 1, .15);EXPECT_NEAR(
    yy / p.size(), 9.01, .25);
  EXPECT_NEAR(w, 1, 1e-9);
}
TEST(Particles, RejectsBadCovariance) {
  EXPECT_FALSE(emcl2::validGnssCov(1, 2, 2, 1));
  EXPECT_FALSE(emcl2::validGnssCov(1, 0, 1, 1));
  EXPECT_TRUE(emcl2::validGnssCov(0, 0, 0, 0));
}
