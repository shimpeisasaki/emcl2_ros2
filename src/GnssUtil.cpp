// SPDX-FileCopyrightText: 2023 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "emcl2/GnssUtil.h"
#include "cmath"
#include "rclcpp/rclcpp.hpp"

namespace emcl2{
GnssUtil::GnssUtil()
{
  gnss_var_ = 5.0;
  pf_var_ = 1.0;
}

double GnssUtil::pfRanGaussian(double sigma)
{
  double x1, x2, w, r;
  do
  {
    do { r = drand48(); } while (r==0.0);
    x1 = 2.0 * r - 1.0;
    do { r = drand48(); } while (r==0.0);
    x2 = 2.0 * r - 1.0;
    w = x1*x1 + x2*x2;
  } while(w > 1.0 || w==0.0);
  return sigma * x2 * sqrt(-2.0*log(w)/w);
}

double GnssUtil::kld()
{
    if(isNAN()) return NAN;
    RCLCPP_DEBUG(rclcpp::get_logger("emcl2_node"), "pf_SIGMA(x, y) = diag(%lf, %lf), gnss_SIGMA(x, y) = diag(%lf, %lf)", pf_sigma_mx_(0, 0), pf_sigma_mx_(1, 1), gnss_sigma_mx_(0, 0), gnss_sigma_mx_(1, 1));
    double kld = log(gnss_sigma_mx_.determinant() / pf_sigma_mx_.determinant());
    kld += (gnss_sigma_mx_.inverse() * pf_sigma_mx_).trace();
    Eigen::Vector2d diff_pos = pf_position_ - gnss_position_;
    kld += diff_pos.transpose() * gnss_sigma_mx_.inverse() * diff_pos;
    kld -= 2.0;
    kld /= 2.0;
    return kld;
}

void GnssUtil::gnssReset(double alpha, double alpha_th, std::vector<emcl2::Particle> & particles)
{
    double beta = alpha < alpha_th ? 1 - alpha / alpha_th : 0.0;
    int particle_num = beta * particles.size();
    RCLCPP_DEBUG(rclcpp::get_logger("emcl2_node"), "beta: %lf, num of replace particle: %d", beta, particle_num);
    RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "reset_sigma(x, y) = (%lf, %lf)", gnss_sigma_mx_(0, 0), gnss_sigma_mx_(1, 1));

    double gnss_std_x = sqrt(gnss_sigma_mx_(0, 0));
    double gnss_std_y = sqrt(gnss_sigma_mx_(1, 1));
    if(gnss_std_x <= 4.0) gnss_std_x = 4.0;
    if(gnss_std_y <= 4.0) gnss_std_y = 4.0;

    for(int i = 0; i < particle_num; ++i)
    {
        particles[i].p_.x_ = gnss_position_[0] + pfRanGaussian(gnss_std_x);
        particles[i].p_.y_ = gnss_position_[1] + pfRanGaussian(gnss_std_y);
        particles[i].p_.t_ = gnss_yaw_ + pfRanGaussian(sqrt(gnss_reset_var_));
    }
}

bool GnssUtil::isNAN()
{
    if(std::isnan(gnss_position_[0]) || std::isnan(gnss_position_[1])) return true;
    return false;
}
}
