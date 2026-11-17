// SPDX-License-Identifier: Apache-2.0
#include "emcl2/emcl2_node.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace emcl2
{
void EMcl2Node::initRecovery()
{
  use_gnss_reset_ = declare_parameter("use_gnss_reset", false);
  antenna_frame_ = declare_parameter("gnss_antenna_frame", std::string(""));
  gnss_max_age_ = declare_parameter("gnss_max_age", 1.0);
  sigma_floor_ = declare_parameter("gnss_sigma_floor", 0.25);
  max_reset_duration_ = declare_parameter("gnss_reset_max_duration", 60.0);
  health_.lost_threshold = declare_parameter("lost_alpha_threshold", 0.3);
  health_.recovered_threshold = declare_parameter("recovered_alpha_threshold", 0.7);
  health_.lost_hold = declare_parameter("lost_hold_sec", 1.0);
  health_.recovered_hold = declare_parameter("recovered_hold_sec", 2.0);
  for (double v:
    {gnss_max_age_, sigma_floor_, max_reset_duration_, health_.lost_hold,
      health_.recovered_hold})
  {
    if (!std::isfinite(v) || v <= 0) {
      throw std::invalid_argument("recovery durations/sigma must be positive and finite");
    }
  }
  if (!(health_.lost_threshold >= 0 && health_.lost_threshold < health_.recovered_threshold &&
    health_.recovered_threshold <= 1))
  {
    throw std::invalid_argument("require 0 <= lost alpha < recovered alpha <= 1");
  }
  if (use_gnss_reset_ && antenna_frame_.empty()) {
    throw std::invalid_argument("gnss_antenna_frame required");
  }
  instance_id_ = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
  health_msg_.instance_id = instance_id_;
  health_msg_.alpha = std::numeric_limits<double>::quiet_NaN();
  health_msg_.state = Health::INITIALIZING;
  health_msg_.reason = "waiting for map and scan";
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  health_pub_ = create_publisher<Health>("localization/status", qos);
  reset_pub_ = create_publisher<Reset>("localization/gnss_reset_status", qos);
  gnss_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "gnss/map_pose", rclcpp::SensorDataQoS(),
    [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
      const auto & c = m->pose.covariance;
      const auto & p = m->pose.pose.position;
      if (m->header.stamp.sec < 0 || m->header.stamp.nanosec >= 1000000000u) {return;}
      const rclcpp::Time stamp(m->header.stamp, get_clock()->get_clock_type());
      const double age = (now() - stamp).seconds();
      if (stamp.nanoseconds() <= 0 || age < 0 || age > gnss_max_age_ ||
      m->header.frame_id != global_frame_id_ ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !validGnssCov(c[0], c[1], c[6], c[7])) {return;}
      if (gnss_pose_ &&
      stamp <= rclcpp::Time(gnss_pose_->header.stamp, get_clock()->get_clock_type())) {return;}
      gnss_pose_ = m;
      gnss_received_ = Steady::now();
    });
  reset_service_ = create_service<Request>(
    "localization/request_gnss_reset",
    std::bind(&EMcl2Node::requestReset, this, std::placeholders::_1, std::placeholders::_2));
  recovery_timer_ =
    create_wall_timer(std::chrono::milliseconds(100), std::bind(&EMcl2Node::recoveryTick, this));
}
bool EMcl2Node::gnssFresh()
{
  if (!gnss_pose_) {return false;}
  const double age =
    (get_clock()->now() -
    rclcpp::Time(gnss_pose_->header.stamp, get_clock()->get_clock_type())).seconds();
  return age >= 0 && age <= gnss_max_age_ &&
         std::chrono::duration<double>(Steady::now() - gnss_received_).count() <= gnss_max_age_;
}
void EMcl2Node::setHealth(uint8_t state, const std::string & reason)
{
  if (health_msg_.state != state) {++health_msg_.transition_id;}
  health_msg_.state = state;
  health_msg_.reason = reason;
  health_msg_.header.stamp = now();
  health_msg_.header.frame_id = global_frame_id_;
  health_pub_->publish(health_msg_);
}
void EMcl2Node::recoveryTick()
{
  if (active_reset_ &&
    std::chrono::duration<double>(Steady::now() - reset_started_).count() >= reset_timeout_)
  {
    finishReset(Reset::TIMED_OUT, "recovery deadline exceeded");
  }
  const double age = now().seconds() - scan_time_stamp_.seconds();
  if (!init_pf_) {
    setHealth(Health::INITIALIZING, "waiting for map");
  } else if (!scan_receive_ || age < 0 || age > scan_max_age_) {
    health_.invalidate();
    setHealth(Health::UNAVAILABLE, "no fresh scan");
  } else if (last_sensor_update_scan_ns_ <= 0 ||
    (now().nanoseconds() - last_sensor_update_scan_ns_) * 1e-9 > scan_max_age_)
  {
    health_.invalidate();
    setHealth(Health::UNAVAILABLE, "no evaluated scan / TF unavailable");
  } else {setHealth(health_msg_.state, health_msg_.reason);}
  if (!reset_msg_.attempt_id.empty()) {
    reset_msg_.header.stamp = now();
    reset_pub_->publish(reset_msg_);
  }
}
void EMcl2Node::observeHealth()
{
  health_msg_.scan_stamp = scan_time_stamp_;
  health_msg_.alpha = pf_->alpha_;
  const double t = std::chrono::duration<double>(Steady::now().time_since_epoch()).count();
  health_.observe(pf_->alpha_, t);
  if (active_reset_ && reset_msg_.particles_replaced && health_.recovered) {
    finishReset(Reset::SUCCEEDED, "LiDAR/map consistency sustained after GNSS reset");
  }
  setHealth(
    active_reset_ ? Health::RECOVERING : (health_.lost ? Health::LOST : Health::TRACKING),
    active_reset_ ? "GNSS recovery in progress" : (health_.lost ? "low LiDAR/map consistency" :
    "LiDAR/map observation available"));
}
void EMcl2Node::finishReset(uint8_t state, const std::string & reason)
{
  reset_msg_.state = state;
  reset_msg_.reason = reason;
  reset_msg_.finished_at = now();
  reset_msg_.header.stamp = now();
  active_reset_ = false;
  reset_pub_->publish(reset_msg_);
  setHealth(health_.lost ? Health::LOST : Health::TRACKING, "reset ended: " + reason);
}
void EMcl2Node::requestReset(
  const Request::Request::SharedPtr request,
  Request::Response::SharedPtr response)
{
  auto old = accepted_requests_.find(request->request_id);
  if (old != accepted_requests_.end()) {*response = old->second;return;}
  auto reject = [&](uint8_t code, const std::string & reason) {
      response->reason_code = code;
      response->reason = reason;
    };
  const double timeout = request->timeout.sec + request->timeout.nanosec * 1e-9;
  if (request->request_id.empty() || request->request_id.size() > 128 || request->timeout.sec < 0 ||
    request->timeout.nanosec >= 1000000000u || timeout <= 0 || timeout > max_reset_duration_)
  {
    reject(Request::Response::INVALID_REQUEST, "invalid ID or timeout");
    return;
  }
  if (active_reset_) {reject(Request::Response::BUSY, "another reset is active");return;}
  if (!use_gnss_reset_) {reject(Request::Response::DISABLED, "GNSS reset disabled");return;}
  const double age = now().seconds() - scan_time_stamp_.seconds();
  if (!init_pf_ || !scan_receive_ || age < 0 || age > scan_max_age_ ||
    last_sensor_update_scan_ns_ <= 0 ||
    (now().nanoseconds() - last_sensor_update_scan_ns_) * 1e-9 > scan_max_age_)
  {
    reject(Request::Response::NOT_READY, "map/evaluated scan unavailable");
    return;
  }
  if (!gnssFresh()) {reject(Request::Response::NO_GNSS, "no fresh map GNSS pose");return;}
  try {
    tf_->lookupTransform(
      base_frame_id_, antenna_frame_, scan_time_stamp_, rclcpp::Duration::from_seconds(
        0));
  } catch (const tf2::TransformException &) {
    reject(Request::Response::NOT_READY, "antenna TF unavailable");
    return;
  }
  response->accepted = true;
  response->reason_code = Request::Response::ACCEPTED;
  response->attempt_id = instance_id_ + "/" + std::to_string(++attempt_counter_);
  response->reason = "accepted";
  accepted_requests_[request->request_id] = *response;
  request_order_.push_back(request->request_id);
  if (request_order_.size() > 128) {
    accepted_requests_.erase(request_order_.front());
    request_order_.pop_front();
  }
  reset_msg_ = Reset();
  reset_msg_.header.frame_id = global_frame_id_;
  reset_msg_.header.stamp = now();
  reset_msg_.request_id = request->request_id;
  reset_msg_.attempt_id = response->attempt_id;
  reset_msg_.started_at = now();
  reset_msg_.state = Reset::RUNNING;
  reset_msg_.reason = "waiting for next valid scan";
  reset_timeout_ = timeout;
  reset_started_ = Steady::now();
  active_reset_ = true;
  health_.invalidate();
  setHealth(Health::RECOVERING, "GNSS reset accepted");
  reset_pub_->publish(reset_msg_);
}
bool EMcl2Node::applyGnssReset()
{
  if (!gnssFresh()) {
    return false;                  // Keep the attempt alive, but never inject stale data.
  }
  geometry_msgs::msg::TransformStamped lever;
  try {
    lever = tf_->lookupTransform(
      base_frame_id_, antenna_frame_, scan_time_stamp_, rclcpp::Duration::from_seconds(
        0));
  } catch (const tf2::TransformException &) {
    return false;
  }
  const auto & p = gnss_pose_->pose.pose.position;
  const auto & c = gnss_pose_->pose.covariance;
  replaceGnssParticles(
    pf_->particles_, p.x, p.y, c[0], c[1], c[7], sigma_floor_,
    lever.transform.translation.x, lever.transform.translation.y, random_);
  reset_msg_.particles_replaced = true;
  reset_msg_.gnss_source_stamp = gnss_pose_->header.stamp;
  reset_msg_.reason = "particles replaced; awaiting LiDAR recovery";
  health_.invalidate();
  reset_pub_->publish(reset_msg_);
  return true;
}
}
