#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class VelocityController final : public rclcpp::Node
{
public:
  explicit VelocityController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::QoS make_best_effort_qos() const;
  uint64_t now_micros();

  void load_parameters();

  void on_velocity_cmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_control_enable(const std_msgs::msg::Bool::SharedPtr msg);
  void on_takeoff_override(const std_msgs::msg::Bool::SharedPtr msg);
  void control_loop_cb();

  px4_msgs::msg::TrajectorySetpoint build_zero_setpoint(uint64_t timestamp_us) const;
  px4_msgs::msg::TrajectorySetpoint build_velocity_setpoint(
    const geometry_msgs::msg::TwistStamped & cmd,
    uint64_t timestamp_us) const;

  // Parameters
  double control_hz_{50.0};
  double cmd_timeout_sec_{0.5};
  double max_xy_speed_mps_{5.0};
  double max_z_speed_mps_{2.0};
  double max_yawspeed_radps_{1.5};

  std::string topic_input_velocity_cmd_;
  std::string topic_offboard_control_enable_;
  std::string topic_offboard_takeoff_override_;
  std::string topic_trajectory_setpoint_out_;

  // Runtime state
  mutable std::mutex cmd_mutex_;
  geometry_msgs::msg::TwistStamped latest_cmd_{};
  rclcpp::Time latest_cmd_stamp_{0, 0, RCL_ROS_TIME};
  bool has_valid_cmd_{false};
  std::atomic_bool control_enable_{false};
  std::atomic_bool takeoff_override_{false};

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeoff_override_sub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
};
