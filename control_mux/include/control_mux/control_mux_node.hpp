#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * ControlMuxNode — velocity command multiplexer.
 *
 * Architecture overview:
 *   offboard_manager  →  "can I control?"  (gate: control_enable, takeoff_override, flight_state)
 *   control_mux       →  "who controls?"   (source selection: aruco / gui / joy / hold)
 *   velocity_controller → "how to execute" (ENU→NED conversion, TrajectorySetpoint output)
 *
 * This node is the *only* publisher to /input/velocity_cmd.
 * All upstream controllers publish to their own candidate topics.
 * The mux applies safety gate logic first, then source arbitration.
 */
class ControlMuxNode final : public rclcpp::Node
{
public:
  explicit ControlMuxNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- Subscription callbacks ----
  void on_aruco_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_gui_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_joy_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void on_control_enable(const std_msgs::msg::Bool::SharedPtr msg);
  void on_takeoff_override(const std_msgs::msg::Bool::SharedPtr msg);
  void on_flight_state(const std_msgs::msg::String::SharedPtr msg);
  void on_source_cmd(const std_msgs::msg::String::SharedPtr msg);

  // ---- Timers ----
  void on_publish_timer();

  // ---- Helpers ----
  void load_parameters();
  rclcpp::QoS make_best_effort_qos() const;

  geometry_msgs::msg::TwistStamped make_zero_velocity();

  // Returns the effective mux state: "EMERGENCY", "TAKEOFF_OVERRIDE", "DISABLED", or "OK"
  std::string compute_gate_state() const;

  void set_active_source(const std::string & source);

  // Candidate input storage
  struct CandidateInput
  {
    geometry_msgs::msg::TwistStamped cmd{};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool has_valid_cmd{false};
  };

  // Parameters
  std::string output_topic_;
  std::string aruco_topic_;
  std::string gui_topic_;
  std::string joy_topic_;
  std::string source_cmd_topic_;
  std::string active_source_topic_;
  std::string control_enable_topic_;
  std::string takeoff_override_topic_;
  std::string flight_state_topic_;

  std::string default_source_;
  double input_timeout_sec_;
  double publish_rate_hz_;

  // Runtime state (protected by mutex)
  mutable std::mutex mutex_;
  std::map<std::string, CandidateInput> candidates_;  // key: "aruco", "gui", "joy"
  std::string selected_source_;                       // from /control_mux/source_cmd
  bool control_enable_{false};
  bool prev_control_enable_{false};
  bool takeoff_override_{false};
  std::string flight_state_;                          // raw string from offboard_manager

  // Publishers / Subscriptions
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_source_pub_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr aruco_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gui_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeoff_override_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flight_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_cmd_sub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};
