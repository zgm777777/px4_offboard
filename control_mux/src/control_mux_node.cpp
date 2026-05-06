#include "control_mux/control_mux_node.hpp"

#include <algorithm>
#include <functional>
#include <sstream>

using namespace std::chrono_literals;

ControlMuxNode::ControlMuxNode(const rclcpp::NodeOptions & options)
: Node("control_mux", options)
{
  load_parameters();

  const auto best_effort_qos = make_best_effort_qos();

  // Publishers
  output_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    output_topic_, rclcpp::QoS(10));
  active_source_pub_ = create_publisher<std_msgs::msg::String>(
    active_source_topic_, rclcpp::QoS(10));

  // Candidate velocity subscriptions
  aruco_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    aruco_topic_,
    rclcpp::QoS(10),
    std::bind(&ControlMuxNode::on_aruco_velocity, this, std::placeholders::_1));

  gui_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    gui_topic_,
    rclcpp::QoS(10),
    std::bind(&ControlMuxNode::on_gui_velocity, this, std::placeholders::_1));

  joy_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    joy_topic_,
    rclcpp::QoS(10),
    std::bind(&ControlMuxNode::on_joy_velocity, this, std::placeholders::_1));

  // Gate state subscriptions (best-effort to match offboard_manager publishing QoS)
  control_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
    control_enable_topic_,
    best_effort_qos,
    std::bind(&ControlMuxNode::on_control_enable, this, std::placeholders::_1));

  takeoff_override_sub_ = create_subscription<std_msgs::msg::Bool>(
    takeoff_override_topic_,
    best_effort_qos,
    std::bind(&ControlMuxNode::on_takeoff_override, this, std::placeholders::_1));

  flight_state_sub_ = create_subscription<std_msgs::msg::String>(
    flight_state_topic_,
    best_effort_qos,
    std::bind(&ControlMuxNode::on_flight_state, this, std::placeholders::_1));

  // Source command subscription
  source_cmd_sub_ = create_subscription<std_msgs::msg::String>(
    source_cmd_topic_,
    rclcpp::QoS(10),
    std::bind(&ControlMuxNode::on_source_cmd, this, std::placeholders::_1));

  // Publish timer
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0))),
    std::bind(&ControlMuxNode::on_publish_timer, this));

  {
    std::lock_guard<std::mutex> lock(mutex_);
    selected_source_ = default_source_;
    candidates_["aruco"] = CandidateInput{};
    candidates_["gui"] = CandidateInput{};
    candidates_["joy"] = CandidateInput{};
  }

  RCLCPP_INFO(
    get_logger(),
    "control_mux started: output=%s default=%s timeout=%.3fs rate=%.1fHz",
    output_topic_.c_str(),
    default_source_.c_str(),
    input_timeout_sec_,
    publish_rate_hz_);
}

// =============================================================================
// Parameter loading
// =============================================================================
void ControlMuxNode::load_parameters()
{
  output_topic_ = declare_parameter<std::string>("output_topic", "/input/velocity_cmd");
  aruco_topic_ = declare_parameter<std::string>("aruco_topic", "/aruco/velocity_cmd");
  gui_topic_ = declare_parameter<std::string>("gui_topic", "/gui/velocity_cmd");
  joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy/velocity_cmd");
  source_cmd_topic_ = declare_parameter<std::string>(
    "source_cmd_topic", "/control_mux/source_cmd");
  active_source_topic_ = declare_parameter<std::string>(
    "active_source_topic", "/control_mux/active_source");
  control_enable_topic_ = declare_parameter<std::string>(
    "control_enable_topic", "/offboard/control_enable");
  takeoff_override_topic_ = declare_parameter<std::string>(
    "takeoff_override_topic", "/offboard/takeoff_override");
  flight_state_topic_ = declare_parameter<std::string>(
    "flight_state_topic", "/offboard/flight_state");

  default_source_ = declare_parameter<std::string>("default_source", "hold");
  input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.5);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);

  input_timeout_sec_ = std::max(0.05, input_timeout_sec_);
  publish_rate_hz_ = std::max(1.0, publish_rate_hz_);
}

rclcpp::QoS ControlMuxNode::make_best_effort_qos() const
{
  return rclcpp::QoS(10).best_effort().durability_volatile();
}

// =============================================================================
// Subscription callbacks
// =============================================================================
void ControlMuxNode::on_aruco_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & cand = candidates_["aruco"];
  cand.cmd = *msg;
  cand.stamp = get_clock()->now();
  cand.has_valid_cmd = true;
}

void ControlMuxNode::on_gui_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & cand = candidates_["gui"];
  cand.cmd = *msg;
  cand.stamp = get_clock()->now();
  cand.has_valid_cmd = true;
}

void ControlMuxNode::on_joy_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & cand = candidates_["joy"];
  cand.cmd = *msg;
  cand.stamp = get_clock()->now();
  cand.has_valid_cmd = true;
}

void ControlMuxNode::on_control_enable(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_enabled = prev_control_enable_;
  control_enable_ = msg->data;
  prev_control_enable_ = control_enable_;

  // When control_enable transitions from true → false, reset source to hold.
  // This prevents a stale gui/joy/aruco source from being inherited on re-arm.
  if (was_enabled && !control_enable_) {
    const std::string old_source = selected_source_;
    selected_source_ = "hold";
    RCLCPP_INFO(get_logger(),
      "control_enable dropped; resetting mux source to 'hold' (was '%s').",
      old_source.c_str());
  }
}

void ControlMuxNode::on_takeoff_override(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  takeoff_override_ = msg->data;
}

void ControlMuxNode::on_flight_state(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  flight_state_ = msg->data;
}

void ControlMuxNode::on_source_cmd(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & cmd = msg->data;

  // Validate source name
  if (cmd != "hold" && cmd != "gui" && cmd != "joy" && cmd != "aruco") {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Unknown source_cmd '%s', keeping current source. Valid: hold/gui/joy/aruco.",
      cmd.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // "hold" is always accepted — it's the safe default.
  if (cmd == "hold") {
    selected_source_ = cmd;
    RCLCPP_INFO(get_logger(), "Source switched to 'hold'.");
    return;
  }

  // Non-hold sources (gui/joy/aruco) require the control gate to be open.
  const bool emergency =
    !flight_state_.empty() &&
    flight_state_.find("EMERGENCY") != std::string::npos;

  if (!control_enable_ || takeoff_override_ || emergency) {
    selected_source_ = "hold";
    RCLCPP_WARN(
      get_logger(),
      "Rejecting source_cmd '%s': control gate is not open "
      "(enable=%s takeoff_override=%s emergency=%s). Falling back to 'hold'.",
      cmd.c_str(),
      control_enable_ ? "true" : "false",
      takeoff_override_ ? "true" : "false",
      emergency ? "true" : "false");
    return;
  }

  selected_source_ = cmd;
  RCLCPP_INFO(get_logger(), "Source switched to '%s'.", cmd.c_str());
}

// =============================================================================
// Gate state computation (Layer 1: safety gating)
// =============================================================================
std::string ControlMuxNode::compute_gate_state() const
{
  // Check for EMERGENCY in flight_state string
  if (!flight_state_.empty()) {
    if (flight_state_.find("EMERGENCY") != std::string::npos) {
      return "EMERGENCY";
    }
  }

  // Check takeoff override
  if (takeoff_override_) {
    return "TAKEOFF_OVERRIDE";
  }

  // Check control enable
  if (!control_enable_) {
    return "DISABLED";
  }

  return "OK";
}

// =============================================================================
// Publish loop (runs at publish_rate_hz_)
// =============================================================================
void ControlMuxNode::on_publish_timer()
{
  // Read gate state under mutex to avoid races with subscription callbacks
  std::string gate;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gate = compute_gate_state();
  }

  // --- Active source string for diagnostics ---
  std::string active_label;
  geometry_msgs::msg::TwistStamped out_cmd;

  if (gate == "EMERGENCY") {
    out_cmd = make_zero_velocity();
    active_label = "EMERGENCY";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "EMERGENCY: zeroing velocity output.");
  } else if (gate == "TAKEOFF_OVERRIDE") {
    out_cmd = make_zero_velocity();
    active_label = "TAKEOFF_OVERRIDE";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Takeoff override active: blocking candidate velocity sources.");
  } else if (gate == "DISABLED") {
    out_cmd = make_zero_velocity();
    active_label = "DISABLED";
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Offboard control disabled: zeroing velocity output.");
  } else {
    // --- Layer 2: source selection ---
    std::string src;
    bool source_timed_out = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      src = selected_source_;
      auto it = candidates_.find(src);
      if (it == candidates_.end() || !it->second.has_valid_cmd) {
        // No valid candidate for this source
        source_timed_out = true;
      } else {
        const double age = (get_clock()->now() - it->second.stamp).seconds();
        if (age > input_timeout_sec_) {
          source_timed_out = true;
        } else {
          out_cmd = it->second.cmd;
        }
      }
    }

    if (src == "hold") {
      out_cmd = make_zero_velocity();
      active_label = "hold";
    } else if (source_timed_out) {
      // Input stale — fall back to hold / zero
      out_cmd = make_zero_velocity();
      active_label = "HOLD";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Source '%s' input stale (timeout=%.3fs), falling back to HOLD.",
        src.c_str(), input_timeout_sec_);
    } else {
      active_label = src;
    }
  }

  // Publish output velocity
  output_pub_->publish(out_cmd);

  // Publish active source
  set_active_source(active_label);
}

// =============================================================================
// Helpers
// =============================================================================
geometry_msgs::msg::TwistStamped ControlMuxNode::make_zero_velocity()
{
  geometry_msgs::msg::TwistStamped msg{};
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "map";
  msg.twist.linear.x = 0.0;
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.z = 0.0;
  return msg;
}

void ControlMuxNode::set_active_source(const std::string & source)
{
  std_msgs::msg::String msg{};
  msg.data = source;
  active_source_pub_->publish(msg);
}

// =============================================================================
// Main
// =============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlMuxNode>());
  rclcpp::shutdown();
  return 0;
}
