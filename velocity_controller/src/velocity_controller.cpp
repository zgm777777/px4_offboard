#include "velocity_controller/velocity_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

using namespace std::chrono_literals;

namespace
{
constexpr double kMinControlHz = 2.0;
constexpr double kMinPositiveValue = 1e-3;

float quiet_nan()
{
  return std::numeric_limits<float>::quiet_NaN();
}

float clamp_abs(const double value, const double max_abs)
{
  const double bounded = std::clamp(value, -max_abs, max_abs);
  return static_cast<float>(bounded);
}
}  // namespace

VelocityController::VelocityController(const rclcpp::NodeOptions & options)
: Node("velocity_controller", options)
{
  load_parameters();

  const auto best_effort_qos = make_best_effort_qos();

  trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    topic_trajectory_setpoint_out_, best_effort_qos);

  velocity_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    topic_input_velocity_cmd_,
    rclcpp::QoS(10),
    std::bind(&VelocityController::on_velocity_cmd, this, std::placeholders::_1));

  control_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
    topic_offboard_control_enable_,
    best_effort_qos,
    std::bind(&VelocityController::on_control_enable, this, std::placeholders::_1));

  takeoff_override_sub_ = create_subscription<std_msgs::msg::Bool>(
    topic_offboard_takeoff_override_,
    best_effort_qos,
    std::bind(&VelocityController::on_takeoff_override, this, std::placeholders::_1));

  control_loop_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / control_hz_)),
    std::bind(&VelocityController::control_loop_cb, this));

  RCLCPP_INFO(
    get_logger(),
    "velocity_controller started: loop_hz=%.2f timeout=%.3f max_xy=%.2f max_z=%.2f max_yaw_rate=%.2f",
    control_hz_,
    cmd_timeout_sec_,
    max_xy_speed_mps_,
    max_z_speed_mps_,
    max_yawspeed_radps_);
}

rclcpp::QoS VelocityController::make_best_effort_qos() const
{
  return rclcpp::QoS(10).best_effort().durability_volatile();
}

uint64_t VelocityController::now_micros()
{
  return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000ULL);
}

void VelocityController::load_parameters()
{
  control_hz_ = declare_parameter<double>("control_hz", 50.0);
  cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
  max_xy_speed_mps_ = declare_parameter<double>("max_xy_speed_mps", 5.0);
  max_z_speed_mps_ = declare_parameter<double>("max_z_speed_mps", 2.0);
  max_yawspeed_radps_ = declare_parameter<double>("max_yawspeed_radps", 1.5);

  topic_input_velocity_cmd_ = declare_parameter<std::string>(
    "topics.input_velocity_cmd", "/input/velocity_cmd");
  topic_offboard_control_enable_ = declare_parameter<std::string>(
    "topics.offboard_control_enable", "/offboard/control_enable");
  topic_offboard_takeoff_override_ = declare_parameter<std::string>(
    "topics.offboard_takeoff_override", "/offboard/takeoff_override");
  topic_trajectory_setpoint_out_ = declare_parameter<std::string>(
    "topics.fmu_in_trajectory_setpoint", "/fmu/in/trajectory_setpoint");

  control_hz_ = std::max(control_hz_, kMinControlHz);
  cmd_timeout_sec_ = std::max(cmd_timeout_sec_, kMinPositiveValue);
  max_xy_speed_mps_ = std::max(max_xy_speed_mps_, 0.0);
  max_z_speed_mps_ = std::max(max_z_speed_mps_, 0.0);
  max_yawspeed_radps_ = std::max(max_yawspeed_radps_, 0.0);
}

void VelocityController::on_velocity_cmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const bool has_stamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
  if (!has_stamp) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Received /input/velocity_cmd without valid timestamp, ignoring command.");
    return;
  }

  std::lock_guard<std::mutex> lock(cmd_mutex_);
  latest_cmd_ = *msg;
  latest_cmd_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
  has_valid_cmd_ = true;
}

void VelocityController::on_control_enable(const std_msgs::msg::Bool::SharedPtr msg)
{
  control_enable_.store(msg->data, std::memory_order_relaxed);
}

void VelocityController::on_takeoff_override(const std_msgs::msg::Bool::SharedPtr msg)
{
  takeoff_override_.store(msg->data, std::memory_order_relaxed);
}

void VelocityController::control_loop_cb()
{
  if (takeoff_override_.load(std::memory_order_relaxed)) {
    return;
  }

  geometry_msgs::msg::TwistStamped cmd_snapshot{};
  rclcpp::Time cmd_time_snapshot{0, 0, get_clock()->get_clock_type()};
  bool has_cmd_snapshot = false;

  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_snapshot = latest_cmd_;
    cmd_time_snapshot = latest_cmd_stamp_;
    has_cmd_snapshot = has_valid_cmd_;
  }

  const auto now = get_clock()->now();
  const bool cmd_timed_out =
    (!has_cmd_snapshot) || ((now - cmd_time_snapshot).seconds() > cmd_timeout_sec_);

  px4_msgs::msg::TrajectorySetpoint sp{};
  if (control_enable_.load(std::memory_order_relaxed) && !cmd_timed_out) {
    sp = build_velocity_setpoint(cmd_snapshot, now_micros());
  } else {
    // Keep publishing explicit zero-velocity setpoints when gated-off or timed-out.
    sp = build_zero_setpoint(now_micros());
  }

  trajectory_setpoint_pub_->publish(sp);
}

px4_msgs::msg::TrajectorySetpoint VelocityController::build_zero_setpoint(uint64_t timestamp_us) const
{
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = timestamp_us;

  msg.position = {quiet_nan(), quiet_nan(), quiet_nan()};
  msg.velocity = {0.0f, 0.0f, 0.0f};
  msg.acceleration = {quiet_nan(), quiet_nan(), quiet_nan()};
  msg.jerk = {quiet_nan(), quiet_nan(), quiet_nan()};
  msg.yaw = quiet_nan();
  msg.yawspeed = 0.0f;

  return msg;
}

px4_msgs::msg::TrajectorySetpoint VelocityController::build_velocity_setpoint(
  const geometry_msgs::msg::TwistStamped & cmd,
  uint64_t timestamp_us) const
{
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = timestamp_us;

  // ENU(world) -> NED(world): (x_east, y_north, z_up) => (x_north, y_east, z_down)
  double v_n = cmd.twist.linear.y;
  double v_e = cmd.twist.linear.x;
  double v_d = -cmd.twist.linear.z;

  const double horizontal_norm = std::hypot(v_n, v_e);
  if (horizontal_norm > max_xy_speed_mps_ && horizontal_norm > kMinPositiveValue) {
    const double scale = max_xy_speed_mps_ / horizontal_norm;
    v_n *= scale;
    v_e *= scale;
  }

  v_d = std::clamp(v_d, -max_z_speed_mps_, max_z_speed_mps_);

  msg.position = {quiet_nan(), quiet_nan(), quiet_nan()};
  msg.velocity = {
    static_cast<float>(v_n),
    static_cast<float>(v_e),
    static_cast<float>(v_d)};
  msg.acceleration = {quiet_nan(), quiet_nan(), quiet_nan()};
  msg.jerk = {quiet_nan(), quiet_nan(), quiet_nan()};
  msg.yaw = quiet_nan();
  msg.yawspeed = clamp_abs(cmd.twist.angular.z, max_yawspeed_radps_);

  return msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityController>());
  rclcpp::shutdown();
  return 0;
}
