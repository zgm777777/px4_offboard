#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "offboard_manager/srv/emergency_disarm.hpp"
#include "offboard_manager/srv/request_arm.hpp"
#include "offboard_manager/srv/request_mode.hpp"
#include "offboard_manager/srv/takeoff.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyModeSwitcher final : public rclcpp::Node
{
public:
  JoyModeSwitcher()
  : Node("joy_mode_switcher")
  {
    topic_joy_ = declare_parameter<std::string>("topic_joy", "/joy");
    topic_velocity_cmd_ = declare_parameter<std::string>("topic_velocity_cmd", "/input/velocity_cmd");
    velocity_cmd_frame_id_ = declare_parameter<std::string>("velocity_cmd_frame_id", "map");

    service_request_mode_ = declare_parameter<std::string>(
      "services.request_mode", "/offboard/request_mode");
    service_request_arm_ = declare_parameter<std::string>(
      "services.request_arm", "/offboard/request_arm");
    service_emergency_disarm_ = declare_parameter<std::string>(
      "services.emergency_disarm", "/offboard/emergency_disarm");
    service_request_takeoff_ = declare_parameter<std::string>(
      "services.request_takeoff", "/offboard/request_takeoff");

    button_mode_ = declare_parameter<int>("buttons.mode", 0);
    button_arm_ = declare_parameter<int>("buttons.arm", 1);
    button_disarm_ = declare_parameter<int>("buttons.disarm", 2);
    button_emergency_ = declare_parameter<int>("buttons.emergency", 3);
    button_takeoff_ = declare_parameter<int>("buttons.takeoff", 6);

    takeoff_altitude_ = declare_parameter<double>("takeoff_altitude", 2.0);
    takeoff_altitude_ = std::max(0.1, takeoff_altitude_);

    min_trigger_interval_sec_ = declare_parameter<double>("min_trigger_interval_sec", 0.25);
    min_trigger_interval_sec_ = std::max(0.0, min_trigger_interval_sec_);

    publish_hz_ = declare_parameter<double>("publish_hz", 30.0);
    publish_hz_ = std::max(2.0, publish_hz_);
    joy_timeout_sec_ = declare_parameter<double>("joy_timeout_sec", 0.5);
    joy_timeout_sec_ = std::max(0.05, joy_timeout_sec_);

    require_deadman_ = declare_parameter<bool>("require_deadman", true);
    button_deadman_ = declare_parameter<int>("buttons.deadman", 5);

    axis_vx_ = declare_parameter<int>("axes.vx", 1);
    axis_vy_ = declare_parameter<int>("axes.vy", 0);
    axis_vz_ = declare_parameter<int>("axes.vz", 4);
    axis_yaw_rate_ = declare_parameter<int>("axes.yaw_rate", 3);

    max_vx_mps_ = declare_parameter<double>("limits.max_vx_mps", 2.0);
    max_vy_mps_ = declare_parameter<double>("limits.max_vy_mps", 2.0);
    max_vz_mps_ = declare_parameter<double>("limits.max_vz_mps", 1.0);
    max_yaw_rate_radps_ = declare_parameter<double>("limits.max_yaw_rate_radps", 1.2);
    deadzone_ = declare_parameter<double>("deadzone", 0.10);

    invert_vx_ = declare_parameter<bool>("invert.vx", false);
    invert_vy_ = declare_parameter<bool>("invert.vy", false);
    invert_vz_ = declare_parameter<bool>("invert.vz", false);
    invert_yaw_rate_ = declare_parameter<bool>("invert.yaw_rate", false);

    max_vx_mps_ = std::max(0.0, max_vx_mps_);
    max_vy_mps_ = std::max(0.0, max_vy_mps_);
    max_vz_mps_ = std::max(0.0, max_vz_mps_);
    max_yaw_rate_radps_ = std::max(0.0, max_yaw_rate_radps_);
    deadzone_ = std::clamp(deadzone_, 0.0, 0.95);

    emergency_reason_ = declare_parameter<std::string>("emergency_reason", "joy_button");

    request_mode_client_ = create_client<offboard_manager::srv::RequestMode>(service_request_mode_);
    request_arm_client_ = create_client<offboard_manager::srv::RequestArm>(service_request_arm_);
    emergency_disarm_client_ =
      create_client<offboard_manager::srv::EmergencyDisarm>(service_emergency_disarm_);
    request_takeoff_client_ = create_client<offboard_manager::srv::Takeoff>(service_request_takeoff_);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      topic_joy_,
      rclcpp::QoS(10),
      std::bind(&JoyModeSwitcher::on_joy, this, std::placeholders::_1));

    velocity_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      topic_velocity_cmd_,
      rclcpp::QoS(10));

    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / publish_hz_)),
      std::bind(&JoyModeSwitcher::publish_velocity_cmd, this));

    RCLCPP_INFO(
      get_logger(),
      "Started. joy=%s vel_cmd=%s mode=%d arm=%d disarm=%d emergency=%d deadman_required=%s",
      topic_joy_.c_str(),
      topic_velocity_cmd_.c_str(),
      button_mode_,
      button_arm_,
      button_disarm_,
      button_emergency_,
      require_deadman_ ? "true" : "false");
  }

private:
  static double clamp_unit(double value)
  {
    return std::clamp(value, -1.0, 1.0);
  }

  static double apply_deadzone(double value, double deadzone)
  {
    const double abs_value = std::fabs(value);
    if (abs_value <= deadzone) {
      return 0.0;
    }

    // Linear remap to keep full scale after deadzone.
    const double sign = value >= 0.0 ? 1.0 : -1.0;
    const double remapped = (abs_value - deadzone) / (1.0 - deadzone);
    return sign * std::clamp(remapped, 0.0, 1.0);
  }

  static double axis_or_zero(const std::vector<float> & axes, int index)
  {
    if (index < 0) {
      return 0.0;
    }
    const auto i = static_cast<size_t>(index);
    if (i >= axes.size()) {
      return 0.0;
    }
    return clamp_unit(static_cast<double>(axes[i]));
  }

  bool is_pressed(const std::vector<int32_t> & buttons, int index) const
  {
    if (index < 0) {
      return false;
    }
    const auto i = static_cast<size_t>(index);
    return i < buttons.size() && buttons[i] != 0;
  }

  bool is_rising_edge(const std::vector<int32_t> & buttons, int index) const
  {
    if (!has_prev_buttons_) {
      return false;
    }
    const bool current = is_pressed(buttons, index);
    const bool previous = is_pressed(prev_buttons_, index);
    return current && !previous;
  }

  bool allow_trigger(rclcpp::Time & last_trigger)
  {
    const auto now = get_clock()->now();
    if ((now - last_trigger).seconds() < min_trigger_interval_sec_) {
      return false;
    }
    last_trigger = now;
    return true;
  }

  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      latest_axes_ = msg->axes;
      latest_buttons_ = msg->buttons;
      last_joy_msg_time_ = get_clock()->now();
      has_joy_msg_ = true;
    }

    if (!has_prev_buttons_) {
      prev_buttons_ = msg->buttons;
      has_prev_buttons_ = true;
      return;
    }

    if (is_rising_edge(msg->buttons, button_mode_) && allow_trigger(last_mode_trigger_)) {
      request_mode();
    }

    if (is_rising_edge(msg->buttons, button_arm_) && allow_trigger(last_arm_trigger_)) {
      request_arm(true);
    }

    if (is_rising_edge(msg->buttons, button_disarm_) && allow_trigger(last_disarm_trigger_)) {
      request_arm(false);
    }

    if (is_rising_edge(msg->buttons, button_emergency_) && allow_trigger(last_emergency_trigger_)) {
      request_emergency_disarm();
    }

    if (is_rising_edge(msg->buttons, button_takeoff_) && allow_trigger(last_takeoff_trigger_)) {
      request_takeoff();
    }

    prev_buttons_ = msg->buttons;
  }

  void publish_velocity_cmd()
  {
    geometry_msgs::msg::TwistStamped msg{};
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = velocity_cmd_frame_id_;

    std::vector<float> axes;
    std::vector<int32_t> buttons;
    rclcpp::Time joy_stamp{0, 0, get_clock()->get_clock_type()};
    bool has_joy = false;

    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      axes = latest_axes_;
      buttons = latest_buttons_;
      joy_stamp = last_joy_msg_time_;
      has_joy = has_joy_msg_;
    }

    const bool joy_stale = (!has_joy) || ((get_clock()->now() - joy_stamp).seconds() > joy_timeout_sec_);
    const bool deadman_ok = !require_deadman_ || is_pressed(buttons, button_deadman_);

    if (!joy_stale && deadman_ok) {
      double vx = apply_deadzone(axis_or_zero(axes, axis_vx_), deadzone_) * max_vx_mps_;
      double vy = apply_deadzone(axis_or_zero(axes, axis_vy_), deadzone_) * max_vy_mps_;
      double vz = apply_deadzone(axis_or_zero(axes, axis_vz_), deadzone_) * max_vz_mps_;
      double yaw_rate = apply_deadzone(axis_or_zero(axes, axis_yaw_rate_), deadzone_) *
        max_yaw_rate_radps_;

      if (invert_vx_) {
        vx = -vx;
      }
      if (invert_vy_) {
        vy = -vy;
      }
      if (invert_vz_) {
        vz = -vz;
      }
      if (invert_yaw_rate_) {
        yaw_rate = -yaw_rate;
      }

      msg.twist.linear.x = vx;
      msg.twist.linear.y = vy;
      msg.twist.linear.z = vz;
      msg.twist.angular.z = yaw_rate;
    }

    velocity_cmd_pub_->publish(msg);
  }

  void request_mode()
  {
    if (!request_mode_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Service unavailable: %s", service_request_mode_.c_str());
      return;
    }

    auto request = std::make_shared<offboard_manager::srv::RequestMode::Request>();
    request_mode_client_->async_send_request(
      request,
      [this](rclcpp::Client<offboard_manager::srv::RequestMode>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(),
          "request_mode: success=%s msg=%s",
          response->success ? "true" : "false",
          response->message.c_str());
      });
  }

  void request_arm(bool arm)
  {
    if (!request_arm_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Service unavailable: %s", service_request_arm_.c_str());
      return;
    }

    auto request = std::make_shared<offboard_manager::srv::RequestArm::Request>();
    request->arm = arm;
    request_arm_client_->async_send_request(
      request,
      [this, arm](rclcpp::Client<offboard_manager::srv::RequestArm>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(),
          "request_arm(%s): success=%s msg=%s",
          arm ? "arm" : "disarm",
          response->success ? "true" : "false",
          response->message.c_str());
      });
  }

  void request_emergency_disarm()
  {
    if (!emergency_disarm_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Service unavailable: %s", service_emergency_disarm_.c_str());
      return;
    }

    auto request = std::make_shared<offboard_manager::srv::EmergencyDisarm::Request>();
    request->reason = emergency_reason_;
    emergency_disarm_client_->async_send_request(
      request,
      [this](rclcpp::Client<offboard_manager::srv::EmergencyDisarm>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(),
          "emergency_disarm: success=%s msg=%s",
          response->success ? "true" : "false",
          response->message.c_str());
      });
  }

  void request_takeoff()
  {
    if (!request_takeoff_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Service unavailable: %s", service_request_takeoff_.c_str());
      return;
    }

    auto request = std::make_shared<offboard_manager::srv::Takeoff::Request>();
    request->altitude = static_cast<float>(takeoff_altitude_);
    request_takeoff_client_->async_send_request(
      request,
      [this](rclcpp::Client<offboard_manager::srv::Takeoff>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(),
          "request_takeoff: success=%s msg=%s",
          response->success ? "true" : "false",
          response->message.c_str());
      });
  }

  std::string topic_joy_;
  std::string topic_velocity_cmd_;
  std::string velocity_cmd_frame_id_;
  std::string service_request_mode_;
  std::string service_request_arm_;
  std::string service_emergency_disarm_;
  std::string service_request_takeoff_;
  std::string emergency_reason_;

  int button_mode_{0};
  int button_arm_{1};
  int button_disarm_{2};
  int button_emergency_{3};
  int button_takeoff_{6};
  int button_deadman_{5};

  int axis_vx_{1};
  int axis_vy_{0};
  int axis_vz_{4};
  int axis_yaw_rate_{3};

  double min_trigger_interval_sec_{0.25};
  double publish_hz_{30.0};
  double joy_timeout_sec_{0.5};
  double deadzone_{0.10};
  double max_vx_mps_{2.0};
  double max_vy_mps_{2.0};
  double max_vz_mps_{1.0};
  double max_yaw_rate_radps_{1.2};
  double takeoff_altitude_{2.0};

  bool require_deadman_{true};
  bool invert_vx_{false};
  bool invert_vy_{false};
  bool invert_vz_{false};
  bool invert_yaw_rate_{false};

  rclcpp::Time last_mode_trigger_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_arm_trigger_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_disarm_trigger_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_emergency_trigger_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_takeoff_trigger_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joy_msg_time_{0, 0, RCL_ROS_TIME};

  bool has_prev_buttons_{false};
  bool has_joy_msg_{false};
  std::vector<int32_t> prev_buttons_;
  std::vector<float> latest_axes_;
  std::vector<int32_t> latest_buttons_;
  std::mutex joy_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_cmd_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Client<offboard_manager::srv::RequestMode>::SharedPtr request_mode_client_;
  rclcpp::Client<offboard_manager::srv::RequestArm>::SharedPtr request_arm_client_;
  rclcpp::Client<offboard_manager::srv::EmergencyDisarm>::SharedPtr emergency_disarm_client_;
  rclcpp::Client<offboard_manager::srv::Takeoff>::SharedPtr request_takeoff_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyModeSwitcher>());
  rclcpp::shutdown();
  return 0;
}
