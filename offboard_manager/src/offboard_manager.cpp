#include "offboard_manager/offboard_manager.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <sstream>
#include <utility>

using namespace std::chrono_literals;

namespace
{
float quiet_nan()
{
  return std::numeric_limits<float>::quiet_NaN();
}
}  // namespace

OffboardManager::OffboardManager(const rclcpp::NodeOptions & options)
: Node("offboard_manager", options)
{
  load_parameters();

  const auto qos = make_qos();

  offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    topic_fmu_in_offboard_control_mode_, qos);
  trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    topic_fmu_in_trajectory_setpoint_, qos);
  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    topic_fmu_in_vehicle_command_, qos);
  takeoff_override_pub_ = create_publisher<std_msgs::msg::Bool>(topic_offboard_takeoff_override_, qos);
  control_enable_pub_ = create_publisher<std_msgs::msg::Bool>(topic_offboard_control_enable_, qos);
  flight_state_pub_ = create_publisher<std_msgs::msg::String>(topic_offboard_flight_state_, qos);
  alert_pub_ = create_publisher<std_msgs::msg::String>(topic_offboard_alert_, qos);

  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    topic_fmu_out_vehicle_status_, qos,
    std::bind(&OffboardManager::on_vehicle_status, this, std::placeholders::_1));
  if (!topic_fmu_out_vehicle_status_alt_.empty() &&
    topic_fmu_out_vehicle_status_alt_ != topic_fmu_out_vehicle_status_)
  {
    vehicle_status_alt_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      topic_fmu_out_vehicle_status_alt_, qos,
      std::bind(&OffboardManager::on_vehicle_status, this, std::placeholders::_1));
  }
  vehicle_land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
    topic_fmu_out_vehicle_land_detected_, qos,
    std::bind(&OffboardManager::on_vehicle_land_detected, this, std::placeholders::_1));
  vehicle_local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    topic_fmu_out_vehicle_local_position_, qos,
    std::bind(&OffboardManager::on_vehicle_local_position, this, std::placeholders::_1));

  service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  request_mode_srv_ = create_service<offboard_manager::srv::RequestMode>(
    service_request_mode_,
    std::bind(
      &OffboardManager::on_request_mode,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3),
    rmw_qos_profile_services_default,
    service_cb_group_);

  request_arm_srv_ = create_service<offboard_manager::srv::RequestArm>(
    service_request_arm_,
    std::bind(
      &OffboardManager::on_request_arm,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3),
    rmw_qos_profile_services_default,
    service_cb_group_);

  emergency_disarm_srv_ = create_service<offboard_manager::srv::EmergencyDisarm>(
    service_emergency_disarm_,
    std::bind(
      &OffboardManager::on_emergency_disarm,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3),
    rmw_qos_profile_services_default,
    service_cb_group_);

  request_takeoff_srv_ = create_service<offboard_manager::srv::Takeoff>(
    service_request_takeoff_,
    std::bind(
      &OffboardManager::on_request_takeoff,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3),
    rmw_qos_profile_services_default,
    service_cb_group_);

  heartbeat_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(heartbeat_hz_, 1.0))),
    std::bind(&OffboardManager::heartbeat_timer_cb, this));

  gate_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(gate_publish_hz_, 1.0))),
    std::bind(&OffboardManager::gate_timer_cb, this));

  state_machine_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / std::max(state_machine_hz_, 1.0))),
    std::bind(&OffboardManager::state_machine_timer_cb, this));

  transition_to(ManagerState::IDLE, "node_started");
  RCLCPP_INFO(get_logger(), "offboard_manager started.");
}

rclcpp::QoS OffboardManager::make_qos() const
{
  return rclcpp::QoS(10).best_effort().durability_volatile();
}

uint64_t OffboardManager::now_micros()
{
  return static_cast<uint64_t>(this->get_clock()->now().nanoseconds() / 1000ULL);
}

void OffboardManager::load_parameters()
{
  heartbeat_hz_ = declare_parameter<double>("heartbeat_hz", 10.0);
  gate_publish_hz_ = declare_parameter<double>("gate_publish_hz", 20.0);
  state_machine_hz_ = declare_parameter<double>("state_machine_hz", 20.0);
  request_timeout_sec_ = declare_parameter<double>("request_timeout_sec", 3.0);
  command_resend_sec_ = declare_parameter<double>("command_resend_sec", 0.2);
  takeoff_timeout_sec_ = declare_parameter<double>("takeoff_timeout_sec", 8.0);
  takeoff_reached_tolerance_m_ = declare_parameter<double>("takeoff_reached_tolerance_m", 0.2);
  debounce_frames_ = declare_parameter<int>("debounce_frames", 2);
  allow_ground_control_while_landed_ = declare_parameter<bool>(
    "allow_ground_control_while_landed", true);

  target_system_ = declare_parameter<int>("target_system", 1);
  target_component_ = declare_parameter<int>("target_component", 1);
  source_system_ = declare_parameter<int>("source_system", 1);
  source_component_ = declare_parameter<int>("source_component", 1);

  topic_fmu_in_offboard_control_mode_ = declare_parameter<std::string>(
    "topics.fmu_in_offboard_control_mode", "/fmu/in/offboard_control_mode");
  topic_fmu_in_vehicle_command_ = declare_parameter<std::string>(
    "topics.fmu_in_vehicle_command", "/fmu/in/vehicle_command");
  topic_fmu_out_vehicle_status_ = declare_parameter<std::string>(
    "topics.fmu_out_vehicle_status", "/fmu/out/vehicle_status");
  topic_fmu_out_vehicle_status_alt_ = declare_parameter<std::string>(
    "topics.fmu_out_vehicle_status_alt", "/fmu/out/vehicle_status_v1");
  topic_fmu_out_vehicle_land_detected_ = declare_parameter<std::string>(
    "topics.fmu_out_vehicle_land_detected", "/fmu/out/vehicle_land_detected");
  topic_fmu_out_vehicle_local_position_ = declare_parameter<std::string>(
    "topics.fmu_out_vehicle_local_position", "/fmu/out/vehicle_local_position");
  topic_fmu_in_trajectory_setpoint_ = declare_parameter<std::string>(
    "topics.fmu_in_trajectory_setpoint", "/fmu/in/trajectory_setpoint");
  topic_offboard_takeoff_override_ = declare_parameter<std::string>(
    "topics.offboard_takeoff_override", "/offboard/takeoff_override");
  topic_offboard_control_enable_ = declare_parameter<std::string>(
    "topics.offboard_control_enable", "/offboard/control_enable");
  topic_offboard_flight_state_ = declare_parameter<std::string>(
    "topics.offboard_flight_state", "/offboard/flight_state");
  topic_offboard_alert_ = declare_parameter<std::string>(
    "topics.offboard_alert", "/offboard/alert");

  service_request_mode_ = declare_parameter<std::string>(
    "services.request_mode", "/offboard/request_mode");
  service_request_arm_ = declare_parameter<std::string>(
    "services.request_arm", "/offboard/request_arm");
  service_emergency_disarm_ = declare_parameter<std::string>(
    "services.emergency_disarm", "/offboard/emergency_disarm");
  service_request_takeoff_ = declare_parameter<std::string>(
    "services.request_takeoff", "/offboard/request_takeoff");

  debounce_frames_ = std::max(1, debounce_frames_);
  gate_publish_hz_ = std::max(10.0, gate_publish_hz_);
  takeoff_timeout_sec_ = std::max(1.0, takeoff_timeout_sec_);
  takeoff_reached_tolerance_m_ = std::clamp(takeoff_reached_tolerance_m_, 0.05, 1.0);
}

void OffboardManager::on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  latest_vehicle_status_ = *msg;
  has_vehicle_status_ = true;
}

void OffboardManager::on_vehicle_land_detected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  latest_land_detected_ = *msg;
  has_land_detected_ = true;
  landed_ = msg->landed;
}

void OffboardManager::on_vehicle_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  latest_local_position_ = *msg;
  has_local_position_ = true;
}

void OffboardManager::heartbeat_timer_cb()
{
  publish_offboard_heartbeat();
  publish_takeoff_position_setpoint();
}

void OffboardManager::gate_timer_cb()
{
  publish_takeoff_override();
  publish_control_enable();
}

void OffboardManager::state_machine_timer_cb()
{
  const auto now = get_clock()->now();

  if (takeoff_active_) {
    const bool timeout = (now - takeoff_started_at_).seconds() > takeoff_timeout_sec_;
    const bool reached_target = has_local_position_ &&
      (latest_local_position_.z <= (takeoff_target_z_ned_ + static_cast<float>(takeoff_reached_tolerance_m_)));
    if (!is_armed_confirmed()) {
      takeoff_active_ = false;
      publish_alert("WARN", "Takeoff override canceled: vehicle no longer armed.");
    } else if (timeout) {
      takeoff_active_ = false;
      publish_alert("WARN", "Takeoff override timeout; fallback to velocity control gate.");
    } else if (reached_target) {
      takeoff_active_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Takeoff override finished: target reached (z=%.2f target=%.2f).",
        latest_local_position_.z,
        takeoff_target_z_ned_);
    }
  }

  if (has_vehicle_status_) {
    if (latest_vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      offboard_confirm_frames_++;
      offboard_lost_frames_ = 0;
    } else {
      offboard_confirm_frames_ = 0;
      offboard_lost_frames_++;
    }

    if (latest_vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      armed_confirm_frames_++;
      disarmed_confirm_frames_ = 0;
    } else {
      armed_confirm_frames_ = 0;
      disarmed_confirm_frames_++;
    }
  }

  if (emergency_flag_ && state_ != ManagerState::EMERGENCY) {
    transition_to(ManagerState::EMERGENCY, "emergency_latched");
  }

  if (pending_mode_request_.active) {
    const bool resend_needed = (now - pending_mode_request_.last_command_sent).seconds() >= command_resend_sec_;
    if (resend_needed) {
      send_mode_command();
      pending_mode_request_.last_command_sent = now;
    }

    if (is_offboard_confirmed()) {
      transition_to(ManagerState::WAITING_ARM, "offboard_confirmed");
      respond_mode_request(true, "OFFBOARD mode confirmed.");
    } else if (now > pending_mode_request_.deadline) {
      publish_alert("WARN", "request_mode timeout, fallback to IDLE.");
      transition_to(ManagerState::IDLE, "request_mode_timeout");
      respond_mode_request(false, "Timeout waiting OFFBOARD confirmation.");
    }
  }

  if (pending_arm_request_.active) {
    const bool resend_needed = (now - pending_arm_request_.last_command_sent).seconds() >= command_resend_sec_;
    if (resend_needed) {
      send_arm_command(pending_arm_request_.arm);
      pending_arm_request_.last_command_sent = now;
    }

    const bool confirmed = pending_arm_request_.arm ? is_armed_confirmed() : is_disarmed_confirmed();
    if (confirmed) {
      if (pending_arm_request_.arm) {
        transition_to(ManagerState::ACTIVE, "arm_confirmed");
      } else {
        transition_to(ManagerState::IDLE, "disarm_confirmed");
      }
      respond_arm_request(true, pending_arm_request_.arm ? "Armed confirmed." : "Disarm confirmed.");
    } else if (now > pending_arm_request_.deadline) {
      publish_alert("WARN", "request_arm timeout, fallback to IDLE.");
      transition_to(ManagerState::IDLE, "request_arm_timeout");
      respond_arm_request(false, "Timeout waiting arm/disarm confirmation.");
    }
  }

  switch (state_) {
    case ManagerState::ACTIVE:
      if (emergency_flag_) {
        transition_to(ManagerState::EMERGENCY, "emergency_latched");
      } else if (landed_ && !allow_ground_control_while_landed_) {
        transition_to(ManagerState::LANDING, "land_detected");
      } else if (offboard_lost_frames_ >= debounce_frames_) {
        transition_to(ManagerState::EXITED, "offboard_lost");
        publish_alert("WARN", "Vehicle exited OFFBOARD mode.");
      }
      break;
    case ManagerState::LANDING:
      if (is_disarmed_confirmed()) {
        transition_to(ManagerState::IDLE, "landing_disarmed");
      }
      break;
    case ManagerState::EXITED:
      if (!pending_mode_request_.active && !pending_arm_request_.active) {
        transition_to(ManagerState::IDLE, "exited_to_idle");
      }
      break;
    default:
      break;
  }
}

void OffboardManager::publish_offboard_heartbeat()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = now_micros();
  msg.position = takeoff_active_;
  msg.velocity = !takeoff_active_;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;
  offboard_control_mode_pub_->publish(msg);

  RCLCPP_DEBUG(
    get_logger(),
    "Published Offboard heartbeat (position=%s velocity=%s).",
    msg.position ? "true" : "false",
    msg.velocity ? "true" : "false");
}

void OffboardManager::publish_takeoff_position_setpoint()
{
  if (!takeoff_active_) {
    return;
  }

  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.timestamp = now_micros();
  sp.position = {
    takeoff_hold_x_ned_,
    takeoff_hold_y_ned_,
    takeoff_target_z_ned_};
  sp.velocity = {quiet_nan(), quiet_nan(), quiet_nan()};
  sp.acceleration = {quiet_nan(), quiet_nan(), quiet_nan()};
  sp.jerk = {quiet_nan(), quiet_nan(), quiet_nan()};
  sp.yaw = quiet_nan();
  sp.yawspeed = quiet_nan();
  trajectory_setpoint_pub_->publish(sp);
}

void OffboardManager::publish_takeoff_override()
{
  std_msgs::msg::Bool msg{};
  msg.data = takeoff_active_;
  takeoff_override_pub_->publish(msg);
}

void OffboardManager::publish_control_enable()
{
  std_msgs::msg::Bool msg{};
  if (takeoff_active_) {
    msg.data = false;
    control_enable_pub_->publish(msg);
    RCLCPP_DEBUG(get_logger(), "Published control_enable=false (takeoff override active).");
    return;
  }

  const bool landed_gate_ok = allow_ground_control_while_landed_ || (!landed_);
  msg.data = (state_ == ManagerState::ACTIVE) && landed_gate_ok && (!emergency_flag_);
  control_enable_pub_->publish(msg);
  RCLCPP_DEBUG(get_logger(), "Published control_enable=%s", msg.data ? "true" : "false");
}

void OffboardManager::publish_flight_state(const std::string & reason)
{
  std_msgs::msg::String msg{};
  std::ostringstream ss;
  ss << state_to_string(state_) << "|" << reason;
  msg.data = ss.str();
  flight_state_pub_->publish(msg);
}

void OffboardManager::publish_alert(const std::string & level, const std::string & message)
{
  std_msgs::msg::String msg{};
  msg.data = level + ":" + message;
  alert_pub_->publish(msg);

  if (level == "ERROR" || level == "CRITICAL") {
    RCLCPP_ERROR(get_logger(), "%s", msg.data.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "%s", msg.data.c_str());
  }
}

void OffboardManager::transition_to(ManagerState next_state, const std::string & reason)
{
  if (state_ == next_state) {
    return;
  }

  state_ = next_state;
  RCLCPP_INFO(
    get_logger(), "State changed to %s (reason=%s)", state_to_string(state_).c_str(), reason.c_str());
  publish_flight_state(reason);
}

std::string OffboardManager::state_to_string(ManagerState s) const
{
  switch (s) {
    case ManagerState::IDLE:
      return "IDLE";
    case ManagerState::REQUESTING_MODE:
      return "REQUESTING_MODE";
    case ManagerState::WAITING_ARM:
      return "WAITING_ARM";
    case ManagerState::ACTIVE:
      return "ACTIVE";
    case ManagerState::LANDING:
      return "LANDING";
    case ManagerState::EMERGENCY:
      return "EMERGENCY";
    case ManagerState::EXITED:
      return "EXITED";
    default:
      return "UNKNOWN";
  }
}

bool OffboardManager::is_offboard_confirmed() const
{
  return offboard_confirm_frames_ >= debounce_frames_;
}

bool OffboardManager::is_armed_confirmed() const
{
  return armed_confirm_frames_ >= debounce_frames_;
}

bool OffboardManager::is_disarmed_confirmed() const
{
  return disarmed_confirm_frames_ >= debounce_frames_;
}

bool OffboardManager::preflight_ok() const
{
  return has_vehicle_status_ && latest_vehicle_status_.pre_flight_checks_pass;
}

void OffboardManager::send_mode_command()
{
  // For PX4, DO_SET_MODE with param1=1 enables custom mode usage, and param2=6 requests OFFBOARD.
  send_vehicle_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
    1.0f,
    6.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f);
}

void OffboardManager::send_arm_command(bool arm)
{
  const float arm_param = arm ?
    static_cast<float>(px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM) :
    static_cast<float>(px4_msgs::msg::VehicleCommand::ARMING_ACTION_DISARM);

  send_vehicle_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    arm_param,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f);
}

void OffboardManager::send_takeoff_command(float altitude)
{
  if (!has_local_position_) {
    return;
  }

  // NED uses positive down. To climb by altitude meters, decrease local z target.
  takeoff_hold_x_ned_ = latest_local_position_.x;
  takeoff_hold_y_ned_ = latest_local_position_.y;
  takeoff_target_z_ned_ = latest_local_position_.z - altitude;
  takeoff_started_at_ = get_clock()->now();
  takeoff_active_ = true;
}

void OffboardManager::send_vehicle_command(
  uint32_t command,
  float param1,
  float param2,
  float param3,
  float param4,
  float param5,
  float param6,
  float param7)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = now_micros();
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.param5 = static_cast<double>(param5);
  msg.param6 = static_cast<double>(param6);
  msg.param7 = param7;
  msg.command = command;
  msg.target_system = static_cast<uint8_t>(target_system_);
  msg.target_component = static_cast<uint8_t>(target_component_);
  msg.source_system = static_cast<uint8_t>(source_system_);
  msg.source_component = static_cast<uint16_t>(source_component_);
  msg.confirmation = 0;
  msg.from_external = true;
  vehicle_command_pub_->publish(msg);
}

void OffboardManager::on_request_mode(
  std::shared_ptr<rclcpp::Service<offboard_manager::srv::RequestMode>> service,
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<offboard_manager::srv::RequestMode::Request> request)
{
  (void)request;

  offboard_manager::srv::RequestMode::Response resp{};

  if (pending_mode_request_.active) {
    resp.success = false;
    resp.message = "request_mode busy: previous request still running.";
    service->send_response(*request_header, resp);
    return;
  }

  if (emergency_flag_) {
    resp.success = false;
    resp.message = "request_mode rejected: emergency latch active.";
    service->send_response(*request_header, resp);
    return;
  }

  if (!has_vehicle_status_) {
    resp.success = false;
    resp.message = "request_mode rejected: vehicle_status not received.";
    service->send_response(*request_header, resp);
    return;
  }

  if (!preflight_ok()) {
    resp.success = false;
    resp.message = "request_mode rejected: pre-flight checks not passed.";
    service->send_response(*request_header, resp);
    return;
  }

  if (is_offboard_confirmed()) {
    transition_to(ManagerState::WAITING_ARM, "offboard_already_confirmed");
    resp.success = true;
    resp.message = "Already in OFFBOARD mode.";
    service->send_response(*request_header, resp);
    return;
  }

  pending_mode_request_.active = true;
  pending_mode_request_.service = std::move(service);
  pending_mode_request_.request_header = std::move(request_header);
  pending_mode_request_.deadline = get_clock()->now() + rclcpp::Duration::from_seconds(request_timeout_sec_);
  pending_mode_request_.last_command_sent = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  transition_to(ManagerState::REQUESTING_MODE, "request_mode_received");
}

void OffboardManager::on_request_arm(
  std::shared_ptr<rclcpp::Service<offboard_manager::srv::RequestArm>> service,
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<offboard_manager::srv::RequestArm::Request> request)
{
  offboard_manager::srv::RequestArm::Response resp{};

  if (pending_arm_request_.active) {
    resp.success = false;
    resp.message = "request_arm busy: previous request still running.";
    service->send_response(*request_header, resp);
    return;
  }

  if (emergency_flag_) {
    resp.success = false;
    resp.message = "request_arm rejected: emergency latch active.";
    service->send_response(*request_header, resp);
    return;
  }

  if (!has_vehicle_status_) {
    resp.success = false;
    resp.message = "request_arm rejected: vehicle_status not received.";
    service->send_response(*request_header, resp);
    return;
  }

  if (request->arm) {
    if (!is_offboard_confirmed()) {
      resp.success = false;
      resp.message = "request_arm rejected: OFFBOARD mode is not confirmed.";
      service->send_response(*request_header, resp);
      return;
    }
  }

  if (!request->arm && is_disarmed_confirmed()) {
    transition_to(ManagerState::IDLE, "already_disarmed");
    resp.success = true;
    resp.message = "Vehicle already disarmed.";
    service->send_response(*request_header, resp);
    return;
  }

  pending_arm_request_.active = true;
  pending_arm_request_.arm = request->arm;
  pending_arm_request_.service = std::move(service);
  pending_arm_request_.request_header = std::move(request_header);
  pending_arm_request_.deadline = get_clock()->now() + rclcpp::Duration::from_seconds(request_timeout_sec_);
  pending_arm_request_.last_command_sent = rclcpp::Time(0, 0, get_clock()->get_clock_type());

  if (request->arm) {
    transition_to(ManagerState::WAITING_ARM, "request_arm_received");
  }
}

void OffboardManager::on_emergency_disarm(
  std::shared_ptr<rclcpp::Service<offboard_manager::srv::EmergencyDisarm>> service,
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<offboard_manager::srv::EmergencyDisarm::Request> request)
{
  offboard_manager::srv::EmergencyDisarm::Response resp{};

  if (!emergency_flag_) {
    emergency_flag_ = true;
    transition_to(ManagerState::EMERGENCY, "emergency_disarm_request");

    // Emergency path is atomic by design: latch flag first, then force disarm and close gate.
    send_arm_command(false);
    publish_control_enable();

    const std::string reason = request->reason.empty() ? "operator_request" : request->reason;
    publish_alert("CRITICAL", "Emergency disarm latched: " + reason);

    if (pending_mode_request_.active) {
      respond_mode_request(false, "Canceled by emergency_disarm.");
    }
    if (pending_arm_request_.active) {
      respond_arm_request(false, "Canceled by emergency_disarm.");
    }

    resp.success = true;
    resp.message = "Emergency disarm executed and latched.";
  } else {
    resp.success = true;
    resp.message = "Emergency already latched.";
  }

  service->send_response(*request_header, resp);
}

void OffboardManager::on_request_takeoff(
  std::shared_ptr<rclcpp::Service<offboard_manager::srv::Takeoff>> service,
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<offboard_manager::srv::Takeoff::Request> request)
{
  offboard_manager::srv::Takeoff::Response resp{};

  if (emergency_flag_) {
    resp.success = false;
    resp.message = "request_takeoff rejected: emergency latch active.";
    service->send_response(*request_header, resp);
    return;
  }

  if (!has_vehicle_status_) {
    resp.success = false;
    resp.message = "request_takeoff rejected: vehicle_status not received.";
    service->send_response(*request_header, resp);
    return;
  }

  if (!has_local_position_) {
    resp.success = false;
    resp.message = "request_takeoff rejected: local_position not received.";
    service->send_response(*request_header, resp);
    return;
  }

  if ((state_ != ManagerState::WAITING_ARM) &&
    (state_ != ManagerState::ACTIVE) &&
    (!is_armed_confirmed()))
  {
    resp.success = false;
    resp.message = "request_takeoff rejected: vehicle is not armed and not in WAITING_ARM/ACTIVE.";
    service->send_response(*request_header, resp);
    return;
  }

  if (request->altitude <= 0.0f) {
    resp.success = false;
    resp.message = "request_takeoff rejected: altitude must be > 0.";
    service->send_response(*request_header, resp);
    return;
  }

  send_takeoff_command(request->altitude);
  resp.success = true;
  resp.message = "Takeoff position override started.";
  service->send_response(*request_header, resp);

  RCLCPP_INFO(
    get_logger(),
    "request_takeoff: climb=%.2fm target_z_ned=%.2f hold=(%.2f, %.2f)",
    request->altitude,
    takeoff_target_z_ned_,
    takeoff_hold_x_ned_,
    takeoff_hold_y_ned_);
}

void OffboardManager::respond_mode_request(bool success, const std::string & message)
{
  if (!pending_mode_request_.active || !pending_mode_request_.service || !pending_mode_request_.request_header) {
    clear_pending_mode_request();
    return;
  }

  offboard_manager::srv::RequestMode::Response resp{};
  resp.success = success;
  resp.message = message;
  pending_mode_request_.service->send_response(*pending_mode_request_.request_header, resp);
  clear_pending_mode_request();
}

void OffboardManager::respond_arm_request(bool success, const std::string & message)
{
  if (!pending_arm_request_.active || !pending_arm_request_.service || !pending_arm_request_.request_header) {
    clear_pending_arm_request();
    return;
  }

  offboard_manager::srv::RequestArm::Response resp{};
  resp.success = success;
  resp.message = message;
  pending_arm_request_.service->send_response(*pending_arm_request_.request_header, resp);
  clear_pending_arm_request();
}

void OffboardManager::clear_pending_mode_request()
{
  pending_mode_request_ = PendingModeRequest{};
}

void OffboardManager::clear_pending_arm_request()
{
  pending_arm_request_ = PendingArmRequest{};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardManager>());
  rclcpp::shutdown();
  return 0;
}
