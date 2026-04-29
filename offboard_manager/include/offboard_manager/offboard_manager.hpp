#pragma once

#include <memory>
#include <string>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "offboard_manager/srv/emergency_disarm.hpp"
#include "offboard_manager/srv/takeoff.hpp"
#include "offboard_manager/srv/request_arm.hpp"
#include "offboard_manager/srv/request_mode.hpp"

class OffboardManager final : public rclcpp::Node
{
public:
  explicit OffboardManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class ManagerState : uint8_t {
    IDLE = 0,
    REQUESTING_MODE,
    WAITING_ARM,
    ACTIVE,
    LANDING,
    EMERGENCY,
    EXITED
  };

  struct PendingModeRequest
  {
    bool active{false};
    std::shared_ptr<rclcpp::Service<offboard_manager::srv::RequestMode>> service;
    std::shared_ptr<rmw_request_id_t> request_header;
    rclcpp::Time deadline{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_command_sent{0, 0, RCL_ROS_TIME};
  };

  struct PendingArmRequest
  {
    bool active{false};
    bool arm{false};
    std::shared_ptr<rclcpp::Service<offboard_manager::srv::RequestArm>> service;
    std::shared_ptr<rmw_request_id_t> request_header;
    rclcpp::Time deadline{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_command_sent{0, 0, RCL_ROS_TIME};
  };

  rclcpp::QoS make_qos() const;
  uint64_t now_micros();

  void load_parameters();

  void on_vehicle_status(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void on_vehicle_land_detected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
  void on_vehicle_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  void on_request_mode(
    std::shared_ptr<rclcpp::Service<offboard_manager::srv::RequestMode>> service,
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<offboard_manager::srv::RequestMode::Request> request);

  void on_request_arm(
    std::shared_ptr<rclcpp::Service<offboard_manager::srv::RequestArm>> service,
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<offboard_manager::srv::RequestArm::Request> request);

  void on_emergency_disarm(
    std::shared_ptr<rclcpp::Service<offboard_manager::srv::EmergencyDisarm>> service,
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<offboard_manager::srv::EmergencyDisarm::Request> request);

  void on_request_takeoff(
    std::shared_ptr<rclcpp::Service<offboard_manager::srv::Takeoff>> service,
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<offboard_manager::srv::Takeoff::Request> request);

  void heartbeat_timer_cb();
  void gate_timer_cb();
  void state_machine_timer_cb();

  void publish_offboard_heartbeat();
  void publish_takeoff_position_setpoint();
  void publish_takeoff_override();
  void publish_control_enable();
  void publish_flight_state(const std::string & reason);
  void publish_alert(const std::string & level, const std::string & message);

  void transition_to(ManagerState next_state, const std::string & reason);
  std::string state_to_string(ManagerState s) const;

  bool is_offboard_confirmed() const;
  bool is_armed_confirmed() const;
  bool is_disarmed_confirmed() const;
  bool preflight_ok() const;

  void send_mode_command();
  void send_arm_command(bool arm);
  void send_takeoff_command(float altitude);
  void send_vehicle_command(
    uint32_t command,
    float param1 = 0.0f,
    float param2 = 0.0f,
    float param3 = 0.0f,
    float param4 = 0.0f,
    float param5 = 0.0f,
    float param6 = 0.0f,
    float param7 = 0.0f);

  void respond_mode_request(bool success, const std::string & message);
  void respond_arm_request(bool success, const std::string & message);

  void clear_pending_mode_request();
  void clear_pending_arm_request();

  // Parameters
  double heartbeat_hz_{10.0};
  double gate_publish_hz_{20.0};
  double state_machine_hz_{20.0};
  double request_timeout_sec_{3.0};
  double command_resend_sec_{0.2};
  double takeoff_timeout_sec_{8.0};
  double takeoff_reached_tolerance_m_{0.2};
  int debounce_frames_{2};
  bool allow_ground_control_while_landed_{false};

  int target_system_{1};
  int target_component_{1};
  int source_system_{1};
  int source_component_{1};

  std::string topic_fmu_in_offboard_control_mode_;
  std::string topic_fmu_in_vehicle_command_;
  std::string topic_fmu_out_vehicle_status_;
  std::string topic_fmu_out_vehicle_status_alt_;
  std::string topic_fmu_out_vehicle_land_detected_;
  std::string topic_fmu_out_vehicle_local_position_;
  std::string topic_fmu_in_trajectory_setpoint_;
  std::string topic_offboard_takeoff_override_;
  std::string topic_offboard_control_enable_;
  std::string topic_offboard_flight_state_;
  std::string topic_offboard_alert_;

  std::string service_request_mode_;
  std::string service_request_arm_;
  std::string service_emergency_disarm_;
  std::string service_request_takeoff_;

  // Runtime state
  ManagerState state_{ManagerState::IDLE};
  bool emergency_flag_{false};
  bool landed_{true};

  bool has_vehicle_status_{false};
  bool has_land_detected_{false};
  bool has_local_position_{false};

  px4_msgs::msg::VehicleStatus latest_vehicle_status_{};
  px4_msgs::msg::VehicleLandDetected latest_land_detected_{};
  px4_msgs::msg::VehicleLocalPosition latest_local_position_{};

  bool takeoff_active_{false};
  float takeoff_target_z_ned_{0.0f};
  float takeoff_hold_x_ned_{0.0f};
  float takeoff_hold_y_ned_{0.0f};
  rclcpp::Time takeoff_started_at_{0, 0, RCL_ROS_TIME};

  int offboard_confirm_frames_{0};
  int armed_confirm_frames_{0};
  int disarmed_confirm_frames_{0};
  int offboard_lost_frames_{0};

  PendingModeRequest pending_mode_request_{};
  PendingArmRequest pending_arm_request_{};

  // ROS interfaces
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeoff_override_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr control_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flight_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_alt_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr gate_timer_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;

  rclcpp::CallbackGroup::SharedPtr service_cb_group_;

  rclcpp::Service<offboard_manager::srv::RequestMode>::SharedPtr request_mode_srv_;
  rclcpp::Service<offboard_manager::srv::RequestArm>::SharedPtr request_arm_srv_;
  rclcpp::Service<offboard_manager::srv::EmergencyDisarm>::SharedPtr emergency_disarm_srv_;
  rclcpp::Service<offboard_manager::srv::Takeoff>::SharedPtr request_takeoff_srv_;
};
