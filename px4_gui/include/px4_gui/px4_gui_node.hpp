#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "offboard_manager/srv/emergency_disarm.hpp"
#include "offboard_manager/srv/request_arm.hpp"
#include "offboard_manager/srv/request_mode.hpp"
#include "offboard_manager/srv/takeoff.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QTimer>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QProgressBar>

class PX4GuiNode final : public QMainWindow
{
  Q_OBJECT

public:
  explicit PX4GuiNode(QWidget *parent = nullptr);
  ~PX4GuiNode();

private slots:
  void spinRos();
  void onRequestMode();
  void onRequestArm();
  void onRequestDisarm();
  void onEmergencyDisarm();
  void onRequestTakeoff();
  void onVelocityChanged();
  void onYawChanged();

private:
  void setupUi();
  void setupRos();

  void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void onLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
  void onFlightState(const std_msgs::msg::String::SharedPtr msg);
  void onAlert(const std_msgs::msg::String::SharedPtr msg);
  void onControlEnable(const std_msgs::msg::Bool::SharedPtr msg);
  void onActiveSource(const std_msgs::msg::String::SharedPtr msg);

  void publishVelocity();
  void publishSourceCmd(const std::string & source);

  std::string navStateString(uint8_t nav_state) const;
  std::string armingStateString(uint8_t arming_state) const;
  std::string armingStateColor(uint8_t arming_state) const;

  void logAppend(const std::string & msg);
  std::string navStateColor(uint8_t nav_state) const;

  // ROS
  rclcpp::Node::SharedPtr ros_node_;
  QTimer *ros_spin_timer_;

  rclcpp::Client<offboard_manager::srv::RequestMode>::SharedPtr mode_client_;
  rclcpp::Client<offboard_manager::srv::RequestArm>::SharedPtr arm_client_;
  rclcpp::Client<offboard_manager::srv::EmergencyDisarm>::SharedPtr emergency_client_;
  rclcpp::Client<offboard_manager::srv::Takeoff>::SharedPtr takeoff_client_;

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flight_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr alert_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_source_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_cmd_pub_;

  // State
  std::mutex state_mutex_;
  std::string flight_state_str_{"--"};
  std::string alert_str_{"--"};
  std::string active_source_str_{"--"};
  bool control_enable_ = false;
  uint8_t nav_state_ = 0;
  uint8_t arming_state_ = 0;
  bool landed_ = true;
  bool preflight_ok_ = false;

  // UI widgets
  QPushButton *btn_mode_;
  QPushButton *btn_arm_;
  QPushButton *btn_disarm_;
  QPushButton *btn_takeoff_;
  QPushButton *btn_emergency_;

  QPushButton *btn_src_hold_;
  QPushButton *btn_src_gui_;
  QPushButton *btn_src_joy_;
  QPushButton *btn_src_aruco_;

  QSlider *slider_vx_;
  QSlider *slider_vy_;
  QSlider *slider_vz_;
  QSlider *slider_yaw_;

  QDoubleSpinBox *spin_vx_;
  QDoubleSpinBox *spin_vy_;
  QDoubleSpinBox *spin_vz_;
  QDoubleSpinBox *spin_yaw_;

  QLabel *label_flight_state_;
  QLabel *label_arming_state_;
  QLabel *label_nav_mode_;
  QLabel *label_landed_;
  QLabel *label_alert_;
  QLabel *label_control_enable_;
  QLabel *label_active_source_;

  QDoubleSpinBox *spin_takeoff_alt_;

  QTextEdit *log_view_;
  int log_line_count_ = 0;

  // Parameters
  std::string velocity_cmd_topic_;
  double max_vx_{2.0};
  double max_vy_{2.0};
  double max_vz_{1.0};
  double max_yaw_{1.2};
};
