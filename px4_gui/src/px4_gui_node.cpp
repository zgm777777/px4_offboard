#include "px4_gui/px4_gui_node.hpp"

#include <algorithm>
#include <sstream>

#include <QFont>
#include <QDateTime>
#include <QApplication>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QMessageBox>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QPushButton>
#include <QScrollBar>
#include <QProgressBar>

using namespace std::chrono_literals;

PX4GuiNode::PX4GuiNode(QWidget *parent)
: QMainWindow(parent)
{
  setupUi();
  setupRos();

  ros_spin_timer_ = new QTimer(this);
  connect(ros_spin_timer_, &QTimer::timeout, this, &PX4GuiNode::spinRos);
  ros_spin_timer_->start(50);

  setWindowTitle("PX4 Offboard Control");
  resize(900, 620);
}

PX4GuiNode::~PX4GuiNode()
{
  ros_spin_timer_->stop();
}

// ---------------------------------------------------------------------------
// UI setup
// ---------------------------------------------------------------------------
void PX4GuiNode::setupUi()
{
  auto *central = new QWidget(this);
  auto *main_layout = new QHBoxLayout(central);

  // ---- Left panel: Mode / Arming controls ----
  auto *left_panel = new QVBoxLayout();

  auto *mode_box = new QGroupBox("Mode & Arming");
  auto *mode_layout = new QVBoxLayout();

  btn_mode_ = new QPushButton("Request OFFBOARD Mode");
  btn_mode_->setMinimumHeight(40);
  btn_mode_->setStyleSheet("background-color: #4a90d9; color: white; font-weight: bold; font-size: 13px;");

  btn_arm_ = new QPushButton("Arm");
  btn_arm_->setMinimumHeight(40);
  btn_arm_->setStyleSheet("background-color: #4caf50; color: white; font-weight: bold; font-size: 13px;");

  btn_disarm_ = new QPushButton("Disarm");
  btn_disarm_->setMinimumHeight(40);
  btn_disarm_->setStyleSheet("background-color: #ff9800; color: white; font-weight: bold; font-size: 13px;");

  btn_takeoff_ = new QPushButton("Takeoff");
  btn_takeoff_->setMinimumHeight(40);
  btn_takeoff_->setStyleSheet("background-color: #2196f3; color: white; font-weight: bold; font-size: 13px;");

  auto *takeoff_row = new QHBoxLayout();
  takeoff_row->addWidget(new QLabel("Altitude (m):"));
  spin_takeoff_alt_ = new QDoubleSpinBox();
  spin_takeoff_alt_->setRange(0.5, 50.0);
  spin_takeoff_alt_->setSingleStep(0.5);
  spin_takeoff_alt_->setValue(5.0);
  spin_takeoff_alt_->setMaximumWidth(80);
  takeoff_row->addWidget(spin_takeoff_alt_);
  takeoff_row->addWidget(btn_takeoff_);

  btn_emergency_ = new QPushButton("EMERGENCY DISARM");
  btn_emergency_->setMinimumHeight(48);
  btn_emergency_->setStyleSheet(
    "background-color: #d32f2f; color: white; font-weight: bold; font-size: 14px;");

  connect(btn_mode_, &QPushButton::clicked, this, &PX4GuiNode::onRequestMode);
  connect(btn_arm_, &QPushButton::clicked, this, &PX4GuiNode::onRequestArm);
  connect(btn_disarm_, &QPushButton::clicked, this, &PX4GuiNode::onRequestDisarm);
  connect(btn_takeoff_, &QPushButton::clicked, this, &PX4GuiNode::onRequestTakeoff);
  connect(btn_emergency_, &QPushButton::clicked, this, &PX4GuiNode::onEmergencyDisarm);

  mode_layout->addWidget(btn_mode_);
  mode_layout->addWidget(btn_arm_);
  mode_layout->addWidget(btn_disarm_);
  mode_layout->addLayout(takeoff_row);
  mode_layout->addSpacing(10);
  mode_layout->addWidget(btn_emergency_);
  mode_box->setLayout(mode_layout);

  left_panel->addWidget(mode_box);

  // ---- Source selection ----
  auto *src_box = new QGroupBox("Control Source");
  auto *src_layout = new QVBoxLayout();

  auto make_src_btn = [](const QString & text, const QString & color, const QString & textColor) {
    auto *btn = new QPushButton(text);
    btn->setMinimumHeight(34);
    btn->setStyleSheet(
      QString("background-color: %1; color: %2; font-weight: bold; font-size: 12px;")
        .arg(color, textColor));
    return btn;
  };

  btn_src_hold_  = make_src_btn("HOLD",  "#616161", "white");
  btn_src_gui_   = make_src_btn("GUI",   "#6a1b9a", "white");
  btn_src_joy_   = make_src_btn("JOY",   "#e65100", "white");
  btn_src_aruco_ = make_src_btn("ArUco", "#1565c0", "white");

  connect(btn_src_hold_,  &QPushButton::clicked, this, [this]() { publishSourceCmd("hold"); });
  connect(btn_src_gui_,   &QPushButton::clicked, this, [this]() { publishSourceCmd("gui"); });
  connect(btn_src_joy_,   &QPushButton::clicked, this, [this]() { publishSourceCmd("joy"); });
  connect(btn_src_aruco_, &QPushButton::clicked, this, [this]() { publishSourceCmd("aruco"); });

  src_layout->addWidget(btn_src_hold_);
  src_layout->addWidget(btn_src_gui_);
  src_layout->addWidget(btn_src_joy_);
  src_layout->addWidget(btn_src_aruco_);
  src_box->setLayout(src_layout);

  left_panel->addWidget(src_box);
  left_panel->addStretch();

  // ---- Center panel: Velocity control ----
  auto *center_panel = new QVBoxLayout();

  auto *vel_box = new QGroupBox("Velocity Control");
  auto *vel_layout = new QGridLayout();

  auto add_slider = [vel_layout, this](const char *label_text,
                                        double max_val, QSlider *&slider, QDoubleSpinBox *&spin) {
    auto *lbl = new QLabel(label_text);
    lbl->setMinimumWidth(50);

    slider = new QSlider(Qt::Horizontal);
    slider->setRange(-100, 100);
    slider->setValue(0);

    spin = new QDoubleSpinBox();
    spin->setRange(-max_val, max_val);
    spin->setSingleStep(0.1);
    spin->setDecimals(2);
    spin->setValue(0.0);
    spin->setSuffix(" m/s");
    spin->setMaximumWidth(120);

    auto *row = new QHBoxLayout();
    row->addWidget(lbl);
    row->addWidget(slider);
    row->addWidget(spin);

    int row_idx = 0;
    if (label_text == QString("Vx")) row_idx = 0;
    else if (label_text == QString("Vy")) row_idx = 1;
    else if (label_text == QString("Vz")) row_idx = 2;
    else row_idx = 3;

    vel_layout->addLayout(row, row_idx, 0);

    QObject::connect(slider, &QSlider::valueChanged, spin, [spin, max_val](int v) {
      spin->blockSignals(true);
      spin->setValue(v / 100.0 * max_val);
      spin->blockSignals(false);
    });
    QObject::connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     slider, [slider, max_val](double v) {
      slider->blockSignals(true);
      slider->setValue(static_cast<int>(v / max_val * 100.0));
      slider->blockSignals(false);
    });
  };

  add_slider("Vx", max_vx_, slider_vx_, spin_vx_);
  add_slider("Vy", max_vy_, slider_vy_, spin_vy_);
  add_slider("Vz", max_vz_, slider_vz_, spin_vz_);
  add_slider("Yaw", max_yaw_, slider_yaw_, spin_yaw_);

  // Fix suffix for yaw
  spin_yaw_->setSuffix(" rad/s");

  connect(slider_vx_, &QSlider::valueChanged, this, &PX4GuiNode::onVelocityChanged);
  connect(slider_vy_, &QSlider::valueChanged, this, &PX4GuiNode::onVelocityChanged);
  connect(slider_vz_, &QSlider::valueChanged, this, &PX4GuiNode::onVelocityChanged);
  connect(slider_yaw_, &QSlider::valueChanged, this, &PX4GuiNode::onYawChanged);
  connect(spin_vx_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &PX4GuiNode::onVelocityChanged);
  connect(spin_vy_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &PX4GuiNode::onVelocityChanged);
  connect(spin_vz_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &PX4GuiNode::onVelocityChanged);
  connect(spin_yaw_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &PX4GuiNode::onYawChanged);

  vel_box->setLayout(vel_layout);
  center_panel->addWidget(vel_box);
  center_panel->addStretch();

  // ---- Right panel: Status ----
  auto *right_panel = new QVBoxLayout();

  auto *status_box = new QGroupBox("Status");
  auto *status_layout = new QGridLayout();

  auto add_status_row = [status_layout](const char *label_text, QLabel *&value_label, int row) {
    auto *lbl = new QLabel(label_text);
    lbl->setMinimumWidth(100);
    value_label = new QLabel("--");
    value_label->setMinimumWidth(180);
    value_label->setStyleSheet("background-color: #2a2a2a; color: #e0e0e0; padding: 4px; border-radius: 3px;");
    QFont font = value_label->font();
    font.setBold(true);
    value_label->setFont(font);
    status_layout->addWidget(lbl, row, 0);
    status_layout->addWidget(value_label, row, 1);
  };

  add_status_row("Flight State:", label_flight_state_, 0);
  add_status_row("Arming State:", label_arming_state_, 1);
  add_status_row("Nav Mode:", label_nav_mode_, 2);
  add_status_row("Landed:", label_landed_, 3);
  add_status_row("Control Enable:", label_control_enable_, 4);
  add_status_row("Vel Source:", label_active_source_, 5);
  add_status_row("Alert:", label_alert_, 6);

  status_box->setLayout(status_layout);
  right_panel->addWidget(status_box);

  // Log area
  auto *log_box = new QGroupBox("Log");
  auto *log_layout = new QVBoxLayout();
  log_view_ = new QTextEdit();
  log_view_->setReadOnly(true);
  log_view_->setMaximumHeight(160);
  log_view_->setStyleSheet("background-color: #1a1a1a; color: #c0c0c0; font-family: monospace; font-size: 11px;");
  log_layout->addWidget(log_view_);
  log_box->setLayout(log_layout);
  right_panel->addWidget(log_box);

  right_panel->addStretch();

  // Assemble
  main_layout->addLayout(left_panel);
  main_layout->addLayout(center_panel);
  main_layout->addLayout(right_panel);

  setCentralWidget(central);
}

// ---------------------------------------------------------------------------
// ROS 2 setup
// ---------------------------------------------------------------------------
void PX4GuiNode::setupRos()
{
  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(false);
  ros_node_ = std::make_shared<rclcpp::Node>("px4_gui", opts);

  // Parameters
  velocity_cmd_topic_ = ros_node_->declare_parameter<std::string>(
    "velocity_cmd_topic", "/gui/velocity_cmd");
  max_vx_ = ros_node_->declare_parameter<double>("max_vx", 2.0);
  max_vy_ = ros_node_->declare_parameter<double>("max_vy", 2.0);
  max_vz_ = ros_node_->declare_parameter<double>("max_vz", 1.0);
  max_yaw_ = ros_node_->declare_parameter<double>("max_yaw", 1.2);

  const auto qos = rclcpp::QoS(10).best_effort().durability_volatile();

  mode_client_ = ros_node_->create_client<offboard_manager::srv::RequestMode>("/offboard/request_mode");
  arm_client_ = ros_node_->create_client<offboard_manager::srv::RequestArm>("/offboard/request_arm");
  emergency_client_ = ros_node_->create_client<offboard_manager::srv::EmergencyDisarm>("/offboard/emergency_disarm");
  takeoff_client_ = ros_node_->create_client<offboard_manager::srv::Takeoff>("/offboard/request_takeoff");

  status_sub_ = ros_node_->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status", qos,
    [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
      onVehicleStatus(msg);
    });

  land_sub_ = ros_node_->create_subscription<px4_msgs::msg::VehicleLandDetected>(
    "/fmu/out/vehicle_land_detected", qos,
    [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
      onLandDetected(msg);
    });

  flight_state_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
    "/offboard/flight_state", qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      onFlightState(msg);
    });

  alert_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
    "/offboard/alert", qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      onAlert(msg);
    });

  control_enable_sub_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
    "/offboard/control_enable", qos,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      onControlEnable(msg);
    });

  active_source_sub_ = ros_node_->create_subscription<std_msgs::msg::String>(
    "/control_mux/active_source", qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      onActiveSource(msg);
    });

  velocity_pub_ = ros_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    velocity_cmd_topic_, rclcpp::QoS(10));

  source_cmd_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
    "/control_mux/source_cmd", rclcpp::QoS(10));
}

// ---------------------------------------------------------------------------
// ROS spin
// ---------------------------------------------------------------------------
void PX4GuiNode::spinRos()
{
  rclcpp::spin_some(ros_node_);
  publishVelocity();
}

// ---------------------------------------------------------------------------
// Velocity publishing
// ---------------------------------------------------------------------------
void PX4GuiNode::publishVelocity()
{
  if (!velocity_pub_) return;

  geometry_msgs::msg::TwistStamped msg{};
  msg.header.stamp = ros_node_->get_clock()->now();
  msg.header.frame_id = "map";

  msg.twist.linear.x = static_cast<float>(spin_vx_->value());
  msg.twist.linear.y = static_cast<float>(spin_vy_->value());
  msg.twist.linear.z = static_cast<float>(spin_vz_->value());
  msg.twist.angular.z = static_cast<float>(spin_yaw_->value());

  velocity_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// Source command publishing
// ---------------------------------------------------------------------------
void PX4GuiNode::publishSourceCmd(const std::string & source)
{
  if (!source_cmd_pub_) return;

  std_msgs::msg::String msg{};
  msg.data = source;
  source_cmd_pub_->publish(msg);
  logAppend("[SRC] Switched to " + source);
}

// ---------------------------------------------------------------------------
// UI callbacks
// ---------------------------------------------------------------------------
void PX4GuiNode::onVelocityChanged()
{
}

void PX4GuiNode::onYawChanged()
{
}

void PX4GuiNode::onRequestMode()
{
  if (!mode_client_->service_is_ready()) {
    logAppend("[WARN] /offboard/request_mode service not available");
    return;
  }

  btn_mode_->setEnabled(false);
  logAppend("[CMD] Requesting OFFBOARD mode...");

  auto request = std::make_shared<offboard_manager::srv::RequestMode::Request>();
  mode_client_->async_send_request(
    request,
    [this](rclcpp::Client<offboard_manager::srv::RequestMode>::SharedFuture future) {
      btn_mode_->setEnabled(true);
      auto response = future.get();
      if (response->success) {
        logAppend("[OK] Mode request: " + response->message);
      } else {
        logAppend("[FAIL] Mode request: " + response->message);
      }
    });
}

void PX4GuiNode::onRequestArm()
{
  if (!arm_client_->service_is_ready()) {
    logAppend("[WARN] /offboard/request_arm service not available");
    return;
  }

  btn_arm_->setEnabled(false);
  logAppend("[CMD] Requesting arm...");

  auto request = std::make_shared<offboard_manager::srv::RequestArm::Request>();
  request->arm = true;
  arm_client_->async_send_request(
    request,
    [this](rclcpp::Client<offboard_manager::srv::RequestArm>::SharedFuture future) {
      btn_arm_->setEnabled(true);
      auto response = future.get();
      if (response->success) {
        logAppend("[OK] Arm: " + response->message);
      } else {
        logAppend("[FAIL] Arm: " + response->message);
      }
    });
}

void PX4GuiNode::onRequestDisarm()
{
  if (!arm_client_->service_is_ready()) {
    logAppend("[WARN] /offboard/request_arm service not available");
    return;
  }

  btn_disarm_->setEnabled(false);
  logAppend("[CMD] Requesting disarm...");

  auto request = std::make_shared<offboard_manager::srv::RequestArm::Request>();
  request->arm = false;
  arm_client_->async_send_request(
    request,
    [this](rclcpp::Client<offboard_manager::srv::RequestArm>::SharedFuture future) {
      btn_disarm_->setEnabled(true);
      auto response = future.get();
      if (response->success) {
        logAppend("[OK] Disarm: " + response->message);
      } else {
        logAppend("[FAIL] Disarm: " + response->message);
      }
    });
}

void PX4GuiNode::onEmergencyDisarm()
{
  auto reply = QMessageBox::warning(this, "Emergency Disarm",
    "Are you sure you want to trigger EMERGENCY DISARM?\n\n"
    "This will immediately disarm the vehicle and latch the emergency flag.",
    QMessageBox::Yes | QMessageBox::No, QMessageBox::No);

  if (reply != QMessageBox::Yes) return;

  if (!emergency_client_->service_is_ready()) {
    logAppend("[WARN] /offboard/emergency_disarm service not available");
    return;
  }

  btn_emergency_->setEnabled(false);
  logAppend("[EMERGENCY] Triggering emergency disarm...");

  auto request = std::make_shared<offboard_manager::srv::EmergencyDisarm::Request>();
  request->reason = "gui_button";
  emergency_client_->async_send_request(
    request,
    [this](rclcpp::Client<offboard_manager::srv::EmergencyDisarm>::SharedFuture future) {
      btn_emergency_->setEnabled(true);
      auto response = future.get();
      if (response->success) {
        logAppend("[EMERGENCY OK] " + response->message);
      } else {
        logAppend("[EMERGENCY FAIL] " + response->message);
      }
    });
}

void PX4GuiNode::onRequestTakeoff()
{
  if (!takeoff_client_->service_is_ready()) {
    logAppend("[WARN] /offboard/request_takeoff service not available");
    return;
  }

  btn_takeoff_->setEnabled(false);
  double alt = spin_takeoff_alt_->value();
  logAppend("[CMD] Requesting takeoff to " + std::to_string(alt) + " m...");

  auto request = std::make_shared<offboard_manager::srv::Takeoff::Request>();
  request->altitude = static_cast<float>(alt);
  takeoff_client_->async_send_request(
    request,
    [this](rclcpp::Client<offboard_manager::srv::Takeoff>::SharedFuture future) {
      btn_takeoff_->setEnabled(true);
      auto response = future.get();
      if (response->success) {
        logAppend("[OK] Takeoff: " + response->message);
      } else {
        logAppend("[FAIL] Takeoff: " + response->message);
      }
    });
}

// ---------------------------------------------------------------------------
// Subscription callbacks
// ---------------------------------------------------------------------------
void PX4GuiNode::onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    nav_state_ = msg->nav_state;
    arming_state_ = msg->arming_state;
    preflight_ok_ = msg->pre_flight_checks_pass;
  }

  auto ns = navStateString(msg->nav_state);
  auto as = armingStateString(msg->arming_state);
  auto as_color = armingStateColor(msg->arming_state);
  auto ns_color = navStateColor(msg->nav_state);

  QMetaObject::invokeMethod(this, [this, ns, as, as_color, ns_color]() {
    label_nav_mode_->setText(QString::fromStdString(ns));
    label_nav_mode_->setStyleSheet(
      QString("background-color: %1; color: white; padding: 4px; border-radius: 3px; font-weight: bold;")
        .arg(QString::fromStdString(ns_color)));

    label_arming_state_->setText(QString::fromStdString(as));
    label_arming_state_->setStyleSheet(
      QString("background-color: %1; color: white; padding: 4px; border-radius: 3px; font-weight: bold;")
        .arg(QString::fromStdString(as_color)));
  });
}

void PX4GuiNode::onLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    landed_ = msg->landed;
  }

  QMetaObject::invokeMethod(this, [this, msg]() {
    if (msg->landed) {
      label_landed_->setText("YES");
      label_landed_->setStyleSheet(
        "background-color: #2e7d32; color: white; padding: 4px; border-radius: 3px; font-weight: bold;");
    } else {
      label_landed_->setText("NO");
      label_landed_->setStyleSheet(
        "background-color: #c62828; color: white; padding: 4px; border-radius: 3px; font-weight: bold;");
    }
  });
}

void PX4GuiNode::onFlightState(const std_msgs::msg::String::SharedPtr msg)
{
  std::string state = msg->data;
  QMetaObject::invokeMethod(this, [this, state]() {
    label_flight_state_->setText(QString::fromStdString(state));
    logAppend("[STATE] " + state);
  });
}

void PX4GuiNode::onAlert(const std_msgs::msg::String::SharedPtr msg)
{
  std::string alert = msg->data;
  QMetaObject::invokeMethod(this, [this, alert]() {
    label_alert_->setText(QString::fromStdString(alert));
    label_alert_->setStyleSheet(
      "background-color: #4a1a1a; color: #ff6666; padding: 4px; border-radius: 3px; font-weight: bold;");
    logAppend("[ALERT] " + alert);
  });
}

void PX4GuiNode::onControlEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
  bool enable = msg->data;
  QMetaObject::invokeMethod(this, [this, enable]() {
    control_enable_ = enable;
    if (enable) {
      label_control_enable_->setText("ENABLED");
      label_control_enable_->setStyleSheet(
        "background-color: #1a4a1a; color: #66ff66; padding: 4px; border-radius: 3px; font-weight: bold;");
    } else {
      label_control_enable_->setText("DISABLED");
      label_control_enable_->setStyleSheet(
        "background-color: #4a1a1a; color: #ff6666; padding: 4px; border-radius: 3px; font-weight: bold;");
    }
  });
}

void PX4GuiNode::onActiveSource(const std_msgs::msg::String::SharedPtr msg)
{
  std::string src = msg->data;
  QMetaObject::invokeMethod(this, [this, src]() {
    active_source_str_ = src;
    label_active_source_->setText(QString::fromStdString(src));

    // Color-code by source type
    QString bg_color;
    if (src == "aruco") {
      bg_color = "#1565c0";   // blue
    } else if (src == "gui") {
      bg_color = "#6a1b9a";   // purple
    } else if (src == "joy") {
      bg_color = "#e65100";   // orange
    } else if (src == "hold" || src == "HOLD") {
      bg_color = "#616161";   // grey
    } else if (src == "EMERGENCY") {
      bg_color = "#b71c1c";   // dark red
    } else if (src == "TAKEOFF_OVERRIDE") {
      bg_color = "#f9a825";   // amber
    } else if (src == "DISABLED") {
      bg_color = "#4a1a1a";   // dim red
    } else {
      bg_color = "#37474f";   // dark blue-grey
    }

    label_active_source_->setStyleSheet(
      QString("background-color: %1; color: white; padding: 4px; border-radius: 3px; font-weight: bold;")
        .arg(bg_color));
  });
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
std::string PX4GuiNode::navStateString(uint8_t nav_state) const
{
  using NavState = px4_msgs::msg::VehicleStatus;
  switch (nav_state) {
    case NavState::NAVIGATION_STATE_MANUAL:          return "MANUAL";
    case NavState::NAVIGATION_STATE_ALTCTL:          return "ALTCTL";
    case NavState::NAVIGATION_STATE_POSCTL:          return "POSCTL";
    case NavState::NAVIGATION_STATE_AUTO_MISSION:    return "AUTO MISSION";
    case NavState::NAVIGATION_STATE_AUTO_LOITER:     return "AUTO LOITER";
    case NavState::NAVIGATION_STATE_AUTO_RTL:        return "AUTO RTL";
    case NavState::NAVIGATION_STATE_ACRO:            return "ACRO";
    case NavState::NAVIGATION_STATE_DESCEND:         return "DESCEND";
    case NavState::NAVIGATION_STATE_TERMINATION:     return "TERMINATION";
    case NavState::NAVIGATION_STATE_OFFBOARD:        return "OFFBOARD";
    case NavState::NAVIGATION_STATE_STAB:            return "STAB";
    case NavState::NAVIGATION_STATE_AUTO_TAKEOFF:    return "AUTO TAKEOFF";
    case NavState::NAVIGATION_STATE_AUTO_LAND:       return "AUTO LAND";
    case NavState::NAVIGATION_STATE_AUTO_FOLLOW_TARGET: return "AUTO FOLLOW";
    case NavState::NAVIGATION_STATE_AUTO_PRECLAND:   return "AUTO PRECLAND";
    case NavState::NAVIGATION_STATE_ORBIT:           return "ORBIT";
    case NavState::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF: return "VTOL TAKEOFF";
    default: {
      std::ostringstream ss;
      ss << "UNKNOWN(" << static_cast<int>(nav_state) << ")";
      return ss.str();
    }
  }
}

std::string PX4GuiNode::armingStateString(uint8_t arming_state) const
{
  using ArmingState = px4_msgs::msg::VehicleStatus;
  switch (arming_state) {
    case ArmingState::ARMING_STATE_DISARMED:       return "DISARMED";
    case ArmingState::ARMING_STATE_ARMED:          return "ARMED";
    default: {
      std::ostringstream ss;
      ss << "UNKNOWN(" << static_cast<int>(arming_state) << ")";
      return ss.str();
    }
  }
}

std::string PX4GuiNode::armingStateColor(uint8_t arming_state) const
{
  using ArmingState = px4_msgs::msg::VehicleStatus;
  switch (arming_state) {
    case ArmingState::ARMING_STATE_ARMED:          return "#2e7d32";
    case ArmingState::ARMING_STATE_DISARMED:       return "#c62828";
    default:                                       return "#616161";
  }
}

std::string PX4GuiNode::navStateColor(uint8_t nav_state) const
{
  using NavState = px4_msgs::msg::VehicleStatus;
  switch (nav_state) {
    case NavState::NAVIGATION_STATE_OFFBOARD:      return "#1565c0";
    case NavState::NAVIGATION_STATE_MANUAL:        return "#616161";
    case NavState::NAVIGATION_STATE_POSCTL:        return "#2e7d32";
    case NavState::NAVIGATION_STATE_ALTCTL:        return "#00838f";
    default:                                       return "#795548";
  }
}

void PX4GuiNode::logAppend(const std::string & msg)
{
  QMetaObject::invokeMethod(this, [this, msg]() {
    QString ts = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    log_view_->append(QString("[%1] %2").arg(ts, QString::fromStdString(msg)));
    log_line_count_++;
    if (log_line_count_ > 500) {
      QTextCursor cursor = log_view_->textCursor();
      cursor.movePosition(QTextCursor::Start);
      cursor.select(QTextCursor::LineUnderCursor);
      cursor.removeSelectedText();
      cursor.deleteChar();
      log_line_count_--;
    }
    log_view_->verticalScrollBar()->setValue(log_view_->verticalScrollBar()->maximum());
  });
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);

  // Dark-ish theme
  app.setStyle("Fusion");
  QPalette dark;
  dark.setColor(QPalette::Window, QColor(53, 53, 53));
  dark.setColor(QPalette::WindowText, Qt::white);
  dark.setColor(QPalette::Base, QColor(25, 25, 25));
  dark.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
  dark.setColor(QPalette::ToolTipBase, Qt::white);
  dark.setColor(QPalette::ToolTipText, Qt::white);
  dark.setColor(QPalette::Text, Qt::white);
  dark.setColor(QPalette::Button, QColor(53, 53, 53));
  dark.setColor(QPalette::ButtonText, Qt::white);
  dark.setColor(QPalette::BrightText, Qt::red);
  dark.setColor(QPalette::Link, QColor(42, 130, 218));
  dark.setColor(QPalette::Highlight, QColor(42, 130, 218));
  dark.setColor(QPalette::Disabled, QPalette::Text, QColor(127, 127, 127));
  dark.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(127, 127, 127));
  app.setPalette(dark);

  PX4GuiNode gui;
  gui.show();

  int ret = app.exec();

  rclcpp::shutdown();
  return ret;
}
