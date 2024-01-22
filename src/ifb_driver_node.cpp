// Copyright 2023 FUTU-RE Co. LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../include/serial/serial.h"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <boost/algorithm/string.hpp>

#include <chrono>
#include <cstdio>
#include <iostream>

// Define constants
// constexpr char PORT_IFB[] = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0667FF564977514867023048-if02";
// constexpr char PORT_IFB[] = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066AFF515049657187144732-if02";
// constexpr char PORT_IFB[] =  "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0673FF30304B4E3043010827-if02";
// constexpr char PORT_IFB[] =  "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF30304B4E3043012534-if02";
// constexpr char PORT_IFB[] =  "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066CFF484971754867113142-if02";
// constexpr char PORT_IFB[] =  "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0669FF555557838667143421-if02"; // 21-09
constexpr char PORT_IFB[] =  "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF3732504E3043103247-if02"; // 2023-07 #2

int BAUD_RATE = 115200;
double IFBSTAT_TIMEOUT_NSEC;  // nsecs
double IFB_TIMEOUT_SEC;
double SERIAL_TIMEOUT;
double WRITE_TIMEOUT;
int ANGLE_STEER_DEG_MAX;
double WHEEL_BASE;      // [m]
double ACC_CMD_FACTOR;  // 1m/s -> cmd=1000rpm@motor
int ACC_CMD_MIN;
int ACC_CMD_MAX;
int STEER_CMD_MIN;
int STEER_CMD_MAX;
double STEER_CMD_FACTOR;
int TIMER_MILLI;
constexpr int AUTOWARE_CTRL_TIMER_MILLI = 100;

class IfbDriver : public rclcpp::Node
{
public:
  IfbDriver() : Node("ifb_driver"), port_ifb_(PORT_IFB), baudrate_(BAUD_RATE), ser_ifb_error_(false)
  {
    BAUD_RATE = declare_parameter<int>("BAUD_RATE", 115200);
    IFBSTAT_TIMEOUT_NSEC = declare_parameter<double>("IFBSTAT_TIMEOUT_NSEC", 1e9);
    IFB_TIMEOUT_SEC = declare_parameter<double>("IFB_TIMEOUT_SEC", 0.5);
    SERIAL_TIMEOUT = declare_parameter<double>("SERIAL_TIMEOUT", 1.0);
    WRITE_TIMEOUT = declare_parameter<double>("WRITE_TIMEOUT", 1.0);
    ANGLE_STEER_DEG_MAX = declare_parameter<int>("ANGLE_STEER_DEG_MAX", 30);
    WHEEL_BASE = declare_parameter<double>("WHEEL_BASE", 1.0);
    ACC_CMD_FACTOR = declare_parameter<double>("ACC_CMD_FACTOR", 1000.0);
    ACC_CMD_MIN = declare_parameter<int>("ACC_CMD_MIN", 0);
    ACC_CMD_MAX = declare_parameter<int>("ACC_CMD_MAX", 3000);
    STEER_CMD_MIN = declare_parameter<int>("STEER_CMD_MIN", -500);
    STEER_CMD_MAX = declare_parameter<int>("STEER_CMD_MAX", 500);
    STEER_CMD_FACTOR = declare_parameter<double>("STEER_CMD_FACTOR", 1000.0);
    TIMER_MILLI = declare_parameter<int>("TIMER_MILLI", 20);
    init();
  }

private:
  void init()
  {
    // Initialize publishers and subscribers
    pub_msg_from_ifb_ = this->create_publisher<std_msgs::msg::String>("/msg_from_ifb", 1);
    pub_autoware_control_mode = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS(1).reliable().keep_last(1));
    controlModeReportMsg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;

    // pub_act_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/act_vel", 1);
    // pub_act_vel_ =
    // this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/act_vel",
    // 1);
    // pub_act_vel_ =
    // this->create_publisher<autoware_auto_vehicle_msgs::msg::VehicleControlCommand>("/your/vehicle_control_command/topic",
    // 1);

    // from vehicle interface to Autoware
    pub_act_vel_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
      "/vehicle/status/velocity_status", 1);
    pub_act_steer_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
      "/vehicle/status/steering_status", 1);

    pub_msg_from_ultrasonic_ =
      this->create_publisher<std_msgs::msg::String>("/msg_from_ultrasonic", 1);
    // sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    //"/cmd_vel", 1, std::bind(&IfbDriver::cb_on_cmd_vel, this, std::placeholders::_1));

    // from autoware
    sub_cmd_vel_ =
      this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", 1,
        std::bind(&IfbDriver::cb_on_cmd_vel, this, std::placeholders::_1));

    read_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(TIMER_MILLI), std::bind(&IfbDriver::read_from_IFB, this));
    this->act_vel_.header.frame_id = "base_link";

    autoware_control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(AUTOWARE_CTRL_TIMER_MILLI), std::bind(&IfbDriver::set_autoware_control, this));
    // Connect to the IFB board
    connect();
  }

  void connect()
  {
    try {
      ser_ =
        std::make_unique<serial::Serial>(port_ifb_, baudrate_, serial::Timeout::simpleTimeout(1.0));
      RCLCPP_INFO(
        this->get_logger(), "Connected to %s at %d baudrate.", port_ifb_.c_str(), baudrate_);
    } catch (const serial::IOException & e) {
      RCLCPP_ERROR(this->get_logger(), "Unable to establish connection. Retrying...");
      rclcpp::sleep_for(std::chrono::seconds(1));
      connect();  // Retry connection
    }
  }

  void cb_on_cmd_vel(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
  {
    // Callback for new vehicle command

    int acc_cmd = static_cast<int>(msg->longitudinal.speed * ACC_CMD_FACTOR);

    // Check limits
    acc_cmd = std::min(std::max(acc_cmd, ACC_CMD_MIN), ACC_CMD_MAX);
    RCLCPP_INFO(this->get_logger(), "Received velocity: %d", acc_cmd);

    write_str("A" + std::to_string(acc_cmd) + "\r\n");

    // Steering
    int str_cmd = static_cast<int>(msg->lateral.steering_tire_angle * STEER_CMD_FACTOR);
    str_cmd = std::min(std::max(str_cmd, STEER_CMD_MIN), STEER_CMD_MAX);
    RCLCPP_INFO(this->get_logger(), "Received steering: %d", str_cmd);

    write_str("S" + std::to_string(str_cmd) + "\r\n");
    return;
  }

  void read_from_IFB()
  {
    if (!read_serial()) {
      RCLCPP_INFO(this->get_logger(), "Serial read fail");
      if (check_ifb_board_timeout()) {
        RCLCPP_INFO(this->get_logger(), "IFB board has timed out");
      }
      return;
    }
    if (!ser_in_.empty()) {
      proc_msg_from_ifb(ser_in_);
    }
  }

  void set_autoware_control()
  {
    this->controlModeReportMsg.stamp = rclcpp::Clock{RCL_SYSTEM_TIME}.now();
    pub_autoware_control_mode->publish(controlModeReportMsg);
  }
  bool check_ifb_board_timeout()
  {
    // True if timed out
    return (rclcpp::Clock().now().nanoseconds() - time_since_ifb_last_) > IFB_TIMEOUT_SEC * 1e9;
  }

  void write_str(const std::string & outstr)
  {
    // Write to serial
    if (ser_ == nullptr || ser_ifb_error_) {
      RCLCPP_INFO(this->get_logger(), "IFB not open when attempting to write...");
      return;
    }

    try {
      // RCLCPP_INFO(this->get_logger(), "IFB outstr: %s", outstr.c_str());
      ser_->write(outstr);
    } catch (const std::exception & e) {
      ser_ifb_error_ = true;
      RCLCPP_INFO(this->get_logger(), "Failed to write to IFB: Exception: %s", e.what());
    }
  }

  void proc_msg_from_ifb(const std::string & in_str)
  {
    // Process incoming data from IFB
    std::vector<std::string> in_str_split;
    boost::split(in_str_split, in_str, boost::is_any_of(","));
    if (in_str_split[0] == "odom") {
      try {
        double speed = std::stod(in_str_split[1]);
        double steer = std::stod(in_str_split[2]);

        act_vel_.longitudinal_velocity = speed / ACC_CMD_FACTOR;
        act_steer_.steering_tire_angle = steer / STEER_CMD_FACTOR;
        act_vel_.header.stamp = rclcpp::Clock{RCL_SYSTEM_TIME}.now();
        act_steer_.stamp = rclcpp::Clock{RCL_SYSTEM_TIME}.now();


      } catch (const std::exception & e) {
        return;
      }
      pub_act_vel_->publish(act_vel_);
      pub_act_steer_->publish(act_steer_);
      update_ifb_last();
    } else if (in_str_split[0] == "info") {
      // Future: update ifb board state
      // Publish ROS topic
      auto message = std_msgs::msg::String();
      message.data = in_str;
      pub_msg_from_ifb_->publish(message);
      update_ifb_last();
    } else if (in_str_split[0] == "ultrasonic") {
      auto message = std_msgs::msg::String();
      message.data = in_str;
      pub_msg_from_ultrasonic_->publish(message);
      update_ifb_last();
    }
  }

  void update_ifb_last() { time_since_ifb_last_ = rclcpp::Clock().now().nanoseconds(); }

  bool read_serial()
  {
    try {
      if (ser_->available()) {
        ser_in_ = ser_->readline();
        ser_->flushInput();
        // RCLCPP_INFO(this->get_logger(), "IFB Received data: %s", ser_in_.c_str());
        return true;
      }

    } catch (const serial::IOException & e) {
      RCLCPP_ERROR(this->get_logger(), "Lost connection. Reconnecting...");
      connect();
    }

    return false;
  }

  std::string port_ifb_;
  int baudrate_;
  std::unique_ptr<serial::Serial> ser_;
  bool ser_ifb_error_;
  std::string ser_in_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_msg_from_ifb_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_msg_from_ultrasonic_;
  autoware_auto_vehicle_msgs::msg::VelocityReport act_vel_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_act_vel_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr pub_autoware_control_mode;
  
  autoware_auto_vehicle_msgs::msg::ControlModeReport controlModeReportMsg;
  
  autoware_auto_vehicle_msgs::msg::SteeringReport act_steer_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_act_steer_;


  // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    sub_cmd_vel_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr autoware_control_timer_;

  int64_t time_since_ifb_last_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IfbDriver>());
  rclcpp::shutdown();
  return 0;
}