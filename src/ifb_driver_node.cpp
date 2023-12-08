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

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/algorithm/string.hpp>
#include "../include/serial/serial.h"

#include <chrono>
#include <iostream>

#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

// Define constants
constexpr char PORT_IFB[] = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0667FF564977514867023048-if02";
constexpr int BAUD_RATE = 115200;
constexpr int IFBSTAT_TIMEOUT_NSEC = 1e9;  // nsecs
constexpr double IFB_TIMEOUT_SEC = 0.5;
constexpr double SERIAL_TIMEOUT = 1.0;
constexpr double WRITE_TIMEOUT = 1.0;
constexpr int ANGLE_STEER_DEG_MAX = 30;
constexpr double WHEEL_BASE = 1.0;  // [m]
constexpr double ACC_CMD_FACTOR = 1000.0; //1m/s -> cmd=1000rpm@motor
constexpr int ACC_CMD_MIN = 0;
constexpr int ACC_CMD_MAX = 3000;
constexpr int STEER_CMD_MIN = -500;
constexpr int STEER_CMD_MAX = 500;
constexpr double STEER_CMD_FACTOR = 1000.0;

class IfbDriver : public rclcpp::Node
{
public:
  IfbDriver()
    : Node("ifb_driver"), port_ifb_(PORT_IFB), baudrate_(BAUD_RATE), ser_ifb_error_(false)
  {
    init();
  }

private:
  void init()
  {
    // Initialize publishers and subscribers
    pub_msg_from_ifb_ = this->create_publisher<std_msgs::msg::String>("/msg_from_ifb", 1);

    // pub_act_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/act_vel", 1);
    // pub_act_vel_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>("/act_vel", 1);
    pub_act_vel_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VehicleControlCommand>("/your/vehicle_control_command/topic", 1);

    pub_msg_from_ultrasonic_ = this->create_publisher<std_msgs::msg::String>("/msg_from_ultrasonic", 1);
    //sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        //"/cmd_vel", 1, std::bind(&IfbDriver::cb_on_cmd_vel, this, std::placeholders::_1));

    sub_cmd_vel_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/cmd_vel", 1, std::bind(&IfbDriver::cb_on_cmd_vel, this, std::placeholders::_1));

    read_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&IfbDriver::read_from_IFB, this));

    // Connect to the IFB board
    connect();
  }

  void connect()
  {
    try
    {
      ser_ = std::make_unique<serial::Serial>(port_ifb_, baudrate_, serial::Timeout::simpleTimeout(1.0));
      RCLCPP_INFO(this->get_logger(), "Connected to %s at %d baudrate.", port_ifb_.c_str(), baudrate_);
    }
    catch (const serial::IOException& e)
    {
      RCLCPP_WARN(this->get_logger(), "Unable to establish connection. Retrying...");
      rclcpp::sleep_for(std::chrono::seconds(1));
      connect();  // Retry connection
    }
  }

  void cb_on_cmd_vel(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
  {
    // Callback for new vehicle command
    RCLCPP_INFO(this->get_logger(), "Received /ackermann_cmd.");

    int acc_cmd = static_cast<int>(msg->longitudinal.acceleration * ACC_CMD_FACTOR);

    // Check limits
    acc_cmd = std::min(std::max(acc_cmd, ACC_CMD_MIN), ACC_CMD_MAX);

    write_str("A" + std::to_string(acc_cmd) + "\r\n");

    // Steering
    int str_cmd = static_cast<int>(msg->lateral.steering_tire_angle * STEER_CMD_FACTOR);

    // Check limits
    str_cmd = std::min(std::max(str_cmd, STEER_CMD_MIN), STEER_CMD_MAX);

    // Send command to IFB board & log
    write_str("S" + std::to_string(str_cmd) + "\r\n");
  }

  void read_from_IFB()
  {
    if (!read_serial())
    {
      RCLCPP_INFO(this->get_logger(), "Serial read fail");
      if (check_ifb_board_timeout())
      {
         RCLCPP_INFO(this->get_logger(), "IFB board has timed out");
       }
       return;
     }
     if (!ser_in_.empty())
     {
       proc_msg_from_ifb(ser_in_);
     }
  }

  bool check_ifb_board_timeout()
  {
    // True if timed out
    return (rclcpp::Clock().now().nanoseconds() - time_since_ifb_last_) > IFB_TIMEOUT_SEC * 1e9;
  }

  void write_str(const std::string& outstr)
  {
    // Write to serial
    if (ser_ == nullptr || ser_ifb_error_)
    {
      RCLCPP_INFO(this->get_logger(), "IFB not open when attempting to write...");
      return;
    }

    try
    {
      RCLCPP_INFO(this->get_logger(), "IFB outstr: %s", outstr.c_str());
      ser_->write(outstr);
    }
    catch (const std::exception& e)
    {
      ser_ifb_error_ = true;
      RCLCPP_INFO(this->get_logger(), "Failed to write to IFB: Exception: %s", e.what());
    }
  }

  void proc_msg_from_ifb(const std::string& in_str)
  {
    // Process incoming data from IFB
    std::vector<std::string> in_str_split;    
    boost::split(in_str_split, in_str, boost::is_any_of(","));
    if (in_str_split[0] == "odom")
    {
      try
      {
        double speed = std::stod(in_str_split[1]);
        double steer = std::stod(in_str_split[2]);

        // act_vel_.longitudinal.acceleration = speed / ACC_CMD_FACTOR;
        // act_vel_.lateral.steering_tire_angle = steer / STEER_CMD_FACTOR;

        act_vel_.long_accel_mps2 = speed / ACC_CMD_FACTOR;
        act_vel_.front_wheel_angle_rad = steer / STEER_CMD_FACTOR;
      }
      catch (const std::exception& e)
      {
        return;
      }

      pub_act_vel_->publish(act_vel_);
      update_ifb_last();
    }
    else if (in_str_split[0] == "info")
    {
      // Future: update ifb board state
      // Publish ROS topic
      auto message = std_msgs::msg::String();
      message.data = in_str;
      pub_msg_from_ifb_->publish(message);
      update_ifb_last();
    }
    else if (in_str_split[0] == "ultrasonic")
    {
      auto message = std_msgs::msg::String();
      message.data = in_str;
      pub_msg_from_ultrasonic_->publish(message);
      update_ifb_last();
    }
  }

  void update_ifb_last()
  {
    time_since_ifb_last_ = rclcpp::Clock().now().nanoseconds();
  }

  bool read_serial()
  {
    try
    {
      if (ser_->available())
      {     
        ser_in_ = ser_->readline();
        std::cout << "IFB Received data: " << ser_in_ << std::endl;
        return true;
      }
      
    }
    catch (const serial::IOException& e)
    {
      RCLCPP_WARN(this->get_logger(), "Lost connection. Reconnecting...");
      connect();
    }
    
    return false;
  }

  std::string port_ifb_;
  int baudrate_;
  std::unique_ptr<serial::Serial> ser_;
  bool ser_ifb_error_;
  std::string ser_in_;
  // geometry_msgs::msg::Twist act_vel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_msg_from_ifb_;
  // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_act_vel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_msg_from_ultrasonic_;

  // autoware_auto_control_msgs::msg::AckermannControlCommand act_vel_;
  // rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr pub_act_vel_;

  autoware_auto_vehicle_msgs::msg::VehicleControlCommand act_vel_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VehicleControlCommand>::SharedPtr pub_act_vel_;

  // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr sub_cmd_vel_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  int64_t time_since_ifb_last_;
};

int main(int argc, char** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world 000 ifb_driver_package package\n");
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IfbDriver>());
  rclcpp::shutdown();

  printf("hello world 111ifb_driver_package package\n");

  return 0;
}


// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world 111ifb_driver_package package\n");
//   return 0;
// }
