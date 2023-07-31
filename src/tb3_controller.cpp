//  Copyright 2023 Shunsuke Kimura

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "tb3_controller_cpp/tb3_controller.hpp"

Tb3Controller::Tb3Controller() : Node("tb3_controller")
{
  this->declare_parameter<std::float_t>("Kp", 1.0);
  this->get_parameter("Kp", Kp_);
  this->declare_parameter<std::float_t>("Kd", 1.0);
  this->get_parameter("Kd", Kd_);
  this->declare_parameter<std::float_t>("T", 0.001);
  this->get_parameter("T", T_);
  this->declare_parameter<std::float_t>("init_xd", 1.0);
  this->get_parameter("init_xd", xd_);
  std::chrono::milliseconds sampling_period{(int)(T_*1000.0)};
  x_ = xd_; // /scan トピックが取得されるまでは，目標値と一致させておくことで制御入力を0とする
  e_pre_ = 0.0;

  using std::placeholders::_1;
  xd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/xd", rclcpp::QoS(10), std::bind(&Tb3Controller::xd_callback, this, _1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(), std::bind(&Tb3Controller::scan_callback, this, _1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(
    sampling_period, std::bind(&Tb3Controller::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "tb3_controller node has been initialised");
  RCLCPP_INFO_STREAM(this->get_logger(), "Kp : " << Kp_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Kd : " << Kd_);
  RCLCPP_INFO_STREAM(this->get_logger(), "T : " << T_);
  RCLCPP_INFO_STREAM(this->get_logger(), "initial xd : " << xd_);
}

void Tb3Controller::xd_callback(std_msgs::msg::Float32::SharedPtr msg)
{
  if(xd_ != msg->data)
  {
    xd_ = msg->data;
    RCLCPP_INFO_STREAM(this->get_logger(), "x_d has been updated : " << xd_);
  }
}

void Tb3Controller::scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 0 ~ 2\pi まで360点取得している
  // RCLCPP_INFO_STREAM(this->get_logger(), "point amount : " << msg->ranges.size());
  // x_ = msg->ranges[0];  // 正面の距離を計測
  x_ = msg->ranges[180];  // 背面の距離を計測
}

void Tb3Controller::timer_callback()
{
  auto message = geometry_msgs::msg::Twist();
  auto e = x_ - xd_;
  auto u = - Kp_ * e - Kd_ * (e - e_pre_)/T_;
  e_pre_ = e;
  if(std::abs(u) > 0.5){
    auto sign = (u > 0) ? 1 : ((u < 0) ? -1 : 0);
    message.linear.x = sign * 0.5;
  }
  else{
    message.linear.x = u;
  }
  cmd_vel_pub_->publish(message);
}
