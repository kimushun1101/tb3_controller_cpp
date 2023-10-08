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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class Tb3Controller : public rclcpp::Node
{
public:
  Tb3Controller() : Node("tb3_controller")
  {
    this->declare_parameter<std::float_t>("Kp", 1.0);
    this->get_parameter("Kp", Kp_);
    this->declare_parameter<std::float_t>("T", 0.01);
    this->get_parameter("T", T_);
    this->declare_parameter<std::float_t>("init_xd", 1.0);
    this->get_parameter("init_xd", xd_);
    std::chrono::milliseconds sampling_period{(int)(T_*1000.0)};
    x_ = xd_; // /scan トピックが取得されるまでは，目標値と一致させておくことで制御入力を0とする

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
    RCLCPP_INFO_STREAM(this->get_logger(), "T : " << T_);
    RCLCPP_INFO_STREAM(this->get_logger(), "initial xd : " << xd_);

    // パラメータ変更のコールバックを登録
    auto parameter_change_cb = std::bind(&Tb3Controller::parameter_callback, this, std::placeholders::_1);
    param_handler_ = this->add_on_set_parameters_callback(parameter_change_cb);
  }

private:
  void xd_callback(std_msgs::msg::Float32::SharedPtr msg)
  {
    if(xd_ != msg->data)
    {
      xd_ = msg->data;
      RCLCPP_INFO_STREAM(this->get_logger(), "x_d has been updated : " << xd_);
    }
  }
  void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 0 ~ 2\pi まで360点取得している
    // RCLCPP_INFO_STREAM(this->get_logger(), "point amount : " << msg->ranges.size());
    // x_ = msg->ranges[0];  // 正面の距離を計測
    x_ = msg->ranges[180];  // 背面の距離を計測
  }
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = - Kp_ * (x_ - xd_);
    cmd_vel_pub_->publish(msg);
  }

  // パラメータ変更のコールバック関数
  rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto &parameter : parameters) {
      if (parameter.get_name() == "Kp") {
        Kp_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "Kp has been updated to: %f", Kp_);
      }
      if (parameter.get_name() == "init_xd") {
        xd_ = parameter.as_double();
        RCLCPP_INFO(this->get_logger(), "xd has been updated to: %f", xd_);
      }
    }
    return result;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr xd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  float Kp_;
  float xd_;
  float x_;
  float T_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_handler_;
};

int main(int argc, char * argv[])
{
  // Ctrl + C を押下したとき速度を0 とする処理
  signal(
    SIGINT, [](int) {
      auto node = rclcpp::Node::make_shared("stop");
      auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      pub->publish(geometry_msgs::msg::Twist());
      rclcpp::shutdown();
    });
  // ここまで

  rclcpp::init(argc, argv);
  auto node = std::make_shared<Tb3Controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
