#include "tb3_controller_cpp/tb3_controller.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

Tb3Controller::Tb3Controller()
: Node("tb3_controller")
{
  this->declare_parameter<std::float_t>("Kp", 0.1);
  this->get_parameter("Kp", Kp_);
  this->declare_parameter<std::float_t>("Kd", 0.01);
  this->get_parameter("Kd", Kd_);
  xd_ = 3.0; //　なんで型指定してないのに動いているのだろう？->ヘッダで定義しているから
  vd_ = 0.0;
  xd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/xd", rclcpp::QoS(10), std::bind(&Tb3Controller::xd_callback, this, _1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(), std::bind(&Tb3Controller::scan_callback, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(10), std::bind(&Tb3Controller::odom_callback, this, _1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); // これはpublishされてる？->scan_callbackでされてる
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::QoS(10),  std::bind(&Tb3Controller::vel_callback, this, _1));
  timer_ = this->create_wall_timer(
      1ms, std::bind(&Tb3Controller::timer_callback, this));
  // プログラム内部のパラメータを確認する方法はpublishするぐらいしかない？あまりにも手間がかかる…->RCLCPP_INFOがある
  // QoS(10)とかSensorDataQoS()はどのような意味があるの？
  // 同じcall_back関数に複数のtopicのsubscriptionを設定したい
  // timer_callbackはどうやって使うのか？

  RCLCPP_INFO(this->get_logger(), "tb3_controller node has been initialised");
}

void Tb3Controller::xd_callback(std_msgs::msg::Float32::SharedPtr msg)
{
  xd_ = msg->data;
}

void Tb3Controller::timer_callback() //ここにコントローラを設計して出力するようにすれば良さそう
{
  // RCLCPP_INFO(this->get_logger(), "Hello!!");
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = - Kp_ * (x_ - xd_) - Kd_ * (v_ - vd_);
  RCLCPP_INFO(this->get_logger(), "I heard: '%f'", message.linear.x);
  cmd_vel_pub_->publish(message);
}
//, geometry_msgs::msg::Twist::SharedPtr msg1
void Tb3Controller::scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) // 速度Dと積分Iをどうやって実装すればよいのか？->callback関数の中でコントローラを組むのは不可能？
{
  x_ = msg->ranges[0];  // 正面の距離を計測
  // 背面の距離を計測したい場合
  // auto point_count = msg->ranges.size();
  // x_ = msg->ranges[point_count / 2];
}

void Tb3Controller::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
}

void Tb3Controller::vel_callback(geometry_msgs::msg::Twist::SharedPtr msg)
{
  v_ = msg->linear.x;
  // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->linear.x);
}


int main(int argc, char * argv[])
{
  signal(
    SIGINT, [](int) {
      auto node = rclcpp::Node::make_shared("stop");
      auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      pub->publish(geometry_msgs::msg::Twist());
      rclcpp::shutdown();
    });
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Tb3Controller>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}