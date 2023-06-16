#include "tb3_controller_cpp/tb3_controller.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

Tb3Controller::Tb3Controller()
: Node("tb3_controller")
{
  this->declare_parameter<std::float_t>("Kp", 0.1);
  this->get_parameter("Kp", Kp_);

  xd_ = 0.0;
  
  xd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/xd", rclcpp::QoS(10), std::bind(&Tb3Controller::xd_callback, this, _1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(), std::bind(&Tb3Controller::scan_callback, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::QoS(10), std::bind(&Tb3Controller::odom_callback, this, _1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  RCLCPP_INFO(this->get_logger(), "tb3_controller node has been initialised");
}

Tb3Controller::~Tb3Controller()
{
  auto node = rclcpp::Node::make_shared("stop");
  // auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  // auto message = geometry_msgs::msg::Twist();
  // pub->publish(message);
  RCLCPP_INFO(this->get_logger(), "tb3_controller node has been terminated");
}

void Tb3Controller::xd_callback(std_msgs::msg::Float32::SharedPtr msg){
  xd_ = msg->data;
}

void Tb3Controller::scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg){
  auto point_count = msg->ranges.size();
  auto x = msg->ranges[point_count/2];
  auto message = geometry_msgs::msg::Twist();
  // message.linear.x = - Kp_ * (x - xd_);
  message.linear.x = 0.3;
  message.angular.z = 0.0;
  cmd_vel_pub_->publish(message);
}

void Tb3Controller::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg){
  current_pose_ = msg->pose.pose;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Tb3Controller>();
  rclcpp::spin(node);
  // rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
