#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

class Tb3Controller : public rclcpp::Node
{
public:
  Tb3Controller();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr xd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  void xd_callback(std_msgs::msg::Float32::SharedPtr msg);
  void vel_callback(geometry_msgs::msg::Twist::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
  void timer_callback();
  float Kp_;
  float Kd_;
  float xd_;
  float vd_;
  float x_;
  float v_;
  geometry_msgs::msg::Pose current_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
};