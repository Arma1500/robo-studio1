#include <cstdio>
#include <iostream>
#include <cmath>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class Reckoning : public rclcpp::Node
{
public:
  Reckoning() : Node("imu_reck")
  {
    // ----------------- Subscribers -------------------
    // Odometry
    odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 
    10, std::bind(&Reckoning::odoCallback, this, std::placeholders::_1));

    //IMU data
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 
    10, std::bind(&Reckoning::imuCallback, this, std::placeholders::_1));

    // ----------------- Publisher -------------------
    measured_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/measured_odo", 10);
  }

private:

  // ----------------- Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // ----------------- Publisher 
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr measured_pub_;

  // ----------------- Variables
  double odo_x_, odo_y_, measured_x_, measured_y_;
  double prev_timestamp_ = 0.0f;
  std::mutex mtx_;

  // ----------------- Callback Functions -----------------
  void odoCallback(const nav_msgs::msg::Odometry::SharedPtr odo_msg){
    std::unique_lock<std::mutex> lck(mtx_);
    odo_x_ = odo_msg->pose.pose.position.x;
    odo_y_ = odo_msg->pose.pose.position.y;

    lck.unlock();
    RCLCPP_INFO(this->get_logger(), "Got Odo");
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg){
    std::unique_lock<std::mutex> lck(mtx_);
    auto stamp = imu_msg->header;
    auto timestamp = stamp.stamp.sec;

    if(prev_timestamp_ == 0.0f){
      prev_timestamp_ = timestamp;
      return;
    }

    double dt = timestamp - prev_timestamp_;
    prev_timestamp_ = timestamp;

    double vel_x =+ imu_msg->linear_acceleration.x * dt;
    double vel_y =+ imu_msg->linear_acceleration.y * dt;
    measured_x_ =+ vel_x * dt;
    measured_y_ =+ vel_y * dt;

    auto measured_angle= imu_msg->orientation;

    lck.unlock();
    RCLCPP_INFO(this->get_logger(), "Got IMU");

    publishMeasuredData(stamp, measured_angle);
  }

  void publishMeasuredData(const std_msgs::msg::Header& current_time, const geometry_msgs::msg::Quaternion& orientation){
    // Calculate difference
    double diff_X = measured_x_ - odo_x_;
    double diff_Y = measured_y_ - odo_y_;
    std::cout<<"X difference: " << diff_X << "Y difference: " << diff_Y << std::endl;

    nav_msgs::msg::Odometry measured_odom;
    measured_odom.header.stamp = current_time.stamp;
    measured_odom.header.frame_id = "odom";
    measured_odom.child_frame_id = "base_link";
    measured_odom.pose.pose.position.x = measured_x_;
    measured_odom.pose.pose.position.y = measured_y_;
    measured_odom.pose.pose.orientation = orientation;

    std::unique_lock<std::mutex> lck(mtx_);
    measured_pub_->publish(measured_odom);
    lck.unlock();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Reckoning>());
  rclcpp::shutdown();
  return 0;
}
