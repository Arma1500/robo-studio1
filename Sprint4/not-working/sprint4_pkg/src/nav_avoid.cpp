#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <mutex>

// For State machine
enum States
  {
    DRIVE,
    AVOID
  };

class NavAvoid : public rclcpp::Node
{
public:
  NavAvoid() : Node("nav_avoid")
  {
    // ------------------- Subscribers -------------------

    // Odometry
    odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 
    10, std::bind(&NavAvoid::odoCallback, this, std::placeholders::_1));

    // Laser Scan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
    10, std::bind(&NavAvoid::scanCallback, this, std::placeholders::_1));

    // ------------------- Publishers -------------------

    // Visual Marker
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("cylinder_marker", 10);

    // Cmd_Vel
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  }

private:

  // ------------------- Subsrcibers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;

  // ------------------- Publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  // ------------------- Private Vars
  std::mutex mtx_;
  States state_;
  bool is_locked_;

  nav_msgs::msg::Odometry odo_;
  sensor_msgs::msg::LaserScan scan_msg_;
  geometry_msgs::msg::Twist cmd_twist_;

  // ------------------- Callback Functions
  // Odometry
  void odoCallback(const nav_msgs::msg::Odometry::SharedPtr odo_msg)
  {
    std::unique_lock<std::mutex> lck(mtx_);
    odo_ = *odo_msg;
    lck.unlock();
    //RCLCPP_INFO(this->get_logger(), "Got Odo");
  }

  // Laser Scan
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "#################################################");

    // Get Scan Message
    std::unique_lock<std::mutex> lck(mtx_);
    scan_msg_ = *msg;
    lck.unlock();

    // Detect cylinders
    std::vector<geometry_msgs::msg::Point> cylinders = detectCylinders();

    // once cylinders are found
    if(!cylinders.empty()){
      RCLCPP_INFO(this->get_logger(), "30cm Cylinder Found!");

      for(auto& cylinder : cylinders){
        geometry_msgs::msg::Point world_cyl = transformToWorld(cylinder);      
        std::cout<<"Cylinder Center x: " << world_cyl.x << ", Cylinder Center y: " << world_cyl.y << std::endl;
        
        //publishCylinderMarker(world_cyl_); // publish to Rviz
        //avoidCyl(world_cyl); // state machine
      }
    }
  }

  // ------------------- Helper Functions
  // Detect Cylinders
  std::vector<geometry_msgs::msg::Point> detectCylinders(void){

    std::vector<geometry_msgs::msg::Point> cyls;

    //containers for objects
    std::vector<std::vector<geometry_msgs::msg::Point>> objects;
    std::vector<geometry_msgs::msg::Point> current_object;

    // varibales that can be adjusted
    double object_tol = 0.5;
    double diameter = 0.30;
    double dia_tol = 0.05;

    // DETECT ALL THE OBJECTS 
    for(unsigned int i = 0; i < scan_msg_.ranges.size(); i++){
      double current_range = scan_msg_.ranges.at(i);

      // if the scan is within the max range that means it is hitting something
      if (!std::isinf(current_range) && !std::isnan(current_range) && current_range < scan_msg_.range_max) {
                geometry_msgs::msg::Point point = polarToCart(i); // convert it to a point

                if (current_object.empty()) {
                    // Start a new segment
                    current_object.push_back(point);

                } else {
                    geometry_msgs::msg::Point previous_point = current_object.back();
                    double distance = std::hypot(point.x - previous_point.x, point.y - previous_point.y);

                    // if the scan is part of the object then push it to the current object
                    if (distance < object_tol) {
                        current_object.push_back(point);

                    } else {
                        if (current_object.size() > 1) {
                            objects.push_back(current_object); // Store valid objects
                        }
                        current_object.clear(); // start a new object

                        current_object.push_back(point);
                    }
                }
            }
    }
    // Push the last object if valid
    if (current_object.size() > 1) {
        objects.push_back(current_object);
    }


    // CHECK WHICH OBJECTS ARE CYLINDERS
    for(auto& object : objects){
      if(object.size() < 4){
        continue; // for objects that are too small to be the 30cm cylinder
      }

      // if the euclidean distance between the first and last segment is within the tolerance then it is the cylinder we need
      geometry_msgs::msg::Point firstPoint = object.front();
      geometry_msgs::msg::Point lastPoint = object.back();
      double distance = std::hypot(lastPoint.x - firstPoint.x, lastPoint.y - firstPoint.y);

      // Check if the segment length is within the cylinder diameter range
      if (std::abs(distance - diameter) <= dia_tol) {
          // Calculate the center point of the segment
          geometry_msgs::msg::Point cyl;
          cyl.x = (firstPoint.x + lastPoint.x) / 2.0;
          cyl.y = (firstPoint.y + lastPoint.y) / 2.0;
          cyls.push_back(cyl);
      }
    }
    
    return cyls;
  }

  // Convert Polar Coordinates to Cartesian coordinates
  geometry_msgs::msg::Point polarToCart(unsigned int index) 
  {
    std::unique_lock<std::mutex> lck(mtx_);
    float angle = scan_msg_.angle_min + scan_msg_.angle_increment * index;
    float range = scan_msg_.ranges.at(index);
    lck.unlock();

    geometry_msgs::msg::Point cart;
    cart.x = static_cast<double>(range * cos(angle));
    cart.y = static_cast<double>(range * sin(angle));
    return cart;
  }

  // Transform from Robot Frame to World Frame
  geometry_msgs::msg::Point transformToWorld(const geometry_msgs::msg::Point& local_pt) {
    geometry_msgs::msg::Point world_pt;

    // Get the robot's position and yaw angle from the odometry message
    double robot_x = odo_.pose.pose.position.x;
    double robot_y = odo_.pose.pose.position.y;

    tf2::Quaternion q(
        odo_.pose.pose.orientation.x,
        odo_.pose.pose.orientation.y,
        odo_.pose.pose.orientation.z,
        odo_.pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Apply the transformation: rotate and translate the local point
    world_pt.x = robot_x + local_pt.x * cos(yaw) - local_pt.y * sin(yaw);
    world_pt.y = robot_y + local_pt.x * sin(yaw) + local_pt.y * cos(yaw);
    world_pt.z = 0.0;  // Assuming the cylinder is on the ground

    return world_pt;
  }

  // Avoid the Cylinder
  void avoidCyl(const geometry_msgs::msg::Point& cyl_pt){

    // varaibles that can be changed    
    double radius = 0.6;
    double linear_vel = 0.3;
    double angular_vel = linear_vel/radius;

    // distance to cylinder center
    double distance = std::hypot(odo_.pose.pose.position.x - cyl_pt.x, odo_.pose.pose.position.y - cyl_pt.y);    
    std::cout<<"distance: " << distance << std::endl;

    // STATE MACHINE
    switch(state_){
      case States::DRIVE :{
        RCLCPP_INFO(this->get_logger(), "Nav2 Running");


        // if the mutex has been locked then unlock it here
        if(is_locked_){
          std::unique_lock<std::mutex> lck(mtx_, std::try_to_lock); // try to take the lock
          
          if(lck.owns_lock()){
            //lck.unlock();
            is_locked_ = false;
          }
        }

        distance = std::hypot(odo_.pose.pose.position.x - cyl_pt.x, odo_.pose.pose.position.y - cyl_pt.y);
        // check if need to change state    
        if(distance < 1){
          state_ = States::AVOID;
        }

        break;
      }
      case States::AVOID :{
        RCLCPP_INFO(this->get_logger(), "Avoiding Cylinder");

        cmd_twist_.linear.x = linear_vel;
        cmd_twist_.angular.z = angular_vel;

        std::unique_lock<std::mutex> lck(mtx_);
        cmd_pub_->publish(cmd_twist_);
        is_locked_ = true;

        distance = std::hypot(odo_.pose.pose.position.x - cyl_pt.x, odo_.pose.pose.position.y - cyl_pt.y);
        if(distance >= 1){
          //cmd_twist_.linear.x = 0;
          //cmd_twist_.angular.z = 0;
          state_ = States::DRIVE; // change state
        }

        break;
      }
    } // switch statement end

  }


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavAvoid>());
  rclcpp::shutdown();
  return 0;
}