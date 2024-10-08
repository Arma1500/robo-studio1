#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <mutex>

class DetectCylinder : public rclcpp::Node
{
public:
  DetectCylinder() : Node("detect_cyl")
  {
    // Subscribers

    // Odometry
    odo_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 
    10, std::bind(&DetectCylinder::odoCallback, this, std::placeholders::_1));

    // Laser Scan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
    10, std::bind(&DetectCylinder::scanCallback, this, std::placeholders::_1));

    // Publisher - visual marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("cylinder_marker", 10);

  }

private:
  // Subsrcibers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub_;

  //Publisher
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Variables
  bool isCylinder_;
  cv::Point cyl_center_;

  sensor_msgs::msg::LaserScan scan_msg;
  nav_msgs::msg::Odometry odo_;

  std::mutex mtx;

  // Callback functions
  // Laser Scan
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    
    std::unique_lock<std::mutex> lck(mtx);
    scan_msg = *msg;
    lck.unlock();
    RCLCPP_INFO(this->get_logger(), "Got a scan");
    
    isCylinder_ = false;
    
    // Variables
    std::vector<std::vector<geometry_msgs::msg::Point>> objects;
    std::vector<geometry_msgs::msg::Point> current_object; 

    geometry_msgs::msg::Point cyl; // final cylinder

    double object_tol = 0.5;
    double diameter = 0.30;
    double dia_tol = 0.05;

    // Find all the objects that are detected by the scan
    for(unsigned int i = 0; i < scan_msg.ranges.size(); i++){
      double current_range = scan_msg.ranges.at(i);

      // if the scan is within the max range that means it is hitting something
      if (!std::isinf(current_range) && !std::isnan(current_range) && current_range < scan_msg.range_max) {
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
                        current_object.clear();
                        current_object.push_back(point);  // Start a new object
                    }
                }
            }
    }

    // Push the last object if valid
    if (current_object.size() > 1) {
        objects.push_back(current_object);
    }

    for(auto& object : objects){
      if(object.size() < 4){
        continue; // for objects that are too small to be the 30cm cylinder
      }

      // if the euclidean distance between the first and last segment is within the tolerance then it is the cylinder we need
      geometry_msgs::msg::Point firstPoint = object.front();
      geometry_msgs::msg::Point lastPoint = object.back();
      double segmentLength = std::hypot(lastPoint.x - firstPoint.x, lastPoint.y - firstPoint.y);

      // Check if the segment length is within the cylinder diameter range
      if (std::abs(segmentLength - diameter) <= dia_tol) {
          // Calculate the center point of the segment
          geometry_msgs::msg::Point cyl;
          cyl.x = (firstPoint.x + lastPoint.x) / 2.0;
          cyl.y = (firstPoint.y + lastPoint.y) / 2.0;
          isCylinder_ = true;
      }
    }

    if(isCylinder_ == true){
      RCLCPP_INFO(this->get_logger(), "30cm Cylinder Found!");

      geometry_msgs::msg::Point world_cyl = transformToWorld(cyl);
      
      cyl_center_.x = world_cyl.x;
      cyl_center_.y = world_cyl.y;
      
      publishCylinderMarker(world_cyl); // publish to Rviz
      drawCylinder(); // Draw on the map
    }

  }

  // Odometry
  void odoCallback(const nav_msgs::msg::Odometry::SharedPtr odo_msg)
  {
    std::unique_lock<std::mutex> lck(mtx);
    odo_ = *odo_msg;
    lck.unlock();
    //RCLCPP_INFO(this->get_logger(), "Got Odo");
  }

  // Convert polar coordinates (from the laser scan) to Cartesian coordinates
  geometry_msgs::msg::Point polarToCart(unsigned int index) {
      std::unique_lock<std::mutex> lck(mtx);
      float angle = scan_msg.angle_min + scan_msg.angle_increment * index;
      float range = scan_msg.ranges.at(index);
      lck.unlock();

      geometry_msgs::msg::Point cart;
      cart.x = static_cast<double>(range * cos(angle));
      cart.y = static_cast<double>(range * sin(angle));
      return cart;
  }

  // Transform from robot pose to goal pose
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

  // FUNCTIONS TO SHOW Cylinder
  // Draw on map function
  void drawCylinder()
  {
    cv::Mat world_Map = cv::imread("/Sprint3/maps/p_world/world_map.png", cv::IMREAD_COLOR);
    cv::Point center = cyl_center_; // copy the center point in here incase I have to change it again

    double default_x = 1100;
    double defauly_y = 1080;

    cv::resize(world_Map, world_Map, cv::Size(default_x,defauly_y), 0, 0, cv::INTER_AREA);

    // reajust center
    center.x = center.x - 1100;
    center.y = center.y - 1080;

    // add cylinder where it needs to be
    cv::Mat modified_world = world_Map.clone();
    cv::circle(modified_world, center, 15, cv::Scalar(255,0,0), 5);

    cv::imshow("Map with Cylinder", modified_world);
    cv::waitKey(0);

    // if it adds it to the map then shut down the node
    rclcpp::shutdown();
    RCLCPP_INFO(this->get_logger(), "Added to the Map!");
  }

  // Publishing the Marker to Rviz
  void publishCylinderMarker(const geometry_msgs::msg::Point& cylinder_position) {
        visualization_msgs::msg::Marker marker;

        // Set the frame ID and time stamp
        marker.header.frame_id = "map";  // map frame
        marker.header.stamp = this->get_clock()->now();

        // Set the namespace and ID
        marker.ns = "cylinder_detection";  // Simple namespace
        marker.id = 0;  // Single marker, use id 0

        // Set marker type and action
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position of the cylinder
        marker.pose.position.x = cylinder_position.x;
        marker.pose.position.y = cylinder_position.y;
        marker.pose.position.z = 0.0;  // Assuming it's on the ground

        // Orientation (no rotation needed)
        marker.pose.orientation.w = 1.0;

        // Set the scale (size of the cylinder)
        marker.scale.x = 0.30;  // 30 cm diameter
        marker.scale.y = 0.30;  // 30 cm diameter
        marker.scale.z = 1.0;   // 1 meter height

        // Set the color (blue, fully opaque)
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        // Set lifetime (optional)
        //marker.lifetime = rclcpp::Duration(0);  // Infinite lifetime

        // Publish the marker
        marker_pub_->publish(marker);
    }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectCylinder>());
  rclcpp::shutdown();

  return 0;
}
