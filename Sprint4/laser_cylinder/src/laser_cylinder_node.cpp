#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <cmath>

class SingleCylinderDetector : public rclcpp::Node {
public:
    SingleCylinderDetector() : Node("single_cylinder_detector_node") {
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/single_cylinder_marker", 10
        );
        pub_cylinder_center_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/detected_cylinder_position", 10
        );
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SingleCylinderDetector::laserScanCallback, this, std::placeholders::_1)
        );

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&SingleCylinderDetector::odometryCallback, this, std::placeholders::_1)
        );

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&SingleCylinderDetector::imageCallback, this, std::placeholders::_1));
    }

private:
    sensor_msgs::msg::LaserScan latest_laser_scan_;
    nav_msgs::msg::Odometry current_odometry_;
    bool green_detected_ = false;  // Store the state of green detection

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS image to OpenCV format
        green_detected_ = false;
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'BGR8'.", msg->encoding.c_str());
            return;
        }
        
        green_detected_ = detectGreen(cv_ptr->image);  // Set global green detection flag
    }

    bool detectGreen(const cv::Mat& image){
        cv::Mat hsv_image;

        // Convert BGR to HSV
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // Define HSV range for green color
        cv::Scalar lower_green(35, 100, 100); // Lower bound for green
        cv::Scalar upper_green(85, 255, 250); // Upper bound for green

        cv::Mat mask;
        // Create a mask using the defined range
        cv::inRange(hsv_image, lower_green, upper_green, mask);

        return cv::countNonZero(mask) > 50;
    }

void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_laser_scan_ = *msg;

    geometry_msgs::msg::Point detected_cylinder;

    // Step 1: Always check for 30cm cylinder within the front 180 degrees
    bool detected_30cm_cylinder = findCylinder(detected_cylinder, 0.26);
    if (detected_30cm_cylinder) {
        // Transform the local cylinder position to world coordinates
        geometry_msgs::msg::Point cylinder_world = transformToWorldCoordinates(detected_cylinder);
        
        // Publish the transformed cylinder position in world coordinates
        pub_cylinder_center_->publish(cylinder_world);

        // Publish the marker with the same world coordinates
        publishCylinderMarker(cylinder_world, 0.30);  // 30cm marker
        RCLCPP_INFO(this->get_logger(), "Detected 30cm cylinder at world position [%f, %f]", cylinder_world.x, cylinder_world.y);
    }

    // Step 2: Check for 20cm cylinder (representing a weed), only detect if green is present
    bool detected_20cm_cylinder = findCylinder(detected_cylinder, 0.16);
    if (detected_20cm_cylinder && green_detected_) {
        // Transform the local cylinder position to world coordinates
        geometry_msgs::msg::Point cylinder_world = transformToWorldCoordinates(detected_cylinder);
            
        // Publish the marker with the same world coordinates
        publishWeedMarker(cylinder_world, 0.20);  // 20cm marker
        RCLCPP_INFO(this->get_logger(), "Detected 20cm weed at world position [%f, %f]", cylinder_world.x, cylinder_world.y);
    } else if (detected_20cm_cylinder && !green_detected_) {
        RCLCPP_INFO(this->get_logger(), "Detected 20cm cylinder, but no green. Ignoring weed detection.");
    }
}




bool findCylinder(geometry_msgs::msg::Point& cylinder, double diameter) {
    cylinder = geometry_msgs::msg::Point();  // Reset the point
    const double proximity_threshold = 0.5;
    const double allowable_tolerance = 0.03;  // Tighten the tolerance to avoid overlap between diameters

    std::vector<geometry_msgs::msg::Point> current_segment;

    // Analyze laser scan points for cylinder detection (0 to 90 degrees)
    for (size_t i = 0; i < 125; i++) {
        double range_value = latest_laser_scan_.ranges[i];

        if (std::isfinite(range_value) && range_value < latest_laser_scan_.range_max) {
            geometry_msgs::msg::Point laser_point = convertToCartesian(i);

            if (current_segment.empty()) {
                current_segment.push_back(laser_point);
            } else {
                geometry_msgs::msg::Point previous_point = current_segment.back();
                double distance_between_points = std::hypot(laser_point.x - previous_point.x, laser_point.y - previous_point.y);

                if (distance_between_points < proximity_threshold) {
                    current_segment.push_back(laser_point);
                } else {
                    // Validate if current segment forms a valid cylinder
                    if (current_segment.size() > 1 && isValidCylinderSegment(current_segment, diameter, allowable_tolerance)) {
                        cylinder = calculateCenterPoint(current_segment);


                        return true;  // Found a cylinder
                    }

                    current_segment.clear();  // Reset segment if not valid
                    current_segment.push_back(laser_point);
                }
            }
        }
    }

    // Analyze laser scan points for cylinder detection (270 to 360 degrees)
    for (size_t i = 235; i < 360; i++) {
        double range_value = latest_laser_scan_.ranges[i];

        if (std::isfinite(range_value) && range_value < latest_laser_scan_.range_max) {
            geometry_msgs::msg::Point laser_point = convertToCartesian(i);

            if (current_segment.empty()) {
                current_segment.push_back(laser_point);
            } else {
                geometry_msgs::msg::Point previous_point = current_segment.back();
                double distance_between_points = std::hypot(laser_point.x - previous_point.x, laser_point.y - previous_point.y);

                if (distance_between_points < proximity_threshold) {
                    current_segment.push_back(laser_point);
                } else {
                    // Validate if current segment forms a valid cylinder
                    if (current_segment.size() > 1 && isValidCylinderSegment(current_segment, diameter, allowable_tolerance)) {
                        cylinder = calculateCenterPoint(current_segment);

                        // Debug: Print the cylinder's size and details
                        RCLCPP_INFO(this->get_logger(), "Cylinder center at x: %f, y: %f", cylinder.x, cylinder.y);

                        return true;  // Found a cylinder
                    }

                    current_segment.clear();  // Reset segment if not valid
                    current_segment.push_back(laser_point);
                }
            }
        }
    }

    // Check the last segment
    if (current_segment.size() > 1 && isValidCylinderSegment(current_segment, diameter, allowable_tolerance)) {
        cylinder = calculateCenterPoint(current_segment);

        // Debug: Print the cylinder's size and details
        RCLCPP_INFO(this->get_logger(), "Cylinder center at x: %f, y: %f", cylinder.x, cylinder.y);

        return true;  // Found a cylinder
    }

    return false;  // No cylinder found
}


    bool isValidCylinderSegment(const std::vector<geometry_msgs::msg::Point>& segment, double desired_diameter, double allowable_tolerance) {
        if (segment.size() < 7) {
            return false;  // Too few points in segment
        }

        double distance = calculateDistance(segment.front(), segment.back());
        if(std::abs(distance - desired_diameter) > allowable_tolerance){
            return false;
        }
        
        // Step 1: Calculate the center point (midpoint of the first and last points)
        geometry_msgs::msg::Point center = calculateCenterPoint(segment);

        // Step 2: Calculate average radius based on the first and last points
        //double expected_radius = desired_diameter / 2.0;

        // Step 3: Check the distance of each point from the center and see if it forms a circle
        double average_radius = 0.0;
        std::vector<double> distances;

        for (const auto& point : segment) {
            double distance = std::hypot(point.x - center.x, point.y - center.y);
            distances.push_back(distance);
            average_radius += distance;
        }
        average_radius /= segment.size();  // Calculate average radius
        

        // Step 4: Verify that the average radius is close to the expected radius
        if (std::abs(average_radius*2 - desired_diameter) > allowable_tolerance) {
            
            return false;  // The average radius deviates too much from the expected radius
        }

        // Step 5: Verify that all points are within a small tolerance from the average radius
        for (const auto& distance : distances) {
            if (std::abs(distance - average_radius) > allowable_tolerance) {
                return false;  // Point is too far from the expected circular shape
            }
        }
        // RCLCPP_INFO(this->get_logger(), "Segment diameter is %f", average_radius*2);
        RCLCPP_INFO(this->get_logger(), "Segment diameter is %f", distance);
        return true;  // Segment is valid, points form a circle
    }

    double calculateDistance(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2) {
        return std::hypot(point2.x - point1.x, point2.y - point1.y);
    }

    geometry_msgs::msg::Point calculateCenterPoint(const std::vector<geometry_msgs::msg::Point>& segment) {
        geometry_msgs::msg::Point firstPoint = segment.front();
        geometry_msgs::msg::Point lastPoint = segment.back();
        geometry_msgs::msg::Point center;

        // Calculate the center point of the segment (midpoint)
        center.x = (firstPoint.x + lastPoint.x) / 2.0;
        center.y = (firstPoint.y + lastPoint.y) / 2.0;

        return center;
    }

    geometry_msgs::msg::Point convertToCartesian(size_t index) {
        float angle = latest_laser_scan_.angle_min + latest_laser_scan_.angle_increment * index;
        float range = latest_laser_scan_.ranges[index];

        geometry_msgs::msg::Point cartesian_point;
        cartesian_point.x = static_cast<double>(range * cos(angle));
        cartesian_point.y = static_cast<double>(range * sin(angle));
        return cartesian_point;
    }

    geometry_msgs::msg::Point transformToWorldCoordinates(const geometry_msgs::msg::Point& local_point) {
        geometry_msgs::msg::Point world_point;

        // Robot's current position from odometry
        double robot_x = current_odometry_.pose.pose.position.x;
        double robot_y = current_odometry_.pose.pose.position.y;

        // Robot's orientation in yaw (from odometry)
        tf2::Quaternion robot_orientation(
            current_odometry_.pose.pose.orientation.x,
            current_odometry_.pose.pose.orientation.y,
            current_odometry_.pose.pose.orientation.z,
            current_odometry_.pose.pose.orientation.w
        );
        
        tf2::Matrix3x3 rotation_matrix(robot_orientation);
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);  // Get yaw angle

        // Transform from local (robot) coordinates to world coordinates
        world_point.x = robot_x + local_point.x * cos(yaw) - local_point.y * sin(yaw);
        world_point.y = robot_y + local_point.x * sin(yaw) + local_point.y * cos(yaw);
        world_point.z = 0.0;  // Ground level

        return world_point;
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odometry_ = *msg;
    }

    void publishCylinderMarker(const geometry_msgs::msg::Point& world_point, double diameter) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map"; // Marker frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cylinder"; 
        marker.id = 0; 

        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = world_point; // Use the calculated world coordinates

        marker.pose.orientation.w = 1.0; // No rotation

        marker.scale.x = diameter; // Diameter
        marker.scale.y = diameter; // Diameter
        marker.scale.z = 0.5; // Height

        marker.color.r = 0.0;
        marker.color.g = 0.2;
        marker.color.b = 1.0; // Blue
        marker.color.a = 1.0; // Opaque

        pub_marker_->publish(marker);
    }

    void publishWeedMarker(const geometry_msgs::msg::Point& world_point, double diameter) {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map"; // Marker frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cylinder"; 
        marker.id = 0; 

        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = world_point; // Use the calculated world coordinates

        marker.pose.orientation.w = 1.0; // No rotation

        marker.scale.x = diameter; // Diameter
        marker.scale.y = diameter; // Diameter
        marker.scale.z = 0.5; // Height

        marker.color.r = 0.0;
        marker.color.g = 1.0; // Green
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Opaque

        pub_marker_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_cylinder_center_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SingleCylinderDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
