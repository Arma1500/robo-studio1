#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserScanProcessor : public rclcpp::Node
{
public:
  LaserScanProcessor() : Node("sprint1_node")
  {
    //Subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
    10, std::bind(&LaserScanProcessor::scanCallback, this, std::placeholders::_1));

    //Publishers
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/nth_scan", 10);
  
  }

private:

  //Callback
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    // show scan info
    RCLCPP_INFO(this->get_logger(), "Scan angle_min: %f radians (%f degrees)", scan->angle_min, scan->angle_min * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "Scan angle_max: %f radians (%f degrees)", scan->angle_max, scan->angle_max * 180.0 / M_PI);

    // set up a scan to publish - clear ranges just incase
    auto nth_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
    nth_scan->ranges.clear();

    // for every nth scan
    nth_scan->angle_increment = scan->angle_increment * nth_point; // adjust the angle - only once

    // Get every nth point from the original scan to publish
    for(size_t i = 0; i < scan->ranges.size(); i += nth_point){
      nth_scan->ranges.push_back(scan->ranges[i]);
    }

    scan_pub_->publish(*nth_scan);

    // for test - I'm actually not sure what I should be seeing here I think they min and max is based on the ranges put into the nth_scan
    RCLCPP_INFO(this->get_logger(), "_______________________nth_scan angle_min: %f radians (%f degrees)", nth_scan->angle_min, nth_scan->angle_min * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "_______________________nth_scan angle_max: %f radians (%f degrees)", nth_scan->angle_max, nth_scan->angle_max * 180.0 / M_PI);

  }

  //Subscriber and Publishers 
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  // constant varaibles
  const double nth_point = 4;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor>());
    rclcpp::shutdown();
    return 0;
}