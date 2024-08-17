#include <cstdio>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using std::placeholders::_1;

class LaserSubscriber : public rclcpp::Node
{
public:
  LaserSubscriber() : Node("laser_sub")
  {
    auto subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LaserSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Ranges between '%f' and '%f'", msg->ranges[0], msg->ranges[0]);
  }

  //rclcpp::Subscription::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserSubscriber>());
  rclcpp::shutdown();
  return 0;
}


