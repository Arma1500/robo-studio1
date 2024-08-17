#include <cstdio>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class ImageProcessor : public rclcpp::Node
{
public:
  ImageProcessor() : Node("my_node")
  {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw", 10, std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_modified", 10);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
      {
          try
          {
              cv::Mat image = cv_bridge::toCvShare(msg, "rgb8")->image;
              RCLCPP_INFO(this->get_logger(), "Image received");
              cv::Point center(image.cols / 2, image.rows / 2);
              cv::circle(image, center, 50, cv::Scalar(0, 255, 0), 3);
              auto modified_msg = cv_bridge::CvImage(msg->header, "rgb8", image).toImageMsg();

              image_pub_->publish(*modified_msg);
          }
          catch (cv_bridge::Exception& e)
          {
              RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
          }
      }
      rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
