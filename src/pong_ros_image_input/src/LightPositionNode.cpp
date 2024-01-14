//==============================================================
// Filename : subpub_light.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description : Ros Node to publish the light position. 
//==============================================================

#include <chrono>
#include <functional>
#include <string>
#include <cstdio>
#include <iostream>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "image_functions.h"
#include "../../pong_ros_core/include/Constants.h"

using std::placeholders::_1;
using namespace pong_ros_constants;

class LightPositionNode : public rclcpp::Node {
  public:
    LightPositionNode()
    : Node("light_position_node") {
      // Subscriptions
      image_input_subscription_ = this -> create_subscription<sensor_msgs::msg::Image>(
      TOPIC_IMAGE_INPUT, 10,std::bind(&LightPositionNode::handle_image_input_susbcription, this, _1));
      
      // Publishers. 
      paddle_pos_publisher_ = this -> create_publisher<std_msgs::msg::Float64>(TOPIC_LIGHT_POSITION, 10);
    }

  private:
   void handle_image_input_susbcription(const sensor_msgs::msg::Image::SharedPtr imageMsg) {
      int imageWidth = getImageWidth(imageMsg);
      int imageHeight = getImageHeight(imageMsg);

      // Grayscaling the image:
      sensor_msgs::msg::Image::SharedPtr grayImage(new sensor_msgs::msg::Image);
      copyImageProperties(grayImage, imageMsg); // copy properties from input image

      for (int y = 0; y < imageHeight; y++) {
          for (int x = 0; x < imageWidth; x++) {
              int brightness = getPixelBrightness(imageMsg, x, y);
              setPixelColor(grayImage, x, y, brightness, brightness, brightness);
          }
      }

      int sumBrightness = 0;
      int sumY = 0;
      const int THRESHOLD = 50;
      for (int y = 0; y < imageHeight; y++) {
        for (int x = 0; x < imageWidth; x++) {
          int brightness = getPixelBrightness(imageMsg, x, y);
          if (brightness > THRESHOLD) {
            setPixelColor(grayImage, x, y, 255, 255, 255);
          }

          if (getPixelBrightness(imageMsg, x, y) == 255) {
            sumBrightness += 255;
            sumY += y * 255;
          }
        }
      }

      double centerY = (double) sumY / sumBrightness;

      // Setting up the message. 

      if (std::isnan(centerY)) {
        centerY = 100.0;
      }

      auto lightPositionMsg = std_msgs::msg::Float64();
      lightPositionMsg.data = centerY;

      // Publish the message
      paddle_pos_publisher_ -> publish(lightPositionMsg);

      RCLCPP_INFO(this -> get_logger(), "lightPosition: %f", lightPositionMsg.data);
    }

    // ROS2 declarations. 
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_input_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr paddle_pos_publisher_;
};

// Initializing the node. 
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPositionNode>());
  rclcpp::shutdown();
  
  return 0;
}
