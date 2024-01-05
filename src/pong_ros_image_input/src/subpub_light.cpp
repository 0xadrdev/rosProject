//==============================================================
// Filename : subpub_light.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
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

using std::placeholders::_1;

class SubpubLight : public rclcpp::Node
{
  public:
    SubpubLight()
    : Node("subpub_light")
    {
      // Subscriptions
      imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image", 10,std::bind(&SubpubLight::image_callback, this, _1));
      
      // Publishing
      paddle_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("light_position", 10);
      
      // Initialize the class object used to compute the ball physics
      //image_ = ball_physics(); 
      //ball_physics pongPhysics_;
      
    }

  private:
  
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      // Confirming data is read
      // RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.header << "' y: '" << msg.width << "'");
      
      //ball_physics pongPhysics_;
      
      int width = getImageWidth(msg);
      int height = getImageHeight(msg);

      RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << width << "' y: '" << height << "'");

      
      //double value = width + height - width;
      // Grayscaling the image:
      sensor_msgs::msg::Image::SharedPtr grayImage(new sensor_msgs::msg::Image);
      copyImageProperties(grayImage, msg); // copy properties from input image
      //grayImage->encoding = sensor_msgs::image_encodings::MONO8; // set the encoding to grayscale
      //grayImage->step = grayImage->width; // set the number of bytes per row to be equal to the image width

      const int THRESHOLD = 50;

      for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
              int brightness = getPixelBrightness(msg, x, y);
              if (brightness > THRESHOLD) {
                  setPixelColor(grayImage, x, y, 255, 255, 255);
              } else {
                  setPixelColor(grayImage, x, y, 0, 0, 0);
              }
              setPixelColor(grayImage, x, y, brightness, brightness, brightness);
          }
      }
      
      // Applying threshold value

      // for (int y = 0; y < height; y++) {
      //     for (int x = 0; x < width; x++) {
      //         int brightness = getPixelBrightness(msg, x, y);
      //         if (brightness > THRESHOLD) {
      //             setPixelColor(grayImage, x, y, 255, 255, 255);
      //         } else {
      //             setPixelColor(grayImage, x, y, 0, 0, 0);
      //         }
      //     }
      // }
      
      // Computing the center of gravity
      int sumBrightness = 0;
      int sumY = 0;

      for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
              int brightness = getPixelBrightness(msg, x, y);
              if (brightness == 255) { // white pixel
                  sumBrightness += brightness;
                  //sumX += x * brightness;
                  sumY += y * brightness;
              }
          }
      }

      //double centerX = (double) sumX / sumBrightness;
      double centerY = (double) sumY / sumBrightness;
     
      
      // Setting up the message. 
      auto paddle_pos_msg = std_msgs::msg::Float64();
      
      if (std::isnan(centerY)) {
          centerY = 100.0;
      }
      paddle_pos_msg.data = centerY;
      
      // Publish the message
      paddle_pos_publisher_->publish(paddle_pos_msg);
      
      RCLCPP_INFO(this->get_logger(), "Publishing light_position: %f", paddle_pos_msg.data);
      
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr paddle_pos_publisher_;
};

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubpubLight>());
  rclcpp::shutdown();
  
  return 0;
}
