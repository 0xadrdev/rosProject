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
#include "../../pong_ros_core/include/Constants.h"

using std::placeholders::_1;
using namespace pong_ros_constants;

class LightPositionNode : public rclcpp::Node {
  public:
    LightPositionNode()
    : Node("subpub_light") {
      // Subscriptions
      image_input_subscription_ = this -> create_subscription<sensor_msgs::msg::Image>(
      TOPIC_IMAGE_INPUT, 10,std::bind(&LightPositionNode::handle_image_input_susbcription, this, _1));
      
      // Publishers. 
      paddle_pos_publisher_ = this -> create_publisher<std_msgs::msg::Float64>(TOPIC_LIGHT_POSITION, 10);
    }

  private:
//     void handle_image_input_susbcription(const sensor_msgs::msg::Image::SharedPtr image) {
//       // Confirming data is read
//       // RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.header << "' y: '" << msg.width << "'");
//       const int THRESHOLD = 50;
      
//       int imageWidth = getImageWidth(image);
//       int imageHeight = getImageHeight(image);

//       RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << imageWidth << "' y: '" << imageHeight << "'");

//       for (int y = 0; y < height; y++) {
//           for (int x = 0; x < width; x++) {
//               int brightness = getPixelBrightness(msg, x, y);
//               setPixelColor(grayImage, x, y, brightness, brightness, brightness);
//           }
//       }

//       // Applying threshold value
//       const int THRESHOLD = 50;

//       for (int y = 0; y < height; y++) {
//           for (int x = 0; x < width; x++) {
//               int imageBrightness = getPixelBrightness(image, x, y);
//               setPixelColor(grayImage, x, y, imageBrightness, imageBrightness, imageBrightness);

//               int brightness = getPixelBrightness(grayImage, x, y);
//               if (brightness > THRESHOLD) {
//                   setPixelColor(grayImage, x, y, 255, 255, 255);
//               } else {
//                   setPixelColor(grayImage, x, y, 0, 0, 0);
//               }
//           }
//       }

//       // Computing the center of gravity
//       int sumBrightness = 0;
//       //int sumX = 0;
//       int sumY = 0;

//       for (int y = 0; y < height; y++) {
//           for (int x = 0; x < width; x++) {
//               int imageBrightness = getPixelBrightness(image, x, y);
//               setPixelColor(grayImage, x, y, imageBrightness, imageBrightness, imageBrightness);

//               int brightness = getPixelBrightness(gra, x, y);
//               if (brightness == 255) { // white pixel
//                   sumBrightness += brightness;
//                   //sumX += x * brightness;
//                   sumY += y * brightness;
//               }
//           }
//       }
      

//       int sumY = 0;
//       int sumBrightness = 0;
//       sensor_msgs::msg::Image::SharedPtr grayImage(new sensor_msgs::msg::Image);
//       copyImageProperties(grayImage, image); // copy properties from input image

//       for (int y = 0; y < imageHeight; y++) {
//         for (int x = 0; x < imageWidth; x++) {
//           int imageBrightness = getPixelBrightness(image, x, y);
//           setPixelColor(grayImage, x, y, imageBrightness, imageBrightness, imageBrightness);
          
//           int brightness = getPixelBrightness(grayImage, x, y);
//           if (brightness > THRESHOLD) {
//             // setPixelColor(grayImage, x, y, 255, 255, 255);
//             sumBrightness += 255; 
//             sumY += y * 255;
//           } 
//         }
//       }
      
//       double centerY = (double) sumY / sumBrightness;
//       if (std::isnan(centerY)) {
//           centerY = 100.0;
//       }

//       auto paddle_pos_msg = std_msgs::msg::Float64();
//       paddle_pos_msg.data = centerY;
//       paddle_pos_publisher_->publish(paddle_pos_msg);
      
//       RCLCPP_INFO(this->get_logger(), "Publishing light_position: %f", paddle_pos_msg.data);
//     }
    
//     // ROS 2 declarations. 
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_input_subscription_;
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr paddle_pos_publisher_;
// };

   void handle_image_input_susbcription(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      // Confirming data is read
      //RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");

      //ball_physics pongPhysics_;

      int imageWidth = getImageWidth(msg);
      int imageHeight = getImageHeight(msg);

      //double value = width + height - width;
      // Grayscaling the image:
      sensor_msgs::msg::Image::SharedPtr grayImage(new sensor_msgs::msg::Image);
      copyImageProperties(grayImage, msg); // copy properties from input image

      for (int y = 0; y < imageHeight; y++) {
          for (int x = 0; x < imageWidth; x++) {
              int brightness = getPixelBrightness(msg, x, y);
              setPixelColor(grayImage, x, y, brightness, brightness, brightness);
          }
      }

      // Applying threshold value
      int sumBrightness = 0;
      int sumY = 0;
      const int THRESHOLD = 50;
      for (int y = 0; y < imageHeight; y++) {
        for (int x = 0; x < imageWidth; x++) {
          int brightness = getPixelBrightness(msg, x, y);
          if (brightness > THRESHOLD) {
            setPixelColor(grayImage, x, y, 255, 255, 255);
          }

          if (getPixelBrightness(msg, x, y) == 255) {
            sumBrightness += 255;
            sumY += y * 255;
          }
        }
      }

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

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_input_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr paddle_pos_publisher_;
    //ball_physics pongPhysics_;
};

// Initializing the node. 
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPositionNode>());
  rclcpp::shutdown();
  
  return 0;
}
