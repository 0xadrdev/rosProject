//==============================================================
// Filename : PaddlesControllerNode.cpp
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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float64.hpp"
#include "../../pong_ros_core/include/Constants.h"

using std::placeholders::_1;
using namespace pong_ros_constants;

class PaddlesControllerNode : public rclcpp::Node {
  public:
    PaddlesControllerNode()
    : Node("paddles_controller_node") {
      // Subscriptions
      keyboard_input_subscription_ = this -> create_subscription<std_msgs::msg::Int16>(
      "/keyboard_input/key", 1, std::bind(&PaddlesControllerNode::handle_keyboard_input_subscription, this, _1));

      light_position_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
      TOPIC_LIGHT_POSITION, 1,std::bind(&PaddlesControllerNode::handle_light_position_subscription, this, _1));
      
      // Publishers. 
      left_paddle_position_publisher_ = this -> create_publisher<std_msgs::msg::Float64>(TOPIC_LEFT_PADDLE_POSITION, 10);
      right_paddle_position_publisher_ = this -> create_publisher<std_msgs::msg::Float64>(TOPIC_RIGHT_PADDLE_POSITION, 10);
    }

  private:
    double leftPaddlePosition = 300;
    double rightPaddlePosition = 300;

    double light_paddle_velocity = 50;
    double keyboard_paddle_velocity = 5;

    void handle_keyboard_input_subscription(const std_msgs::msg::Int16::SharedPtr keyCodeMsg) {
      // Confirming data is read
      // RCLCPP_INFO_STREAM(this -> get_logger(), "I heard '" << keyCodeMsg -> data << "'");
      int key_code = keyCodeMsg -> data;

      // Setting up the message. 
      auto paddle_position_msg = std_msgs::msg::Float64();
      
      if (key_code == 115 && leftPaddlePosition < 535) { 
          paddle_position_msg.data = leftPaddlePosition + keyboard_paddle_velocity;
      	  leftPaddlePosition = leftPaddlePosition + keyboard_paddle_velocity;
      } else if (key_code == 119 && leftPaddlePosition > 65) {
      	  paddle_position_msg.data = leftPaddlePosition - keyboard_paddle_velocity;
      	  leftPaddlePosition = leftPaddlePosition - keyboard_paddle_velocity;
      } else {
      	  paddle_position_msg.data = leftPaddlePosition; 
      }

      auto right_paddle_position_msg = std_msgs::msg::Float64();

      if (key_code == 66 && rightPaddlePosition < 535) {
          right_paddle_position_msg.data = rightPaddlePosition + keyboard_paddle_velocity;
      	  rightPaddlePosition = rightPaddlePosition + keyboard_paddle_velocity;
      } else if (key_code == 65 && rightPaddlePosition > 65) {
      	  right_paddle_position_msg.data = rightPaddlePosition - keyboard_paddle_velocity;
      	  rightPaddlePosition = rightPaddlePosition - keyboard_paddle_velocity;
      } else {
      	  right_paddle_position_msg.data = rightPaddlePosition; 
      }
      
      // Publish the message
      left_paddle_position_publisher_ -> publish(paddle_position_msg);
      right_paddle_position_publisher_ -> publish(right_paddle_position_msg);
      
      // RCLCPP_INFO(this -> get_logger(), "Publishing first_paddle_position: %1f", paddle_position_msg.data);
    }
    
    void handle_light_position_subscription(const std_msgs::msg::Float64::SharedPtr lightPositionMsg) {
      // Confirming data is read
      // RCLCPP_INFO_STREAM(this -> get_logger(), "I heard '" << lightPositionMsg -> data << "'");
      
      int lightPosition = lightPositionMsg -> data;
      
      // Setting up the message. 
      auto right_paddle_position_msg = std_msgs::msg::Float64();
      
      if (lightPosition >= 150 && rightPaddlePosition <= 5) {
          right_paddle_position_msg.data = rightPaddlePosition + light_paddle_velocity;
      	  rightPaddlePosition = rightPaddlePosition + light_paddle_velocity;
      } else if (lightPosition <= 100 && rightPaddlePosition >= 80) {
      	  right_paddle_position_msg.data = rightPaddlePosition - light_paddle_velocity;
      	  rightPaddlePosition = rightPaddlePosition - light_paddle_velocity;
      } else {
      	  right_paddle_position_msg.data = rightPaddlePosition; 
      }
      
      // Publish the message
      right_paddle_position_publisher_ -> publish(right_paddle_position_msg);
      
      // RCLCPP_INFO(this -> get_logger(), "Publishing second_paddle_position: %1f", right_paddle_position_msg.data);
    }

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr keyboard_input_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr light_position_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_paddle_position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_paddle_position_publisher_;
};

// Initializing the node. 
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PaddlesControllerNode>());
  rclcpp::shutdown();
  
  return 0;
}

