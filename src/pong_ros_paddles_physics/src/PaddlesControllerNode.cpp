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

class PaddlesControllerNode : public rclcpp::Node
{
  public:
    PaddlesControllerNode()
    : Node("paddles_controller_node")
    {
      // Subscriptions
      keyboard_input_subscription_ = this -> create_subscription<std_msgs::msg::Int16>(
        "/keyboard_input/key", 1, std::bind(&PaddlesControllerNode::handle_keyboard_input_subscription, this, _1));

      light_position_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
      "light_position", 1,std::bind(&PaddlesControllerNode::handle_light_position_subscription, this, _1));
      
      // Publishing
      left_paddle_position_publisher_ = this -> create_publisher<std_msgs::msg::Float64>(TOPIC_LEFT_PADDLE_POSITION, 10);
      right_paddle_position_publisher_ = this -> create_publisher<std_msgs::msg::Float64>(TOPIC_RIGHT_PADDLE_POSITION, 10);
      
      // Initialize the class object used to compute the ball physics
      // Could be done, but the simplicity of this task allows for implementation in the ros2 node itself.
      
    }

  private:

    // Paddle position

    double leftPaddlePosition = 300;
    double rightPaddlePosition = 300;

    // Ros2 declarations. 

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr keyboard_input_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr light_position_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_paddle_position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_paddle_position_publisher_;

    // Subscriptions handlers. 
  
    void handle_keyboard_input_subscription(const std_msgs::msg::Int16::SharedPtr msg)
    {
      // Confirming data is read
      // RCLCPP_INFO_STREAM(this -> get_logger(), "I heard '" << msg -> data << "'");
      int key_code = msg -> data;
      
      // Setting up the message. 
      auto paddle_position_msg = std_msgs::msg::Float64();
      
      double paddle_velocity = 5;
      if (key_code == 115 && leftPaddlePosition < 535) { // 115  ->  W
          paddle_position_msg.data = leftPaddlePosition + paddle_velocity;
      	  leftPaddlePosition = leftPaddlePosition + paddle_velocity;
      } else if (key_code == 119 && leftPaddlePosition > 65) { // 119  ->  S
      	  paddle_position_msg.data = leftPaddlePosition - paddle_velocity;
      	  leftPaddlePosition = leftPaddlePosition - paddle_velocity;
      } else {
      	  paddle_position_msg.data = leftPaddlePosition; 
      }

      auto right_paddle_position_msg = std_msgs::msg::Float64();

      if (key_code == 66 && rightPaddlePosition < 535) {
          right_paddle_position_msg.data = rightPaddlePosition + paddle_velocity;
      	  rightPaddlePosition = rightPaddlePosition + paddle_velocity;
      } else if (key_code == 65 && rightPaddlePosition > 65) {
      	  right_paddle_position_msg.data = rightPaddlePosition - paddle_velocity;
      	  rightPaddlePosition = rightPaddlePosition - paddle_velocity;
      } else {
      	  right_paddle_position_msg.data = rightPaddlePosition; 
      }
      
      // Publish the message
      left_paddle_position_publisher_ -> publish(paddle_position_msg);
      right_paddle_position_publisher_ -> publish(right_paddle_position_msg);
      
      RCLCPP_INFO(this -> get_logger(), "Publishing first_paddle_position: %1f", paddle_position_msg.data);
      // RCLCPP_INFO(this -> get_logger(), "Publishing second_paddle_position: %1f", right_paddle_position_msg.data);
    }
    
    void handle_light_position_subscription(const std_msgs::msg::Float64::SharedPtr msg)
    {
      // Confirming data is read
      // RCLCPP_INFO_STREAM(this -> get_logger(), "I heard '" << msg -> data << "'");
      
      int lightInput = msg -> data; // Holding the light high means low value, holding it low means high value.
      
      // Setting up the message. 
      auto right_paddle_position_msg = std_msgs::msg::Float64();
      
      double paddle_velocity = 50;
      if (lightInput >= 150 && rightPaddlePosition <= 5) {
          right_paddle_position_msg.data = rightPaddlePosition + paddle_velocity;
      	  rightPaddlePosition = rightPaddlePosition + paddle_velocity;
      } else if (lightInput <= 100 && rightPaddlePosition >= 80) {
      	  right_paddle_position_msg.data = rightPaddlePosition - paddle_velocity;
      	  rightPaddlePosition = rightPaddlePosition - paddle_velocity;
      } else {
      	  right_paddle_position_msg.data = rightPaddlePosition; 
      }
      
      // Publish the message
      right_paddle_position_publisher_ -> publish(right_paddle_position_msg);
      
      // RCLCPP_INFO(this -> get_logger(), "Publishing second_paddle_position: %1f", right_paddle_position_msg.data);
    }
};

// Initializing the node. 

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PaddlesControllerNode>());
  rclcpp::shutdown();
  
  return 0;
}

