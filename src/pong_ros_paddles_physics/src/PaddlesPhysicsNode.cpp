//==============================================================
// Filename : PaddlesPhysicsNode.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description : ROS Node to manage the paddles of the game. 
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
#include "PaddlesPhysics.h"

using std::placeholders::_1;
using namespace pong_ros_constants;

class PaddlesPhysicsNode : public rclcpp::Node {
  public:
    PaddlesPhysicsNode()
    : Node("paddles_physics_node") {

      // Subscriptions
      keyboard_input_subscription_ = this -> create_subscription<std_msgs::msg::Int16>(
      "/keyboard_input/key", 1, std::bind(&PaddlesPhysicsNode::handle_keyboard_input_subscription, this, _1));

      light_position_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
      TOPIC_LIGHT_POSITION, 1,std::bind(&PaddlesPhysicsNode::handle_light_position_subscription, this, _1));
      
      // Publishers. 
      left_paddle_position_publisher_ = this -> create_publisher<std_msgs::msg::Float64>(TOPIC_LEFT_PADDLE_POSITION, 10);

      right_paddle_position_publisher_ = this -> create_publisher<std_msgs::msg::Float64>(TOPIC_RIGHT_PADDLE_POSITION, 10);

      // Instance of the PaddlesPhysics()
      paddles_controller_ = PaddlesPhysics();
    }

  private:
    void handle_keyboard_input_subscription(const std_msgs::msg::Int16::SharedPtr keyCodeMsg) {
      // RCLCPP_INFO(this -> get_logger(), "keyCodeMsg received: %1f", keyCodeMsg -> data);

      int keyCode = keyCodeMsg -> data;
      double leftPaddlePosition = paddles_controller_.getLeftPaddlePosition();
      double rightPaddlePosition = paddles_controller_.getRightPaddlePosition();

      if (keyCode == 115 && leftPaddlePosition < 535) { // Pressed W
        paddles_controller_.updateLeftPaddleUp();
      } else if (keyCode == 119 && leftPaddlePosition > 65) { // Presed S
        paddles_controller_.updateLeftPaddleDown();
      } 

      paddles_controller_.setRightPaddleVelocity(5);

      if (keyCode == 66 && rightPaddlePosition < 535) { // Presed up arrow. 
        paddles_controller_.updateRightPaddleUp();
      } else if (keyCode == 65 && rightPaddlePosition > 65) { // Presed down arrow. 
        paddles_controller_.updateRightPaddleDown();

      } 

      // Publish the new paddles positions. 
      auto left_paddle_position_msg = std_msgs::msg::Float64();
      left_paddle_position_msg.data = paddles_controller_.getLeftPaddlePosition();
      left_paddle_position_publisher_ -> publish(left_paddle_position_msg);

      auto right_paddle_position_msg = std_msgs::msg::Float64();
      right_paddle_position_msg.data = paddles_controller_.getRightPaddlePosition();
      right_paddle_position_publisher_ -> publish(right_paddle_position_msg);

      // RCLCPP_INFO(this -> get_logger(), "New left paddle position: ", left_paddle_position_msg.data);
      // RCLCPP_INFO(this -> get_logger(), "New right paddle position from keyboard: ", right_paddle_position_msg.data);
    }
    
    void handle_light_position_subscription(const std_msgs::msg::Float64::SharedPtr lightPositionMsg) {
      // RCLCPP_INFO(this -> get_logger(), "lightPositionMsg received: %1f", lightPositionMsg -> data);
      
      int lightPosition = lightPositionMsg -> data;
      double rightPaddlePosition = paddles_controller_.getRightPaddlePosition();

      paddles_controller_.setRightPaddleVelocity(50);

      if (lightPosition >= 150 && rightPaddlePosition <= 535) {
        paddles_controller_.updateRightPaddleUp();
      } else if (lightPosition <= 100 && rightPaddlePosition >= 65) {
        paddles_controller_.updateRightPaddleDown();
      }
      
      // Publish the new paddle position. 
      auto right_paddle_position_msg = std_msgs::msg::Float64();
      right_paddle_position_msg.data = paddles_controller_.getRightPaddlePosition();
      right_paddle_position_publisher_ -> publish(right_paddle_position_msg);
      
      // RCLCPP_INFO(this -> get_logger(), "New right paddle position from light %1f", right_paddle_position_msg.data);
    }

    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr keyboard_input_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr light_position_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_paddle_position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_paddle_position_publisher_;

    PaddlesPhysics paddles_controller_; 
};

// Initializing the node. 
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PaddlesPhysicsNode>());
  rclcpp::shutdown();
  
  return 0;
}

