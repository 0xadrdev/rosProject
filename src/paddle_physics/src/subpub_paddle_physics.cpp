#include <chrono>
#include <functional>
#include <string>
#include <cstdio>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float64.hpp"

//#include "ball_physics.h"

using std::placeholders::_1;

class SubpubPaddlePhysics : public rclcpp::Node
{
  public:
    SubpubPaddlePhysics()
    : Node("subpub_paddle_physics")
    {
      // Subscriptions
      keyCodeSub_ = this -> create_subscription<std_msgs::msg::Int16>(
        "/keyboard_input/key", 
        1,
        std::bind(&SubpubPaddlePhysics::key_callback, this, _1));

      lightPosSub_ = this -> create_subscription<std_msgs::msg::Float64>(
      "light_position", 1,std::bind(&SubpubPaddlePhysics::light_callback, this, _1));
      
      // Publishing
      paddle_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("first_paddle_position", 10);
      right_paddle_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("second_paddle_position", 10);
      
      // Initialize the class object used to compute the ball physics
      // Could be done, but the simplicity of this task allows for implementation in the ros2 node itself.
      
    }

  private:
  
    void key_callback(const std_msgs::msg::Int16::SharedPtr msg)
    {
      // Confirming data is read
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard '" << msg->data << "'");
      int keyInput = msg->data;
      
      // Setting up the message. 
      auto paddle_pos_msg = std_msgs::msg::Float64();
      
      double velY = 5;
      if (keyInput == 115 && paddlePosition <= 520) { // 115 -> W
          paddle_pos_msg.data = paddlePosition + velY;
      	  paddlePosition = paddlePosition + velY;
      } else if (keyInput == 119 && paddlePosition >= 80) { // 119 -> S
      	  paddle_pos_msg.data = paddlePosition - velY;
      	  paddlePosition = paddlePosition - velY;
      } else {
      	  paddle_pos_msg.data = paddlePosition; 
      }

      auto right_paddle_pos_msg = std_msgs::msg::Float64();

      if (keyInput == 66 && rightPaddlePosition <= 520) {
          right_paddle_pos_msg.data = rightPaddlePosition + velY;
      	  rightPaddlePosition = rightPaddlePosition + velY;
      } else if (keyInput == 65 && rightPaddlePosition >= 80) {
      	  right_paddle_pos_msg.data = rightPaddlePosition - velY;
      	  rightPaddlePosition = rightPaddlePosition - velY;
      } else {
      	  right_paddle_pos_msg.data = rightPaddlePosition; 
      }
      
      // Publish the message
      paddle_pos_publisher_->publish(paddle_pos_msg);
      right_paddle_pos_publisher_->publish(right_paddle_pos_msg);
      
      RCLCPP_INFO(this->get_logger(), "Publishing first_paddle_position: %1f", paddle_pos_msg.data);
      RCLCPP_INFO(this->get_logger(), "Publishing second_paddle_position: %1f", right_paddle_pos_msg.data);
    }
    
    void light_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
      // Confirming data is read
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard '" << msg->data << "'");
      
      int lightInput = msg->data; // Holding the light high means low value, holding it low means high value.
      
      // Setting up the message. 
      auto right_paddle_pos_msg = std_msgs::msg::Float64();
      
      double velY = 50;
      if (lightInput >= 150 && rightPaddlePosition <= 520) {
          right_paddle_pos_msg.data = rightPaddlePosition + velY;
      	  rightPaddlePosition = rightPaddlePosition + velY;
      } else if (lightInput <= 100 && rightPaddlePosition >= 80) {
      	  right_paddle_pos_msg.data = rightPaddlePosition - velY;
      	  rightPaddlePosition = rightPaddlePosition - velY;
      } else {
      	  right_paddle_pos_msg.data = rightPaddlePosition; 
      }
      
      // Publish the message
      right_paddle_pos_publisher_->publish(right_paddle_pos_msg);
      
      RCLCPP_INFO(this->get_logger(), "Publishing second_paddle_position: %1f", right_paddle_pos_msg.data);
      
    }
    
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr keyCodeSub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lightPosSub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr paddle_pos_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_paddle_pos_publisher_;

    // Initializing the paddle position
    double paddlePosition = 300;
    double rightPaddlePosition = 300;
};


// Initializing and running the subscribed and publishing node
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubpubPaddlePhysics>());
  rclcpp::shutdown();
  
  return 0;
}

