//==============================================================
// Filename : PongControllerNode.cpp
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
#include "std_msgs/msg/float64.hpp"
#include "pong_ros_interfaces/msg/ball_position.hpp" 
#include "pong_ros_interfaces/msg/ball_velocity.hpp" 
#include "PongController.h"
#include "../../pong_ros_core/include/Constants.h"

using std::placeholders::_1;
using namespace pong_ros_constants;

class PongControllerNode:public rclcpp::Node
{
  public:
    PongControllerNode()
    : Node("pong_controller_node") {
      // Subscriptions
      ball_position_subscription_ = this->create_subscription<pong_ros_interfaces::msg::BallPosition>(
      TOPIC_BALL_POSITION, 10, std::bind(&PongControllerNode::handle_ball_position_subscription, this, _1));

      left_paddle_position_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      TOPIC_LEFT_PADDLE_POSITION, 10, std::bind(&PongControllerNode::handle_left_paddle_position_subscription, this, _1));
      
      right_paddle_position_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      TOPIC_RIGHT_PADDLE_POSITION, 10, std::bind(&PongControllerNode::handle_right_paddle_position_subscription, this, _1));
      
      // Publishing
      ball_velocity_publisher_ = this->create_publisher<pong_ros_interfaces::msg::BallVelocity>(TOPIC_BALL_VELOCITY, 10);
      
      // Initialize the class object used to compute the logic
      pongController_ = PongController();
      
      // Initialize a timer for the iteration speed of the game.
      timer_ = this -> create_wall_timer(std::chrono::milliseconds(1000 / 60), std::bind(&PongControllerNode::timer_callback, this));
    }
    
    private:

      // Subscriptions handlers. 

      void handle_ball_position_subscription(const pong_ros_interfaces::msg::BallPosition & msg) {
        // Confirming data is read correctly
        // RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");
        
        // Update the ball position using the message from the subscription
          pongController_.setBallPositionX(msg.x);
          pongController_.setBallPositionY(msg.y);
      }
      
      void handle_left_paddle_position_subscription(const std_msgs::msg::Float64::SharedPtr msg)
      {
        // RCLCPP_INFO(this->get_logger(), "Received first paddle position: %f", msg->data);
        pongController_.setLeftPaddlePosition(msg->data);
      }

      void handle_right_paddle_position_subscription(const std_msgs::msg::Float64::SharedPtr msg)
      {
        // RCLCPP_INFO(this->get_logger(), "Received second paddle position: %f", msg->data);
        pongController_.setRightPaddlePosition(msg->data);
      }
      
      void timer_callback() {
          auto ball_vel_msg = pong_ros_interfaces::msg::BallVelocity();
          
          // Using the logic class
          pongController_.checkCollision();

          pongController_.updateBallVelocity();

          pongController_.incrementVelocity();

          ball_vel_msg.x = pongController_.getBallVelocityX();
          ball_vel_msg.y = pongController_.getBallVelocityY();

          // Publishing the updated ball velocity. 
          ball_velocity_publisher_ -> publish(ball_vel_msg);

          // RCLCPP_INFO(this->get_logger(), "Publishing ball_velocity: (%f, %f)", ball_vel_msg.x, ball_vel_msg.y);
      }

      // ROS 2 declarations. 

      rclcpp::TimerBase::SharedPtr timer_;

      rclcpp::Subscription<pong_ros_interfaces::msg::BallPosition>::SharedPtr ball_position_subscription_;
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_paddle_position_subscription_;
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_paddle_position_subscription_;

      rclcpp::Publisher<pong_ros_interfaces::msg::BallVelocity>::SharedPtr ball_velocity_publisher_;

      PongController pongController_;
};


// Initialing the node. 

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PongControllerNode>());
    rclcpp::shutdown();

    return 0;
}


