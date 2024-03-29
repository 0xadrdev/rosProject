//==============================================================
// Filename : PongControllerNode.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description : Ros Node to control the game. 
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

#include "PongController.h"
#include "../../pong_ros_core/include/Constants.h"

using std::placeholders::_1;
using namespace pong_ros_constants;

class PongControllerNode : public rclcpp::Node {
  public:
    PongControllerNode()
    : Node("pong_controller_node") {

      // Subscriptions. 
      left_paddle_position_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
      TOPIC_LEFT_PADDLE_POSITION, 10, std::bind(&PongControllerNode::handle_left_paddle_position_subscription, this, _1));
      
      right_paddle_position_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
      TOPIC_RIGHT_PADDLE_POSITION, 10, std::bind(&PongControllerNode::handle_right_paddle_position_subscription, this, _1));
      
      // Publishers. 
      ball_position_publisher_ = this -> create_publisher<pong_ros_interfaces::msg::BallPosition>(TOPIC_BALL_POSITION, 10);
      
      // Instance the pongController class. 
      pong_controller_ = PongController();

      // Setting up a timer to call requestAnimationFrame at 60 FPS. 
      request_animation_frame_ = this -> create_wall_timer(std::chrono::milliseconds(1000 / 60), std::bind(&PongControllerNode::requestAnimationFrame, this));
    }
    
  private:
    void handle_left_paddle_position_subscription(const std_msgs::msg::Float64::SharedPtr leftPaddlePositionMsg) {
      // RCLCPP_INFO(this->get_logger(), "Left paddle position : %f", leftPaddlePositionMsg -> data);
      pong_controller_.setLeftPaddlePosition(leftPaddlePositionMsg -> data);
    }

    void handle_right_paddle_position_subscription(const std_msgs::msg::Float64::SharedPtr rightPaddlePositionMsg) {
      // RCLCPP_INFO(this->get_logger(), "Right paddle Position: %f", rightPaddlePositionMsg -> data);
      pong_controller_.setRightPaddlePosition(rightPaddlePositionMsg -> data);
    }

    void requestAnimationFrame() {
      pong_controller_.determineCollisionType();

      pong_controller_.incrementVelocity(); // Increments the velocity of the ball if it bounces the paddles. 

      pong_controller_.updateBallPosition();

      auto ball_position_msg = pong_ros_interfaces::msg::BallPosition();
      ball_position_msg.x = pong_controller_.getBallPositionX();
      ball_position_msg.y = pong_controller_.getBallPositionY();

      ball_position_publisher_ -> publish(ball_position_msg);

      int collisionDetected = pong_controller_.getCollisionType();
      if (collisionDetected == NO_COLLISION) return;
      // RCLCPP_INFO(this->get_logger(), "Collision Detected: %d", collisionDetected);
      // RCLCPP_INFO(this->get_logger(), "New ball position: (%f, %f)", ball_position_msg.x, ball_position_msg.y);
    }

    // ROS 2 declarations. 
    BallPhysics ball_physics_;
    PongController pong_controller_;
    rclcpp::TimerBase::SharedPtr request_animation_frame_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_paddle_position_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_paddle_position_subscription_;
    rclcpp::Publisher<pong_ros_interfaces::msg::BallPosition>::SharedPtr ball_position_publisher_;
};

// Initializing the node. 
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PongControllerNode>());
  rclcpp::shutdown();

  return 0;
}


