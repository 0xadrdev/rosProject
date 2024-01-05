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

#include "BallPhysics.h"
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
      // ball_position_subscription_ = this -> create_subscription<pong_ros_interfaces::msg::BallPosition>(
      // TOPIC_BALL_POSITION, 10, std::bind(&PongControllerNode::handle_ball_position_subscription, this, _1));

      left_paddle_position_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
      TOPIC_LEFT_PADDLE_POSITION, 10, std::bind(&PongControllerNode::handle_left_paddle_position_subscription, this, _1));
      
      right_paddle_position_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
      TOPIC_RIGHT_PADDLE_POSITION, 10, std::bind(&PongControllerNode::handle_right_paddle_position_subscription, this, _1));
      
      // Publishers. 
      ball_position_publisher_ = this -> create_publisher<pong_ros_interfaces::msg::BallPosition>(TOPIC_BALL_POSITION, 10);
      // ball_velocity_publisher_ = this -> create_publisher<pong_ros_interfaces::msg::BallVelocity>(TOPIC_BALL_VELOCITY, 10);
      
      // Initialize the class object used to compute the logic
      pong_controller_ = PongController();

      ball_physics_ = BallPhysics();

      // Initialize a timer for the iteration speed of the game.
      timer_ = this -> create_wall_timer(std::chrono::milliseconds(1000 / 60), std::bind(&PongControllerNode::timer_callback, this));
    }
    
    private:

      // Subscriptions handlers. 
      void handle_ball_position_subscription(const pong_ros_interfaces::msg::BallPosition & ballPositionMsg) {
        // Confirming data is read correctly
        // RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");
        pong_controller_.setBallPosition(ballPositionMsg.x, ballPositionMsg.y);
      }
      
      void handle_left_paddle_position_subscription(const std_msgs::msg::Float64::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Received first paddle position: %f", msg->data);
        pong_controller_.setLeftPaddlePosition(msg -> data);
      }

      void handle_right_paddle_position_subscription(const std_msgs::msg::Float64::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Received second paddle position: %f", msg->data);
        pong_controller_.setRightPaddlePosition(msg -> data);
      }
      
      // void timer_callback() {
      //     auto ball_vel_msg = pong_ros_interfaces::msg::BallVelocity();
          
      //     // Using the logic class
      //     pong_controller_.determineCollisionType();

      //     pong_controller_.updateBallVelocity();

      //     pong_controller_.incrementVelocity();

      //     ball_vel_msg.x = pong_controller_.getBallVelocityX();
      //     ball_vel_msg.y = pong_controller_.getBallVelocityY();

      //     // Publishing the updated ball velocity. 
      //     ball_velocity_publisher_ -> publish(ball_vel_msg);

      //     // RCLCPP_INFO(this->get_logger(), "Publishing ball_velocity: (%f, %f)", ball_vel_msg.x, ball_vel_msg.y);
      // }


      void timer_callback() {

          pong_controller_.setBallPosition(ball_physics_.getBallPositionX(), ball_physics_.getBallPositionY());
          pong_controller_.setBallVelocity(ball_physics_.getBallVelocityX(), ball_physics_.getBallVelocityY());
          
          // Using the logic class
          pong_controller_.determineCollisionType();

          pong_controller_.updateBallVelocity();

          pong_controller_.incrementVelocity();

          ball_physics_.setBallPosition(pong_controller_.getBallPositionX() + pong_controller_.getBallVelocityX(), pong_controller_.getBallPositionY() +  pong_controller_.getBallVelocityY());
          ball_physics_.setBallVelocity(pong_controller_.getBallVelocityX(), pong_controller_.getBallVelocityY());

          auto ball_position_msg = pong_ros_interfaces::msg::BallPosition();

          ball_position_msg.x = ball_physics_.getBallPositionX();
          ball_position_msg.y = ball_physics_.getBallPositionY();

          // Publishing the updated ball position. 
          ball_position_publisher_ -> publish(ball_position_msg);

          // RCLCPP_INFO(this->get_logger(), "Publishing ball_velocity: (%f, %f)", ball_position_msg.x, ball_position_msg.y);
      }

      // ROS 2 declarations. 
      rclcpp::TimerBase::SharedPtr timer_;
      // rclcpp::Subscription<pong_ros_interfaces::msg::BallPosition>::SharedPtr ball_position_subscription_;
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_paddle_position_subscription_;
      rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_paddle_position_subscription_;
      rclcpp::Publisher<pong_ros_interfaces::msg::BallPosition>::SharedPtr ball_position_publisher_;
      // rclcpp::Publisher<pong_ros_interfaces::msg::BallVelocity>::SharedPtr ball_velocity_publisher_;
      PongController pong_controller_;
      BallPhysics ball_physics_;
};

// Initializing the node. 
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PongControllerNode>());
    rclcpp::shutdown();

    return 0;
}


