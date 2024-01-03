//==============================================================
// Filename :
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
#include "pong_ros_interfaces/msg/vel.hpp" 
#include "logic.h"

using std::placeholders::_1;

class SubpubLogic : public rclcpp::Node
{
  public:
    SubpubLogic()
    : Node("subpub_logic") {
      // Subscriptions
      ball_position_subscription_ = this->create_subscription<pong_ros_interfaces::msg::BallPosition>(
      "ball_position", 10, std::bind(&SubpubLogic::handle_ball_position_subscription, this, _1));

      left_paddle_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "first_paddle_position", 10, std::bind(&SubpubLogic::handle_left_paddle_subscription, this, _1));
      
      right_paddle_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "second_paddle_position", 10, std::bind(&SubpubLogic::handle_right_paddle_subscription, this, _1));
      
      // Publishing
      ball_velocity_publisher_ = this->create_publisher<pong_ros_interfaces::msg::Vel>("ball_velocity", 10);
      
      // Initialize the class object used to compute the logic
      pongLogic_ = logic();
      
      // Initialize a timer for the iteration speed of the game.
      timer_ = this -> create_wall_timer(std::chrono::milliseconds(1000 / 60), std::bind(&SubpubLogic::timer_callback, this));
    }
    
    private:
    void handle_ball_position_subscription(const pong_ros_interfaces::msg::BallPosition & msg) {
    	// Confirming data is read correctly
	    // RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");
    	
    	// Update the ball position using the message from the subscription
        pongLogic_.setBallPositionX(msg.x);
        pongLogic_.setBallPositionY(msg.y);
    }
    
    void handle_left_paddle_subscription(const std_msgs::msg::Float64::SharedPtr msg)
    {
      // RCLCPP_INFO(this->get_logger(), "Received first paddle position: %f", msg->data);
      pongLogic_.setPadPosLeft(msg->data);
    }

    void handle_right_paddle_subscription(const std_msgs::msg::Float64::SharedPtr msg)
    {
      // RCLCPP_INFO(this->get_logger(), "Received second paddle position: %f", msg->data);
      pongLogic_.setPadPosRight(msg->data);
    }
    
    void timer_callback() {
        auto ball_vel_msg = pong_ros_interfaces::msg::Vel();
        
        // Using the logic class
        pongLogic_.checkCollision();
        pongLogic_.updateBallVelocity();
        ball_vel_msg.x = pongLogic_.getBallVelocityX();
        ball_vel_msg.y = pongLogic_.getBallVelocityY();

	      // Publishing the updated ball velocity. 
        ball_velocity_publisher_ -> publish(ball_vel_msg);

        // RCLCPP_INFO(this->get_logger(), "Publishing ball_velocity: (%f, %f)", ball_vel_msg.x, ball_vel_msg.y);
        
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<pong_ros_interfaces::msg::BallPosition>::SharedPtr ball_position_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_paddle_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_paddle_subscription_;

    rclcpp::Publisher<pong_ros_interfaces::msg::Vel>::SharedPtr ball_velocity_publisher_;

    logic pongLogic_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubpubLogic>());
    rclcpp::shutdown();

    return 0;
}


