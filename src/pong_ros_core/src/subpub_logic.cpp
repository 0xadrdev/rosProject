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
#include "pong_ros_interfaces/msg/ball.hpp" 
#include "pong_ros_interfaces/msg/vel.hpp" 
#include "logic.h"

using std::placeholders::_1;

class SubpubLogic : public rclcpp::Node
{
  public:
    SubpubLogic()
    : Node("subpub_logic")
    {
      // Subscriptions
      ballPosSub_ = this->create_subscription<pong_ros_interfaces::msg::Ball>(
      "ball_position", 10, std::bind(&SubpubLogic::ball_callback, this, _1));
      firstPaddleSub_ = this->create_subscription<std_msgs::msg::Float64>(
      "first_paddle_position", 10, std::bind(&SubpubLogic::first_paddle_callback, this, _1));
      secondPaddleSub_ = this->create_subscription<std_msgs::msg::Float64>(
      "second_paddle_position", 10, std::bind(&SubpubLogic::second_paddle_callback, this, _1));
      
      // Publishing
      ball_vel_publisher_ = this->create_publisher<pong_ros_interfaces::msg::Vel>("ball_velocity", 10);
      
      // Initialize the class object used to compute the logic
      pongLogic_ = logic();
      
      // Initialize a timer for the iteration speed of the game
      timer_ = this->create_wall_timer(std::chrono::milliseconds(16), std::bind(&SubpubLogic::timer_callback, this));
      
    }
    
    private:
    void ball_callback(const pong_ros_interfaces::msg::Ball & msg) {
    	// Confirming data is read correctly
	// RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");
    	
    	// Update the ball position using the message from the subscription
        pongLogic_.setBallPosX(msg.x);
        pongLogic_.setBallPosY(msg.y);
        //pongLogic_.setBallPosX(msg->x);
        //pongLogic_.setBallPosY(msg->y);
    }
    
    void first_paddle_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
      // RCLCPP_INFO(this->get_logger(), "Received first paddle position: %f", msg->data);
      pongLogic_.setPadPosLeft(msg->data);
      //pongLogic_.setPadPosRight(msg->data);	// Only done for testing (control both paddles with one input, REMOVE LATER
    }

    void second_paddle_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
      // RCLCPP_INFO(this->get_logger(), "Received second paddle position: %f", msg->data);
      pongLogic_.setPadPosRight(msg->data);
    }
    
    void timer_callback() {
        
        auto ball_vel_msg = pong_ros_interfaces::msg::Vel();
        
        // Using the logic class
        pongLogic_.checkCollision();
        pongLogic_.updateVelocity();
        ball_vel_msg.x = pongLogic_.getBallVelX();
        ball_vel_msg.y = pongLogic_.getBallVelY();

	// Publishing the updated ball velocity. 
        ball_vel_publisher_->publish(ball_vel_msg);

        // RCLCPP_INFO(this->get_logger(), "Publishing ball_velocity: (%f, %f)", ball_vel_msg.x, ball_vel_msg.y);
        
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<pong_ros_interfaces::msg::Ball>::SharedPtr ballPosSub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr firstPaddleSub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr secondPaddleSub_;
    rclcpp::Publisher<pong_ros_interfaces::msg::Vel>::SharedPtr ball_vel_publisher_;
    logic pongLogic_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SubpubLogic>());
    
    rclcpp::shutdown();

    return 0;
}


