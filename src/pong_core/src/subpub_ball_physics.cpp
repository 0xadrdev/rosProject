#include <chrono>
#include <functional>
#include <string>
#include <cstdio>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tutorial_interfaces/msg/ball.hpp" 
#include "tutorial_interfaces/msg/vel.hpp" 

#include "ball_physics.h"

using std::placeholders::_1;

class SubpubBallPhysics : public rclcpp::Node
{
  public:
    SubpubBallPhysics()
    : Node("subpub_ball_physics")
    {
      // Subscriptions
      ballVelSub_ = this->create_subscription<tutorial_interfaces::msg::Vel>(
      "ball_velocity", 10,std::bind(&SubpubBallPhysics::vel_callback, this, _1));
      
      // Publishing
      ball_pos_publisher_ = this->create_publisher<tutorial_interfaces::msg::Ball>("ball_position", 10);
      
      // Initialize the class object used to compute the ball physics
      pongPhysics_ = ball_physics(); 
      //ball_physics pongPhysics_;
      
    }

  private:
  
    void vel_callback(const tutorial_interfaces::msg::Vel & msg)
    {
      // Confirming data is read
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");
      
      //ball_physics pongPhysics_;
      
      pongPhysics_.updatePosition(msg.x, msg.y);
      
      // Setting up the message. 
      auto ball_pos_msg = tutorial_interfaces::msg::Ball();
      
      double ball_pos_x = pongPhysics_.getBallPosX(); // Use the class object to compute physics
      double ball_pos_y = pongPhysics_.getBallPosY(); // Use the class object to compute physics
      
      ball_pos_msg.x = ball_pos_x;
      ball_pos_msg.y = ball_pos_y;
      
      // Publish the message
      ball_pos_publisher_->publish(ball_pos_msg);
      
      RCLCPP_INFO(this->get_logger(), "Publishing ball_position: (%f, %f)", ball_pos_msg.x, ball_pos_msg.y);
      
    }
    
    rclcpp::Subscription<tutorial_interfaces::msg::Vel>::SharedPtr ballVelSub_;
    rclcpp::Publisher<tutorial_interfaces::msg::Ball>::SharedPtr ball_pos_publisher_;
    ball_physics pongPhysics_;
};


//int main(int argc, char ** argv) {
int main(int argc, char* argv[]) {

  
  //ball_physics pongPhysics_;
  
  // Initializing and running the subscribed and publishing node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubpubBallPhysics>());
  rclcpp::shutdown();
  
  return 0;
}

