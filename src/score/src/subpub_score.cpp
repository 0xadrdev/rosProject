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
#include "tutorial_interfaces/msg/ball.hpp" 
#include "tutorial_interfaces/msg/score.hpp" 

#include "score.h"

using std::placeholders::_1;

class SubpubScore : public rclcpp::Node
{
  public:
    SubpubScore()
    : Node("subpub_score")
    {
      // Subscriptions
      ballPosSub_ = this->create_subscription<tutorial_interfaces::msg::Ball>(
      "ball_position", 10,std::bind(&SubpubScore::pos_callback, this, _1));
      
      // Publishing
      score_publisher_ = this->create_publisher<tutorial_interfaces::msg::Score>("score", 10);
      
      // Initialize the class object used to compute the ball physics
      score_ = score(); 
      /////////////////////Object initialization///////////////////////////////
      
    }

  private:
  
    void pos_callback(const tutorial_interfaces::msg::Ball & msg)
    {
      // Confirming data is read
      // RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");
      
      // Computing score updates using object
      score_.setBallPosX(msg.x);
      score_.setBallPosY(msg.y);
      score_.updateScore();
      /////////////////////Object computation///////////////////////////////
      
      // Setting up the message. 
      auto score_msg = tutorial_interfaces::msg::Score();
      
      int score_first = score_.getScoreFirst(); // Use the class object to compute physics
      int score_second = score_.getScoreSecond(); // Use the class object to compute physics
      
      score_msg.first = score_first;
      score_msg.second = score_second;
      
      // Publish the message
      score_publisher_->publish(score_msg);
      
      // RCLCPP_INFO(this->get_logger(), "Publishing score: (%d - %d)", score_msg.first, score_msg.second);
      
    }
    
    rclcpp::Subscription<tutorial_interfaces::msg::Ball>::SharedPtr ballPosSub_;
    rclcpp::Publisher<tutorial_interfaces::msg::Score>::SharedPtr score_publisher_;
    // Initialize the class object used to compute the ball physics
    score score_;
    /////////////////////Object initialization///////////////////////////////
};


int main(int argc, char* argv[]) {
  
  // Initializing and running the subscribed and publishing node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubpubScore>());
  rclcpp::shutdown();
  
  return 0;
}

