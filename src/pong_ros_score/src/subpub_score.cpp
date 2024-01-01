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
#include "pong_ros_interfaces/msg/score.hpp" 

#include "score.h"

using std::placeholders::_1;

class SubpubScore : public rclcpp::Node {
  public:
    SubpubScore() : Node("subpub_score") {

      // Subscriptions
      ball_position_subscription = this -> create_subscription<pong_ros_interfaces::msg::Ball>(
      "ball_position", 10,std::bind(&SubpubScore::handle_ball_position_subscription, this, _1));
      
      // Publishers 
      score_publisher_ = this -> create_publisher<pong_ros_interfaces::msg::Score>("score", 10);
      
      // Initialize the class object used to compute the ball physics
      score_ = score(); 
    }

  private:
  
    void handle_ball_position_subscription(const pong_ros_interfaces::msg::Ball & msg) {
      // Confirming data is read

      // RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");
      
      // Computing score updates using object

      // TODO: Solo publico el mensaje cuando cambia el score, es decir cuando updateScore me devuelve True; 
      
      score_.setBallPosX(msg.x);
      score_.setBallPosY(msg.y);

      score_.updateScore();
      
      // Setting up the message. 
      auto score_message = pong_ros_interfaces::msg::Score();
      
      int score_player_left = score_.getScoreFirst(); // Use the class object to compute physics
      int score_player_right = score_.getScoreSecond(); // Use the class object to compute physics
      
      score_message.first = score_player_left;
      score_message.second = score_player_right;
      
      // Publish the message
      score_publisher_ -> publish(score_message);
      
      // RCLCPP_INFO(this->get_logger(), "Publishing score: (%d - %d)", score_message.first, score_message.second);
    }
    
    rclcpp::Subscription<pong_ros_interfaces::msg::Ball>::SharedPtr ball_position_subscription;
    rclcpp::Publisher<pong_ros_interfaces::msg::Score>::SharedPtr score_publisher_;
    score score_;
};


int main(int argc, char* argv[]) {
  // Initializing and running the subscribed and publishing node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubpubScore>());
  rclcpp::shutdown();
  
  return 0;
}

