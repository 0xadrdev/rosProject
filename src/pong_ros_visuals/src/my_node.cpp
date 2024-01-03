//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include <cstdio>
#include <iostream>
#include <SDL2/SDL.h>
#include <memory>

#include "../include/SDL2_UI.h"
#include "../include/Pong_field.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "pong_ros_interfaces/msg/ball_position.hpp" 
#include "pong_ros_interfaces/msg/score.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber() : Node("visualization_node") {
      ball_position_subscription_ = this -> create_subscription<pong_ros_interfaces::msg::BallPosition>(
      "ball_position", 10, std::bind(&MinimalSubscriber::handle_ball_position_subscription, this, _1));

      left_paddle_position_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
      "first_paddle_position", 10, std::bind(&MinimalSubscriber::handle_left_paddle_subscription, this, _1));

      right_paddle_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
      "second_paddle_position", 10, std::bind(&MinimalSubscriber::handle_right_paddle_subscription, this, _1));

      score_players_subscription = this -> create_subscription<pong_ros_interfaces::msg::Score>(
      "score", 10, std::bind(&MinimalSubscriber::handle_players_score_subscription, this, _1));
      
      // Initialize the Pong_field object
      pong_field_ = std::make_shared<Pong_field>();
      pong_field_ -> render();
    }

  private:
  
    void handle_ball_position_subscription(const pong_ros_interfaces::msg::BallPosition & msg) const {
      // RCLCPP_INFO_STREAM(this -> get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");

      pong_field_ -> setBallPosition(msg.x, msg.y);
    }
    
    void handle_left_paddle_subscription(const std_msgs::msg::Float64::SharedPtr msg) const {
      // RCLCPP_INFO(this -> get_logger(), "Received first paddle position: %f", msg -> data);

      pong_field_ -> setLeftPaddlePosition(msg -> data);
    }

    void handle_right_paddle_subscription(const std_msgs::msg::Float64::SharedPtr msg) const {
      // RCLCPP_INFO(this -> get_logger(), "Received second paddle position: %f", msg -> data);

      pong_field_ -> setRightPaddlePosition(msg -> data);
    }

    void handle_players_score_subscription(const pong_ros_interfaces::msg::Score & msg) const {
      // RCLCPP_INFO(this -> get_logger(), "Received score: %d - %d", msg.first, msg.second);

      // pong_field_ -> setFieldText(std::to_string(msg.first) + std::to_string(msg.second));
      pong_field_ -> setFieldText(std::to_string(msg.first) + std::string("  ") + std::to_string(msg.second));
      pong_field_ -> render();
    }
    
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_paddle_position_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_paddle_subscription_;

    rclcpp::Subscription<pong_ros_interfaces::msg::BallPosition>::SharedPtr ball_position_subscription_;
    rclcpp::Subscription<pong_ros_interfaces::msg::Score>::SharedPtr  score_players_subscription;
    
    std::shared_ptr<Pong_field> pong_field_ ;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  
  return 0;
}
