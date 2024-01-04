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
#include "pong_ros_interfaces/msg/players_scores.hpp" 

#include "PlayersScores.h"

using std::placeholders::_1;

class PlayersScoresNode : public rclcpp::Node {
  public:
    PlayersScoresNode() : Node("players_scores_node") {

      // Subscriptions
      ball_position_subscription = this -> create_subscription<pong_ros_interfaces::msg::BallPosition>(
      "ball_position", 10,std::bind(&PlayersScoresNode::handle_ball_position_subscription, this, _1));
      
      // Publishers 
      score_publisher_ = this -> create_publisher<pong_ros_interfaces::msg::PlayersScores>("score", 10);
      
      // Initialize the class object used to handle the players scores. 
      players_scores_ = PlayersScores(); 
    }

  private:
  
    void handle_ball_position_subscription(const pong_ros_interfaces::msg::BallPosition & msg) {
      // Confirming data is read

      // RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");
      
      // TODO: Solo publico el mensaje cuando cambia el score, es decir cuando updateScore me devuelve True; 
      
      players_scores_.setBallPositionX(msg.x);
      players_scores_.setBallPositionY(msg.y);

      players_scores_.updatePlayersScores();
      
      // Setting up the new score. 
      auto new_players_score_msg = pong_ros_interfaces::msg::PlayersScores();
      
      int score_left_player = players_scores_.getScoreLeftPlayer();
      int score_right_player = players_scores_.getScoreRightPlayer();
      
      new_players_score_msg.left_player = score_left_player;
      new_players_score_msg.right_player = score_right_player;
      
      // Publish the message
      score_publisher_ -> publish(new_players_score_msg);
      
      // RCLCPP_INFO(this->get_logger(), "Publishing score: (%d - %d)", new_players_score_msg.left_player, new_players_score_msg.right_player);
    }

    // ROS2 declarations. 
    
    rclcpp::Subscription<pong_ros_interfaces::msg::BallPosition>::SharedPtr ball_position_subscription;
    rclcpp::Publisher<pong_ros_interfaces::msg::PlayersScores>::SharedPtr score_publisher_;

    PlayersScores players_scores_;
};

// Initializing the node. 

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlayersScoresNode>());
  rclcpp::shutdown();
  
  return 0;
}

