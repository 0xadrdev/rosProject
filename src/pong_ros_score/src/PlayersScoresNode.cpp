//==============================================================
// Filename :PlayersScoresNode.cpp
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
#include "../../pong_ros_core/include/Constants.h"

using std::placeholders::_1;
using namespace pong_ros_constants;

class PlayersScoresNode : public rclcpp::Node {
  public:
    PlayersScoresNode() 
    : Node("players_scores_node") {
      // Subscriptions
      ball_position_subscription = this -> create_subscription<pong_ros_interfaces::msg::BallPosition>(
      TOPIC_BALL_POSITION, 10,std::bind(&PlayersScoresNode::handle_ball_position_subscription, this, _1));
      
      // Publishers 
      score_publisher_ = this -> create_publisher<pong_ros_interfaces::msg::PlayersScores>(TOPIC_PLAYERS_SCORES, 10);
      
      // Instance of the PlayersScores class. 
      players_scores_ = PlayersScores(); 
    }

  private:
    void handle_ball_position_subscription(const pong_ros_interfaces::msg::BallPosition & ballPositionMsg) {
      // Confirming data is read

      // RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << ballPositionMsg.x << "' y: '" << ballPositionMsg.y << "'");
      
      // TODO: Solo publico el mensaje cuando cambia el score, es decir cuando updateScore me devuelve True; 
      
      players_scores_.setBallPositionX(ballPositionMsg.x);
      players_scores_.setBallPositionY(ballPositionMsg.y);

      // Update the scores depending on the ball position received. 
      bool isScoreMade = players_scores_.updatePlayersScores();
      
      if (!isScoreMade) {
        return;
      }

      // Setting up the new score. 
      int score_left_player = players_scores_.getScoreLeftPlayer();
      int score_right_player = players_scores_.getScoreRightPlayer();
      
      auto new_players_score_msg = pong_ros_interfaces::msg::PlayersScores();
      new_players_score_msg.left_player = score_left_player;
      new_players_score_msg.right_player = score_right_player;
      
      // Publish the message
      score_publisher_ -> publish(new_players_score_msg);
      
      // RCLCPP_INFO(this->get_logger(), "Publishing score: (%d - %d)", new_players_score_msg.left_player, new_players_score_msg.right_player);
    }

    // ROS2 declarations. 
    PlayersScores players_scores_;
    
    rclcpp::Subscription<pong_ros_interfaces::msg::BallPosition>::SharedPtr ball_position_subscription;
    rclcpp::Publisher<pong_ros_interfaces::msg::PlayersScores>::SharedPtr score_publisher_;
};

// Initializing the node. 
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlayersScoresNode>());
  rclcpp::shutdown();
  
  return 0;
}

