//==============================================================
// Filename : PongVisualizationNode.cpp
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include <cstdio>
#include <iostream>
#include <SDL2/SDL.h>
#include <memory>

#include "../include/PongRosSdl.h"
#include "../include/PongVisualization.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "pong_ros_interfaces/msg/ball_position.hpp" 
#include "pong_ros_interfaces/msg/players_scores.hpp"
#include "../../pong_ros_core/include/Constants.h"

using std::placeholders::_1;
using namespace pong_ros_constants;

class PongVisualizationNode : public rclcpp::Node {
  public:
    PongVisualizationNode() 
    : Node("pong_visualization_node") {
      ball_position_subscription_ = this -> create_subscription<pong_ros_interfaces::msg::BallPosition>(
      TOPIC_BALL_POSITION, 10, std::bind(&PongVisualizationNode::handle_ball_position_subscription, this, _1));

      left_paddle_position_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
      TOPIC_LEFT_PADDLE_POSITION, 10, std::bind(&PongVisualizationNode::handle_left_paddle_subscription, this, _1));

      right_paddle_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
      TOPIC_RIGHT_PADDLE_POSITION, 10, std::bind(&PongVisualizationNode::handle_right_paddle_subscription, this, _1));

      score_players_subscription = this -> create_subscription<pong_ros_interfaces::msg::PlayersScores>(
      TOPIC_PLAYERS_SCORES, 10, std::bind(&PongVisualizationNode::handle_players_score_subscription, this, _1));
      
      // Initialize the PongVisualization object
      pong_visualization_ = std::make_shared<PongVisualization>();
      pong_visualization_ -> render();
    }

  private:
    void handle_ball_position_subscription(const pong_ros_interfaces::msg::BallPosition & ballPositionMsg) const {
      // RCLCPP_INFO_STREAM(this -> get_logger(), "I heard x: '" << ballPositionMsg.x << "' y: '" << ballPositionMsg.y << "'");

      pong_visualization_ -> setBallPosition(ballPositionMsg.x, ballPositionMsg.y);
      pong_visualization_ -> render();
    }
    
    void handle_left_paddle_subscription(const std_msgs::msg::Float64::SharedPtr leftPaddlePositionMsg) const {
      // RCLCPP_INFO(this -> get_logger(), "Received first paddle position: %f", leftPaddlePositionMsg -> data);

      pong_visualization_ -> setLeftPaddlePosition(leftPaddlePositionMsg -> data);
    }

    void handle_right_paddle_subscription(const std_msgs::msg::Float64::SharedPtr rightPaddlePosition) const {
      // RCLCPP_INFO(this -> get_logger(), "Received second paddle position: %f", rightPaddlePosition -> data);

      pong_visualization_ -> setRightPaddlePosition(rightPaddlePosition -> data);
    }

    void handle_players_score_subscription(const pong_ros_interfaces::msg::PlayersScores & playersScores) const {
      // RCLCPP_INFO(this -> get_logger(), "Received score: %d - %d", playersScores.left_player, playersScores.right_player);

      // pong_visualization_ -> setFieldText(std::to_string(playersScores.left_player) + std::to_string(playersScores.right_player));
      pong_visualization_ -> setFieldText(std::to_string(playersScores.left_player) + std::string("  ") + std::to_string(playersScores.right_player));
    }

    // ROS2 declarations. 
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_paddle_position_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_paddle_subscription_;
    rclcpp::Subscription<pong_ros_interfaces::msg::BallPosition>::SharedPtr ball_position_subscription_;
    rclcpp::Subscription<pong_ros_interfaces::msg::PlayersScores>::SharedPtr  score_players_subscription;
    std::shared_ptr<PongVisualization> pong_visualization_ ;
};

// Initialing the node. 
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PongVisualizationNode>());
  rclcpp::shutdown();
  
  return 0;
}
