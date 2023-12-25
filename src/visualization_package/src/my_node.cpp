#include <cstdio>
#include <iostream>
#include <SDL2/SDL.h>
#include <memory>

#include "../include/SDL2_UI.h"
#include "../include/Pong_field.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tutorial_interfaces/msg/ball.hpp" 
#include "tutorial_interfaces/msg/score.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("visualization_node")
    {
      ballPosSub_ = this->create_subscription<tutorial_interfaces::msg::Ball>(
      "ball_position", 10, std::bind(&MinimalSubscriber::ball_callback, this, _1));

      firstPaddleSub_ = this->create_subscription<std_msgs::msg::Float64>(
      "first_paddle_position", 10, std::bind(&MinimalSubscriber::first_paddle_callback, this, _1));

      secondPaddleSub_ = this->create_subscription<std_msgs::msg::Float64>(
      "second_paddle_position", 10, std::bind(&MinimalSubscriber::second_paddle_callback, this, _1));

      scoreSub_ = this->create_subscription<tutorial_interfaces::msg::Score>(
      "score", 10, std::bind(&MinimalSubscriber::score_callback, this, _1));
      
      // Initialize the Pong_field object
      field_ = std::make_shared<Pong_field>();
      field_->DrawField();
      
    }

  private:
  
    void ball_callback(const tutorial_interfaces::msg::Ball & msg) const {
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");

      field_->setXYBall(msg.x, msg.y);
    }
    
    void first_paddle_callback(const std_msgs::msg::Float64::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Received first paddle position: %f", msg->data);

      field_->setYBatLeft(msg->data);
    }

    void second_paddle_callback(const std_msgs::msg::Float64::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Received second paddle position: %f", msg->data);

      field_->setYBatRight(msg->data);
    }

    void score_callback(const tutorial_interfaces::msg::Score & msg) const {
      RCLCPP_INFO(this->get_logger(), "Received score: %d - %d", msg.first, msg.second);

      field_ -> setFieldText(std::to_string(msg.first) + std::string(" - ") + std::to_string(msg.second));
      field_ -> DrawField();
    }
    
    rclcpp::Subscription<tutorial_interfaces::msg::Ball>::SharedPtr ballPosSub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr firstPaddleSub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr secondPaddleSub_;
    rclcpp::Subscription<tutorial_interfaces::msg::Score>::SharedPtr scoreSub_;
    
    std::shared_ptr<Pong_field> field_;
};


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  
  return 0;
}
