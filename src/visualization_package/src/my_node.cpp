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
    //: Node("minimal_subscriber")
    {
      //subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      //"topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      //ballPosSub_ = this->create_subscription<std_msgs::msg::Float64>(
      ballPosSub_ = this->create_subscription<tutorial_interfaces::msg::Ball>(
      "ball_position", 10, std::bind(&MinimalSubscriber::ball_callback, this, _1));
      firstPaddleSub_ = this->create_subscription<std_msgs::msg::Float64>(
      "first_paddle_position", 10, std::bind(&MinimalSubscriber::first_paddle_callback, this, _1));
      secondPaddleSub_ = this->create_subscription<std_msgs::msg::Float64>(
      "second_paddle_position", 10, std::bind(&MinimalSubscriber::second_paddle_callback, this, _1));
      //scoreSub_ = this->create_subscription<std_msgs::msg::Int32>(
      scoreSub_ = this->create_subscription<tutorial_interfaces::msg::Score>(
      "score", 10, std::bind(&MinimalSubscriber::score_callback, this, _1));
      
      // Initialize the Pong_field object
      field_ = std::make_shared<Pong_field>();
      field_->DrawField();
      
    }

  private:
    //void ball_callback(const std_msgs::msg::Float64::SharedPtr msg) const
    void ball_callback(const tutorial_interfaces::msg::Ball & msg) const {
      //RCLCPP_INFO(this->get_logger(), "Received ball position: %f", msg->data);
      RCLCPP_INFO_STREAM(this->get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");
      //field_->setXYBall(msg->data, msg->data);
      field_->setXYBall(msg.x, msg.y);
    }
    
    void first_paddle_callback(const std_msgs::msg::Float64::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Received first paddle position: %f", msg->data);
      field_->setYBatLeft(msg->data);
      //field_->setYBatRight(msg->data); // Only for testing purposes. [REMOVE]
    }

    void second_paddle_callback(const std_msgs::msg::Float64::SharedPtr msg) const {
      RCLCPP_INFO(this->get_logger(), "Received second paddle position: %f", msg->data);
      field_->setYBatRight(msg->data);
    }

    //void score_callback(const std_msgs::msg::Int32::SharedPtr msg) const
    void score_callback(const tutorial_interfaces::msg::Score & msg) const {
      //RCLCPP_INFO(this->get_logger(), "Received score: %d", msg->data);
      //field_->setFieldText(std::to_string(msg->data) + std::string(" - 0"));
      RCLCPP_INFO(this->get_logger(), "Received score: %d - %d", msg.first, msg.second);
      field_->setFieldText(std::to_string(msg.first) + std::string(" - ") + std::to_string(msg.second));
      field_->DrawField();
    }
    
    //rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ballPosSub_;
    rclcpp::Subscription<tutorial_interfaces::msg::Ball>::SharedPtr ballPosSub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr firstPaddleSub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr secondPaddleSub_;
    rclcpp::Subscription<tutorial_interfaces::msg::Score>::SharedPtr scoreSub_;
    //rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    
    std::shared_ptr<Pong_field> field_;
};


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  
  return 0;
}
