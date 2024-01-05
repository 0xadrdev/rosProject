//==============================================================
// Filename : BallPhysicsNode.cpp
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
#include "pong_ros_interfaces/msg/ball_velocity.hpp"
#include "../../pong_ros_core/include/Constants.h"
#include "BallPhysics.h"

using std::placeholders::_1;
using namespace pong_ros_constants;


class BallPhysicsNode: public rclcpp::Node {
  public: 
    BallPhysicsNode()
    : Node("ball_physics_node") {

      // Subscriptions
      ball_velocity_subscription_ = this -> create_subscription<pong_ros_interfaces::msg::BallVelocity>(
      TOPIC_BALL_VELOCITY, 10, std::bind( & BallPhysicsNode::handle_ball_velocity_subscription, this, _1));

      // Publishers. 
      ball_position_publisher_ = this -> create_publisher<pong_ros_interfaces::msg::BallPosition>(TOPIC_BALL_POSITION, 10);

      // Create instance of the BallPhysics class. 
      ball_physics_ = BallPhysics();
    }

  private:
    // Subscriptions handlers. 
    void handle_ball_velocity_subscription(const pong_ros_interfaces::msg::BallVelocity & ballVelocityMsg) {
      // Confirming data is read
      RCLCPP_INFO_STREAM(this -> get_logger(), "I heard x: '" << ballVelocityMsg.x << "' y: '" << ballVelocityMsg.y << "'");

      ball_physics_.setBallPosition(ballVelocityMsg.x + ball_physics_.getBallPositionX(), ballVelocityMsg.y + ball_physics_.getBallPositionY());

      // Setting up the new ball position message. 
      double ball_position_x = ball_physics_.getBallPositionX();
      double ball_position_y = ball_physics_.getBallPositionY();
      
      auto new_ball_position_msg = pong_ros_interfaces::msg::BallPosition();
      new_ball_position_msg.x = ball_position_x;
      new_ball_position_msg.y = ball_position_y;

      // Publishing the new ball position message. 
      ball_position_publisher_ -> publish(new_ball_position_msg);

      RCLCPP_INFO(this -> get_logger(), "Publishing ball_position: (%f, %f)", new_ball_position_msg.x, new_ball_position_msg.y);
    }

    // ROS2 declarations. 
    rclcpp::Subscription < pong_ros_interfaces::msg::BallVelocity > ::SharedPtr ball_velocity_subscription_;
    rclcpp::Publisher < pong_ros_interfaces::msg::BallPosition > ::SharedPtr ball_position_publisher_;

    BallPhysics ball_physics_;
};

// Initializing the node. 
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared < BallPhysicsNode > ());
  rclcpp::shutdown();

  return 0;
}