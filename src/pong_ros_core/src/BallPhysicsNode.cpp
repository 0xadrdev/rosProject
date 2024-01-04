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
#include "pong_ros_interfaces/msg/ball_velocity.hpp"

#include "BallPhysics.h"

using std::placeholders::_1;

class BallPhysicsNode: public rclcpp::Node {
  public: BallPhysicsNode(): Node("ball_physics_node") {

    // Subscriptions
    ball_velocity_subscription_ = this -> create_subscription <pong_ros_interfaces::msg::BallVelocity> (
      "ball_velocity", 10, std::bind( & BallPhysicsNode::handle_ball_velocity_subscription, this, _1));

    // Publishers. 
    ball_position_publisher_ = this -> create_publisher <pong_ros_interfaces::msg::BallPosition> ("ball_position", 10);

    // Initialize the class object used to compute the ball physics
    ball_physics_ = BallPhysics();
  }

  private:

    // Subscriptions handlers. 

    void handle_ball_velocity_subscription(const pong_ros_interfaces::msg::BallVelocity & msg) {
      // Confirming data is read
      // RCLCPP_INFO_STREAM(this -> get_logger(), "I heard x: '" << msg.x << "' y: '" << msg.y << "'");

      ball_physics_.updateBallPosition(msg.x, msg.y);

      // Setting up the new ball position message. 

      double ball_position_x = ball_physics_.getBallPositionX(); // Use the class object to compute physics
      double ball_position_y = ball_physics_.getBallPositionY(); // Use the class object to compute physics

      auto new_ball_position_msg = pong_ros_interfaces::msg::BallPosition();
      
      new_ball_position_msg.x = ball_position_x;
      new_ball_position_msg.y = ball_position_y;

      // Publishing the new ball position message. 
      ball_position_publisher_ -> publish(new_ball_position_msg);

      // RCLCPP_INFO(this -> get_logger(), "Publishing ball_position: (%f, %f)", new_ball_position_msg.x, new_ball_position_msg.y);

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