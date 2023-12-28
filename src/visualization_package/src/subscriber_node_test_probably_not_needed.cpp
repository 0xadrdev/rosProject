//==============================================================
// Filename :
// Authors : Jordi Perez Diago 
// Group : 
// License : Apache license 2.0
// Description :
//==============================================================

#include <memory>

#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
//      subscription_ = this->create_subscription<std_msgs::msg::String>(
      subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
//    void topic_callback(const std_msgs::msg::String & msg) const
    void topic_callback(const std_msgs::msg::Float64::SharedPtr msg) const
    {
//      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
      // RCLCPP_INFO(this->get_logger(), "I heard: '%1f'", msg->data);
    }
//    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
