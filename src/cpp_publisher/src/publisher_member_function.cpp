#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tutorial_interfaces/msg/ball.hpp"                                            // Using the custom interface
#include "tutorial_interfaces/msg/score.hpp" 

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
   public:
    MinimalPublisher() : Node("minimal_publisher") {
        //ball_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("ball_position", 10);
        ball_pos_publisher_ = this->create_publisher<tutorial_interfaces::msg::Ball>("ball_position", 10);
        first_paddle_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("first_paddle_position", 10);
        second_paddle_pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("second_paddle_position", 10);
        //score_publisher_ = this->create_publisher<std_msgs::msg::Float64>("score", 10);
        //score_publisher_ = this->create_publisher<std_msgs::msg::Int32>("score", 10);
        score_publisher_ = this->create_publisher<tutorial_interfaces::msg::Score>("score", 10);

        timer_ = this->create_wall_timer(200ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

   private:
    void timer_callback() {
    	
    	// Initializing incremental increase of variables for testing:
	static float ball_pos_x = 0;
        static float ball_pos_y = 0;
        static float first_paddle_pos = 0;
        static float second_paddle_pos = 0;
            
        auto ball_pos_msg = tutorial_interfaces::msg::Ball();
        //ball_pos_msg.x = rand() % 1000;
        //ball_pos_msg.y = rand() % 600;
        ball_pos_msg.x = ball_pos_x;
        ball_pos_msg.y = ball_pos_y;

        auto first_paddle_pos_msg = std_msgs::msg::Float64();
        //first_paddle_pos_msg.data = rand() % 600;
        first_paddle_pos_msg.data = first_paddle_pos;

        auto second_paddle_pos_msg = std_msgs::msg::Float64();
        //second_paddle_pos_msg.data = rand() % 600;
        second_paddle_pos_msg.data = second_paddle_pos;

        auto score_msg = tutorial_interfaces::msg::Score();
        score_msg.first = rand() % 10;
        score_msg.second = rand() % 10;

        ball_pos_publisher_->publish(ball_pos_msg);
        first_paddle_pos_publisher_->publish(first_paddle_pos_msg);
        second_paddle_pos_publisher_->publish(second_paddle_pos_msg);
        score_publisher_->publish(score_msg);

        RCLCPP_INFO(this->get_logger(), "Publishing ball_position: (%f, %f)", ball_pos_msg.x, ball_pos_msg.y);
        RCLCPP_INFO(this->get_logger(), "Publishing first_paddle_position: %f", first_paddle_pos_msg.data);
        RCLCPP_INFO(this->get_logger(), "Publishing second_paddle_position: %f", second_paddle_pos_msg.data);
        RCLCPP_INFO(this->get_logger(), "Publishing score: %d - %d", score_msg.first, score_msg.second);
        
        // Incrementally increasing the variables
        ball_pos_x += 5; // Incrementing ball_pos_x by 5 on each timer_callback
        ball_pos_y += 3; // Incrementing ball_pos_y by 3 on each timer_callback
        first_paddle_pos += 2; // Incrementing first_paddle_pos by 2 on each timer_callback
        second_paddle_pos += 1; // Incrementing second_paddle_pos by 1 on each timer_callback
	
	// Checking end conditions. 
        if (ball_pos_x > 1000) {
            ball_pos_x = 0;
        }

        if (ball_pos_y > 600) {
            ball_pos_y = 0;
        }

        if (first_paddle_pos > 600) {
            first_paddle_pos = 0;
        }

        if (second_paddle_pos > 600) {
            second_paddle_pos = 0;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<tutorial_interfaces::msg::Ball>::SharedPtr ball_pos_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr first_paddle_pos_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr second_paddle_pos_publisher_;
    rclcpp::Publisher<tutorial_interfaces::msg::Score>::SharedPtr score_publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<MinimalPublisher>());

    rclcpp::shutdown();

    return 0;
}
