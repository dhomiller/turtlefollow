#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlefollow/msg/mode.hpp>

using std::placeholders::_1;

class Follow : public rclcpp::Node
{
  public:
    Follow()
    : Node("follow")
    {
      subscription_user_turtle_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10, std::bind(&Follow::user_callback, this, _1));
      subscription_ai_turtle_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle2/pose", 10, std::bind(&Follow::ai_callback, this, _1));
      subscription_mode_ = this->create_subscription<turtlefollow::msg::Mode>(
      "turtle2/mode", 10, std::bind(&Follow::mode_callback, this, _1));
      publisher_ai_turtle_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 10);
      
      mode = "RUN";
    }

  private:
    void user_callback(const turtlesim::msg::Pose msg)
    {
      //update user position
      user_location = msg;
      //publish with updated position
      publish();
    }
    void ai_callback(const turtlesim::msg::Pose msg)
    {
      //update ai position
      ai_location = msg;
      //publish with updated position
      publish();
    }
    void mode_callback(const turtlefollow::msg::Mode msg)
    {
      mode = msg.mode;
      RCLCPP_INFO(this->get_logger(), "Entering mode: '%s'", mode.c_str());
      publish();
    }

    void publish() const
    {
      //calculate direction. There may be some domain errors here.
      float x_dist = user_location.x-ai_location.x;
      float y_dist = user_location.y-ai_location.y;
      float theta = atan2(y_dist, x_dist); //angle [-pi, pi] of target, centered on ai
      float dist = sqrt(x_dist*x_dist + y_dist*y_dist);
      float angular_vel;
      float linear_vel;
      
      if(mode=="FOLLOW") {
        angular_vel = tan((theta-ai_location.theta)/2)*5; //turn toward target
        linear_vel = dist-0.4 < 0 ? 0 : (dist-0.4)*cos(theta-ai_location.theta);
      }
      else if(mode=="RUN") {
        angular_vel = tan((theta-ai_location.theta-M_PI)/2)*5; //turn away from target
        linear_vel = 5-dist < 0 ? 0 : (5-dist)*-1*cos(theta-ai_location.theta);
      }
      else {
        angular_vel = 0;
        linear_vel = 0;
      }
    
      //limit angular and linear velocity
      if(fabs(angular_vel) > 5.0) {
        angular_vel = angular_vel / fabs(angular_vel) * 5.0;
      }
      if(fabs(linear_vel) > 5.0) {
        linear_vel = linear_vel / fabs(linear_vel) * 5.0;
      }
      
      
      
      
      
      //form message
      geometry_msgs::msg::Twist msg;
      msg.angular.z = angular_vel;
      msg.linear.x = linear_vel;
      //publish
      publisher_ai_turtle_->publish(msg);

    //   RCLCPP_INFO(this->get_logger(), "My angle: '%f'", ai_location.theta);
    //   RCLCPP_INFO(this->get_logger(), "Angle to target: '%f'", theta);
    //   RCLCPP_INFO(this->get_logger(), "Angular velocity: '%f'", angular_vel);
    //   auto message = std_msgs::msg::String();
    //   message.data = "Hello, world! " + std::to_string(count_++);
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //   publisher_->publish(message);
    }
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_user_turtle_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_ai_turtle_;
    rclcpp::Subscription<turtlefollow::msg::Mode>::SharedPtr subscription_mode_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ai_turtle_;


    turtlesim::msg::Pose user_location;
    turtlesim::msg::Pose ai_location;
    std::string mode;

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Follow>());
  rclcpp::shutdown();
  return 0;
}