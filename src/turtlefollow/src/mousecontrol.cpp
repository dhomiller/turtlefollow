#include <chrono>
#include <functional>
#include <string>

#include <xcb/xcb.h>
#include <xcb/xproto.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MouseControl : public rclcpp::Node
{
  public:
    MouseControl()
    : Node("mousecontrol"), count_(0)
    {
      subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10, std::bind(&MouseControl::subscription_callback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
      timer_ = this->create_wall_timer(
      50ms, std::bind(&MouseControl::timer_callback, this));
      
      // establish connection to X server and find screen
      connection = xcb_connect(NULL, NULL);
      screen = xcb_setup_roots_iterator (xcb_get_setup (connection)).data;
      //xcb_flush (connection); //not necessary - events not used.
    }

  private:
    void timer_callback()
    {
      
      xcb_query_pointer_cookie_t cookie = xcb_query_pointer(connection, screen->root);
      xcb_query_pointer_reply_t *reply = xcb_query_pointer_reply(connection, cookie, NULL); 

      //calculate direction. There may be some domain errors here.
      float x_dist = (reply->win_x-350)/40.0;
      float y_dist = (350-reply->win_y)/40.0;
      float theta = atan2(y_dist, x_dist); //angle [-pi, pi] of pointer
      float dist = sqrt(x_dist*x_dist + y_dist*y_dist);
      float angular_vel = tan((theta-angle)/2)*5; //turn toward angle of pointer
      float linear_vel = dist-0.4 < 0 ? 0 : dist-0.4;

      geometry_msgs::msg::Twist msg;
      msg.angular.z = angular_vel;
      msg.linear.x = linear_vel;
      publisher_->publish(msg);
    }
    void subscription_callback(const turtlesim::msg::Pose msg)
    {
      //update user angle
      angle = msg.theta;
    }

    
    float angle; // angle of the user's turtle
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    size_t count_;
    xcb_connection_t *connection;
    xcb_screen_t *screen;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MouseControl>());
  rclcpp::shutdown();
  return 0;
}