#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <turtlefollow/msg/mode.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ModeSwitch : public rclcpp::Node
{
  public:
    ModeSwitch()
    : Node("modeswitch"), count_(0)
    {
      publisher_ = this->create_publisher<turtlefollow::msg::Mode>("turtle2/mode", 10);
      timer_ = this->create_wall_timer(
      5000ms, std::bind(&ModeSwitch::timer_callback, this));
      mode = "RUN";
    }

  private:
    void timer_callback()
    {
      mode = (mode=="FOLLOW") ? "RUN" : "FOLLOW";
      turtlefollow::msg::Mode msg;
      msg.mode = mode;
      publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<turtlefollow::msg::Mode>::SharedPtr publisher_;
    size_t count_;
    std::string mode;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModeSwitch>());
  rclcpp::shutdown();
  return 0;
}