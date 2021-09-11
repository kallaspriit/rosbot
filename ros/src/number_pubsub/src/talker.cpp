// #include <cstdio>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world chat package\n");
//   return 0;
// }

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("talker"), count(0)
  {
    publisher = this->create_publisher<tutorial_interfaces::msg::Num>("num_topic", 10);
    timer = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = tutorial_interfaces::msg::Num();
    message.num = count++;
    publisher->publish(message);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher;
  size_t count;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();

  return 0;
}
