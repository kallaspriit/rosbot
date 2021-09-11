#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"

using std::placeholders::_1;

class Listener : public rclcpp::Node
{
public:
  Listener()
  : Node("listener")
  {
    subscription =
      this->create_subscription<tutorial_interfaces::msg::Num>(
      "num_topic", 10,
      std::bind(&Listener::topic_callback, this, _1));
  }

private:
  void topic_callback(const tutorial_interfaces::msg::Num::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->num);
  }

  rclcpp::Subscription<tutorial_interfaces::msg::Num>::SharedPtr subscription;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();

  return 0;
}
