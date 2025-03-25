#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
using namespace std::chrono_literals;

class SimpleFlipperRotator : public rclcpp::Node
{
public:
  SimpleFlipperRotator() : Node("simple_flipper_rotator")
  {
    // Create a publisher for the front left flipper joint command.
    publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/front_left_flipper_controller/commands", 10);

    // Create a timer to publish the command periodically.
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SimpleFlipperRotator::publishCommand, this));
  }

private:
  void publishCommand()
  {
    auto command_msg = std_msgs::msg::Float64();
    command_msg.data = 3.2;  // Desired position command for testing.
    RCLCPP_INFO(this->get_logger(), "Publishing command: %.2f", command_msg.data);
    publisher_->publish(command_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleFlipperRotator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
