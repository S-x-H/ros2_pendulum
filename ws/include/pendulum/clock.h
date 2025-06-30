#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

class Clock : public rclcpp::Node {
public:
  Clock();

private:
  void timerCallback();
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr clockPub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::TimeReference tick_;

  double duration_;
};