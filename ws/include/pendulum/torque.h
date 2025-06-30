#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

class Torque : public rclcpp::Node {
public:
  Torque();

private:
  void clockCb(sensor_msgs::msg::TimeReference::ConstSharedPtr timeMsg);
  bool approximateModulo(double t);

  // bit of a weird hack as this is not a temperature, but the temperature type
  // is effective a header and a double, which is what we need here
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr torquePub_;
  rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr clockSub_;

  double modPrecision_;
};