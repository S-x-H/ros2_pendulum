#include "pendulum/torque.h"
#include "pendulum/utils.h"

Torque::Torque() : Node("torque") {
  rclcpp::QoS qos(rclcpp::KeepLast(10));

  clockSub_ = this->create_subscription<sensor_msgs::msg::TimeReference>(
      "sim_clock", qos.reliable(),
      std::bind(&Torque::clockCb, this, std::placeholders::_1));

  torquePub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
      "pendulum_torque", qos.reliable());

  // precision of time modulo is given as update step / 2, in seconds
  int updateStep;
  this->declare_parameter<int>("update_step", 1);
  this->get_parameter<int>("update_step", updateStep);
  modPrecision_ = double(updateStep) / 1000 / 2;
}

void Torque::clockCb(sensor_msgs::msg::TimeReference::ConstSharedPtr timeMsg) {
  sensor_msgs::msg::Temperature msg;
  msg.header.stamp = timeMsg->header.stamp;

  double t = timeToFloat(timeMsg->header.stamp);
  if (approximateModulo(t)) {
    msg.temperature = 1;
  } else if (approximateModulo(t - 0.5)) {
    msg.temperature = -1;
  } else {
    msg.temperature = 0;
  }

  torquePub_->publish(msg);
}

bool Torque::approximateModulo(double t) {
  // an approximate modulo is used because time is given to 9 d.p., and an exact
  // modulo would be unlikely to ever equal 0
  return abs(t - round(t)) < modPrecision_;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Torque>());
  rclcpp::shutdown();
  return 0;
}