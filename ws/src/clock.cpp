#include "pendulum/clock.h"

Clock::Clock() : Node("clock")
{
  // rate at which to tick the clock, in milliseconds
  // this tick will be used to synchronise messages between nodes
  int updateStep;
  this->declare_parameter<int>("update_step", 1);
  this->get_parameter<int>("update_step", updateStep);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(updateStep), std::bind(&Clock::timerCallback, this));

  rclcpp::QoS qos(rclcpp::KeepLast(10));
  clockPub_ = this->create_publisher<sensor_msgs::msg::TimeReference>("sim_clock", qos.reliable());

  // save time reference as system start time of simulation
  tick_.time_ref = this->now();
}

void Clock::timerCallback()
{
  // generate sim time, update tick message timestamp and publish
  rclcpp::Duration simTime = this->now() - tick_.time_ref;
  int seconds = int(floor(float(simTime.nanoseconds()) / 1000000000));
  int nanoseconds = simTime.nanoseconds() - seconds * 1000000000;
  tick_.header.stamp.sec = seconds;
  tick_.header.stamp.nanosec = nanoseconds;
  clockPub_->publish(tick_);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Clock>());
  rclcpp::shutdown();
  return 0;
}