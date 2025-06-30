#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/time_synchronizer.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

class Pendulum : public rclcpp::Node
{
public:
  Pendulum();

private:
  void clockCb(const sensor_msgs::msg::TimeReference::ConstSharedPtr timeMsg);
  void
  clockTorqueCb(const sensor_msgs::msg::TimeReference::ConstSharedPtr &timeMsg,
                const sensor_msgs::msg::Temperature::ConstSharedPtr &torqueMsg);

  void updateState(double dt);

  rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr clockSub_;

  // bit of a weird hack as this is not a temperature, but the temperature type
  // is effective a header and a double, which is what we need here
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr angularPosPub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr angularVelPub_;

  using ClockTorquePolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::TimeReference, sensor_msgs::msg::Temperature>;
  using ClockTorqueSync = message_filters::Synchronizer<ClockTorquePolicy>;
  message_filters::Subscriber<sensor_msgs::msg::TimeReference> clockSubFilt_;
  message_filters::Subscriber<sensor_msgs::msg::Temperature> torqueSubFilt_;
  std::shared_ptr<message_filters::Synchronizer<ClockTorquePolicy>> clockTorqueFilter_;

  std::optional<builtin_interfaces::msg::Time> lastTime_;

  double pos_;
  double vel_;
  std::optional<double> torque_;

  double mass_;
  double length_;
  double gravity_;

  unsigned int publishIncrement_;
  unsigned int publishCount_;
  double duration_;
};