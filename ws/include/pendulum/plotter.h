#include <fstream>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/time_synchronizer.hpp>

class Plotter : public rclcpp::Node
{
public:
  Plotter();
  ~Plotter();

private:
  void poseVelCb(const sensor_msgs::msg::Temperature::ConstSharedPtr &posMsg,
                 const sensor_msgs::msg::Temperature::ConstSharedPtr &velMsg);
  void poseVelTorqueCb(
      const sensor_msgs::msg::Temperature::ConstSharedPtr &torqueMsg,
      const sensor_msgs::msg::Temperature::ConstSharedPtr &posMsg,
      const sensor_msgs::msg::Temperature::ConstSharedPtr &velMsg);

  using PoseVelPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Temperature,
                                                                  sensor_msgs::msg::Temperature>;
  using PoseVelSync = message_filters::Synchronizer<PoseVelPolicy>;

  using PoseVelTorquePolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Temperature,
                                                                        sensor_msgs::msg::Temperature,
                                                                        sensor_msgs::msg::Temperature>;
  using PoseVelTorqueSync = message_filters::Synchronizer<PoseVelTorquePolicy>;

  // bit of a weird hack as this is not a temperature, but the temperature type
  // is effective a header and a double, which is what we need here
  message_filters::Subscriber<sensor_msgs::msg::Temperature> torqueSubFilt_;
  message_filters::Subscriber<sensor_msgs::msg::Temperature> angularPosSubFilt_;
  message_filters::Subscriber<sensor_msgs::msg::Temperature> angularVelSubFilt_;

  std::shared_ptr<message_filters::Synchronizer<PoseVelPolicy>> poseVelFilter_;
  std::shared_ptr<message_filters::Synchronizer<PoseVelTorquePolicy>> poseVelTorqueFilter_;

  std::ofstream fileStream_;

  std::optional<double> torque_;
  bool plotDegrees_;
};