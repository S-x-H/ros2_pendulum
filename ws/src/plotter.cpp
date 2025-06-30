#include "pendulum/plotter.h"
#include "pendulum/utils.h"
#include <rclcpp/qos.hpp>

Plotter::Plotter() : Node("plotter")
{
  rclcpp::QoS qos(rclcpp::KeepLast(10));

  angularPosSubFilt_.subscribe(this, "pendulum_angular_position", qos);
  angularVelSubFilt_.subscribe(this, "pendulum_angular_velocity", qos);

  this->declare_parameter<bool>("plot_degrees", false);
  this->get_parameter<bool>("plot_degrees", plotDegrees_);

  bool useConstTorque;
  this->declare_parameter<bool>("use_const_torque", true);
  this->get_parameter<bool>("use_const_torque", useConstTorque);

  if (useConstTorque)
  {
    // if a constant torque is provided, sync pose and velocity only
    double torque;
    this->declare_parameter<double>("torque", 0.0);
    this->get_parameter<double>("torque", torque);
    torque_ = torque;

    poseVelFilter_.reset(new PoseVelSync(PoseVelPolicy(10), angularPosSubFilt_, angularVelSubFilt_));
    poseVelFilter_->registerCallback(std::bind(&Plotter::poseVelCb, this, std::placeholders::_1, std::placeholders::_2));
  }
  else
  {
    // if torque is taken from a topic, sync and subscribe to torque too
    torqueSubFilt_.subscribe(this, "pendulum_torque", qos);

    poseVelTorqueFilter_.reset(
        new PoseVelTorqueSync(PoseVelTorquePolicy(10), torqueSubFilt_, angularPosSubFilt_, angularVelSubFilt_));
    poseVelTorqueFilter_->registerCallback(
        std::bind(&Plotter::poseVelTorqueCb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  std::string logName;
  this->declare_parameter<std::string>("log_name", "pendulum_log");
  this->get_parameter<std::string>("log_name", logName);
  if (plotDegrees_)
  {
    logName = logName + "_degrees.csv";
  }
  else
  {
    logName = logName + "_radians.csv";
  }

  try
  {
    fileStream_.open(logName.c_str());
    if (fileStream_.is_open())
    {
      RCLCPP_INFO(this->get_logger(), "Writing to file: %s", logName.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to create file: %s", logName.c_str());
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Exception thrown creating file: %s. Exception was: %s", logName.c_str(), e.what());
  }

  // write column headings
  fileStream_ << "Time (s)"
              << ","
              << "Torque (Nm)"
              << ",";
  if (plotDegrees_)
  {
    fileStream_ << "Position (°)"
                << ","
                << "Velocity (°/s)"
                << "\n";
  }
  else
  {
    fileStream_ << "Position (rad)"
                << ","
                << "Velocity (rad/s)"
                << "\n";
  }
}

Plotter::~Plotter() { fileStream_.close(); }

void Plotter::poseVelCb(
    const sensor_msgs::msg::Temperature::ConstSharedPtr &posMsg,
    const sensor_msgs::msg::Temperature::ConstSharedPtr &velMsg)
{
  if (!torque_)
  {
    RCLCPP_ERROR(this->get_logger(), "Config error no source of torque value.");
    return;
  }

  fileStream_ << posMsg->header.stamp.sec << ".";
  fileStream_ << std::setfill('0') << std::setw(9) << posMsg->header.stamp.nanosec;
  fileStream_ << "," << torque_.value() << ",";

  if (plotDegrees_)
  {
    fileStream_ << radToDeg(posMsg->temperature) << ","
                << radToDeg(velMsg->temperature) << "\n";
  }
  else
  {
    fileStream_ << posMsg->temperature << "," << velMsg->temperature << "\n";
  }
}

void Plotter::poseVelTorqueCb(
    const sensor_msgs::msg::Temperature::ConstSharedPtr &torqueMsg,
    const sensor_msgs::msg::Temperature::ConstSharedPtr &posMsg,
    const sensor_msgs::msg::Temperature::ConstSharedPtr &velMsg)
{
  torque_ = torqueMsg->temperature;
  poseVelCb(posMsg, velMsg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Plotter>());
  rclcpp::shutdown();
  return 0;
}