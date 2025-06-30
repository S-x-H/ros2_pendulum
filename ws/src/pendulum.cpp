#include "pendulum/pendulum.h"
#include "pendulum/utils.h"
#include <math.h>

Pendulum::Pendulum() : Node("pendulum") {
  rclcpp::QoS qos(rclcpp::KeepLast(10));

  angularPosPub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
      "pendulum_angular_position", qos.reliable());

  angularVelPub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
      "pendulum_angular_velocity", qos.reliable());

  // initial values given in degrees but converted to radians for the calcs
  this->declare_parameter<double>("pos_0", 0);
  this->get_parameter<double>("pos_0", pos_);
  this->declare_parameter<double>("vel_0", 0);
  this->get_parameter<double>("vel_0", vel_);
  pos_ = degToRad(pos_);
  vel_ = degToRad(vel_);

  bool useConstTorque;
  this->declare_parameter<bool>("use_const_torque", true);
  this->get_parameter<bool>("use_const_torque", useConstTorque);

  if (useConstTorque) {
    // if a constant torque is provided, subscribe to clock only
    double torque;
    this->declare_parameter<double>("torque", 0);
    this->get_parameter<double>("torque", torque);
    torque_ = torque;

    clockSub_ = this->create_subscription<sensor_msgs::msg::TimeReference>(
        "sim_clock", qos.reliable(),
        std::bind(&Pendulum::clockCb, this, std::placeholders::_1));
  } else {
    // if torque is taken from a topic, synchronise clock and torque
    clockSubFilt_.subscribe(this, "sim_clock", qos.get_rmw_qos_profile());
    torqueSubFilt_.subscribe(this, "pendulum_torque",
                             qos.get_rmw_qos_profile());

    clockTorqueFilter_.reset(new ClockTorqueSync(
        ClockTorquePolicy(10), clockSubFilt_, torqueSubFilt_));
    clockTorqueFilter_->registerCallback(std::bind(&Pendulum::clockTorqueCb,
                                                   this, std::placeholders::_1,
                                                   std::placeholders::_2));
  }

  this->declare_parameter<double>("mass", 0);
  this->get_parameter<double>("mass", mass_);

  this->declare_parameter<double>("length", 0);
  this->get_parameter<double>("length", length_);

  this->declare_parameter<double>("gravity", 0);
  this->get_parameter<double>("gravity", gravity_);

  // publish at a lower rate than update rate, if this is not 1
  this->declare_parameter<int>("publish_increment", 2);
  this->get_parameter<unsigned int>("publish_increment", publishIncrement_);
  publishCount_ = publishIncrement_;

  // duration for which to run simulation, or infinite if -1
  this->declare_parameter<double>("duration", -1);
  this->get_parameter<double>("duration", duration_);
}

void Pendulum::clockCb(
    sensor_msgs::msg::TimeReference::ConstSharedPtr timeMsg) {
  if (!torque_) {
    RCLCPP_ERROR(this->get_logger(), "CONFIG ERROR no source of torque value.");
    return;
  }

  auto incomingStamp = timeMsg->header.stamp;

  // this will trigger the first loop of simulation only
  if (!lastTime_) {
    lastTime_ = incomingStamp;
    return;
  }

  double dt = timeToFloat(incomingStamp) - timeToFloat(lastTime_.value());
  lastTime_ = incomingStamp;

  // update state with Forward-Euler integration
  double accel = -gravity_ / length_ * sin(pos_) +
                 (torque_.value() / (mass_ * length_ * length_));
  vel_ = vel_ + accel * dt;
  pos_ = pos_ + vel_ * dt;

  // track publish rate, if not publishing every update step
  if (publishCount_ > 1) {
    publishCount_--;
  } else {
    sensor_msgs::msg::Temperature posMsg;
    posMsg.header.stamp = incomingStamp;
    posMsg.temperature = pos_;

    sensor_msgs::msg::Temperature velMsg;
    velMsg.header.stamp = incomingStamp;
    velMsg.temperature = vel_;

    angularPosPub_->publish(posMsg);
    angularVelPub_->publish(velMsg);

    publishCount_ = publishIncrement_;
  }

  if (duration_ > 0) {
    if (timeToFloat(incomingStamp) > duration_) {
      RCLCPP_INFO(this->get_logger(),
                  "%f seconds of simulation complete, shutting down.",
                  duration_);
      rclcpp::shutdown();
    }
  }
}

void Pendulum::clockTorqueCb(
    const sensor_msgs::msg::TimeReference::ConstSharedPtr &timeMsg,
    const sensor_msgs::msg::Temperature::ConstSharedPtr &torqueMsg) {
  torque_ = torqueMsg->temperature;
  clockCb(timeMsg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pendulum>());
  rclcpp::shutdown();
  return 0;
}