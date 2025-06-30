
#include <rclcpp/rclcpp.hpp>

inline double timeToFloat(builtin_interfaces::msg::Time time) {
  return double(time.sec) + double(time.nanosec) / 1000000000;
}

inline double degToRad(double deg) { return deg * (M_PI / 180); }
inline double radToDeg(double rad) { return rad * (180 / M_PI); }