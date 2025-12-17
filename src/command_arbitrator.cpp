#include "base_controller/command_arbitrator.hpp"

void CommandArbitrator::setTimeoutMs(int timeout_ms)
{
  timeout_ms_ = timeout_ms;
}

void CommandArbitrator::addSource(const std::string & name, int priority)
{
  Source s;
  s.name = name;
  s.priority = priority;
  sources_.push_back(s);
}

void CommandArbitrator::updateSource(
  const std::string & name,
  const geometry_msgs::msg::Twist & cmd,
  const rclcpp::Time & stamp)
{
  for (auto & s : sources_) {
    if (s.name == name) {
      s.last = cmd;
      s.stamp = stamp;
      return;
    }
  }
}

geometry_msgs::msg::Twist CommandArbitrator::select(
  const rclcpp::Time & now,
  std::string & active_source,
  bool emergency_stop) const
{
  geometry_msgs::msg::Twist stop;
  stop.linear.x = 0.0;
  stop.angular.z = 0.0;

  if (emergency_stop) {
    active_source = "emergency";
    return stop;
  }

  const Source * best = nullptr;

  for (const auto & s : sources_) {
    double age_ms = (now - s.stamp).seconds() * 1000.0;
    if (age_ms > timeout_ms_) continue;
    if (!best || s.priority > best->priority) {
      best = &s;
    }
  }

  if (!best) {
    active_source = "none";
    return stop;
  }

  active_source = best->name;
  return best->last;
}
