#pragma once

#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include "geometry_msgs/msg/twist.hpp"

/**
 * @brief Deterministic command arbitration with priority and timeout.
 *
 * Safety intent:
 *  - No dynamic allocation after configuration
 *  - Deterministic selection
 *  - Emergency stop has absolute priority
 */
class CommandArbitrator
{
public:
  struct Source
  {
    geometry_msgs::msg::Twist last;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    int priority{0};
    std::string name;
  };

  // Configuration (call in lifecycle::on_configure)
  void setTimeoutMs(int timeout_ms);
  void addSource(const std::string & name, int priority);

  // Runtime updates
  void updateSource(
    const std::string & name,
    const geometry_msgs::msg::Twist & cmd,
    const rclcpp::Time & stamp
  );

  // Arbitration
  geometry_msgs::msg::Twist select(
    const rclcpp::Time & now,
    std::string & active_source,
    bool emergency_stop) const;

private:
  int timeout_ms_{200};
  std::vector<Source> sources_;
};
