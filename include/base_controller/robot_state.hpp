#pragma once

#include <rclcpp/time.hpp>

struct RobotState
{
  // Energy & motion
  bool motion_enabled{false};     // разрешение движения оператором
  bool driver_ready{false};       // драйвер подтвердил готовность
  bool driver_fault{false};       // fault от драйвера

  // Safety
  bool estop_hw_active{false};    // аппаратный E-Stop
  bool velocity_zero{true};       // реально ли стоим (позже энкодеры)

  // Feedback watchdog
  rclcpp::Time last_feedback_ts{0, 0, RCL_ROS_TIME};
};
