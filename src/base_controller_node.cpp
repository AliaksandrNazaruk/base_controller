#include <chrono>
#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "std_msgs/msg/bool.hpp"

#include "base_controller/async_driver_client.hpp"

using namespace std::chrono_literals;

class BaseControllerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit BaseControllerNode()
  : rclcpp_lifecycle::LifecycleNode("base_controller")
  {}

  // ============================================================
  // Lifecycle
  // ============================================================

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    control_rate_hz_    = declare_parameter<double>("control_rate_hz", 20.0);
    cmd_vel_timeout_ms_ = declare_parameter<int>("cmd_vel_timeout_ms", 200);
    max_linear_mps_     = declare_parameter<double>("max_linear_mps", 0.6);
    max_angular_rps_    = declare_parameter<double>("max_angular_rps", 1.2);
    driver_endpoint_ =
      declare_parameter<std::string>("driver_endpoint", "http://localhost:8105");

    // --- Command source metadata ---
    cmd_nav_.priority    = 10;
    cmd_nav_.name        = "nav";

    cmd_teleop_.priority = 50;
    cmd_teleop_.name     = "teleop";

    cmd_safe_.priority   = 90;
    cmd_safe_.name       = "safe";

    driver_client_ = std::make_unique<AsyncDriverClient>(driver_endpoint_);

    // --- Command inputs ---

    cmd_nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_nav_.last  = *msg;
        cmd_nav_.stamp = now();
      });

    cmd_teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_teleop", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_teleop_.last  = *msg;
        cmd_teleop_.stamp = now();
      });

    cmd_safe_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_safe", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_safe_.last  = *msg;
        cmd_safe_.stamp = now();
      });

    // --- Emergency stop (latched) ---
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop",
      rclcpp::QoS(1).reliable(),   // ❌ убрать transient_local()
      std::bind(&BaseControllerNode::emergencyStopCallback, this, std::placeholders::_1)
    );

    // --- Diagnostics ---
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10)
    );

    RCLCPP_INFO(get_logger(), "Configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&BaseControllerNode::controlLoop, this)
    );

    RCLCPP_INFO(get_logger(), "Activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    control_timer_.reset();
    sendStop();
    RCLCPP_WARN(get_logger(), "Deactivated → STOP");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    control_timer_.reset();
    sendStop();

    cmd_nav_sub_.reset();
    cmd_teleop_sub_.reset();
    cmd_safe_sub_.reset();
    estop_sub_.reset();
    diag_pub_.reset();

    emergency_stop_ = false;
    driver_client_.reset();

    RCLCPP_INFO(get_logger(), "Cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    sendStop();
    RCLCPP_WARN(get_logger(), "Shutdown");
    return CallbackReturn::SUCCESS;
  }

private:
  // ============================================================
  // Command arbitration
  // ============================================================

  struct CmdSource
  {
    geometry_msgs::msg::Twist last;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    int priority{0};
    std::string name;
  };

  CmdSource cmd_nav_;
  CmdSource cmd_teleop_;
  CmdSource cmd_safe_;

  geometry_msgs::msg::Twist selectCommand(std::string & active_source)
  {
    geometry_msgs::msg::Twist stop;
    stop.linear.x  = 0.0;
    stop.angular.z = 0.0;

    if (emergency_stop_) {
      active_source = "emergency";
      return stop;
    }

    auto now_t = now();
    CmdSource * best = nullptr;

    auto consider = [&](CmdSource & src) {
      const double age_ms =
        (now_t - src.stamp).seconds() * 1000.0;
      if (age_ms > cmd_vel_timeout_ms_) return;
      if (!best || src.priority > best->priority) {
        best = &src;
      }
    };

    consider(cmd_nav_);
    consider(cmd_teleop_);
    consider(cmd_safe_);

    if (!best) {
      active_source = "none";
      return stop;
    }

    active_source = best->name;
    return best->last;
  }

  // ============================================================
  // Emergency stop
  // ============================================================

  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      if (!emergency_stop_) {
        RCLCPP_ERROR(get_logger(), "!!! EMERGENCY STOP ACTIVATED !!!");
        sendStop();
      }
      emergency_stop_ = true;
    } else {
      RCLCPP_WARN(get_logger(), "Emergency stop RESET");
      emergency_stop_ = false;
    }
  }

  // ============================================================
  // Control loop
  // ============================================================

  void controlLoop()
  {
    if (get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    if (!driver_client_) {
      return;
    }

    driver_ok_ = driver_client_->isHealthy();

    std::string active_source;
    geometry_msgs::msg::Twist cmd = selectCommand(active_source);

    watchdog_triggered_ = (active_source == "none");

    publishDiagnostics(active_source);

    driver_client_->submit(cmd);
  }

  void sendStop()
  {
    geometry_msgs::msg::Twist stop;
    stop.linear.x  = 0.0;
    stop.angular.z = 0.0;

    if (driver_client_) {
      driver_client_->submit(stop);
    }
  }

  // ============================================================
  // Diagnostics
  // ============================================================

  void publishDiagnostics(const std::string & active_source)
  {
    if (!diag_pub_) return;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "base_controller";
    status.hardware_id = "base_controller";

    if (emergency_stop_) {
      status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "EMERGENCY STOP ACTIVE";
    }
    else if (!driver_ok_) {
      status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Driver unhealthy";
    }
    else if (watchdog_triggered_) {
      status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "No active command";
    }
    else {
      status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "OK";
    }

    diagnostic_msgs::msg::KeyValue kv;

    kv.key = "active_source";
    kv.value = active_source;
    status.values.push_back(kv);

    kv.key = "emergency_stop";
    kv.value = emergency_stop_ ? "true" : "false";
    status.values.push_back(kv);

    kv.key = "driver_ok";
    kv.value = driver_ok_ ? "true" : "false";
    status.values.push_back(kv);

    array.status.push_back(status);
    diag_pub_->publish(array);
  }

private:
  // ============================================================
  // ROS interfaces
  // ============================================================

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_safe_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ============================================================
  // State
  // ============================================================

  bool emergency_stop_{false};
  bool driver_ok_{false};
  bool watchdog_triggered_{false};

  // ============================================================
  // Parameters
  // ============================================================

  double control_rate_hz_{20.0};
  int cmd_vel_timeout_ms_{200};
  double max_linear_mps_{0.6};
  double max_angular_rps_{1.2};
  std::string driver_endpoint_;

  // ============================================================
  // Driver
  // ============================================================

  std::unique_ptr<AsyncDriverClient> driver_client_;
};

// ============================================================
// main
// ============================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BaseControllerNode>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
