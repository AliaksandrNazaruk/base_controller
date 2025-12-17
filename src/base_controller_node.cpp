#include <chrono>
#include <memory>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "base_controller/robot_state.hpp"
#include "base_controller/async_driver_client.hpp"
#include "base_controller/command_arbitrator.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

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
  // Robot state update
  // ============================================================

  void updateRobotState()
  {
    if (!driver_client_) {
      robot_state_.driver_fault = true;
      robot_state_.driver_ready = false;
      return;
    }

    robot_state_.driver_fault = !driver_client_->isHealthy();

    // TODO: driver_ready must be independent from fault
    // ready = initialized + enabled + no inhibit
    robot_state_.driver_ready = !robot_state_.driver_fault;

    if (!robot_state_.driver_fault) {
      robot_state_.last_feedback_ts = now();
    }

    // Stubs for future HW integration
    robot_state_.estop_hw_active = false;
    robot_state_.velocity_zero  = true;
  }

  // ============================================================
  // SAFETY FSM
  // ============================================================

  enum class SafetyState {
    SAFE,
    OPERATIONAL,
    EMERGENCY
  };

  // ============================================================
  // Lifecycle
  // ============================================================

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    control_rate_hz_    = declare_parameter<double>("control_rate_hz", 20.0);
    cmd_vel_timeout_ms_ = declare_parameter<int>("cmd_vel_timeout_ms", 200);
    feedback_timeout_ms_= declare_parameter<int>("feedback_timeout_ms", 500);
    driver_endpoint_    =
      declare_parameter<std::string>("driver_endpoint", "http://localhost:8105");

    driver_client_ = std::make_unique<AsyncDriverClient>(driver_endpoint_);

    arbitrator_.setTimeoutMs(cmd_vel_timeout_ms_);
    arbitrator_.addSource("nav",    10);
    arbitrator_.addSource("teleop", 50);
    arbitrator_.addSource("safe",   90);

    cmd_nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        arbitrator_.updateSource("nav", *msg, now());
      });

    cmd_teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_teleop", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        arbitrator_.updateSource("teleop", *msg, now());
      });

    cmd_safe_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_safe", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        arbitrator_.updateSource("safe", *msg, now());
      });

    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", rclcpp::QoS(1).reliable(),
      std::bind(&BaseControllerNode::emergencyStopCallback, this, std::placeholders::_1)
    );

    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10
    );

    reset_emergency_srv_ =
      create_service<std_srvs::srv::Trigger>(
        "/reset_emergency",
        std::bind(&BaseControllerNode::resetEmergencyCallback, this,
                  std::placeholders::_1, std::placeholders::_2)
      );

    enable_motion_srv_ =
      create_service<std_srvs::srv::Trigger>(
        "/enable_motion",
        std::bind(&BaseControllerNode::enableMotionCallback, this,
                  std::placeholders::_1, std::placeholders::_2)
      );

    safety_state_      = SafetyState::SAFE;
    emergency_latched_ = false;
    robot_state_.motion_enabled = false;

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
    safety_state_ = SafetyState::SAFE;

    RCLCPP_WARN(get_logger(), "Deactivated → SAFE");
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
    driver_client_.reset();

    emergency_latched_ = false;
    safety_state_      = SafetyState::SAFE;

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
  // Services
  // ============================================================

  void enableMotionCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (safety_state_ != SafetyState::SAFE || robot_state_.driver_fault) {
      response->success = false;
      response->message = "Cannot enable motion";
      return;
    }

    robot_state_.motion_enabled = true;

    RCLCPP_WARN(get_logger(), "Motion ENABLED by operator");

    response->success = true;
    response->message = "Motion enabled";
  }

  void resetEmergencyCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (safety_state_ != SafetyState::EMERGENCY) {
      response->success = false;
      response->message = "No active emergency";
      return;
    }

    emergency_latched_ = false;
    robot_state_.motion_enabled = false;
    safety_state_ = SafetyState::SAFE;

    RCLCPP_WARN(get_logger(), "Emergency reset by operator");

    response->success = true;
    response->message = "Emergency reset";
  }

  // ============================================================
  // Emergency stop
  // ============================================================

  void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !emergency_latched_) {
      emergency_latched_ = true;
      RCLCPP_ERROR(get_logger(), "!!! EMERGENCY STOP LATCHED !!!");
    }
  }

  // ============================================================
  // Control loop
  // ============================================================

  void controlLoop()
  {
    updateRobotState();
    updateSafetyState();

    geometry_msgs::msg::Twist cmd = computeOutputCommand();
    publishDiagnostics();

    if (driver_client_) {
      driver_client_->submit(cmd);
    }
  }

  void updateSafetyState()
  {
    if (emergency_latched_) {
      safety_state_ = SafetyState::EMERGENCY;
      return;
    }

    if (!robot_state_.motion_enabled ||
        get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
        robot_state_.driver_fault ||
        !robot_state_.driver_ready) {
      safety_state_ = SafetyState::SAFE;
      return;
    }

    const double feedback_age_ms =
      (now() - robot_state_.last_feedback_ts).seconds() * 1000.0;

    if (feedback_age_ms > feedback_timeout_ms_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Driver feedback timeout (%.0f ms) → SAFE STOP", feedback_age_ms);
      safety_state_ = SafetyState::SAFE;
      return;
    }

    std::string src;
    arbitrator_.select(now(), src, false);

    safety_state_ = (src == "none")
      ? SafetyState::SAFE
      : SafetyState::OPERATIONAL;
  }

  geometry_msgs::msg::Twist computeOutputCommand()
  {
    geometry_msgs::msg::Twist stop;

    if (safety_state_ != SafetyState::OPERATIONAL) {
      return stop;
    }

    std::string src;
    return arbitrator_.select(now(), src, false);
  }

  void sendStop()
  {
    geometry_msgs::msg::Twist stop;
    if (driver_client_) {
      driver_client_->submit(stop);
    }
  }

  // ============================================================
  // Diagnostics
  // ============================================================

  void publishDiagnostics()
  {
    if (!diag_pub_) return;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "base_controller";
    status.hardware_id = "base_controller";

    switch (safety_state_) {
      case SafetyState::EMERGENCY:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "EMERGENCY";
        break;
      case SafetyState::SAFE:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "SAFE";
        break;
      case SafetyState::OPERATIONAL:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "OPERATIONAL";
        break;
    }

    array.status.push_back(status);
    diag_pub_->publish(array);
  }

private:
  // ROS
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_emergency_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_motion_srv_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_safe_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  RobotState robot_state_;
  SafetyState safety_state_{SafetyState::SAFE};
  bool emergency_latched_{false};

  CommandArbitrator arbitrator_;
  std::unique_ptr<AsyncDriverClient> driver_client_;

  int feedback_timeout_ms_{500};
  double control_rate_hz_{20.0};
  int cmd_vel_timeout_ms_{200};
  std::string driver_endpoint_;
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
