#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>

#include "geometry_msgs/msg/twist.hpp"

class AsyncDriverClient
{
public:
  explicit AsyncDriverClient(const std::string& endpoint);
  ~AsyncDriverClient();

  void submit(const geometry_msgs::msg::Twist& cmd);
  bool isHealthy() const;

private:
  void workerLoop();
  bool sendCommand(const geometry_msgs::msg::Twist& cmd);

private:
  std::string endpoint_;

  std::thread worker_;
  std::atomic<bool> running_{true};

  mutable std::mutex mutex_;
  std::optional<geometry_msgs::msg::Twist> pending_cmd_;

  std::atomic<bool> last_ok_{true};
};
