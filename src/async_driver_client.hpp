#include "httplib/httplib.h"
#include "async_driver_client.hpp"
#include "httplib.h"
#include <nlohmann/json.hpp>

AsyncDriverClient::AsyncDriverClient(const std::string& endpoint)
: endpoint_(endpoint)
{
  worker_ = std::thread(&AsyncDriverClient::workerLoop, this);
}

AsyncDriverClient::~AsyncDriverClient()
{
  running_ = false;
  if (worker_.joinable())
    worker_.join();
}

void AsyncDriverClient::submit(const geometry_msgs::msg::Twist& cmd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_cmd_ = cmd;  // overwrite → latest wins
}

bool AsyncDriverClient::isHealthy() const
{
  return last_ok_;
}

void AsyncDriverClient::workerLoop()
{
  httplib::Client cli(endpoint_);
  cli.set_connection_timeout(0, 200000); // 200 ms
  cli.set_read_timeout(0, 200000);

  while (running_) {
    std::optional<geometry_msgs::msg::Twist> cmd;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_cmd_) {
        cmd = pending_cmd_;
        pending_cmd_.reset();
      }
    }

    if (cmd) {
      last_ok_ = sendCommand(*cmd);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

bool AsyncDriverClient::sendCommand(const geometry_msgs::msg::Twist& cmd)
{
  nlohmann::json payload = {
    {"linear_mps", cmd.linear.x},
    {"angular_rps", cmd.angular.z},
    {"timeout_ms", 100}
  };

  httplib::Client cli(endpoint_);
  auto res = cli.Post("/drive", payload.dump(), "application/json");

  return res && res->status == 200;
}

