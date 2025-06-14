#ifndef ros2_console_rest_api_node__REST_API_SERVER_HPP_
#define ros2_console_rest_api_node__REST_API_SERVER_HPP_

#include <httplib.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <log_viewer_base/log_viewer_base.hpp>

#include <atomic>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace ros2_console_rest_api_node
{

using json = nlohmann::json;

class RestApiServer : public rclcpp::Node
{
public:
  explicit RestApiServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RestApiServer();

  void start_server(const std::string & host = "0.0.0.0", int port = 8080);
  void stop_server();
  
  std::shared_ptr<log_viewer_base::LogViewerBase> log_viewer_base_;

private:
  std::unique_ptr<httplib::Server> server_;
  std::thread server_thread_;
  std::atomic<bool> running_{false};

  rclcpp::TimerBase::SharedPtr update_graph_timer_;
  rclcpp::TimerBase::SharedPtr debug_logs_timer_;
  rclcpp::TimerBase::SharedPtr monitor_state_timer_;

  void setup_routes();
  void setup_cors(httplib::Response & res);
  
  void handle_root(const httplib::Request & req, httplib::Response & res);
  void handle_nodes(const httplib::Request & req, httplib::Response & res);
  void handle_node_logs(const httplib::Request & req, httplib::Response & res);
  void handle_all_logs(const httplib::Request & req, httplib::Response & res);
  void handle_topics(const httplib::Request & req, httplib::Response & res);
  void handle_services(const httplib::Request & req, httplib::Response & res);
  
  std::string get_current_timestamp_string() const;
};

}  // namespace ros2_console_rest_api_node

#endif  // ros2_console_rest_api_node__REST_API_SERVER_HPP_