#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <unordered_map>


#include "ros2_console_rest_api_node/rest_api_server.hpp"

std::atomic<bool> g_shutdown_requested(false);

void signal_handler(int signum) {
  std::cout << "Received signal " << signum << ", shutting down..." << std::endl;
  g_shutdown_requested.store(true);
}

namespace ros2_console_rest_api_node
{

RestApiServer::RestApiServer(const rclcpp::NodeOptions & options)
: Node("rest_api_server", options)
{
  rclcpp::NodeOptions log_viewer_options = options;
  
  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("log_buffer_size", 2000));  // Increase buffer size
  log_viewer_options.parameter_overrides(params);
  
  log_viewer_base_ = std::make_shared<log_viewer_base::LogViewerBase>(log_viewer_options);
  
  if (log_viewer_base_->is_paused()) {
    std::cout << "Unpausing log viewer base" << std::endl;
    log_viewer_base_->set_paused_flag(false);
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  update_graph_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    [this]() {
      if (log_viewer_base_) {
        log_viewer_base_->update_graph();
      }
    });
    
  debug_logs_timer_ = this->create_wall_timer(
    std::chrono::seconds(30),
    [this]() {
      if (log_viewer_base_) {
        auto pending_logs = log_viewer_base_->get_pending_logs();
        if (pending_logs.empty()) {
          std::cout << "No logs received yet." << std::endl;
        }
      }
    });

  monitor_state_timer_ = this->create_wall_timer(
    std::chrono::seconds(30),
    [this]() {
      if (log_viewer_base_) {
        auto nodes = log_viewer_base_->get_nodes();
        auto all_logs = log_viewer_base_->get_pending_logs();
      }
    });
}

RestApiServer::~RestApiServer()
{
  stop_server();
}

void RestApiServer::start_server(const std::string & host, int port)
{
  if (running_) {
    std::cout << "Server is already running" << std::endl;
    return;
  }

  server_ = std::make_unique<httplib::Server>();
  setup_routes();

  running_ = true;
  server_thread_ = std::thread([this, host, port]() {
    std::cout << "Starting HTTP server on " << host << ":" << port << std::endl;
    
    server_->set_read_timeout(5, 0);
    server_->set_write_timeout(5, 0);
    server_->set_idle_interval(0, 100000);
    
    if (!server_->bind_to_port(host.c_str(), port)) {
      std::cerr << "Failed to bind to port " << port << std::endl;
      running_ = false;
      return;
    }
    
    if (!server_->listen_after_bind()) {
      std::cerr << "Failed to start server" << std::endl;
      running_ = false;
      return;
    }
  });
}

void RestApiServer::stop_server()
{
  if (!running_) {
    return;
  }

  running_ = false;
  
  if (server_) {
    server_->stop();
  }

  if (server_thread_.joinable()) {
    server_thread_.join();
  }
}

void RestApiServer::setup_routes()
{
  server_->Get("/", [this](const httplib::Request & req, httplib::Response & res) {
    handle_root(req, res);
  });

  server_->Get("/nodes", [this](const httplib::Request & req, httplib::Response & res) {
    handle_nodes(req, res);
  });

  server_->Get(R"(/nodes/(.*)/logs)", [this](const httplib::Request & req, httplib::Response & res) {
    handle_node_logs(req, res);
  });
  
  server_->Get("/logs", [this](const httplib::Request & req, httplib::Response & res) {
    handle_all_logs(req, res);
  });
  
  server_->Get("/topics", [this](const httplib::Request & req, httplib::Response & res) {
    handle_topics(req, res);
  });
  
  server_->Get("/services", [this](const httplib::Request & req, httplib::Response & res) {
    handle_services(req, res);
  });

  server_->set_error_handler([this](const httplib::Request &, httplib::Response & res) {
    json error_response = {
      {"error", "Not Found"},
      {"status", 404}
    };
    res.status = 404;
    setup_cors(res);
    res.set_content(error_response.dump(), "application/json");
  });
}

void RestApiServer::setup_cors(httplib::Response & res)
{
  res.set_header("Access-Control-Allow-Origin", "*");
  res.set_header("Access-Control-Allow-Methods", "GET, OPTIONS");
  res.set_header("Access-Control-Allow-Headers", "Content-Type");
}

void RestApiServer::handle_root(const httplib::Request &, httplib::Response & res)
{
  json response = {
    {"message", "ROS2 Log Viewer REST API"},
    {"version", "1.0.0"},
    {"endpoints", {
      {"/", "API information"},
      {"/nodes", "List all nodes"},
      {"/nodes/{node_name}/logs", "Get logs for specific node"},
      {"/logs", "Get all logs with optional severity filter"},
      {"/topics", "List all topics with publishers/subscribers"},
      {"/services", "List all services with providers"}
    }}
  };
  setup_cors(res);
  res.set_content(response.dump(2), "application/json");
}

void RestApiServer::handle_nodes(const httplib::Request &, httplib::Response & res)
{
  json response = json::array();
  if (log_viewer_base_) {
    size_t node_count = 0;
    
    for (const auto & node : log_viewer_base_->get_nodes()) {
      json node_info = {
        {"name", node.full_name}
      };
      
      std::vector<std::string> node_filter = {node.full_name};
      auto logs = log_viewer_base_->get_filtered_logs(node_filter);
      node_info["log_count"] = logs.size();
      
      if (!node.publishers.empty()) {
        json pub_info = json::object();
        for (const auto& [topic, types] : node.publishers) {
          pub_info[topic] = types;
        }
        node_info["publishers"] = pub_info;
      }
      
      if (!node.subscribers.empty()) {
        json sub_info = json::object();
        for (const auto& [topic, types] : node.subscribers) {
          sub_info[topic] = types;
        }
        node_info["subscribers"] = sub_info;
      }
      node_count++;
      response.push_back(node_info);
    }
  }
  setup_cors(res);
  res.set_content(response.dump(), "application/json");
}

void RestApiServer::handle_node_logs(const httplib::Request & req, httplib::Response & res)
{
  std::string node_name = req.matches[1];
  std::string decoded_node_name;
  for (size_t i = 0; i < node_name.length(); ++i) {
    if (node_name[i] == '%' && i + 2 < node_name.length()) {
      std::string hex = node_name.substr(i + 1, 2);
      if (hex == "2F" || hex == "2f") {
        decoded_node_name += '/';
        i += 2;
      } else {
        decoded_node_name += node_name[i];
      }
    } else {
      decoded_node_name += node_name[i];
    }
  }
  
  bool node_exists = false;
  if (log_viewer_base_) {
    for (const auto & node : log_viewer_base_->get_nodes()) {
      if (node.full_name.substr(1) == decoded_node_name) {
        node_exists = true;
        break;
      }
    }
  }
  
  if (!node_exists) {
    json error_response = {
      {"error", "Node not found"},
      {"node", node_name},
      {"status", 404}
    };
    res.status = 404;
    setup_cors(res);
    res.set_content(error_response.dump(), "application/json");
    return;
  }

  json response = json::array();
  if (log_viewer_base_) {
    std::vector<std::string> node_filter = {node_name};
    auto logs = log_viewer_base_->get_filtered_logs(node_filter);
    size_t log_idx = 0;
    for (const auto & log : logs) {
      try {
        if (log.msg.length() > 10000) {
          continue;
        }

        log_idx++;
        json log_entry = {
          {"timestamp", {
            {"sec", log.stamp.sec},
            {"nanosec", log.stamp.nanosec}
          }},
          {"level", log_viewer_base::level_to_string(static_cast<log_viewer_base::LogLevel>(log.level))},
          {"name", log.name},
          {"message", log.msg},
          {"file", log.file},
          {"function", log.function},
          {"line", log.line}
        };
        response.push_back(log_entry);
      } catch (const std::exception& e) {
        std::cout << "Error processing log entry: " << e.what() << std::endl;
      }
    }
  }

  setup_cors(res);
  res.set_content(response.dump(), "application/json");
}

void RestApiServer::handle_all_logs(const httplib::Request & req, httplib::Response & res)
{
  std::string severity_filter = "";
  if (req.has_param("severity")) {
    severity_filter = req.get_param_value("severity");
  }
  
  size_t limit = 100;
  if (req.has_param("limit")) {
    try {
      limit = std::stoul(req.get_param_value("limit"));
    } catch (const std::exception& e) {
      std::cout << "Invalid limit parameter: " << e.what() << ", using default limit of " << limit << std::endl;
    }
  }

  json response = json::array();
  if (log_viewer_base_) {
    auto all_pending_logs = log_viewer_base_->get_pending_logs();
    
    size_t count = 0;
    for (const auto & log : all_pending_logs) {
      try {
        if (log.msg.length() > 10000) {
          std::cout << "Skipping suspiciously large log message (length: " 
                    << log.msg.length() << ")" << std::endl;
          continue;
        }
        
        if (!severity_filter.empty()) {
          std::string log_level = log_viewer_base::level_to_string(
            static_cast<log_viewer_base::LogLevel>(log.level));
          
          if (log_level != severity_filter) {
            continue;
          }
        }
        
        json log_entry = {
          {"timestamp", {
            {"sec", log.stamp.sec},
            {"nanosec", log.stamp.nanosec}
          }},
          {"level", log_viewer_base::level_to_string(static_cast<log_viewer_base::LogLevel>(log.level))},
          {"name", log.name},
          {"message", log.msg},
          {"file", log.file},
          {"function", log.function},
          {"line", log.line}
        };
      
        response.push_back(log_entry);
        if (++count >= limit) {
          break;
        }
      } catch (const std::exception& e) {
        std::cout << "Error processing log entry: " << e.what() << std::endl;
      }
    }
  }
  setup_cors(res);
  res.set_content(response.dump(), "application/json");
}

void RestApiServer::handle_topics(const httplib::Request &, httplib::Response & res)
{
  json response = json::object();
  if (log_viewer_base_) {
    auto topics_map = log_viewer_base_->get_topics();
    for (const auto & [topic_name, node_names] : topics_map) {
      response[topic_name] = node_names;
    }
  }
  setup_cors(res);
  res.set_content(response.dump(), "application/json");
}

void RestApiServer::handle_services(const httplib::Request &, httplib::Response & res)
{
  json response = json::object();
  if (log_viewer_base_) {
    auto services_map = log_viewer_base_->get_services();
    for (const auto & [service_name, node_names] : services_map) {
      response[service_name] = node_names;
    }
  }
  setup_cors(res);
  res.set_content(response.dump(), "application/json");
}

std::string RestApiServer::get_current_timestamp_string() const
{
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S");
  ss << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
  
  return ss.str();
}

}  // namespace ros2_console_rest_api_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  
  auto node = std::make_shared<ros2_console_rest_api_node::RestApiServer>(options);
  
  int port = 8080;
  std::string host = "0.0.0.0";
  
  std::cout << "REST API Server node created successfully." << std::endl;
  
  const char* domain_id_env = std::getenv("ROS_DOMAIN_ID");
  if (domain_id_env) {
    std::cout << "Using ROS_DOMAIN_ID: " << domain_id_env << std::endl;
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  if (node->has_parameter("port")) {
    port = node->get_parameter("port").as_int();
  } else {
    node->declare_parameter("port", port);
  }
  
  if (node->has_parameter("host")) {
    host = node->get_parameter("host").as_string();
  } else {
    node->declare_parameter("host", host);
  }
  
  node->start_server(host, port);
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  if (node->log_viewer_base_) {
    std::cout << "Adding LogViewerBase node to executor" << std::endl;
    executor.add_node(node->log_viewer_base_);
  } else {
    std::cerr << "ERROR: Could not add LogViewerBase node to executor (null pointer)" << std::endl;
  }
  
  std::cout << "REST API Server is running at http://" 
            << ((host == "0.0.0.0") ? "localhost" : host)
            << ":" << port << "/" << std::endl;
  std::cout << "Press Ctrl+C to terminate the server." << std::endl;
  
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  
  std::thread status_thread([node]() {
    while (!g_shutdown_requested) {
      std::this_thread::sleep_for(std::chrono::seconds(15));
      if (node->log_viewer_base_ && !g_shutdown_requested) {
        auto pending_logs = node->log_viewer_base_->get_pending_logs();
      }
    }
  });
  
  try {
    while (!g_shutdown_requested) {
      executor.spin_some(std::chrono::milliseconds(100));
    }
  } catch (const std::exception & e) {
    std::cerr << "Exception in main thread: " << e.what() << std::endl;
    g_shutdown_requested.store(true);
  }
  
  if (status_thread.joinable()) {
    status_thread.join();
  }
  
  node->stop_server();
  rclcpp::shutdown();
  
  return 0;
}