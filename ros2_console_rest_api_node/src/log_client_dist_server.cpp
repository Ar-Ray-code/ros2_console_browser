#include <rclcpp/rclcpp.hpp>
#include <httplib.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <regex>
#include <thread>
#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>

class LogClientDistServer : public rclcpp::Node
{
public:
    LogClientDistServer() : Node("log_client_dist_server")
    {
        declare_parameter("target_ip", "0.0.0.0");
        declare_parameter("target_port", 8081);
        declare_parameter("target_ip_keyword", "target-ip");
        declare_parameter("qr_distribute_count", 1);
        declare_parameter("show_qr_code", false);
        declare_parameter("qr_location", "cli");
        declare_parameter("rest_api_server_ip", "localhost");
        declare_parameter("rest_api_server_port", 8080);
        
        target_ip_ = get_parameter("target_ip").as_string();
        target_port_ = get_parameter("target_port").as_int();
        target_ip_keyword_ = get_parameter("target_ip_keyword").as_string();
        qr_distribute_count_ = get_parameter("qr_distribute_count").as_int();
        show_qr_code_ = get_parameter("show_qr_code").as_bool();
        qr_location_ = get_parameter("qr_location").as_string();
        rest_api_server_ip_ = get_parameter("rest_api_server_ip").as_string();
        rest_api_server_port_ = get_parameter("rest_api_server_port").as_int();
        
        std::cout << "Starting Log Client Distribution Server" << std::endl;
        std::cout << "Client Distribution Server will run on " << target_ip_ << ":" << target_port_ << std::endl;
        std::cout << "REST API Server expected at " << rest_api_server_ip_ << ":" << rest_api_server_port_ << std::endl;
        
        client_html_path_ = findClientHtml();
        if (client_html_path_.empty()) {
            std::cerr << "Could not find client.html file. Please ensure it exists in the expected locations." << std::endl;
            return;
        }
        
        std::cout << "Client HTML path: " << client_html_path_ << std::endl;
        
        if (show_qr_code_) {
            generateAndShowQRCode();
        }
        startServer();
    }
    
private:
    std::string target_ip_;
    int target_port_;
    std::string target_ip_keyword_;
    int qr_distribute_count_;
    bool show_qr_code_;
    std::string qr_location_;
    std::string rest_api_server_ip_;
    int rest_api_server_port_;
    std::string client_html_path_;
    std::unique_ptr<httplib::Server> server_;
    std::thread server_thread_;
    std::vector<std::string> qr_urls_;
    
    std::string findClientHtml()
    {
        std::vector<std::string> possible_paths = {
            "./example/client.html",
            "../example/client.html",
            "../../example/client.html",
            "./client.html",
            "../client.html",
            "./ros2_console_rest_api_node/example/client.html",
            std::filesystem::current_path() / "example" / "client.html"
        };
        
        for (const auto& path : possible_paths) {
            if (std::filesystem::exists(path)) {
                return std::filesystem::absolute(path);
            }
        }
        
        try {
            for (const auto& entry : std::filesystem::recursive_directory_iterator(".")) {
                if (entry.is_regular_file() && entry.path().filename() == "client.html") {
                    return std::filesystem::absolute(entry.path());
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error searching for client.html: " << e.what() << std::endl;
        }
        
        return "";
    }
    
    std::string loadAndModifyClientHtml()
    {
        std::ifstream file(client_html_path_);
        if (!file.is_open()) {
            std::cerr << "Failed to open client.html file: " << client_html_path_ << std::endl;
            return "";
        }
        
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string content = buffer.str();
        
        std::string api_base;
        if (rest_api_server_ip_ == "localhost" || rest_api_server_ip_ == "0.0.0.0") {
            std::string server_ip = getServerIP();
            api_base = "http://" + server_ip + ":" + std::to_string(rest_api_server_port_);
        } else {
            api_base = "http://" + rest_api_server_ip_ + ":" + std::to_string(rest_api_server_port_);
        }
        
        std::cout << "Replacing localhost with: " << api_base << std::endl;
        
        content = std::regex_replace(content, std::regex("http://localhost:8080"), api_base);
        content = std::regex_replace(content, std::regex("http://localhost:" + std::to_string(rest_api_server_port_)), api_base);
        content = std::regex_replace(content, std::regex("'http://localhost:8080'"), "'" + api_base + "'");
        content = std::regex_replace(content, std::regex("\"http://localhost:8080\""), "\"" + api_base + "\"");
        content = std::regex_replace(content, std::regex("`http://localhost:8080`"), "`" + api_base + "`");
        content = std::regex_replace(content, std::regex("const API_BASE = 'http://localhost:8080';"), "const API_BASE = '" + api_base + "';");
        content = std::regex_replace(content, std::regex("const API_BASE = \"http://localhost:8080\";"), "const API_BASE = \"" + api_base + "\";");
        
        if (rest_api_server_port_ != 8080) {
            content = std::regex_replace(content, std::regex(":8080"), ":" + std::to_string(rest_api_server_port_));
        }
        
        content = std::regex_replace(content, std::regex("<strong>Server URL:</strong> http://localhost:8080"), "<strong>Server URL:</strong> " + api_base);
        std::cout << "HTML modification completed. API_BASE set to: " << api_base << std::endl;
        
        return content;
    }
    
    std::string getServerIP()
    {
        if (target_ip_ != "0.0.0.0" && target_ip_ != "localhost") {
            return target_ip_;
        }
        
        std::vector<std::string> ips = getAvailableIPs();
        
        for (const auto& ip : ips) {
            if (ip.find(target_ip_keyword_) != std::string::npos) {
                std::cout << "Selected IP with keyword '" << target_ip_keyword_ << "': " << ip << std::endl;
                return ip;
            }
        }
        
        if (!ips.empty()) {
            std::cout << "Using first available IP: " << ips[0] << std::endl;
            return ips[0];
        }
        
        std::cout << "No suitable IP found, using localhost" << std::endl;
        return "localhost";
    }
    
    void startServer()
    {
        server_ = std::make_unique<httplib::Server>();
        
        server_->set_pre_routing_handler([](const httplib::Request&, httplib::Response& res) {
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
            res.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization, X-Requested-With");
            res.set_header("Access-Control-Max-Age", "86400");
            return httplib::Server::HandlerResponse::Unhandled;
        });
        
        server_->Options(".*", [](const httplib::Request&, httplib::Response& res) {
            res.status = 200;
            return;
        });
        
        server_->Get("/", [this](const httplib::Request&, httplib::Response& res) {
            std::string content = loadAndModifyClientHtml();
            if (content.empty()) {
                res.status = 500;
                res.set_content("Failed to load client.html", "text/plain");
                return;
            }
            res.set_content(content, "text/html");
        });
        
        server_->Get("/client.html", [this](const httplib::Request&, httplib::Response& res) {
            std::string content = loadAndModifyClientHtml();
            if (content.empty()) {
                res.status = 500;
                res.set_content("Failed to load client.html", "text/plain");
                return;
            }
            res.set_content(content, "text/html");
        });
        
        server_->Get("/info", [this](const httplib::Request&, httplib::Response& res) {
            std::string server_ip;
            std::string api_base;
            if (rest_api_server_ip_ == "localhost" || rest_api_server_ip_ == "0.0.0.0") {
                server_ip = getServerIP();
                api_base = "http://" + server_ip + ":" + std::to_string(rest_api_server_port_);
            } else {
                server_ip = rest_api_server_ip_;
                api_base = "http://" + rest_api_server_ip_ + ":" + std::to_string(rest_api_server_port_);
            }
            
            std::string json_response = "{\n";
            json_response += "  \"server\": \"Log Client Distribution Server\",\n";
            json_response += "  \"target_ip\": \"" + target_ip_ + "\",\n";
            json_response += "  \"target_port\": " + std::to_string(target_port_) + ",\n";
            json_response += "  \"rest_api_server_port\": " + std::to_string(rest_api_server_port_) + ",\n";
            json_response += "  \"detected_server_ip\": \"" + server_ip + "\",\n";
            json_response += "  \"api_base_url\": \"" + api_base + "\",\n";
            json_response += "  \"client_html_path\": \"" + client_html_path_ + "\"\n";
            json_response += "}";
            
            res.set_content(json_response, "application/json");
        });
        
        
        server_thread_ = std::thread([this]() {
            std::cout << "HTTP Server starting on " << target_ip_ << ":" << target_port_ << std::endl;
            if (!server_->listen(target_ip_.c_str(), target_port_)) {
                std::cerr << "Failed to start HTTP server on " << target_ip_ << ":" << target_port_ << std::endl;
            }
        });
        
        server_->Get("/qr", [this](const httplib::Request&, httplib::Response& res) {
            std::string qr_html = generateQRCodeHtml();
            res.set_content(qr_html, "text/html");
        });
        
        server_->Get("/qr/json", [this](const httplib::Request&, httplib::Response& res) {
            std::string json_response = "{\"qr_urls\": [";
            for (size_t i = 0; i < qr_urls_.size(); ++i) {
                json_response += "\"" + qr_urls_[i] + "\"";
                if (i < qr_urls_.size() - 1) json_response += ",";
            }
            json_response += "]}";
            res.set_content(json_response, "application/json");
        });
        
        std::cout << "Client Distribution Server started successfully" << std::endl;
        std::string display_ip = (target_ip_ == "0.0.0.0") ? getServerIP() : target_ip_;

        std::cout << "=== Access URLs ===" << std::endl;
        std::cout << "Client HTML: http://" << display_ip << ":" << target_port_ << std::endl;
        if (rest_api_server_ip_ == "localhost" || rest_api_server_ip_ == "0.0.0.0") {
            std::cout << "REST API:    http://" << getServerIP() << ":" << rest_api_server_port_ << std::endl;
        } else {
            std::cout << "REST API:    http://" << rest_api_server_ip_ << ":" << rest_api_server_port_ << std::endl;
        }
        if (show_qr_code_) {
            std::cout << "QR Codes:    http://" << display_ip << ":" << target_port_ << "/qr" << std::endl;
        }
        std::cout << "==================" << std::endl;
    }
    
public:
    void generateAndShowQRCode()
    {
        std::vector<std::string> ips = getAvailableIPs();
        
        std::cout << "Available IPs for QR code generation:" << std::endl;
        for (size_t i = 0; i < ips.size(); ++i) {
            std::cout << "  IP " << i + 1 << ": " << ips[i] << std::endl;
        }
        
        for (int i = 0; i < qr_distribute_count_ && i < static_cast<int>(ips.size()); ++i) {
            std::string url = "http://" + ips[i] + ":" + std::to_string(target_port_);
            qr_urls_.push_back(url);
            std::cout << "Generated QR URL " << i + 1 << ": " << url << std::endl;
        }
        
        // QRコード表示
        if (qr_location_ == "cli") {
            showQRCodeCLI();
        } else if (qr_location_ == "browser") {
            showQRCodeBrowser();
        } else if (qr_location_ == "gtk") {
            showQRCodeGTK();
        }
    }
    
    std::vector<std::string> getAvailableIPs()
    {
        std::vector<std::string> ips;
        if (target_ip_ != "0.0.0.0" && target_ip_ != "localhost") {
            ips.push_back(target_ip_);
        }
        
        std::string cmd = "hostname -I 2>/dev/null | tr ' ' '\n' | head -5";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (pipe) {
            char buffer[128];
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
                std::string ip(buffer);
                ip.erase(ip.find_last_not_of("\n\r ") + 1);
                if (!ip.empty() && ip != "127.0.0.1") {
                    ips.push_back(ip);
                }
            }
            pclose(pipe);
        }
        
        if (ips.empty()) {
            ips.push_back("localhost");
        }
        
        return ips;
    }
    
    void showQRCodeCLI()
    {
        std::cout << "=== QR Code URLs ===" << std::endl;
        for (size_t i = 0; i < qr_urls_.size(); ++i) {
            std::cout << "QR Code " << i + 1 << ": " << qr_urls_[i] << std::endl;
            
            std::string qr_cmd = "qrencode -t ANSI '" + qr_urls_[i] + "' 2>/dev/null";
            if (system(qr_cmd.c_str()) != 0) {
                std::cout << "Manual QR generation needed for: " << qr_urls_[i] << std::endl;
                std::cout << "Install qrencode with: sudo apt install qrencode" << std::endl;
            }
        }
        std::cout << "====================" << std::endl;
    }
    
    void showQRCodeBrowser()
    {
        if (!qr_urls_.empty()) {
            std::string url = "http://" + getServerIP() + ":" + std::to_string(target_port_) + "/qr";
            std::string cmd = "xdg-open '" + url + "' 2>/dev/null &";
            if (system(cmd.c_str()) != 0) {
                std::cout << "Could not open browser. Access QR codes at: " << url << std::endl;
            }
        }
    }
    
    void showQRCodeGTK()
    {
        std::cout << "GTK QR code display not implemented yet." << std::endl;
        showQRCodeCLI();
    }
    
    std::string generateQRCodeHtml()
    {
        std::string html = R"(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>QR Codes - ROS2 Log Client</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 20px;
            background-color: #f5f5f5;
        }
        .container {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            max-width: 800px;
            margin: 0 auto;
        }
        .qr-item {
            border: 1px solid #ddd;
            margin: 20px 0;
            padding: 20px;
            border-radius: 8px;
            text-align: center;
        }
        .qr-url {
            font-size: 18px;
            font-weight: bold;
            margin-bottom: 15px;
            color: #333;
        }
        .qr-code {
            margin: 20px 0;
        }
        .qr-instructions {
            color: #666;
            font-size: 14px;
            margin-top: 10px;
        }
    </style>
    <script src="https://cdn.jsdelivr.net/npm/qrcode@1.5.3/build/qrcode.min.js"></script>
</head>
<body>
    <div class="container">
        <h1>QR Codes for ROS2 Log Client</h1>
        <p>Scan these QR codes to access the log client from your mobile device or other computers:</p>
        
)";
        
        for (size_t i = 0; i < qr_urls_.size(); ++i) {
            html += "        <div class=\"qr-item\">\n";
            html += "            <div class=\"qr-url\">" + qr_urls_[i] + "</div>\n";
            html += "            <div class=\"qr-code\" id=\"qr" + std::to_string(i) + "\"></div>\n";
            html += "            <div class=\"qr-instructions\">Scan this QR code to access the log client</div>\n";
            html += "        </div>\n";
        }
        
        html += R"(
    </div>
    
    <script>
        // Generate QR codes
)";
        
        for (size_t i = 0; i < qr_urls_.size(); ++i) {
            html += "        QRCode.toCanvas(document.getElementById('qr" + std::to_string(i) + "'), '" + qr_urls_[i] + "', {width: 256, height: 256});\n";
        }
        
        html += R"(
    </script>
</body>
</html>)";
        
        return html;
    }
    
    ~LogClientDistServer()
    {
        if (server_) {
            server_->stop();
        }
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LogClientDistServer>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
