# ROS2 Console Browser - VibeCoding Project Instructions

## Project Overview
This project creates custom client applications that consume the ROS2 Console REST API. The existing `ros2_console_rest_api_node` provides a robust REST API for accessing ROS2 system information and logs at `http://localhost:8080`.

## Important Rules
- **NEVER modify the `ros2_console_rest_api_node` directory** - this is the stable API server
- **Always create new projects in separate directories** at the repository root
- **Use descriptive project names** like `flutter_ros2_client`, `qt_ros2_viewer`, `react_log_dashboard`, etc.

## API Server Information
The REST API server runs on `http://localhost:8080` and provides these endpoints:

### Core Endpoints
- `GET /` - API information and metadata
- `GET /nodes` - List all ROS2 nodes with publisher/subscriber info
- `GET /nodes/{node_name}/logs` - Get logs for specific node (supports namespaced nodes)
- `GET /logs?severity={level}&limit={count}` - Get filtered system logs
- `GET /topics` - List all ROS2 topics and associated nodes
- `GET /services` - List all ROS2 services and providers

### Response Formats
**Node List Response:**
```json
[
  {
    "name": "string",
    "log_count": number,
    "publishers": {"topic_name": ["message_types"]},
    "subscribers": {"topic_name": ["message_types"]}
  }
]
```

**Log Entry Response:**
```json
[
  {
    "timestamp": {"sec": number, "nanosec": number},
    "level": "INFO|WARN|ERROR|DEBUG",
    "name": "logger_name",
    "message": "log_message",
    "file": "source_file.cpp",
    "function": "function_name",
    "line": number
  }
]
```

## Development Guidelines

### Starting the API Server
```bash
docker compose up
```
This starts the API server at `http://localhost:8080` and serves the example client at `http://localhost:8081`.

### Testing API Endpoints
- View nodes: `http://localhost:8080/nodes`
- View talker logs: `http://localhost:8080/nodes/talker/logs`
- View all topics: `http://localhost:8080/topics`
- View services: `http://localhost:8080/services`

### Project Structure for New Clients
```
project_root/
├── your_client_name/
│   ├── README.md              # Project-specific documentation
│   ├── src/                   # Source code
│   ├── assets/               # Static assets if needed
│   └── [framework-specific files]
```

### Common Client Features to Implement
1. **Node Browser** - Display available ROS2 nodes
2. **Real-time Log Viewer** - Stream logs from selected nodes
3. **Topic/Service Explorer** - Browse ROS2 graph structure
4. **Dark/Light Mode** - Follow the example client's theme support
5. **Auto-refresh** - Periodic updates for live monitoring

### Framework-Specific Examples

#### Web-based Clients (React, Vue, Angular)
- Use `fetch()` or axios for API calls
- Implement WebSocket-like polling for real-time updates
- Handle CORS (already configured on server)
- Consider responsive design for mobile access

#### Native Desktop (Qt, Electron, Flutter Desktop)
- Use HTTP client libraries appropriate for framework
- Implement native UI components for better performance
- Consider system tray integration for background monitoring

#### Mobile (Flutter, React Native)
- Optimize for touch interfaces
- Consider push notifications for critical log events
- Handle network connectivity changes gracefully

### Error Handling Best Practices
- Always check HTTP status codes
- Provide user-friendly error messages
- Implement retry logic for network failures
- Handle empty log responses gracefully

### Performance Considerations
- Implement pagination for large log datasets
- Use appropriate refresh intervals (example uses 100ms)
- Cache node/topic/service lists when appropriate
- Consider virtualization for large log lists

## Reference Implementation
Study `ros2_console_rest_api_node/example/client.html` for:
- API usage patterns
- Error handling approaches
- Real-time update strategies
- Dark/light mode implementation
- Responsive design principles

## Testing Your Client
1. Start the API server: `docker compose up`
2. Verify API connectivity: `curl http://localhost:8080/nodes`
3. Test your client against live ROS2 data
4. Check error handling with invalid endpoints
5. Verify real-time updates work as expected

## Common Pitfalls to Avoid
- Don't hardcode localhost URLs in production clients
- Handle URL encoding for namespaced node names (e.g., `/namespace/node`)
- Don't assume logs are always available for every node
- Implement proper cleanup for intervals/timers
- Test with both empty and large datasets

## Support Files
- `openapi.yaml` - Complete API specification for code generation
- `ros2_console_rest_api_node/example/` - Reference HTML/CSS implementation