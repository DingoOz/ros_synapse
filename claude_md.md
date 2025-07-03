# TurtleBot3 Controller - Qt6 C++ Application

## Project Overview
A comprehensive Qt6 C++ application for managing TurtleBot3 robots with ROS2 Jazzy on Ubuntu 24.04. The application provides a modern GUI for remote robot control, ROS2 command execution, SSH terminal access, and real-time process monitoring.

## Architecture
- **Main Window**: Tabbed interface with dark theme
- **Command Builder**: Form-based ROS2 command generation with parameter configuration
- **SSH Manager**: Remote connection and terminal access to TurtleBot3
- **ROS2 Executor**: Local and remote ROS2 command execution
- **Process Monitor**: Real-time monitoring of running ROS2 nodes and processes
- **Configuration Manager**: Settings persistence and profile management

## Target System
- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy Jalopy
- **Qt**: Qt6 (Core, Widgets, Network)
- **Build System**: CMake with ament_cmake

## Key Features
### 1. Quick Launch Tab
- Pre-configured buttons for common TurtleBot3 operations:
  - Robot Bringup (`ros2 launch turtlebot3_bringup robot.launch.py`)
  - Teleoperation (`ros2 run turtlebot3_teleop teleop_keyboard`)
  - SLAM (`ros2 launch turtlebot3_slam slam.launch.py`)
  - Navigation (`ros2 launch turtlebot3_navigation2 navigation2.launch.py`)
  - RViz2 visualization
  - Gazebo simulation
- Quick diagnostic commands (battery status, sensor checks, network diagnostics)
- Real-time output display

### 2. Command Builder Tab
- Dynamic parameter forms based on selected ROS2 packages
- Support for launch files and executable nodes
- Parameter validation and type checking (string, int, double, bool)
- Command preview before execution
- Save/load command templates as JSON
- Built-in command database for TurtleBot3 operations

### 3. Terminal Tab
- SSH connection to remote TurtleBot3
- Interactive terminal with command history
- Support for key-based authentication
- Connection profiles for multiple robots
- Real-time command execution and output

### 4. Monitor Tab
- Tree view of running ROS2 processes
- Process status monitoring (CPU, memory, PID)
- Start/stop/kill process controls
- Node health checking
- Resource usage tracking

### 5. Configuration Tab
- Robot connection settings (IP, username, SSH port)
- ROS2 environment configuration (workspace, domain ID)
- TurtleBot3 model selection (burger, waffle, waffle_pi)
- Settings persistence and profile management

## Technical Implementation

### Core Classes
```cpp
MainWindow          // Main application window with tabbed interface
CommandWidget       // ROS2 command builder with parameter forms
SSHManager         // SSH connection and remote command execution
ROS2Executor       // Local ROS2 process management
ProcessMonitor     // System monitoring and process tracking
ConfigManager      // Settings and configuration persistence
```

### Key Technologies
- **Qt6 Widgets**: Modern GUI with dark theme
- **QProcess**: Process execution and management
- **QTcpSocket**: Network communication for SSH
- **QSettings**: Configuration persistence
- **QTimer**: Periodic status updates
- **QSplitter**: Resizable layout panels

### SSH Implementation
- Uses system SSH client via QProcess
- Support for password and key-based authentication
- Connection health monitoring with periodic checks
- Automatic reconnection on connection loss
- Terminal emulation with command history

### ROS2 Integration
- Environment variable management (ROS_DOMAIN_ID, workspace paths)
- Command validation and parsing
- Process lifecycle management
- Emergency stop functionality
- Remote execution capabilities

## File Structure
```
turtlebot3_controller/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── main.cpp
│   ├── mainwindow.cpp
│   ├── commandwidget.cpp
│   ├── sshmanager.cpp
│   ├── ros2executor.cpp
│   ├── processmonitor.cpp
│   └── configmanager.cpp
├── include/
│   ├── mainwindow.h
│   ├── commandwidget.h
│   ├── sshmanager.h
│   ├── ros2executor.h
│   ├── processmonitor.h
│   └── configmanager.h
└── CLAUDE.md
```

## Build Instructions
```bash
# Create workspace
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

# Clone/create project
# Copy all source files to turtlebot3_controller/

# Install Qt6 dependencies
sudo apt update
sudo apt install qt6-base-dev qt6-tools-dev cmake

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_controller

# Run
source install/setup.bash
ros2 run turtlebot3_controller turtlebot3_controller
```

## Usage Workflow
1. **Initial Setup**: Configure robot IP and credentials in Configuration tab
2. **Connection**: Use "Connect" button to establish SSH connection to TurtleBot3
3. **Quick Operations**: Use Quick Launch tab for common tasks
4. **Custom Commands**: Build custom ROS2 commands in Command Builder tab
5. **Monitoring**: Track running processes and system status in Monitor tab
6. **Terminal Access**: Use Terminal tab for direct command-line access

## Security Considerations
- SSH key-based authentication recommended over passwords
- Connection validation and timeout handling
- Secure credential storage (consider Qt Keychain integration)
- Network connection error handling

## Future Enhancements
- Integration with ROS2 parameter server
- Log file management and analysis
- Multi-robot fleet management
- Custom plugin architecture for additional robot types
- Integration with robot diagnostic tools

## Development Notes
- Use Qt6's modern signal/slot syntax
- Implement proper error handling for all network operations
- Follow Qt coding conventions and memory management
- Consider internationalization (i18n) for multi-language support
- Implement comprehensive logging for debugging

## Dependencies
- Qt6 (Core, Widgets, Network)
- ROS2 Jazzy
- CMake 3.16+
- OpenSSH client
- TurtleBot3 packages

## Testing Strategy
- Unit tests for command parsing and validation
- Integration tests for SSH connectivity
- UI tests for widget interactions
- Network failure simulation tests
- ROS2 environment validation tests