# ROS Synapse

A desktop program to help you launch your robot.

This program seeks to solve the requirement to have several terminal windows open at once with separate command flags and logs to be managed. 

Currently for Linux only

Uses Qt6 and TOML settings

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-green.svg)
![Qt6](https://img.shields.io/badge/Qt-6-brightgreen.svg)

## Future goals

- To use Generative AI to judge the status of the robot in realtime.


## Assumed Enivronment

- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy Jalopy
- **Qt6**: Core, Widgets, Network components
- **CMake**: 3.16 or higher

## Installation

### 1. Install Dependencies

```bash
# Install Qt6 and build tools
sudo apt update
sudo apt install qt6-base-dev qt6-tools-dev cmake

# Install ROS2 dependencies
sudo apt install ros-jazzy-rclcpp ros-jazzy-rcl-interfaces

# Install TOML library
sudo apt install libtomlplusplus-dev
```

### 2. Build the Application

```bash
# Clone the repository
git clone <repository-url>
cd ros_synapse

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Build
mkdir build && cd build
cmake ..
make

# Alternatively, if built as ROS2 package:
# colcon build --packages-select ros_synapse
```

## Usage

### Running the Application

```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Run from build directory
cd build
./ros_synapse
```

### Configuration

Edit `settings.toml` to customize:

```toml
[ui]
dark_mode = true
font_size = 12

[controls]
# Customize dropdown options for each row
row1_dropdown1_options = ["ros2", "rostopic", "rosnode"]
row1_dropdown1_default = "ros2"

[ros2]
workspace = "~/ros2_ws"
domain_id = 0
```

### Testing Log Viewer

Use the included test script to generate sample log messages:

```bash
# Terminal 1: Run ROS Synapse
source /opt/ros/jazzy/setup.bash
./ros_synapse

# Terminal 2: Generate test logs
source /opt/ros/jazzy/setup.bash
python3 test_log_publisher.py
```

## Architecture

### Core Components

- **MainWindow**: Central application hub with tabbed interface
- **CommandWidget**: Dynamic command builder with 3-row control layout  
- **ConfigManager**: TOML-based configuration system
- **ROS2 Integration**: Live log subscription and node management

### File Structure

```
ros_synapse/
├── CMakeLists.txt          # Build configuration
├── package.xml             # ROS2 package manifest
├── settings.toml           # Application configuration
├── src/                    # Source implementations
├── include/                # Header files
├── test_log_publisher.py   # Test script for log generation
└── README.md              # This file
```

## Development

### Code Style

The project follows [Google C++ Style Guide](cpp_coding_standard.markdown) conventions:

- **Variables**: `snake_case` with trailing underscore for members (`member_`)
- **Functions**: `CamelCase` (`OnLogMessageReceived`)
- **Constants**: `k` prefix (`kDefaultTimeout`)
- **Include Guards**: `ROS_SYNAPSE_INCLUDE_FILENAME_H_`

### Adding New Features

1. Update TOML configuration in `settings.toml` if needed
2. Modify relevant classes following the established architecture
3. Ensure ROS2 environment is sourced during development
4. Test with the provided `test_log_publisher.py` script

### Building with ROS2

```bash
# As a ROS2 package (alternative build method)
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros_synapse
source install/setup.bash
ros2 run ros_synapse ros_synapse
```

## Contributing

1. Follow the Google C++ Style Guide
2. Test changes with both standalone and ROS2 package builds
3. Update documentation for new features
4. Ensure compatibility with ROS2 Jazzy

## License

This project is licensed under MIT


