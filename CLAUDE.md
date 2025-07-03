# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS Synapse is a Qt6-based launcher and AI ROS log monitor application that provides a modern GUI for ROS2 command execution, SSH management, and process monitoring. The application is designed as a standalone Qt6 application (not a ROS2 package) with TOML-based configuration.

## Build and Development Commands

### Prerequisites
```bash
sudo apt install qt6-base-dev qt6-tools-dev cmake libtomlplusplus-dev
```

### Build Process
```bash
# Build the application
mkdir build && cd build
cmake ..
make

# Run the application (from build directory)
./ros_synapse
```

### Development Workflow
- The application executable runs from the `build/` directory
- Configuration file `settings.toml` is located in the project root and accessed via `../settings.toml` from the executable
- No ROS2 workspace or colcon build required - this is a standalone Qt6 application

## Architecture Overview

### Core Component Architecture
The application follows a Qt6 MVC-style architecture with distinct component responsibilities:

- **MainWindow**: Central hub managing the tabbed interface and coordinating between components
- **CommandWidget**: Dynamic UI generator for ROS2 commands with 3-row control layout
- **ConfigManager**: TOML-based configuration system using tomlplusplus library
- **SSHManager**: Remote connection handling with automatic path resolution
- **ROS2Executor**: Process management for local/remote ROS2 command execution
- **ProcessMonitor**: Real-time system monitoring with tree widget display

### Configuration System
The application uses a hybrid configuration approach:
- **TOML Configuration** (`settings.toml`): Primary configuration for UI controls, dropdown options, and application settings
- **Qt Settings**: Fallback system for legacy configuration persistence
- **ConfigManager**: Bridges TOML parsing with Qt application components

Key TOML structure:
```toml
[controls]
row{N}_dropdown{M}_options = ["option1", "option2"]
row{N}_dropdown{M}_default = "default_value"

[ui]
dark_mode = true
font_size = 12

[ros2]
workspace = "~/ros2_ws"
domain_id = 0
```

### Widget Communication Pattern
Components communicate through Qt's signal/slot mechanism:
- `CommandWidget` emits `CommandReady(QString)` when user builds commands
- `SSHManager` emits connection state changes to update UI
- `ConfigManager` provides centralized TOML configuration access

## Code Style and Standards

The project follows Google C++ Style Guide conventions:
- **Naming**: `snake_case` for variables with trailing underscore for members (`member_`), `CamelCase` for functions and classes
- **Include Guards**: Format `ROS_SYNAPSE_INCLUDE_FILENAME_H_`
- **File Structure**: Headers in `include/`, implementations in `src/`
- **Constants**: Use `k` prefix (`kDefaultTimeout`)

## Key Implementation Details

### TOML Integration
- Uses tomlplusplus library for configuration parsing
- ConfigManager handles file path resolution (`../settings.toml` from build directory)
- Graceful fallback when TOML file not found
- Type-safe value extraction with exception handling

### Dynamic UI Generation
CommandWidget creates control rows programmatically:
- Reads dropdown options from TOML configuration
- Populates editable QComboBox widgets with default values
- Connects button signals using lambda captures for row identification

### Build System Integration
CMakeLists.txt includes:
- Qt6 with MOC/UIC automatic processing
- tomlplusplus library via PkgConfig
- Proper header inclusion and library linking
- C++17 standard requirement

## Important File Locations

- `settings.toml`: Primary configuration file (must be in project root)
- `include/`: All header files with class definitions
- `src/`: Implementation files
- `build/`: Build output directory (executable location)

## Common Development Scenarios

When modifying control layouts: Update both the TOML configuration structure and the corresponding parsing logic in ConfigManager.

When adding new UI components: Follow the established pattern of header declaration, implementation in src/, and integration through MainWindow's tabbed interface.

When working with TOML configuration: Remember the file path resolution from build directory and ensure proper exception handling for missing keys or malformed data.