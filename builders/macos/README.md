# macOS Builders

Homebrew formula for ROS2 Humble on macOS.

## Quick Start

### Install from Tap

```bash
brew tap travka/ros2
brew install ros2-humble
```

### Build from Source

```bash
./build.sh
```

## Requirements

- macOS 11+ (Big Sur or later)
- Apple Silicon (M1/M2/M3) or Intel
- Homebrew
- Xcode Command Line Tools

## Dependencies

- CMake
- Python 3.11
- PyYAML
- TinyXML2
- Eigen
- Console Bridge
- OpenSSL 3
- spdlog
- assimp
- bullet

## Usage

After installation, source the ROS2 environment:

```bash
source /opt/ros/humble/setup.bash
```

Or add to your shell profile:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.zshrc
```

## Known Issues

- ROS2 on macOS is experimental
- Some features may not work due to platform differences
- Performance may be lower than on Linux
