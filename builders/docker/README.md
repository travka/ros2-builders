# Docker Builders

Multi-platform ROS2 Humble Docker images for ARM64 and AMD64.

## Quick Start

### Build locally
```bash
./docker-build.sh --platform linux/arm64
```

### Build and push to Docker Hub
```bash
./docker-build.sh --push --tag v1.0.0
```

### Build multi-platform
```bash
./docker-build.sh --platform linux/arm64,linux/amd64 --push
```

## Images

| Tag | Platform | Description |
|-----|----------|-------------|
| `latest` | arm64, amd64 | Latest stable release |
| `arm64` | arm64 | Apple Silicon optimized |
| `amd64` | amd64 | Intel/AMD optimized |

## Docker Hub

https://hub.docker.com/r/travka/ros2-humble
