# ROS 2 Humble Hawksbill Docker Images

**Image:** `travka/ros2-humble`

## Overview
Official ROS 2 Humble Hawksbill base images with desktop tools, colcon, rosdep, and vcstool. Built and published automatically from GitHub Actions with multi-arch support (linux/amd64, linux/arm64), SBOM generation, and security scanning.

## Tags
- `latest` → default branch build
- `vX.Y.Z` → release tags
- `X.Y` / `X` → semver convenience tags
- `branch` tags (for non-default branches)
- `pr-*` tags (for pull requests)
- `sha-*` tags (per-commit identifiers)

## Included
- ROS 2 Humble desktop
- colcon common extensions
- rosdep, vcstool
- Locale and UTF-8 defaults

## Usage
```bash
# Pull latest stable
docker pull travka/ros2-humble:latest

# Pull a specific release
docker pull travka/ros2-humble:v1.2.3

# Run a shell
docker run -it --rm travka/ros2-humble:latest bash

# Source ROS 2
source /opt/ros/humble/setup.bash
```

## Build & Security
- Multi-arch builds via Buildx (amd64/arm64)
- SBOM generated for releases and uploaded as artifacts
- Vulnerability scans (Trivy/Grype) with gates on HIGH/CRITICAL
- OCI labels populated (title, description, source, license, docs)

## Links
- Source: https://github.com/travka/ros2-builders
- Docker Hub: https://hub.docker.com/r/travka/ros2-humble
- Docs: https://docs.ros.org/en/humble/
