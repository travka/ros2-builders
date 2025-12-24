# Development Guide for ROS2 Builders

This document provides detailed information about the architecture, development setup, and technical implementation details of ROS2 Builders.

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Repository Structure](#repository-structure)
- [Component Details](#component-details)
- [Local Development](#local-development)
- [CI/CD Pipeline](#cicd-pipeline)
- [Release Process](#release-process)
- [Debugging](#debugging)
- [Performance Optimization](#performance-optimization)

## Architecture Overview

ROS2 Builders provides automated build infrastructure for ROS2 packages across multiple platforms:

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS2 Builders                           │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐      ┌──────────────┐      ┌───────────┐ │
│  │   Docker     │      │   macOS      │      │    AUR    │ │
│  │   Builders   │      │   Homebrew   │      │   PKG     │ │
│  │              │      │              │      │           │ │
│  │ Multi-arch   │      │ arm64/amd64  │      │ Arch      │ │
│  │ amd64/arm64  │      │ Formula      │      │ Linux     │ │
│  └──────────────┘      └──────────────┘      └───────────┘ │
│         │                     │                     │       │
│         └─────────────────────┴─────────────────────┘       │
│                           │                                  │
│                    ┌──────▼──────┐                          │
│                    │    CI/CD    │                          │
│                    │  Workflows  │                          │
│                    └──────┬──────┘                          │
│                           │                                  │
│              ┌────────────┼────────────┐                    │
│              │            │            │                    │
│         ┌────▼───┐  ┌────▼───┐  ┌────▼───┐                 │
│         │ Build  │  │  Test  │  │Security│                 │
│         │  Jobs  │  │  Jobs  │  │  Scans │                 │
│         └────────┘  └────────┘  └────────┘                 │
└─────────────────────────────────────────────────────────────┘
```

### Key Design Principles

1. **Multi-Platform Support**: Build for Docker (amd64/arm64), macOS (Homebrew), and Arch Linux (AUR)
2. **Security-First**: Automated vulnerability scanning and SBOM generation
3. **Automated Testing**: Comprehensive testing at all stages
4. **Fast Iteration**: Caching and parallel builds for quick feedback

## Repository Structure

```
ros2-builders/
├── .github/
│   ├── workflows/
│   │   ├── build-docker.yml      # Docker image builds
│   │   ├── build-macos.yml       # macOS formula builds
│   │   ├── build-aur.yml         # AUR package builds
│   │   ├── test-docker.yml       # Docker image tests
│   │   ├── test-macos.yml        # macOS formula tests
│   │   ├── security-scan.yml     # Security scanning
│   │   └── release.yml           # Release automation
│   ├── settings/
│   │   └── repository.yaml       # Repository settings
│   ├── dependabot.yml            # Dependency updates
│   └── CODEOWNERS                # Code ownership
├── builders/
│   ├── docker/
│   │   ├── Dockerfile            # Docker image definition
│   │   ├── README.md             # Docker usage docs
│   │   └── docker-build.sh       # Build script
│   ├── macos/
│   │   ├── ros2-humble.rb        # Homebrew formula
│   │   ├── build.sh              # Build script
│   │   └── README.md             # macOS usage docs
│   ├── aur/
│   │   ├── PKGBUILD              # Arch package definition
│   │   ├── .SRCINFO              # Package metadata
│   │   └── README.md             # AUR usage docs
│   └── scripts/
│       └── release.sh            # Release automation
├── .trivyignore                  # Trivy scan exclusions
├── CHANGELOG.md                  # Version history
├── CONTRIBUTING.md               # Contribution guidelines
├── DEVELOPMENT.md                # This file
├── LICENSE                       # Apache-2.0 license
└── README.md                     # Main documentation
```

## Component Details

### Docker Builder

#### Dockerfile Structure

```dockerfile
ARG ROS_DISTRO=humble
ARG UBUNTU_VERSION=jammy
FROM ubuntu:${UBUNTU_VERSION}

# Base image configuration
ARG ROS_DISTRO=humble

# Labels for metadata
LABEL maintainer="travka <https://github.com/travka>"
LABEL org.opencontainers.image.source="https://github.com/travka/ros2-builders"

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# ROS2 installation
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

WORKDIR /root
```

#### Build Script (`docker-build.sh`)

The build script provides:
- Multi-platform build support (`--platform`)
- Push to Docker Hub (`--push`)
- Custom tagging (`--tag`)
- BuildKit integration

### macOS Homebrew Builder

#### Formula Structure (`ros2-humble.rb`)

```ruby
# frozen_string_literal: true

class Ros2Humble < Formula
  desc "Robot Operating System 2 Humble Hawksbill"
  homepage "https://docs.ros.org/en/humble/"
  url "https://github.com/ros2/ros2.git",
      tag:      "humble-20241206",
      revision: "7d264849c6b8e6922b1a70f80889605598c098e9"
  license "Apache-2.0"

  # Dependencies
  depends_on "cmake" => :build
  depends_on "python@3.11"
  depends_on "openssl@3"
  # ... more dependencies

  # Install method
  def install
    # Installation logic
  end

  # Test block
  test do
    # Test logic
  end
end
```

### AUR Builder

#### PKGBUILD Structure

```bash
pkgname=ros2-humble
pkgver=20241206
pkgrel=1
pkgdesc="Robot Operating System 2 - Humble Hawksbill"
arch=('x86_64' 'aarch64')
url="https://docs.ros.org/en/humble/"
license=('Apache')
depends=('python' 'cmake' 'rosdep' 'colcon-common-extensions')
# ... more package definition

package() {
    # Packaging logic
}
```

## Local Development

### Prerequisites

#### For Docker Development

```bash
# Install Docker
# macOS: Download Docker Desktop
# Linux: sudo apt install docker.io docker-compose

# Install Buildx for multi-arch builds
docker buildx install

# Verify installation
docker buildx version
```

#### For macOS Development

```bash
# Install Homebrew
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install required tools
brew install cmake python@3.11
```

### Building Locally

#### Docker

```bash
# Build for current platform
cd builders/docker
./docker-build.sh

# Build for specific platform
./docker-build.sh --platform linux/arm64

# Build for multiple platforms
./docker-build.sh --platform linux/amd64,linux/arm64

# Build with custom tag
./docker-build.sh --tag my-custom-tag

# Build and push
./docker-build.sh --push
```

#### macOS

```bash
# Validate formula
ruby -c builders/macos/ros2-humble.rb

# Audit formula
brew audit --strict builders/macos/ros2-humble.rb

# Test installation
brew install --formula builders/macos/ros2-humble.rb

# Create bottle
cd $(brew --repo)
brew bottle --no-rebuild ./builders/macos/ros2-humble.rb
```

### Testing Locally

#### Docker Tests

```bash
# Build test image
docker build -t ros2-humble-test ./builders/docker

# Run ROS2 command
docker run --rm ros2-humble-test ros2 --version

# Test workspace
docker run -v $PWD/test_ws:/ros2_ws/src ros2-humble-test
```

#### macOS Tests

```bash
# Run formula tests
brew test ros2-humble

# Manual verification
ros2 --version
colcon --version
python3 -c "import rclpy; print('rclpy imported successfully')"
```

## CI/CD Pipeline

### Workflow Triggers

| Workflow | Triggers |
|----------|----------|
| `build-docker.yml` | Push to main, tags, PRs |
| `build-macos.yml` | Push to main, PRs |
| `test-docker.yml` | Push to main, changes to docker files |
| `test-macos.yml` | Push to main, changes to macOS files |
| `security-scan.yml` | Push, PRs, weekly schedule |
| `release.yml` | Tag push (v\*.\*.\*) |

### Caching Strategy

#### Docker Layer Caching

```yaml
- name: Cache Docker layers
  uses: actions/cache@v4
  with:
    path: /tmp/.buildx-cache
    key: ${{ runner.os }}-buildx-${{ github.sha }}
    restore-keys: |
      ${{ runner.os }}-buildx-
```

#### Homebrew Cache

```yaml
- name: Set up Homebrew cache
  uses: actions/cache@v4
  with:
    path: |
      ~/Library/Caches/Homebrew
      ~/.cache/Homebrew
    key: ${{ runner.os }}-homebrew-${{ hashFiles('**/ros2-humble.rb') }}
```

### Parallel Builds

Docker builds use matrix strategy for parallel execution:

```yaml
strategy:
  matrix:
    platform: [linux/amd64, linux/arm64]
  fail-fast: false
```

## Release Process

### Version Tagging

```bash
# Tag version
git tag v1.0.0

# Push tag
git push origin v1.0.0
```

### Automated Release Steps

1. **GitHub Release Created**
   - Auto-generated notes
   - Attached SBOM files
   - Links to Docker images

2. **Docker Images Built**
   - Multi-platform builds
   - Versioned and latest tags
   - Pushed to Docker Hub

3. **Homebrew Formula Updated**
   - Version bump
   - Committed to tap repository
   - Bottles created

4. **Notifications**
   - Release summary
   - Installation instructions

### Release Checklist

- [ ] All CI/CD tests pass
- [ ] Security scans clean
- [ ] CHANGELOG.md updated
- [ ] Documentation updated
- [ ] Version bumped appropriately
- [ ] No breaking changes without warning

## Debugging

### Debugging Docker Builds

```bash
# Enable BuildKit debugging
BUILDKIT_STEP_LOG_MAX_SIZE=10000000 docker buildx build ...

# Interactive debugging
docker run -it --rm ros2-humble:test bash

# View build logs
docker buildx build --progress=plain ...
```

### Debugging GitHub Actions

```bash
# Enable step debugging
# Add to workflow:
# env:
#   ACTIONS_STEP_DEBUG: true
#   ACTIONS_RUNNER_DEBUG: true

# Use tmate for SSH access
- name: Setup tmate session
  uses: mxschmitt/action-tmate@v3
```

### Debugging Formula Installation

```bash
# Verbose installation
brew install --verbose --debug ./builders/macos/ros2-humble.rb

# Check logs
brew config
brew doctor
```

## Performance Optimization

### Build Time Optimization

1. **Use Layer Caching**: Order Dockerfile instructions for optimal caching
2. **Parallel Builds**: Matrix strategy for multi-platform builds
3. **Dependency Caching**: Cache pip, apt, and Homebrew packages
4. **Selective Rebuilds**: Only rebuild changed components

### Image Size Optimization

1. **Multi-stage builds**: Separate build and runtime environments
2. **Cleanup in same layer**: Remove unnecessary files immediately
3. **Use .dockerignore**: Exclude unnecessary files from build context
4. **Minimal base images**: Use alpine or slim variants where possible

### CI/CD Optimization

1. **Conditional execution**: Skip unnecessary jobs
2. **Artifact caching**: Reuse build artifacts
3. **Incremental builds**: Build only what changed
4. **Resource limits**: Configure runner resources appropriately

## Additional Resources

- [ROS2 Documentation](https://docs.ros.org/)
- [Docker Documentation](https://docs.docker.com/)
- [Homebrew Formula Cookbook](https://docs.brew.sh/Formula-Cookbook)
- [Arch Linux Packaging Guidelines](https://wiki.archlinux.org/title/Arch_package_guidelines)
- [GitHub Actions Documentation](https://docs.github.com/actions)
