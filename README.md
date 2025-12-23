# ROS2 Builders

Automated build infrastructure for ROS2 Humble packages on macOS (Homebrew) and Docker (multi-arch).

[![CodeQL](https://github.com/travka/ros2-builders/actions/workflows/security-scan.yml/badge.svg)](https://github.com/travka/ros2-builders/actions/workflows/security-scan.yml)
[![Docker](https://github.com/travka/ros2-builders/actions/workflows/build-docker.yml/badge.svg)](https://github.com/travka/ros2-builders/actions/workflows/build-docker.yml)

## Overview

This repository provides CI/CD pipelines to build and distribute ROS2 packages:

- **Docker**: Multi-platform images (ARM64/AMD64) pushed to [Docker Hub](https://hub.docker.com/u/travka)
- **macOS**: Homebrew formula for easy installation on Apple Silicon

## Features

### Build Targets

| Platform | Status | Architecture |
|----------|--------|--------------|
| Docker Hub (travka/ros2-humble) | ✅ | arm64, amd64 |
| macOS Homebrew (travka/ros2) | 🚧 | arm64, amd64 |

### Security (Free Tools)

| Tool | Purpose | Scan Target |
|------|---------|-------------|
| [CodeQL](https://codeql.github.com/) | Static analysis | Source code |
| [Trivy](https://aquasecurity.github.io/trivy/) | Vulnerability scan | Containers, files |
| [Grype](https://github.com/anchore/grype) | Vulnerability scan | Containers |
| [Semgrep](https://semgrep.dev/) | Custom SAST rules | Source code |
| [Syft](https://github.com/anchore/syft) | SBOM generation | Containers |
| [Dependabot](https://docs.github.com/en/code-security/dependabot) | Dependency updates | Package files |

## Quick Start

### Docker Images

Pull and run ROS2 Humble:

```bash
# ARM64 (Apple Silicon)
docker pull travka/ros2-humble:arm64
docker run -it --rm travka/ros2-humble:arm64

# AMD64
docker pull travka/ros2-humble:amd64
docker run -it --rm travka/ros2-humble:amd64
```

### macOS Homebrew

Add tap and install:

```bash
brew tap travka/ros2
brew install ros2-humble
```

## Repository Structure

```
ros2-builders/
├── builders/
│   ├── docker/           # Docker build files
│   ├── macos/            # macOS Homebrew formula
│   └── scripts/          # Utility scripts
├── .github/
│   └── workflows/        # GitHub Actions CI/CD
├── .trivyignore          # Trivy exclusions
└── README.md
```

## Building Locally

### Docker

```bash
cd builders/docker
./docker-build.sh --platform linux/arm64
```

### macOS

```bash
cd builders/macos
./build.sh
```

## CI/CD Pipeline

### Workflows

- `build-docker.yml`: Build and push Docker images
- `build-macos.yml`: Validate Homebrew formula
- `security-scan.yml`: Run all security checks
- `release.yml`: Create releases with Git tags

### Security Gates

All builds must pass:
- ✅ CodeQL analysis
- ✅ Trivy vulnerability scan
- ✅ No high/critical CVEs
- ✅ Dependabot checks green

### Branch Protection

- `main` branch requires:
  - Pull request reviews
  - Status checks pass
  - No direct pushes

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

### Pull Request Requirements

- All security scans pass
- CODEOWNERS approve
- Dependabot updates merged

## Releases

Releases are automated via Git tags:

```bash
git tag v1.0.0
git push origin v1.0.0
```

Triggered actions:
- Build Docker images
- Update Homebrew formula version
- Generate SBOM
- Create GitHub release

## License

Apache-2.0

## Support

- Issues: [GitHub Issues](https://github.com/travka/ros2-builders/issues)
- Discussions: [GitHub Discussions](https://github.com/travka/ros2-builders/discussions)
