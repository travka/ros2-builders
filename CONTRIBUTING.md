# Contributing to ROS2 Builders

Thank you for your interest in contributing to ROS2 Builders! This document provides guidelines and instructions for contributing to this project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Pull Request Process](#pull-request-process)
- [Coding Standards](#coding-standards)
- [Testing Guidelines](#testing-guidelines)
- [Documentation](#documentation)

## Code of Conduct

This project adheres to a code of conduct. By participating, you are expected to uphold this standard. Please report unacceptable behavior to the repository maintainers.

## Getting Started

### Prerequisites

- Git
- Docker (for Docker image development)
- macOS with Homebrew (for formula development)
- GitHub account with fork permissions

### Setting Up Development Environment

1. **Fork the repository**

   ```bash
   # Fork on GitHub, then clone your fork
   git clone https://github.com/your-username/ros2-builders.git
   cd ros2-builders
   ```

2. **Add upstream remote**

   ```bash
   git remote add upstream https://github.com/travka/ros2-builders.git
   git fetch upstream
   ```

3. **Create a development branch**

   ```bash
   git checkout -b feature/your-feature-name
   ```

### Development Tools

#### Docker Development

```bash
# Build locally for testing
cd builders/docker
./docker-build.sh --platform linux/amd64

# Build and push
./docker-build.sh --platform linux/amd64,linux/arm64 --push --tag v1.0.0
```

#### macOS Formula Development

```bash
# Validate formula syntax
ruby -c builders/macos/ros2-humble.rb

# Audit formula
brew audit --strict builders/macos/ros2-humble.rb

# Test installation
brew install --formula builders/macos/ros2-humble.rb
```

## Development Workflow

1. **Branch Naming**

   Use these branch name prefixes:
   - `feature/` - New features
   - `fix/` - Bug fixes
   - `docs/` - Documentation changes
   - `test/` - Test updates
   - `refactor/` - Code refactoring

2. **Make Your Changes**

   - Follow the coding standards below
   - Add tests for new functionality
   - Update documentation as needed

3. **Commit Your Changes**

   ```bash
   git add .
   git commit -m "type(scope): description"
   ```

   Commit types:
   - `feat` - New feature
   - `fix` - Bug fix
   - `docs` - Documentation
   - `style` - Code style (formatting, etc.)
   - `refactor` - Code refactoring
   - `test` - Adding or updating tests
   - `chore` - Maintenance tasks

4. **Sync with Upstream**

   ```bash
   git fetch upstream
   git rebase upstream/main
   ```

5. **Push and Create Pull Request**

   ```bash
   git push origin feature/your-feature-name
   # Then create PR on GitHub
   ```

## Pull Request Process

### Before Submitting

1. **Run tests locally**

   ```bash
   # Docker tests
   docker build -t ros2-humble-test ./builders/docker

   # macOS formula tests
   ruby -c builders/macos/ros2-humble.rb
   brew audit builders/macos/ros2-humble.rb
   ```

2. **Update documentation**

   - Update README.md if relevant
   - Add/update CHANGELOG.md for user-facing changes
   - Update inline code comments

3. **Check security**

   - No hardcoded secrets or credentials
   - Dependencies are up-to-date
   - Run `trivy` or equivalent scanner

### Pull Request Template

When creating a PR, please include:

- **Description**: What does this PR do?
- **Type**: Feature, bug fix, docs, etc.
- **Changes**: List of files changed
- **Testing**: How did you test this?
- **Screenshots**: For UI changes (if applicable)
- **Related Issues**: Links to related issues

### PR Review Process

1. Automated checks must pass:
   - All CI/CD workflows
   - Security scans
   - CodeQL analysis
   - Linting/formatting

2. At least one maintainer approval required

3. Address review comments promptly

4. Keep PRs focused and small for easier review

## Coding Standards

### Shell Scripts

- Use `#!/bin/bash` with `set -e` for error handling
- Use variables for repeated values
- Add comments for non-obvious logic
- Follow POSIX compatibility where possible

### YAML Files

- Use 2-space indentation
- Quote strings that contain special characters
- Use consistent key ordering
- Add comments for complex workflows

### Dockerfile

- Minimize layers
- Use specific version tags (not `latest`)
- Clean up in the same layer
- Order instructions for caching

### Homebrew Formula

- Follow Homebrew formula style guide
- Use frozen_string_literal: true
- Document non-obvious dependencies
- Include comprehensive test block

### GitHub Actions Workflows

- Use environment variables for repeated values
- Implement proper caching
- Use matrix strategies for parallel builds
- Add clear step names

## Testing Guidelines

### Unit Tests

- Test individual functions and components
- Mock external dependencies
- Test edge cases and error conditions

### Integration Tests

- Test Docker image builds and runs
- Test macOS formula installation
- Test end-to-end workflows

### Automated Tests

All tests run automatically on:
- Push to main branch
- Pull requests
- Scheduled runs (weekly)

### Manual Testing

Before submitting PRs, manually test:
- Docker image functionality
- macOS formula installation
- Documentation accuracy

## Documentation

### When to Update Documentation

- Adding new features
- Changing existing functionality
- Fixing bugs that affect users
- Updating dependencies

### Documentation Files

- `README.md` - Main project documentation
- `CONTRIBUTING.md` - Contribution guidelines (this file)
- `DEVELOPMENT.md` - Development setup and architecture
- `CHANGELOG.md` - Release notes and version history
- Inline code comments - Complex logic explanations

### Documentation Style

- Use clear, concise language
- Provide examples for usage
- Keep sections organized
- Use code blocks for code snippets
- Include diagrams where helpful

## Security

### Reporting Security Issues

- Do not report security issues in public issues
- Email security concerns to the maintainers
- Wait for confirmation before disclosing

### Security Best Practices

- Never commit secrets or credentials
- Use environment variables for sensitive data
- Regularly update dependencies
- Run security scanners before committing
- Review dependencies for known vulnerabilities

## Questions or Need Help?

- Open an issue for bugs or feature requests
- Use GitHub Discussions for questions
- Check existing issues and discussions first

## License

By contributing, you agree that your contributions will be licensed under the Apache-2.0 License.
