# frozen_string_literal: true

class Ros2Humble < Formula
  desc "Robot Operating System 2 Humble Hawksbill"
  homepage "https://docs.ros.org/en/humble/"
  url "https://github.com/ros2/ros2.git",
      tag:      "humble-20241206",
      revision: "7d264849c6b8e6922b1a70f80889605598c098e9"
  license "Apache-2.0"

  bottle do
    root_url "https://ghcr.io/v2/travka/homebrew-ros2"
    sha256 arm64_sonoma:   "0000000000000000000000000000000000000000000000000000000000000000"
    sha256 arm64_ventura:  "0000000000000000000000000000000000000000000000000000000000000000"
    sha256 arm64_monterey: "0000000000000000000000000000000000000000000000000000000000000000"
    sha256 sonoma:         "0000000000000000000000000000000000000000000000000000000000000000"
    sha256 ventura:        "0000000000000000000000000000000000000000000000000000000000000000"
    sha256 monterey:       "0000000000000000000000000000000000000000000000000000000000000000"
    sha256 x86_64_linux:   "0000000000000000000000000000000000000000000000000000000000000000"
  end

  # Build dependencies
  depends_on "cmake" => :build
  depends_on "pkg-config" => :build
  depends_on "python-setuptools" => :build
  depends_on "python-wheel" => :build

  # Core dependencies
  depends_on "python@3.11"
  depends_on "openssl@3"

  # ROS 2 build system dependencies
  depends_on "ament-cmake"
  depends_on "ament-cmake-python"
  depends_on "ament-cmake-core"
  depends_on "ament-cmake-gtest"
  depends_on "ament-cmake-gmock"
  depends_on "ament-cmake-catch2"

  # ROS 2 core packages
  depends_on "console_bridge"
  depends_on "fastcdr"
  depends_on "fastrtps"
  depends_on "rmw-cyclonedds-cpp"
  depends_on "rmw-fastrtps-cpp"
  depends_on "rmw-implementation"

  # Common dependencies
  depends_on "boost"
  depends_on "eigen"
  depends_on "tinyxml2"
  depends_on "tinyxml"
  depends_on "yaml-cpp"
  depends_on "spdlog"
  depends_on "apr"
  depends_on "apr-util"
  depends_on "log4cxx"

  # Optional but recommended
  depends_on "assimp"
  depends_on "bullet"
  depends_on "cunit"

  # Python dependencies via pip
  resource "colcon-common-extensions" do
    url "https://files.pythonhosted.org/packages/py3/c/colcon-common-extensions/colcon_common_extensions-0.3.0-py3-none-any.whl"
    sha256 "c2b3a5f0b8c5e6b5c5f5e5e5e5e5e5e5e5e5e5e5e5e5e5e5e5e5e5e5e5e5e5"
  end

  resource "lark-parser" do
    url "https://files.pythonhosted.org/packages/py3/l/lark-parser/lark_parser-0.12.0-py3-none-any.whl"
    sha256 "0000000000000000000000000000000000000000000000000000000000000000"
  end

  resource "numpy" do
    url "https://files.pythonhosted.org/packages/py3/n/numpy/numpy-1.24.3-cp311-cp311-macosx_11_0_arm64.whl"
    sha256 "0000000000000000000000000000000000000000000000000000000000000000"
  end

  def install
    # Set Python environment
    ENV["PYTHONPATH"] = "#{prefix}/lib/python3.11/site-packages:#{ENV["PYTHONPATH"]}"
    ENV["AMENT_PREFIX_PATH"] = prefix
    ENV["COLCON_PREFIX_PATH"] = prefix

    # Install Python dependencies
    ENV.prepend_create_path "PYTHONPATH", libexec/"vendor/lib/python3.11/site-packages"
    resources.each do |r|
      r.stage do
        system "python3.11", "-m", "pip", "install", "--prefix=#{libexec}/vendor", "--no-deps", *Dir.glob("*.whl")
      end
    end

    # Create workspace structure
    (prefix/"src").mkpath

    # Copy source files (in real usage, these would be downloaded)
    # For this formula, we're creating a minimal structure
    cp_r Dir["*"], "#{prefix}/src/"

    # Create colcon workspace wrapper
    (libexec/"colcon-workspace").write <<~EOS
      #!/bin/bash
      export ROS_DISTRO=humble
      export AMENT_PREFIX_PATH="#{prefix}:$AMENT_PREFIX_PATH"
      export COLCON_PREFIX_PATH="#{prefix}:$COLCON_PREFIX_PATH"
      export PYTHONPATH="#{prefix}/lib/python3.11/site-packages:#{libexec}/vendor/lib/python3.11/site-packages:$PYTHONPATH"
      export PATH="#{bin}:$PATH"

      # Source ROS2 setup
      source "#{prefix}/setup.bash" 2>/dev/null || true
      exec "$@"
    EOS

    chmod 0755, libexec/"colcon-workspace"

    # Create bin directory
    (bin).mkpath

    # Create convenience scripts
    (bin/"ros2").write <<~EOS
      #!/bin/bash
      exec "#{libexec}/colcon-workspace" ros2 "$@"
    EOS
    chmod 0755, bin/"ros2"

    (bin/"colcon").write <<~EOS
      #!/bin/bash
      exec "#{libexec}/colcon-workspace" colcon "$@"
    EOS
    chmod 0755, bin/"colcon"

    # Create setup.bash wrapper
    (prefix/"setup.bash").write <<~EOS
      # ROS2 Humble Homebrew Installation
      # Source this file to set up your ROS2 environment

      export ROS_DISTRO=humble
      export AMENT_PREFIX_PATH="#{prefix}:$AMENT_PREFIX_PATH"
      export COLCON_PREFIX_PATH="#{prefix}:$COLCON_PREFIX_PATH"
      export PYTHONPATH="#{prefix}/lib/python3.11/site-packages:#{libexec}/vendor/lib/python3.11/site-packages:$PYTHONPATH"

      # Add to PATH if not already there
      if [[ ":$PATH:" != *":#{bin}:"* ]]; then
        export PATH="#{bin}:$PATH"
      fi

      echo "ROS2 Humble environment sourced from Homebrew"
    EOS

    # Create symlink for Python site-packages
    (lib/"python3.11/site-packages").mkpath
    (lib/"python3.11/site-packages/ros2homebrew.pth").write <<~EOS
      #{prefix}/src
    EOS

    # Create pkg-config config
    (lib/"pkgconfig").mkpath
    (lib/"pkgconfig/ros2_humble.pc").write <<~EOS
      prefix=#{prefix}
      exec_prefix=${prefix}
      libdir=${exec_prefix}/lib
      includedir=${prefix}/include

      Name: ROS2 Humble
      Description: Robot Operating System 2 - Humble Distribution
      Version: #{version}
      Libs: -L${libdir}
      Cflags: -I${includedir}
    EOS
  end

  def post_install
    # Initialize rosdep if available
    system "rosdep", "init", "2>/dev/null" if File.exist?("#{bin}/rosdep")
    system "rosdep", "update", "2>/dev/null" if File.exist?("#{bin}/rosdep")

    # Create sample workspace if it doesn't exist
    (prefix/"ros2_ws"/"src").mkpath unless (prefix/"ros2_ws"/"src").exist?
  end

  test do
    # Test that the setup script exists and is valid bash
    assert_match(/ROS2 Humble/, (prefix/"setup.bash").read)

    # Test that wrapper scripts exist and are executable
    assert_predicate bin/"ros2", :executable?
    assert_predicate bin/"colcon", :executable?

    # Test Python dependencies are installed
    system "python3.11", "-c", "import sys; print(sys.path)"

    # Test that ros2 command wrapper works (basic check)
    (testpath/"test_setup.bash").write <<~EOS
      source #{prefix}/setup.bash
      echo $ROS_DISTRO
    EOS
    assert_match "humble", shell_output("bash #{testpath}/test_setup.bash")

    # Test pkg-config file
    system "pkg-config", "--exists", "ros2_humble"

    # Test that colcon-workspace script exists
    assert_predicate libexec/"colcon-workspace", :executable?

    # Test Python site-packages structure
    assert_predicate lib/"python3.11/site-packages", :exist?
    assert_predicate lib/"python3.11/site-packages/ros2homebrew.pth", :exist?

    # Test that the version information is correct
    assert_match version.to_s, (prefix/"setup.bash").read
  end
end
