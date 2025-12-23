# frozen_string_literal: true

class Ros2Humble < Formula
  desc "Robot Operating System 2 Humble Hawksbill"
  homepage "https://docs.ros.org/en/humble/"
  url "https://github.com/ros2/ros2.git",
      tag:      "rolling",
      revision: "0000000000000000000000000000000000000000"
  license "Apache-2.0"

  depends_on "cmake"
  depends_on "python@3.11"
  depends_on "pyyaml"
  depends_on "tinyxml2"
  depends_on "eigen"
  depends_on "console_bridge"
  depends_on "openssl@3"
  depends_on "spdlog"
  depends_on "assimp"
  depends_on "bullet"

  # ROS 2 dependencies
  resource "ament_cmake" do
    url "https://github.com/ament/ament_cmake/archive/refs/tags/humble.tar.gz"
    sha256 "0000000000000000000000000000000000000000000000000000000000000000"
  end

  resource "rcutils" do
    url "https://github.com/ros2/rcutils/archive/refs/tags/humble.tar.gz"
    sha256 "0000000000000000000000000000000000000000000000000000000000000000"
  end

  def install
    ENV["PYTHONPATH"] = "#{prefix}/lib/python3.11/site-packages:#{ENV["PYTHONPATH"]}"

    mkdir_p "#{prefix}/src/ros2"
    cp_r Dir["*"], "#{prefix}/src/ros2"

    # Install ROS 2 workspace
    cd prefix do
      system "cmake", "-S", "src/ros2", "-B", "build", *std_cmake_args
      system "cmake", "--build", "build"
      system "cmake", "--install", "build"
    end
  end

  test do
    system "python3.11", "-c", "import rclpy"
    system "#{bin}/ros2", "--version"
  end
end
