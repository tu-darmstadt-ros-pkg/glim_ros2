#include <iostream>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim_ros/utils.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto glim = std::make_shared<glim::GlimROS>(options);

  // Get, validate and log dump path
  std::string dump_path = "/tmp/dump";
  glim->declare_parameter<std::string>("dump_path", dump_path);
  glim->get_parameter<std::string>("dump_path", dump_path);
  std::string dump_path_timestamped;
  dump_path_timestamped = glim_ros::create_timestamped_dir(dump_path);
  spdlog::info("dump_path: {}", dump_path);
  spdlog::info("dump_path_timestamped: {}", dump_path_timestamped);

  rclcpp::spin(glim);

  rclcpp::shutdown();

  glim->wait();
  glim->save(dump_path_timestamped);

  return 0;
}