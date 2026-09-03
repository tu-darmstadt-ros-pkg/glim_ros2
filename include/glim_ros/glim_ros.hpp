#pragma once

#include <any>
#include <deque>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#ifdef BUILD_WITH_CV_BRIDGE
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace glim {
class TimeKeeper;
class CloudPreprocessor;
class AsyncOdometryEstimation;
class AsyncSubMapping;
class AsyncGlobalMapping;

class ExtensionModule;
class GenericTopicSubscription;

class GlimROS : public rclcpp::Node {
public:
  GlimROS(const rclcpp::NodeOptions& options);
  ~GlimROS();

  bool needs_wait();
  void timer_callback();

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
#ifdef BUILD_WITH_CV_BRIDGE
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
#endif
  size_t points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void wait(bool auto_quit = false);
  void save(const std::string& path);

  /**
   * @brief Discard the current map and rebuild the mapping pipeline from scratch
   * @note  Equivalent to restarting the node: odometry, local and global mapping are recreated, so
   *        the new map starts empty with its origin at the pose the sensor currently has. Extension
   *        modules stay loaded and are notified via GlimROSCallbacks::on_reset.
   * @return true if the pipeline was rebuilt. If false, GLIM is left inactive (dropping all input)
   *         because the mapping modules could not be reloaded.
   */
  bool reset_pipeline();

  const std::vector<std::shared_ptr<GenericTopicSubscription>>& extension_subscriptions();

private:
  //! Creates the processing modules. Requires GlobalConfig to be initialized.
  bool create_pipeline();
  //! Destroys the processing modules, stopping their worker threads.
  void destroy_pipeline();

  void reset_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

private:
  //! Guards the processing modules against a concurrent reset. Held while they are swapped out.
  std::mutex pipeline_mutex;

  std::unique_ptr<glim::TimeKeeper> time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;

  std::shared_ptr<glim::AsyncOdometryEstimation> odometry_estimation;
  std::unique_ptr<glim::AsyncSubMapping> sub_mapping;
  std::unique_ptr<glim::AsyncGlobalMapping> global_mapping;

  bool keep_raw_points;
  double imu_time_offset;
  double points_time_offset;
  double acc_scale;
  bool dump_on_unload;
  std::string dump_path;

  std::string intensity_field, ring_field;

  // Extension modulles
  std::vector<std::shared_ptr<ExtensionModule>> extension_modules;
  std::vector<std::shared_ptr<GenericTopicSubscription>> extension_subs;

  // ROS-related
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service;
#ifdef BUILD_WITH_CV_BRIDGE
  image_transport::Subscriber image_sub;
#endif
};

}  // namespace glim
