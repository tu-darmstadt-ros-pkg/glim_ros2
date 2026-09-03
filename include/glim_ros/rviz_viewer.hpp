#pragma once

#include <any>
#include <atomic>
#include <thread>
#include <chrono>

#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>

namespace spdlog {
class logger;
}

namespace glim {

class TrajectoryManager;

/**
 * @brief Rviz-based viewer
 */
class RvizViewer : public ExtensionModuleROS2 {
public:
  RvizViewer();
  ~RvizViewer();

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;

private:
  void set_callbacks();
  //! Drops the trajectory and map cached for a pipeline that was reset (see GlimROSCallbacks::on_reset)
  void reset_state();
  //! Replaces the latched ~/map with an empty cloud so RViz drops the previous map immediately
  void publish_empty_map();
  bool update_T_imu_base();
  void odometry_new_frame(const EstimationFrame::ConstPtr& new_frame, bool corrected);
  void submap_on_new_keyframe(int id, const EstimationFrame::ConstPtr& keyframe);
  void submap_on_new_submap(const SubMap::ConstPtr& submap);
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps);
  void publish_global_map_if_due();
  void invoke(const std::function<void()>& task);

  void spin_once();

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Time last_globalmap_pub_time;

  std::string imu_frame_id;
  std::string lidar_frame_id;
  std::string base_frame_id;
  std::string odom_frame_id;
  std::string map_frame_id;
  bool publish_imu2lidar;
  // If true, publish odom/map at the first base_frame pose (Athena/FAST-LIO style)
  // instead of the first IMU pose. Requires a TF from imu_frame_id to base_frame_id.
  bool base_centric_frames;
  std::atomic_bool T_imu_base_valid{false};
  Eigen::Isometry3d T_imu_base = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_base_imu = Eigen::Isometry3d::Identity();
  double tf_time_offset;
  int global_map_pub_interval;
  int global_map_pub_n_points;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_pub;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_points_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_scanend_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_pub;  // Odometry in LiDAR frame
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_scanend_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_scanend_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> pose_corrected_with_cov_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> pose_scanend_corrected_with_cov_pub;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_pub;  // Pose in LiDAR frame
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_scanend_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> lidar_pose_corrected_with_cov_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>> lidar_pose_scanend_corrected_with_cov_pub;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> imu_bias_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>> imu_bias_with_cov_pub;

  std::mutex trajectory_mutex;
  std::unique_ptr<TrajectoryManager> trajectory;

  std::vector<gtsam_points::PointCloud::ConstPtr> submaps;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> submap_poses;

  // Keyframes of the unfinished submap (published as a provisional map until the submap completes)
  std::vector<gtsam_points::PointCloud::ConstPtr> pending_keyframe_frames;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> pending_keyframe_poses;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};
}  // namespace glim