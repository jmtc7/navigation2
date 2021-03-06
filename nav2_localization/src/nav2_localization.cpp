// Copyright (c) 2021 Khaled SAAD and Jose M. TORRES-CAMARA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <random>
#include <memory>  // For make_shared<>
#include "nav2_localization/nav2_localization.hpp"
#include "nav2_util/string_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "lifecycle_msgs/msg/state.hpp"

namespace nav2_localization
{

LocalizationServer::LocalizationServer()
: nav2_util::LifecycleNode("localization_server", "", true),
  initial_pose_set_(false),
  sample_motion_model_loader_("nav2_localization", "nav2_localization::SampleMotionModel"),
  default_sample_motion_model_id_("DiffDriveOdomMotionModel"),
  matcher2d_loader_("nav2_localization", "nav2_localization::Matcher2d"),
  default_matcher2d_id_("LikelihoodFieldMatcher2d"),
  solver_loader_("nav2_localization", "nav2_localization::Solver"),
  default_solver_id_("MCLSolver2d"),
  default_types_{
    "nav2_localization::DiffDriveOdomMotionModel",
    "nav2_localization::LikelihoodFieldMatcher2d",
    "nav2_localization::MCLSolver2d"}
{
  RCLCPP_INFO(get_logger(), "Creating localization server");

  declare_parameter("sample_motion_model_id", default_sample_motion_model_id_);
  declare_parameter("matcher2d_id", default_matcher2d_id_);
  declare_parameter("solver_id", default_solver_id_);
  declare_parameter("scan_topic", "scan");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("map_frame_id", "map");
  declare_parameter("transform_tolerance", 1.0);
  declare_parameter("localization_plugins", default_ids_);
}

LocalizationServer::~LocalizationServer()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
LocalizationServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring localization interface");

  double temp;

  get_parameter("sample_motion_model_id", sample_motion_model_id_);
  get_parameter("matcher2d_id", matcher2d_id_);
  get_parameter("solver_id", solver_id_);
  get_parameter("scan_topic", scan_topic_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("map_frame_id", map_frame_id_);
  get_parameter("transform_tolerance", temp);
  transform_tolerance_ = tf2::durationFromSec(temp);

  initTransforms();
  initMessageFilters();
  initPubSub();
  initPlugins();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationServer::on_activate(const rclcpp_lifecycle::State & state)
{
  sample_motion_model_->activate();
  matcher2d_->activate();
  solver_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  initial_pose_sub_.reset();

  // Scan
  scan_connection_.disconnect();
  scan_filter_.reset();
  scan_sub_.reset();

  // Transforms
  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  sample_motion_model_->cleanup();
  matcher2d_->cleanup();
  solver_->cleanup();

  return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn
LocalizationServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
LocalizationServer::mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "A new map was received.");
  if (!first_map_received_) {
    matcher2d_->setMap(msg);
    first_map_received_ = true;
  }
}

void
LocalizationServer::initTransforms()
{
  // Initialize transform listener and broadcaster
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    rclcpp_node_->get_node_base_interface(),
    rclcpp_node_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);
}


void
LocalizationServer::initMessageFilters()
{
  // LaserScan msg
  laser_scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
    rclcpp_node_.get(), scan_topic_, rmw_qos_profile_sensor_data);

  laser_scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10, rclcpp_node_);

  laser_scan_connection_ = laser_scan_filter_->registerCallback(
    std::bind(
      &LocalizationServer::laserReceived,
      this, std::placeholders::_1));

  // PointCloud msg
  scan_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
    rclcpp_node_.get(), scan_topic_, rmw_qos_profile_sensor_data);

  scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
    *scan_sub_, *tf_buffer_, odom_frame_id_, 10, rclcpp_node_);

  scan_connection_ = scan_filter_->registerCallback(
    std::bind(
      &LocalizationServer::scanReceived,
      this, std::placeholders::_1));
}

void
LocalizationServer::initPubSub()
{
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&LocalizationServer::mapReceived, this, std::placeholders::_1));

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&LocalizationServer::initialPoseReceived, this, std::placeholders::_1));
}

void
LocalizationServer::initPlugins()
{
  auto node = shared_from_this();

  get_parameter("localization_plugins", localization_ids_);
  if (localization_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  try {
    sample_motion_model_type_ = nav2_util::get_plugin_type_param(node, sample_motion_model_id_);
    sample_motion_model_ =
      sample_motion_model_loader_.createUniqueInstance(sample_motion_model_type_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(),
      "Failed to create sample motion model. Exception: %s",
      ex.what());
    exit(-1);
  }

  sample_motion_model_->configure(node);

  try {
    matcher2d_type_ = nav2_util::get_plugin_type_param(node, matcher2d_id_);
    matcher2d_ = matcher2d_loader_.createUniqueInstance(matcher2d_type_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(get_logger(), "Failed to create matcher2d. Exception: %s", ex.what());
    exit(-1);
  }

  matcher2d_->configure(node);

  try {
    solver_type_ = nav2_util::get_plugin_type_param(node, solver_id_);
    solver_ = solver_loader_.createUniqueInstance(solver_type_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(get_logger(), "Failed to create solver. Exception: %s", ex.what());
    exit(-1);
  }

  solver_->configure(
    node,
    sample_motion_model_,
    matcher2d_,
    geometry_msgs::msg::TransformStamped(),
    geometry_msgs::msg::TransformStamped());
}

void
LocalizationServer::initialPoseReceived(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  solver_->initFilter(msg);
  initial_pose_set_ = true;
}

void
LocalizationServer::laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
  sensor_msgs::msg::PointCloud2 scan;
  laser_to_pc_projector_.projectLaser(*laser_scan, scan);
  std::shared_ptr<sensor_msgs::msg::PointCloud2> scan_ptr;
  scan_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(scan);
  scanReceived(scan_ptr);
}

void
LocalizationServer::scanReceived(sensor_msgs::msg::PointCloud2::ConstSharedPtr scan)
{
  // Since the sensor data is continually being published by the simulator or robot,
  // we don't want our callbacks to fire until we're in the active state
  if ((!get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) ||
    (!initial_pose_set_)) {return;}

  geometry_msgs::msg::TransformStamped odom_to_base_msg;
  try {
    odom_to_base_msg = tf_buffer_->lookupTransform(
      odom_frame_id_,
      base_frame_id_,
      scan->header.stamp,
      transform_tolerance_);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return;
  }

  // TODO(unassigned): should this run only once?
  geometry_msgs::msg::TransformStamped sensor_pose;
  try {
    sensor_pose = tf_buffer_->lookupTransform(
      base_frame_id_,
      scan->header.frame_id,
      scan->header.stamp,
      transform_tolerance_);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return;
  }
  matcher2d_->setSensorPose(sensor_pose);

  // The estimated robot's pose in the global frame
  geometry_msgs::msg::TransformStamped map_to_base_msg =
    solver_->solve(odom_to_base_msg, scan);

  tf2::Stamped<tf2::Transform> odom_to_base_tf;
  tf2::fromMsg(odom_to_base_msg, odom_to_base_tf);

  tf2::Stamped<tf2::Transform> map_to_base_tf;
  tf2::fromMsg(map_to_base_msg, map_to_base_tf);

  geometry_msgs::msg::TransformStamped map_to_odom_msg;
  map_to_odom_msg.transform = tf2::toMsg(odom_to_base_tf.inverseTimes(map_to_base_tf));
  map_to_odom_msg.header.stamp =
    tf2_ros::toMsg(tf2_ros::fromMsg(scan->header.stamp) + transform_tolerance_);
  map_to_odom_msg.header.frame_id = map_frame_id_;
  map_to_odom_msg.child_frame_id = odom_frame_id_;

  tf_broadcaster_->sendTransform(map_to_odom_msg);
}

}  // namespace nav2_localization
