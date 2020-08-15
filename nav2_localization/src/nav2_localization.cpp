#include <chrono>

#include "nav2_localization/nav2_localization.hpp"
#include "nav2_util/string_utils.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace nav2_localization
{

LocalizationServer::LocalizationServer()
: LifecycleNode("localization_server", "", true),
  sample_motion_model_loader_("nav2_localization", "nav2_localization::SampleMotionModel"),
  default_sample_motion_model_id_("DummyMotionSampler"),
  matcher2d_loader_("nav2_localization", "nav2_localization::Matcher2D"),
  default_matcher2d_id_("DummyMatcher2D"),
  solver_loader_("nav2_localization", "nav2_localization::Solver"),
  default_solver_id_("DummySolver")
{
    RCLCPP_INFO(get_logger(), "Creating localization server");

    declare_parameter("sample_motion_model_id", default_sample_motion_model_id_);
    declare_parameter("matcher2d_id", default_matcher2d_id_);
    declare_parameter("solver_id", default_solver_id_);
    declare_parameter("first_map_only", true);
    declare_parameter("laser_scan_topic_", "scan");
    declare_parameter("odom_frame_id", "odom");
}

LocalizationServer::~LocalizationServer()
{
    RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
LocalizationServer::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Configuring localization interface");

    get_parameter("sample_motion_model_id", sample_motion_model_id_);
    get_parameter("matcher2d_id", matcher2d_id_);
    get_parameter("solver_id", solver_id_);
    get_parameter("first_map_only", first_map_only_);
    get_parameter("laser_scan_topic", scan_topic_);
    get_parameter("odom_frame_id", odom_frame_id_);

    last_time_printed_msg_ = now();

    initTransforms();
    initMessageFilters();
    initPubSub();
    initPlugins();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
    initial_pose_sub_.reset();

    // Laser Scan
    laser_scan_connection_.disconnect();
    laser_scan_filter_.reset();
    laser_scan_sub_.reset();

    // Transforms
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalizationServer::on_activate(const rclcpp_lifecycle::State & state)
{
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
    if (first_map_only_ && first_map_received_) {
        return;
    }
    map_ = *msg;
    first_map_received_ = true;
}

void
LocalizationServer::initMessageFilters()
{
    laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
        rclcpp_node_.get(), scan_topic_, rmw_qos_profile_sensor_data);

    laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10, rclcpp_node_);

    laser_scan_connection_ = laser_scan_filter_->registerCallback(
        std::bind(
            &LocalizationServer::laserReceived,
            this, std::placeholders::_1));
}

void
LocalizationServer::initTransforms()
{
    // Initilize transform listener and broadcaster
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        rclcpp_node_->get_node_base_interface(),
        rclcpp_node_->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);
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

    try {
        sample_motion_model_type_ = nav2_util::get_plugin_type_param(node, sample_motion_model_id_);
        sample_motion_model_ = sample_motion_model_loader_.createUniqueInstance(sample_motion_model_type_);
    } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(get_logger(), "Failed to create sample motion model. Exception: %s", ex.what());
        exit(-1);
    }

    sample_motion_model_->configure(node);

    try {
        matcher2d_type_ = nav2_util::get_plugin_type_param(node, matcher2d_id_);
        matcher2d_ = matcher2d_loader_.createUniqueInstance(sample_motion_model_type_);
    } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(get_logger(), "Failed to create matcher2d. Exception: %s", ex.what());
        exit(-1);
    }

    // TODO
    // matcher2d__->configure();

    try {
        solver_type_ = nav2_util::get_plugin_type_param(node, solver_id_);
        solver_ = solver_loader_.createUniqueInstance(solver_type_);
    } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(get_logger(), "Failed to create solver. Exception: %s", ex.what());
        exit(-1);
    }

    solver_->configure();
}

void
LocalizationServer::initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // TODO
}

bool
LocalizationServer::checkElapsedTime(std::chrono::seconds check_interval, rclcpp::Time last_time)
{
    rclcpp::Duration elapsed_time = now() - last_time;
    if (elapsed_time.nanoseconds() * 1e-9 > check_interval.count()) {
        return true;
    }
    return false;
}

void
LocalizationServer::laserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
{
    // Since the sensor data is continually being published by the simulator or robot,
    // we don't want our callbacks to fire until we're in the active state
    if (!get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {return;}
    if (!first_map_received_) {
        if (checkElapsedTime(2s, last_time_printed_msg_)) {
            RCLCPP_WARN(get_logger(), "Waiting for map....");
            last_time_printed_msg_ = now();
        }
        return;
    }

    std::string laser_scan_frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
    last_laser_received_ts_ = now();
    int laser_index = -1;
    geometry_msgs::msg::PoseStamped laser_pose; 
}

} //nav2_localiztion
