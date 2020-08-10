#ifndef NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_PLUGINS_HPP_
#define NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_PLUGINS_HPP_

#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nav2_localization
{
class DummyMotionSampler : public nav2_localization::SampleMotionModel
{
public:
    DummyMotionSampler(){}

    geometry_msgs::msg::Pose getMostLikelyPose(
        const nav_msgs::msg::Odometry& prev_odom,
        const nav_msgs::msg::Odometry& curr_odom,
        const geometry_msgs::msg::Pose& prev_pose);

    void configure(
        const double& alpha1,
        const double& alpha2,
        const double& alpha3,
        const double& alpha4,
        const double& alpha5);
};
} // nav2_localization

#endif // NAV2_LOCALIZATION__SAMPLE_MOTION_MODEL_PLUGINS_HPP_