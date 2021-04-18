// Copyright (c) 2021 Jose M. TORRES-CAMARA and Khaled SAAD
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

#include "nav2_localization/plugins/solvers/mcl_solver2d.hpp"

#include <vector>
#include <memory>

#include "pluginlib/class_list_macros.hpp"
#include "nav2_localization/interfaces/solver_base.hpp"
#include "nav2_localization/particle_filter.hpp"

namespace nav2_localization
{
geometry_msgs::msg::TransformStamped MCLSolver2d::solve(
  const geometry_msgs::msg::TransformStamped & curr_odom,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scan)
{
  // Motion update
  pf_->predict(prev_odom_, curr_odom);
  prev_odom_ = curr_odom;

  // Measurement update
  pf_->update(scan);

  geometry_msgs::msg::PoseWithCovarianceStamped curr_pose;
  curr_pose = pf_->getMostLikelyPose();

  prev_pose_ = curr_pose;

  return curr_pose;
}

void MCLSolver2d::initFilter(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose) {
    pf_->init(1000, *pose); // TODO init # of particles
  }

void MCLSolver2d::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  SampleMotionModel::Ptr & motionSampler,
  Matcher2d::Ptr & matcher,
  const geometry_msgs::msg::TransformStamped & odom,
  const geometry_msgs::msg::TransformStamped & pose)
{
  node_ = node;

  node_->declare_parameter("num_particles", 1000);

  motionSampler_ = motionSampler;
  matcher_ = matcher;

  // Get configuration and generate PF
  int number_of_particles;
  node_->get_parameter("num_particles", number_of_particles);

  pf_ = std::make_shared<ParticleFilter>(number_of_particles);

  prev_odom_ = odom;
  prev_pose_ = pose;
}

void MCLSolver2d::activate()
{}

void MCLSolver2d::deactivate()
{}

void MCLSolver2d::cleanup()
{}

}  // namespace nav2_localization

PLUGINLIB_EXPORT_CLASS(nav2_localization::MCLSolver2d, nav2_localization::Solver)
