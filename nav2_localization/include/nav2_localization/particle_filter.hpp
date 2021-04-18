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

#ifndef NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_
#define NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_

#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_localization/interfaces/sample_motion_model_base.hpp"
#include "nav2_localization/interfaces/matcher2d_base.hpp"
#include <Eigen/Core>

namespace nav2_localization
{
class ParticleFilter
{
public:
	// TODO(unassigned): Pass initial pose
	ParticleFilter(
	  SampleMotionModel::Ptr & motion_sampler, Matcher2d::Ptr & matcher);
	void init(const int & initial_number_of_particles,
	          geometry_msgs::msg::PoseWithCovarianceStamped pose);
 // Motion Update:
  void predict(const geometry_msgs::msg::TransformStamped& prev_odom,
  	           const geometry_msgs::msg::TransformStamped& curr_odom);
  // Measurement Update:
  void update(sensor_msgs::msg::PointCloud2::ConstSharedPtr laser_scan);
	// Calculate a best estimate pose:
  geometry_msgs::msg::PoseWithCovarianceStamped getMostLikelyPose() const;

	class ParticlesWeightConstIterator;

private:
  void resample(std_msgs::msg::Time); // Part of measurement update.

	void updateMostLikelyPose(std_msgs::msg::Time);

	struct Pose2D {
		Eigen::Vector2d position_;
		double orientation_;

		Pose2D(geometry_msgs::msg::Pose pose); {}
		geometry_msgs::msg::Pose ToTPoseMsg() const;
	};

	struct Pose2DWithCov : public Pose2D{
		Eigen::Matrix3d covariance_;

		geometry_msgs::msg::PoseWithCovariance ToPoseWithCovarianceMsg() const;
	};

	struct Particle
	{
		Pose2D pose;
		double weight;
	};

	using Particles = std::vector<Particle>;
	Particles particles_;
	Pose2DWithCov best_est_;
	std_msgs::msg::Time best_est_timestamp_;
	Particles prev_particles_;

	SampleMotionModel::Ptr motion_sampler_;
  Matcher2d::Ptr matcher_;

	std::string frame_id_;
};
}  // namespace nav2_localization

#endif  // NAV2_LOCALIZATION__PARTICLE_FILTER_HPP_