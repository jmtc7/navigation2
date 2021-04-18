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

#include "nav2_localization/particle_filter.hpp"

#include <random>

namespace  nav2_localization
{

class ParticleFilter::ParticlesWeightConstIterator : public Particles::const_iterator
{
  ParticlesWeightConstIterator(Particles::const_iterator iter);

  const decltype(Particle::weight)& operator->() const;
};

ParticleFilter::ParticlesWeightConstIterator::ParticlesWeightConstIterator(
  std::vector<Particle>::const_iterator iter)
  : std::vector<Particle>::const_iterator{iter} {}

const decltype(Particle::weight)&
ParticleFilter::ParticlesWeightConstIterator::operator->() const
{
  return operator->().weight;
}

std::discrete_distribution<>
MakeDiscreteDistribution(const ParticleFilter::Particles& particles) {
  return std::discrete_distribution<>(
  ParticleFilter::ParticlesWeightConstIterator(particles.cbegin()),
  ParticleFilter::ParticlesWeightConstIterator(particles.cend()));
}

ParticleFilter::Pose2D::Pose2D(geometry_msgs::msg::Pose pose)
		: position_(pose.position.x, pose.position.y),
      orientation_(tf2::getYaw(pose.orientation)) {}

geometry_msgs::msg::Pose ParticleFilter::Pose2D::ToTPoseMsg() const {
  geometry_msgs::msg::Pose pose;
  pose.position.x = position_[0];
  pose.position.y = position_[1];
  pose.orientation = tf::createQuaternionFromYaw(orientation_);
  return pose;
}

geometry_msgs::msg::PoseWithCovariance
ParticleFilter::Pose2DWithCov::ToPoseWithCovarianceMsg() const {
  geometry_msgs::msg::PoseWithCovariance pose_with_cov;
  pose_with_cov.pose = ToPoseMsg();

  // Copy in the covariance, converting from 3-D to 6-D
  auto& cov = pose_with_cov.covariance;
  for (size_t i = 0; i < 2; ++i) {
    for (size_t j = 0; j < 2; ++j) {
      cov[6*i+j] = covariance_[i][j];
    }
  }
  cov[6*5+5] = covariance_[2][2];

  return pose_with_cov;
}

void ParticleFilter::init(const int & initial_number_of_particles,
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose) {
  decltype(Particle::weight) initial_weight = 1.f/initial_number_of_particles;
  Particle p{initial_pose.pose, initial_weight};
  particles_.resize(initial_number_of_particles, p);
  // TODO: generate randomly distributed poses according to covariance
}

void ParticleFilter::predict(
  const geometry_msgs::msg::TransformStamped& prev_odom,
  const geometry_msgs::msg::TransformStamped& curr_odom) {
  for (auto & p : particles_) {
    p.pose = motion_sampler_->getMostLikelyPose(prev_odom, curr_odom, p.pose);
  }
}

void ParticleFilter::update(sensor_msgs::msg::PointCloud2::ConstSharedPtr laser_scan) {
  decltype(Particle::weight) total_weight = 0;

  // Update particle weights with measurement likelihoods.
  for (auto& p : particles_) {
    p.weight *= matcher_->getScanProbability(laser_scan, p.pose);
    total_weight += p.weight;
  }

  // Normalize weights.
  for (auto& p : particles_) {
    p.weight /= total_weight;
  }

  resample(laser_scan->header.stamp);
}

void ParticleFilter::resample(std_msgs::msg::Time timestamp) {
  std::swap(particles_, prev_particles_);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> weighted_particles(MakeDiscreteDistribution(prev_particles_));
 
  decltype(Particle::weight) default_weight = 1.0/particles_.size();
  // TODO(AMCL): resize particles if neccessary
  for (size_t idx = 0; idx < particles_.size(); ++idx) {
    // Sample an index from the discrete_distribution given by weights in particle set.
    size_t sample_idx = weighted_particles(gen);
    
    particles_[idx].pose = prev_particles_[sample_idx].pose;
    particles_[idx].weight = default_weight;
  }

  updateMostLikelyPose(timestamp);
}

void ParticleFilter::updateMostLikelyPose(std_msgs::msg::Time timestamp) {
  // TODO: calculate means and variances
}

geometry_msgs::msg::PoseWithCovarianceStamped ParticleFilter::getMostLikelyPose() const {
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  // Fill in the header. (TODO: fill in seq?)
  pose.header.frame_id = frame_id_;
  pose.header.stamp = best_est_timestamp_;
  // Copy in the pose.
  pose.pose = best_est_.ToPoseWithCovarianceMsg();
  return pose;
}

}  // namespace nav2_localization
