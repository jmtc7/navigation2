#ifndef NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_
#define NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace nav2_localization
{

/**
 * @class Matcher2d
 * @brief Abstract interface for a 2D matcher for localization purposes to adhere to with pluginlib
 */
class Matcher2d
{
public:
	using Ptr = std::shared_ptr<nav2_localization::Matcher2d>;

    /**
     * @brief Computes how likely is it to read a measurement given a position in a map
     * @param scan 2D laser-like measurement
     * @param pose Hypothesis on the robot's pose
     * @param map 2D occupancy grid map of the environment where the robot is
     * @return Probability of how likely is it to perceive the inputted measurement assuming that the robot is in the given pose inside the provided map
     */
	virtual float getLikelihood(
		const sensor_msgs::msg::LaserScan& scan,
		const geometry_msgs::msg::Pose& pose,
		const nav_msgs::msg::OccupancyGrid& map) = 0;

protected:
    Matcher2d(){}
};  
} // nav2_localization

#endif // NAV2_LOCALIZATION__MATCHER2D_BASE_HPP_
