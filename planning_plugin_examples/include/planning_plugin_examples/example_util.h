#ifndef CONSTRAINED_PLANNING_EXAMPLE_UTIL
#define CONSTRAINED_PLANNING_EXAMPLE_UTIL

#include <string>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

/** Everyting that has to do with visualization in Rviz
 * is grouped in this class.
 * */
class Visuals
{
public:
  Visuals(const std::string& reference_frame, ros::NodeHandle& node_handle)
  {
    rvt_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(reference_frame, "/rviz_visual_tools");
    rvt_->loadRobotStatePub("/display_robot_state");
    rvt_->enableBatchPublishing();
    rvt_->deleteAllMarkers();
    rvt_->trigger();
    ros::Duration(0.1).sleep();
  }

  void plotPose(const Eigen::Isometry3d pose)
  {
    rvt_->publishAxis(pose, rviz_visual_tools::LARGE);
    rvt_->trigger();
  }

  moveit_visual_tools::MoveItVisualToolsPtr rvt_;
  ros::Publisher display_publisher;
};

/** Inverse of the Conversion matrix from roll-pitch-yaw velocity to angular velocity.
 *
 * w = B(rpy) * rpy_dot
 * w = angular velocity, B = matrix returned by this function,
 * rpy = roll-pitch-yaw angles
 * rpy_dot = roll-pitch-yaw time derivatives.
 * 
 * This function directly calculates B^-1
 * and contains a singularity for ry = +/- pi / 2
 *
 * from: 
 * https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
 * 
 * */
Eigen::Matrix3d angularVelocityToRPYRates(double rx, double ry)
{
  double TOLERANCE{ 1e-9 }; /* TODO what tolerance to use here? */
  Eigen::Matrix3d E;
  double cosy{ std::cos(ry) };

  // check for singular case
  if (std::abs(cosy) < TOLERANCE)
  {
    ROS_ERROR_STREAM("Singularity in orientation path constraints.");
  }

  double cosx{ std::cos(rx) };
  double sinx{ std::sin(rx) };
  double siny{ std::sin(ry) };
  E << 1, sinx * siny / cosy, -cosx * siny / cosy, 0, cosx, sinx, 0, -sinx / cosy, cosx / cosy;
  return E;
}

Eigen::Vector3d rotationToRPY(const Eigen::Matrix3d& r)
{
  Eigen::Vector3d xyz = r.eulerAngles(0, 1, 2);
  // in MoveIt some wrapping is done sometimes, but I'm not sure why or when to apply it yet
  // probably to convert the eigen convention: [0:pi]x[-pi:pi]x[-pi:pi] to a MoveIt convention?
  // but it looks the same to me
  // it does not seem to work in some cases.
  // xyz(0) = std::min(fabs(xyz(0)), M_PI - fabs(xyz(0)));
  // xyz(1) = std::min(fabs(xyz(1)), M_PI - fabs(xyz(1)));
  // xyz(2) = std::min(fabs(xyz(2)), M_PI - fabs(xyz(2)));
  return xyz;
}

Eigen::Vector3d poseToRPY(const Eigen::Isometry3d& p)
{
  return rotationToRPY(p.rotation());
}

#endif  // CONSTRAINED_PLANNING_EXAMPLE_UTIL