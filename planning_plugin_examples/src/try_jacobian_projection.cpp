#include <ros/ros.h>

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // toMsg(...)

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

/** I quickly moved some code to header files to have some structure */
#include "planning_plugin_examples/example_util.h"
#include "planning_plugin_examples/example_robot.h"

/** Change this parameters for different robots or planning plugins. */
const std::string FIXED_FRAME = "world";
const std::string PLANNING_GROUP = "manipulator";

namespace rvt = rviz_visual_tools;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/** Compute both the exact and numerical Jacobian for
 * the end-effector's roll pitch yaw velocity
 * and print the error.
 * */
void compareOrientationJacobians(Robot& robot, const int num_runs = 100)
{
  Eigen::VectorXd q_rand;
  for (int i{ 0 }; i < num_runs; ++i)
  {
    q_rand = robot.getRandomJointPositions();

    auto Jexact = robot.jacobianOrientation(q_rand);
    auto Japprox = robot.numericalJacobianOrientation(q_rand);
    Eigen::MatrixXd Jerror = Japprox - Jexact;

    double sum_error = Jerror.lpNorm<1>();
    std::cout << "error: " << sum_error << std::endl;
  }
}

/** 3D pose error expressed using roll pitch yaw angles.
 * (a sequence of elementary rotations around X, Y and then Z, moveing axis).
 * */
Vector6d poseError(const Eigen::Isometry3d& current, const Eigen::Isometry3d& target)
{
  Vector6d error;
  error.head(3) = current.translation() - target.translation();
  error.tail(3) = poseToRPY(current) - poseToRPY(target);
  return error;
}

/** Move a robot from a start joint configuration q_start to an end-effector goal pose.
 * */
void tspaceProject(Robot& robot, Visuals& visuals, const Eigen::VectorXd& q_start, const Eigen::Isometry3d goal_pose)
{
  double step_size = 0.05;
  Eigen::VectorXd q_current{ q_start };
  Eigen::VectorXd delta_q{ q_start };
  Vector6d error = poseError(robot.fk(q_start), goal_pose);
  for (int i{ 0 }; i < 8; ++i)
  {
    error = poseError(robot.fk(q_current), goal_pose);

    // calculate jacobian, clean this up.
    Eigen::MatrixXd Ja = robot.jacobian(q_current);
    Ja.bottomRows(3) = robot.jacobianOrientation(q_current);

    // Resolved motion rate control
    Eigen::MatrixXd Jpinv = Ja.completeOrthogonalDecomposition().pseudoInverse();
    delta_q = Jpinv * error;
    q_current += step_size * delta_q;

    // debugging
    robot.plot(visuals.rvt_, q_current);
    std::cout << "tspace error: " << error.transpose() << std::endl;
    std::cout << "jspace step: " << delta_q.transpose() << std::endl;
    std::cout << "Jacobian: \n";
    std::cout << Ja << std::endl;
    ros::Duration(0.1).sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jacobian_projection");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  Visuals visuals(FIXED_FRAME, node_handle);
  Robot robot(PLANNING_GROUP);

  // compareOrientationJacobians(robot);

  Eigen::VectorXd q_start(6);
  q_start << 0, -1.5, 1.5, 0, 0, 0;
  robot.plot(visuals.rvt_, q_start);

  Eigen::Isometry3d start_pose = robot.fk(q_start);
  Eigen::Isometry3d goal_pose = start_pose * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY());
  goal_pose.translation()[1] += 0.1;

  ROS_INFO_STREAM("start pose: " << start_pose.translation().transpose());

  visuals.plotPose(start_pose);
  ros::Duration(0.1).sleep();
  visuals.plotPose(goal_pose);

  Vector6d error = poseError(start_pose, goal_pose);
  std::cout << error.transpose() << std::endl;

  tspaceProject(robot, visuals, q_start, goal_pose);

 

  // // auto error_pose = goal_pose.inverse() * start_pose;
  // // auto rpy_error = error_pose.rotation().eulerAngles(0, 1, 2);
  // // std::cout << "RPY error: " << rpy_error[0] << ", " << rpy_error[1] << ", " << rpy_error[2] << "\n";
  // // // exact analytical jacobian for rpy angles
  // // auto Ja = angularVelocityToRPYRates(rpy_error[0], rpy_error[1]) * J.bottomRows(3);
  // // std::cout << Ja << std::endl;

  // ROS_INFO_STREAM("Jacobian at start pose: ");
  // std::cout << robot.jacobianOrientation(q_start) << std::endl;

  // ROS_INFO_STREAM("Approximate Jacobian at start pose: ");
  // std::cout << robot.numericalJacobianOrientation(q_start) << std::endl;

  ros::shutdown();
  return 0;
}