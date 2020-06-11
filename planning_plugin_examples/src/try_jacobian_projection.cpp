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

/** Change this parameters for different robots or planning plugins. */
const std::string FIXED_FRAME = "world";
const std::string PLANNING_GROUP = "manipulator";
const std::string ROBOT_DESCRIPTION = "robot_description";

namespace rvt = rviz_visual_tools;

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
    rvt_->publishAxis(pose, rvt::LARGE);
    rvt_->trigger();
  }

  moveit_visual_tools::MoveItVisualToolsPtr rvt_;
  ros::Publisher display_publisher;
};

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

Eigen::Vector3d poseToRPY(const Eigen::Isometry3d& p)
{
  return p.rotation().eulerAngles(0, 1, 2);
}

Eigen::Vector3d rotationToRPY(const Eigen::Matrix3d& r)
{
  return r.eulerAngles(0, 1, 2);
}

/** Hide all MoveIt stuff in a class
 *
 * q = vector with joint positions
 */
class Robot
{
public:
  Robot()
  {
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(ROBOT_DESCRIPTION);
    robot_model_ = robot_model_loader_->getModel();
    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    joint_model_group_ = robot_state_->getJointModelGroup(PLANNING_GROUP);
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    planning_scene_->getCurrentStateNonConst().setToDefaultValues();
  }

  const Eigen::Isometry3d fk(const std::vector<double>& q, const std::string& frame = "tool_tip") const
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getGlobalLinkTransform(frame);
  }

  const Eigen::Isometry3d fk(const Eigen::VectorXd q, const std::string& frame = "tool_tip") const
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getGlobalLinkTransform(frame);
  }

  Eigen::MatrixXd jacobian(const Eigen::VectorXd q)
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    return robot_state_->getJacobian(joint_model_group_);
  }

  Eigen::MatrixXd numericalJacobianOrientation(const Eigen::VectorXd q)
  {
    const double h{ 1e-6 }; /* step size for numerical derivation */
    // const std::size_t ndof {q.size()};
    const std::size_t ndof = q.size();

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(3, ndof);

    // helper matrix for differentiation.
    Eigen::MatrixXd Ih = h * Eigen::MatrixXd::Identity(ndof, ndof);

    for (std::size_t dim{ 0 }; dim < 6; ++dim)
    {
      auto rpy = poseToRPY(fk(q));
      auto rpy_plus_h = poseToRPY(fk(q + Ih.col(dim)));
      Eigen::Vector3d col = (rpy_plus_h - rpy) / h;
      J.col(dim) = col;
    }
    return J;
  }

  Eigen::MatrixXd jacobianOrientation(const Eigen::VectorXd q)
  {
    // const std::size_t ndof {q.size()};
    const std::size_t ndof = q.size();
    auto rpy = poseToRPY(fk(q));
    return angularVelocityToRPYRates(rpy[0], rpy[1]) * jacobian(q).bottomRows(3);
  }

  void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, const std::vector<double>& q)
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    mvt->publishRobotState(robot_state_, rvt::DEFAULT);
    mvt->trigger();
  }

  void plot(moveit_visual_tools::MoveItVisualToolsPtr mvt, const Eigen::VectorXd q)
  {
    robot_state_->setJointGroupPositions(joint_model_group_, q);
    mvt->publishRobotState(robot_state_, rvt::DEFAULT);
    mvt->trigger();
  }

  Eigen::VectorXd getRandomJointPositions()
  {
    Eigen::VectorXd joint_values;
    robot_state_->setToRandomPositions(joint_model_group_);
    robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);
    return joint_values;
  }

private:
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  planning_scene::PlanningScenePtr planning_scene_; /* I should probably use the planning scene monitor */
};

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jacobian_projection");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  Visuals visuals("world", node_handle);
  Robot robot;

  compareOrientationJacobians(robot);

  // Eigen::VectorXd q_start(6);
  // q_start << 0, -1.5, 1.5, 0, 0, 0;
  // robot.plot(visuals.rvt_, q_start);

  // auto start_pose = robot.fk(q_start);
  // visuals.rvt_->publishAxis(start_pose, rvt::LARGE);
  // visuals.rvt_->trigger();
  // // visuals.plotPose(start_pose);

  // Eigen::Isometry3d goal_pose = start_pose * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY());
  // goal_pose.translation()[1] += 0.1;

  // visuals.plotPose(goal_pose);

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