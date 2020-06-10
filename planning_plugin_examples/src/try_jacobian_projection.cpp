#include <ros/ros.h>

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

private:
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  planning_scene::PlanningScenePtr planning_scene_; /* I should probably use the planning scene monitor */
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jacobian_projection");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  Visuals visuals("world", node_handle);
  Robot robot;

  Eigen::VectorXd q_start(6);
  q_start << 0, -1.5, 1.5, 0, 0, 0;
  robot.plot(visuals.rvt_, q_start);

  auto start_pose = robot.fk(q_start);
  visuals.plotPose(start_pose);

  Eigen::Isometry3d goal_pose = start_pose * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY());
  goal_pose.translation()[1] += 0.1;

  visuals.plotPose(goal_pose);

  auto J = robot.jacobian(q_start);
  ROS_INFO_STREAM("Jacobian at start pose: ");
  std::cout << J << std::endl;

  ros::shutdown();
  return 0;
}