#include "constrained_planning_plugin/compl_planning_context.h"

#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_state/conversions.h>

namespace compl_interface
{
COMPLPlanningContext::COMPLPlanningContext(const std::string& name, const std::string& group,
                                           moveit::core::RobotModelConstPtr robot_model)
  : PlanningContext(name, group), robot_model_(robot_model), joint_model_group_(robot_model->getJointModelGroup(group))
{
  robot_state_.reset(new moveit::core::RobotState(robot_model));
  robot_state_->setToDefaultValues();
  compl_interface_ = COMPLInterfacePtr(new COMPLInterface());
}

void COMPLPlanningContext::clear()
{
}

bool COMPLPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  // ROS_INFO_STREAM("Solving a motion planning request.");

  // std::vector<double> tmp_joint_positions {0, 0, 0, 0, 0, 0, 0};
  // robot_state_->setJointGroupPositions(joint_model_group_,tmp_joint_positions);
  // auto fk = robot_state_->getGlobalLinkTransform("panda_hand");
  // ROS_INFO_STREAM("Forward kinematics: " << fk.translation());

  // TODO figure out how to do selection based on request content
  bool use_current_state{ false };
  Eigen::VectorXd start_joint_positions(7);
  if (use_current_state)
  {
    auto start_state = planning_scene_->getCurrentState();
    start_state.copyJointGroupPositions(joint_model_group_, start_joint_positions);
  }
  else
  {
    // read start state from planning request
    robot_state::RobotState start_state(robot_model_);
    moveit::core::robotStateMsgToRobotState(request_.start_state, start_state);
    start_state.copyJointGroupPositions(joint_model_group_, start_joint_positions);
    // or cast joint positions from std vector to Eigen?
  }

  ROS_INFO_STREAM("Start state: " << start_joint_positions);

  // extract goal from planning request
  Eigen::VectorXd goal_joint_positions(7);
  ROS_INFO_STREAM("num goal constraints: " << request_.goal_constraints.size());
  std::size_t joint_index{ 0 };
  for (auto& joint_constraint : request_.goal_constraints[0].joint_constraints)
  {
    ROS_INFO_STREAM("name: " << joint_constraint.joint_name << " value: " << joint_constraint.position);
    goal_joint_positions[joint_index] = joint_constraint.position;
    joint_index++;
  }
  ROS_INFO_STREAM("goal state: " << goal_joint_positions);

  compl_interface_->preSolve(robot_model_, "panda_arm");

  auto success = compl_interface_->solve(start_joint_positions, goal_joint_positions);

  if (success)
  {
    compl_interface_->postSolve();
    // res.trajectory_ = createRobotTrajectoryFromSolution(compl_interface_->getSolutionPath());
  }

  return success;
}

bool COMPLPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  return false;
}

bool COMPLPlanningContext::terminate()
{
  return true;
}

robot_trajectory::RobotTrajectoryPtr
COMPLPlanningContext::createRobotTrajectoryFromSolution(std::vector<Eigen::VectorXd> path)
{
  auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, request_.group_name);
  auto state = std::make_shared<moveit::core::RobotState>(planning_scene_->getCurrentState());

  for (std::size_t path_index = 0; path_index < path.size(); ++path_index)
  {
    size_t joint_index = 0;
    for (const moveit::core::JointModel* jm : trajectory->getGroup()->getActiveJointModels())
    {
      assert(jm->getVariableCount() == 1);
      state->setVariablePosition(jm->getFirstVariableIndex(), path[path_index][joint_index++]);
    }
    trajectory->addSuffixWayPoint(state, 0.0);
  }
}
}  // namespace compl_interface