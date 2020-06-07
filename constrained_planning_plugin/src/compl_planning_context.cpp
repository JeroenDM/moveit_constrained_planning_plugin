#include "constrained_planning_plugin/compl_planning_context.h"

#include <moveit/planning_interface/planning_interface.h>

namespace compl_interface
{
COMPLPlanningContext::COMPLPlanningContext(const std::string& name, const std::string& group, moveit::core::RobotModelConstPtr robot_model)
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

  compl_interface_->preSolve(robot_model_, "panda_arm");

  auto success = compl_interface_->solve();

  if (success)
  {
    compl_interface_->postSolve();
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
}  // namespace compl_interface