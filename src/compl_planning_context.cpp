#include "constrained_planning_plugin/compl_planning_context.h"

#include <moveit/planning_interface/planning_interface.h>

namespace compl_interface
{
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

  compl_interface_->preSolve();

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