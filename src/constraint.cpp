#include "constrained_planning_plugin/constraint.h"

namespace compl_interface
{
COMPLConstraint::COMPLConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group)
  : robot_model_(robot_model), link_name_("panda_hand"), ob::Constraint(7, 1)
{
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
  joint_model_group_ = robot_state_->getJointModelGroup(group);

  // hardcoded TODO link name
  // joint_model_group_->getEndEffectorTips()
}

Eigen::Isometry3d COMPLConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_positions) const
{
  robot_state_->setJointGroupPositions(joint_model_group_, joint_positions);
  return robot_state_->getGlobalLinkTransform(link_name_);
}

void COMPLConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const
{
  auto fk = forwardKinematics(x);
  auto rpy =  fk.rotation().eulerAngles(0, 1, 2);

  // y rotation zero
  out[0] = rpy[1];
}
}  // namespace compl_interface