#include "constrained_planning_plugin/constraint.h"

namespace compl_interface
{
COMPLConstraint::COMPLConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group)
  : robot_model_(robot_model), ob::Constraint(7, 1)
{
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
  joint_model_group_ = robot_state_->getJointModelGroup(group);

  link_name_ = joint_model_group_->getLinkModelNames().back();

  ROS_INFO_STREAM("Created OMPL constraints for link: " << link_name_);
}

Eigen::Isometry3d COMPLConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_positions) const
{
  robot_state_->setJointGroupPositions(joint_model_group_, joint_positions);
  return robot_state_->getGlobalLinkTransform(link_name_);
}

void COMPLConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const
{
  auto fk = forwardKinematics(x);
  auto rpy = fk.rotation().eulerAngles(0, 1, 2);

  // y rotation zero
  // out[0] = rpy[0];
  // out[0] = 0.0;

  // fixed x_position
  // 0.3 is the know position of the end-effector
  // at the start and end of the trajectory
  // out[0] = fk.translation()[0] - 0.3;

  // try bounds constraints
  // robot starts with end-effector at 0.6 m and ends
  // with end-effector at 0.7 m
  // here we try to stay in between these bounds
  double z_position = fk.translation()[2];
  if (z_position > 0.7)
  {
    out[0] = z_position - 0.7;
  }
  else if (z_position < 0.6)
  {
    out[0] = 0.6 - z_position;
  }
  else
  {
    out[0] = 0.0;
  }
}

// void COMPLConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x,
//                                Eigen::Ref<Eigen::MatrixXd> out) const
// {
//   robot_state_->setJointGroupPositions(joint_model_group_, x);

//   auto J = robot_state_->getJacobian(joint_model_group_);
//   // out = J.row(3);
//   out = J.row(0);
  
// }
}  // namespace compl_interface