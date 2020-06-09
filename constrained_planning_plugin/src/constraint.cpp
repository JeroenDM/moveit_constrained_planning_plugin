#include "constrained_planning_plugin/constraint.h"

namespace compl_interface
{
COMPLConstraint::COMPLConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                                 moveit_msgs::Constraints constraints)
  : robot_model_(robot_model), dimension_(3), ob::Constraint(7, 3)
{
  ROS_INFO_STREAM("--- creating constraints from this input: ---");
  ROS_INFO_STREAM(constraints);
  ROS_INFO_STREAM("---------------------------------------------");

  double TOL{ 1e-6 }; /* use a tolerance for equality constraints. */
  position_bounds_ = { { 0.3 - TOL, 0.3 + TOL }, { -0.3, 0.3 }, { 0.6, 0.7 } };

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
  for (std::size_t i{ 0 }; i < dimension_; ++i)
  {
    out[i] = position_bounds_[i].distance(fk.translation()[i]);
  }
}

void COMPLConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const
{
  // ASSUME out is filled with zeros by default
  robot_state_->setJointGroupPositions(joint_model_group_, x);
  auto fk = forwardKinematics(x);
  auto J = robot_state_->getJacobian(joint_model_group_);

  for (std::size_t i{ 0 }; i < dimension_; ++i)
  {
    double fk_i = fk.translation()[i];
    if (fk_i > position_bounds_[i].upper)
    {
      out.row(i) = J.row(i);
    }
    else if (fk_i < position_bounds_[i].lower)
    {
      out.row(i) = -J.row(i);
    }
  }
}
}  // namespace compl_interface