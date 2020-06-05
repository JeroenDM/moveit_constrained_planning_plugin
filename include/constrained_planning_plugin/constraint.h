#ifndef COMPL_INTERFACE_CONSTRAINT_H
#define COMPL_INTERFACE_CONSTRAINT_H

#include <ompl/base/Constraint.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace compl_interface
{
namespace ob = ompl::base;

class COMPLConstraint : public ob::Constraint
{
public:
  COMPLConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group);

  void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;

  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_positions) const;

private:
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  const std::string link_name_;
};

}  // namespace compl_interface

#endif  // COMPL_INTERFACE_CONSTRAINT_H
