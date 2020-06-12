#include "constrained_planning_plugin/constraint.h"

#include <eigen_conversions/eigen_msg.h>
#include <cmath>

namespace compl_interface
{
COMPLConstraint::COMPLConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                                 moveit_msgs::Constraints constraints)
  : robot_model_(robot_model), dimension_(3), ob::Constraint(7, 3)
{
  ROS_INFO_STREAM("--- creating constraints from this input: ---");
  ROS_INFO_STREAM(constraints);
  ROS_INFO_STREAM("---------------------------------------------");

  // hardcoded constraints for debugging:
  // double TOL{ 1e-6 }; /* use a tolerance for equality constraints. */
  // position_bounds_ = { { 0.3 - TOL, 0.3 + TOL }, { -0.3, 0.3 }, { 0.6, 0.7 } };

  // get out the position constraints from somewhere deep inside this message.
  position_bounds_ = positionConstraintMsgToBoundVector(constraints.position_constraints.at(0));
  ROS_INFO_STREAM("Parsed constraints" << position_bounds_[0]);
  ROS_INFO_STREAM("Parsed constraints" << position_bounds_[1]);
  ROS_INFO_STREAM("Parsed constraints" << position_bounds_[2]);

  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
  joint_model_group_ = robot_state_->getJointModelGroup(group);

  link_name_ = joint_model_group_->getLinkModelNames().back();

  ROS_INFO_STREAM("Created OMPL constraints for link: " << link_name_);

  // orientation constraints experiments
  tf::quaternionMsgToEigen(constraints.orientation_constraints.at(0).orientation, desired_ee_quat_);
  orientation_bounds_ = orientationConstraintMsgToBoundVector(constraints.orientation_constraints.at(0));
  ROS_INFO_STREAM("Parsed ori constraints" << orientation_bounds_[0]);
  ROS_INFO_STREAM("Parsed ori constraints" << orientation_bounds_[1]);
  ROS_INFO_STREAM("Parsed ori constraints" << orientation_bounds_[2]);
}

Eigen::Isometry3d COMPLConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_positions) const
{
  robot_state_->setJointGroupPositions(joint_model_group_, joint_positions);
  return robot_state_->getGlobalLinkTransform(link_name_);
}

void COMPLConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const
{
  // auto fk = forwardKinematics(x);
  // for (std::size_t i{ 0 }; i < dimension_; ++i)
  // {
  //   out[i] = position_bounds_[i].distance(fk.translation()[i]);
  // }

  auto fk = forwardKinematics(x);
  auto diff = fk.rotation().transpose() * desired_ee_quat_;
  auto rpy = diff.eulerAngles(0, 1, 2);
  // out[0] = orientation_bounds_[1].distance(rpy[1]);
  // // out[0] = rpy[0];
  // // out[1] = rpy[1];
  // // out[2] = rpy[2];
  for (std::size_t i{ 0 }; i < dimension_; ++i)
  {
    out[i] = orientation_bounds_[i].distance(rpy[i]);
  }
  // std::cout << out[0]  << ", "<< out[1] << ", " << out[2] <<  ", " << std::endl;
}

void COMPLConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const
{
  // ASSUME out is filled with zeros by default
  // robot_state_->setJointGroupPositions(joint_model_group_, x);
  // auto fk = forwardKinematics(x);
  // auto J = robot_state_->getJacobian(joint_model_group_);

  // for (std::size_t i{ 0 }; i < dimension_; ++i)
  // {
  //   double fk_i = fk.translation()[i];
  //   if (fk_i > position_bounds_[i].upper)
  //   {
  //     out.row(i) = J.row(i);
  //   }
  //   else if (fk_i < position_bounds_[i].lower)
  //   {
  //     out.row(i) = -J.row(i);
  //   }
  // }

  robot_state_->setJointGroupPositions(joint_model_group_, x);
  auto fk = forwardKinematics(x);
  auto J = robot_state_->getJacobian(joint_model_group_);

  auto diff = fk.rotation().transpose() * desired_ee_quat_;
  auto rpy = diff.eulerAngles(0, 1, 2);

  auto Ja = angularVelocityToRPYRates(rpy[0], rpy[1]) * J.bottomRows(3);
  // out = Ja;

   for (std::size_t i{ 0 }; i < dimension_; ++i)
  {
    if (rpy[i] > orientation_bounds_[i].upper)
    {
      out.row(i) = Ja.row(i);
    }
    else if (rpy[i] < orientation_bounds_[i].lower)
    {
      out.row(i) = -Ja.row(i);
    }
  }
}

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
std::vector<Bound> positionConstraintMsgToBoundVector(moveit_msgs::PositionConstraint pos_con)
{
  auto pos = pos_con.constraint_region.primitive_poses.at(0).position;
  auto dims = pos_con.constraint_region.primitives.at(0).dimensions;

  // dimension of -1 signifies no constraints, so set to infinity
  for (auto& dim : dims)
  {
    if (dim == -1)
      dim = INF;
  }

  return { { pos.x - dims[0] / 2, pos.x + dims[0] / 2 },
           { pos.y - dims[1] / 2, pos.y + dims[1] / 2 },
           { pos.z - dims[2] / 2, pos.z + dims[2] / 2 } };
}

std::vector<Bound> orientationConstraintMsgToBoundVector(moveit_msgs::OrientationConstraint ori_con)
{
  // Eigen::Quaterniond quat;
  // tf::quaternionMsgToEigen(ori_con.orientation, quat);
  // auto rxyz = quat.matrix().eulerAngles(0, 1, 2);
  std::vector<double> dims{ ori_con.absolute_x_axis_tolerance, ori_con.absolute_y_axis_tolerance,
                            ori_con.absolute_z_axis_tolerance };

  // dimension of -1 signifies no constraints, so set to infinity
  for (auto& dim : dims)
  {
    if (dim == -1)
      dim = INF;
  }

  // express orientation constraints in a local reference frame
  // return { { rxyz.x() - dims[0] / 2, rxyz.x() + dims[0] / 2 },
  //          { rxyz.y() - dims[1] / 2, rxyz.y() + dims[1] / 2 },
  //          { rxyz.z() - dims[2] / 2, rxyz.z() + dims[2] / 2 } };
  return { { -dims[0] / 2, dims[0] / 2 }, { -dims[1] / 2, dims[1] / 2 }, { -dims[2] / 2, dims[2] / 2 } };
}

std::ostream& operator<<(std::ostream& os, const Bound& bound)
{
  os << "Bounds: ";
  os << "( " << bound.lower;
  os << ", " << bound.upper << " )";
  return os;
}

}  // namespace compl_interface