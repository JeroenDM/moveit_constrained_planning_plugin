#include "constrained_planning_plugin/constraint.h"

#include <eigen_conversions/eigen_msg.h>
#include <cmath>

namespace compl_interface
{
/******************************************
 * Generic constraint methods
 * ****************************************/

PositionConstraint::PositionConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                                       moveit_msgs::Constraints constraints, const unsigned int num_dofs)
  : robot_model_(robot_model), ob::Constraint(num_dofs, 3)
{
  // Setup Moveit's robot model for kinematic calculations
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
  joint_model_group_ = robot_state_->getJointModelGroup(group);

  // Parse constraints
  ROS_INFO_STREAM("Creating position constraints from for shape (" << num_dofs << ", 3)");

  // get out the position constraints from somewhere deep inside this message.
  // do this in init method to call the virtual method of the a child class.
  // fillBoundsFromConstraintsMsg(constraints);

  // use end-effector link by default TODO make this input
  link_name_ = joint_model_group_->getLinkModelNames().back();

  ROS_INFO_STREAM("Created OMPL constraints for link: " << link_name_);
}

void PositionConstraint::init(moveit_msgs::Constraints constraints)
{
  fillBoundsFromConstraintsMsg(constraints);
}

Eigen::Isometry3d PositionConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  robot_state_->setJointGroupPositions(joint_model_group_, joint_values);
  return robot_state_->getGlobalLinkTransform(link_name_);
}

Eigen::MatrixXd PositionConstraint::geometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  robot_state_->setJointGroupPositions(joint_model_group_, joint_values);
  return robot_state_->getJacobian(joint_model_group_);
}

void PositionConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const
{
  auto current_values = calcCurrentValues(x);
  for (std::size_t i{ 0 }; i < bounds_.size(); ++i)
  {
    out[i] = bounds_[i].distance(current_values[i]);
  }
}

void PositionConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const
{
  // !! ASSUME out is filled with zeros by default !!
  auto current_values = calcCurrentValues(x);
  auto current_jacobian = calcCurrentJacobian(x);

  for (std::size_t i{ 0 }; i < bounds_.size(); ++i)
  {
    if (current_values[i] > bounds_[i].upper)
    {
      out.row(i) = current_jacobian.row(i);
    }
    else if (current_values[i] < bounds_[i].lower)
    {
      out.row(i) = -current_jacobian.row(i);
    }
  }
}

/******************************************
 * Position constraints specific
 * ****************************************/

void PositionConstraint::fillBoundsFromConstraintsMsg(moveit_msgs::Constraints constraints)
{
  bounds_.clear();
  bounds_ = positionConstraintMsgToBoundVector(constraints.position_constraints.at(0));
  ROS_INFO_STREAM("Parsed x constraints" << bounds_[0]);
  ROS_INFO_STREAM("Parsed y constraints" << bounds_[1]);
  ROS_INFO_STREAM("Parsed z constraints" << bounds_[2]);

  // extract target / nominal value
  geometry_msgs::Point position =
      constraints.position_constraints.at(0).constraint_region.primitive_poses.at(0).position;
  target_ << position.x, position.y, position.z;
}

Eigen::Vector3d PositionConstraint::calcCurrentValues(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return forwardKinematics(x).translation();
}

Eigen::MatrixXd PositionConstraint::calcCurrentJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  return geometricJacobian(x).topRows(3);
}

/******************************************
 * Orientation constraints specific
 * ****************************************/
void RPYConstraints::fillBoundsFromConstraintsMsg(moveit_msgs::Constraints constraints)
{
  bounds_.clear();
  bounds_ = orientationConstraintMsgToBoundVector(constraints.orientation_constraints.at(0));
  ROS_INFO_STREAM("Parsed rx / roll constraints" << bounds_[0]);
  ROS_INFO_STREAM("Parsed ry / pitch constraints" << bounds_[1]);
  ROS_INFO_STREAM("Parsed rz / yaw constraints" << bounds_[2]);

  // extract target / nominal value
  // for orientation we can save the target in different formats, probably quaternion is the best one here
  // we could use a 3 vector to be uniform with position constraints, but this makes us vulnerable to
  // singularities, wich could occur here as the target can be an arbitrary orientation
  tf::quaternionMsgToEigen(constraints.orientation_constraints.at(0).orientation, target_as_quat_);

  // so we could do this:
  target_ = target_as_quat_.toRotationMatrix().eulerAngles(0, 1, 2);
  // but calcCurrentValues and calcCurrentJacobian use target_as_quat_
}

Eigen::Vector3d RPYConstraints::calcCurrentValues(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // I'm not sure yet whether I want the error expressed in the current ee_frame, or target_frame,
  // or world frame. This implementation expressed the error in the end-effector frame.
  Eigen::Matrix3d error = forwardKinematics(x).rotation().transpose() * target_as_quat_;
  return error.eulerAngles(0, 1, 2);
}

Eigen::MatrixXd RPYConstraints::calcCurrentJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
{
  // use the euler angles of the current end-effector orientation expressed in the world frame
  // we need this to convert the geometric jacobian to an analytical one that can be used for rpy angles
  // the jacobian is expressed in the world frame, so should the rpy angles I suppose...
  auto rpy = forwardKinematics(x).rotation().eulerAngles(0, 1, 2);
  return angularVelocityToRPYRates(rpy[0], rpy[1]) * geometricJacobian(x).bottomRows(3);
}

/******************************************
 * Some utilities
 * ****************************************/

std::shared_ptr<PositionConstraint> createConstraint(robot_model::RobotModelConstPtr robot_model,
                                                     const std::string& group, moveit_msgs::Constraints constraints)
{
  std::size_t num_dofs = robot_model->getJointModelGroup(group)->getVariableCount();
  std::size_t num_pos_con = constraints.position_constraints.size();
  std::size_t num_ori_con = constraints.orientation_constraints.size();

  if (num_pos_con > 0 && num_ori_con > 0)
  {
    ROS_ERROR_STREAM("Combining position and orientation constraints not supported yet.");
    return nullptr;
  }
  else if (num_pos_con > 0)
  {
    if (num_pos_con > 1)
    {
      ROS_ERROR_STREAM("Only a single position constraints supported. Using the first one.");
    }
    auto pos_con = std::make_shared<PositionConstraint>(robot_model, group, constraints, num_dofs);
    pos_con->init(constraints);
    return pos_con;
  }
  else if (num_ori_con > 0)
  {
    if (num_ori_con > 1)
    {
      ROS_ERROR_STREAM("Only a single orientation constraints supported. Using the first one.");
    }
    auto ori_con = std::make_shared<RPYConstraints>(robot_model, group, constraints, num_dofs);
    ori_con->init(constraints);
    return ori_con;
  }
  else
  {
    ROS_ERROR_STREAM("No constraints found in planning request.");
    return nullptr;
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